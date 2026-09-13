"""Shared host-side framing + protocol codec for the LoRa mesh host protocol
(v2 plan §Host protocol). This is THE one framing implementation used by both
tools/host-cli/mesh_cli.py and tools/mcp/pico/server.py.

Framing rules (COBS over-zero encoding, single 0x00 delimiter; the vetted
`cobs` package is the raw codec, wrapped here with the exact mesh-core rules):

- wire segment (between delimiters) <= 302 bytes; decoded frame <= 300 bytes;
- literal 0x00 inside a segment, a block run overrunning the segment, or an
  overlong segment is MALFORMED: the decoder consumes it through the next
  delimiter (bounded discard) and re-arms — recovery is always via delimiter;
- one frame may span packets; several frames may share one packet;
- encoder convention (hostproto tests assert it): payload b"" -> b"\x01\x00".

Command lifecycle (§Host protocol): every command first receives CMD_ACK.
ok=0 is followed by ERROR and ends that command; ok=1 admits it. Terminal
responses: PONG (PING), INFO (GET_INFO), QR (EXPORT_QR), CONTACTS_END
(GET_CONTACTS), EVT_SEND_RESULT (SEND_TEXT), EVT_VERIFY_RESULT (VERIFY);
CMD_ACK(ok=1) itself is terminal only for every other command. The CLI
serializes commands, keeps at most ONE unresolved send/verify (10 s / 15 s
abandon timeouts) and never auto-resends.
"""
from __future__ import annotations

import struct
import time
from typing import Callable
from collections import deque

from cobs import cobs as _cobs_codec

import proto  # literal type table (tools/host-cli/proto.py)

# ---------------------------------------------------------------------------
# Raw COBS framing
# ---------------------------------------------------------------------------


class BadFrame(Exception):
    """Malformed or overlong wire segment (bounded discard, no event)."""


class BadEvent(Exception):
    """Well-framed event whose body violates the §Host protocol shapes."""


class BusyError(Exception):
    """A send/verify result is still unresolved when issuing another."""


class SessionStale(Exception):
    """An async send/verify terminal was lost: the 3 core one-unresolved
    session can no longer tell that node's next terminal from the matched
    one. Reuse this connection only after an explicit host reconnect."""
    """A command was issued while a send/verify result is still unresolved."""


def encode_frame(payload: bytes) -> bytes:
    """COBS-encode a protocol frame body and append the 0x00 delimiter."""
    if len(payload) > proto.MAX_FRAME:
        raise BadFrame(f"payload too long: {len(payload)} > {proto.MAX_FRAME}")
    return _cobs_codec.encode(bytes(payload)) + b"\x00"


class Step:
    NONE = "none"
    FRAME = "frame"
    MALFORMED = "malformed"


class CobsDecoder:
    """Streaming packet-boundary-correct decoder mirroring hostproto.rs."""

    MAX_WIRE_SEGMENT = 302
    MAX_FRAME = proto.MAX_FRAME

    def __init__(self) -> None:
        self.raw = bytearray()
        self.decoded = b""

    def reset(self) -> None:
        self.raw.clear()
        self.decoded = b""

    @property
    def frame(self) -> bytes:
        return self.decoded

    def feed(self, chunk: bytes) -> tuple[str, int]:
        """Consume up to and including the first delimiter; return
        (step, consumed). Malformed/overlong segments report MALFORMED after
        being consumed up through their delimiter and re-arm for the next."""
        idx = chunk.find(b"\x00")
        if idx < 0:
            consumed = len(chunk)
            if len(self.raw) + consumed > self.MAX_WIRE_SEGMENT:
                self.raw.clear()
                return Step.MALFORMED, consumed
            self.raw.extend(chunk)
            return Step.NONE, consumed
        consumed = idx + 1
        segment = chunk[:idx]
        if len(self.raw) + len(segment) > self.MAX_WIRE_SEGMENT:
            self.raw.clear()
            return Step.MALFORMED, consumed
        self.raw.extend(segment)
        raw = bytes(self.raw)
        self.raw.clear()
        if not raw:
            return Step.NONE, consumed  # bare delimiter: not a protocol frame
        try:
            decoded = _cobs_codec.decode(raw)
        except Exception:
            return Step.MALFORMED, consumed
        if decoded == b"":
            return Step.NONE, consumed  # empty-decoding frame: no event
        self.decoded = decoded
        return Step.FRAME, consumed


def feed_all(chunk: bytes, decoder: CobsDecoder) -> tuple[list[bytes], int]:
    """Decode every frame in one chunk. Malformed/overlong segments are
    bounded-discarded; returns (frames, malformed_count)."""
    frames: list[bytes] = []
    malformed = 0
    rest = chunk
    while rest:
        step, used = decoder.feed(rest)
        rest = rest[used:]
        if step == Step.FRAME:
            frames.append(decoder.frame)
        elif step == Step.MALFORMED:
            malformed += 1
    return frames, malformed


# ---------------------------------------------------------------------------
# Wire primitives and shape validators
# ---------------------------------------------------------------------------


def _u(n: int, size: int) -> bytes:
    return int(n).to_bytes(size, "big")


def check_idx(idx: int, what: str = "contact_idx") -> int:
    if not isinstance(idx, int) or isinstance(idx, bool):
        raise BadEvent(f"{what}: not an integer")
    if not 0 <= idx <= proto.MAX_CONTACT_IDX:
        raise BadEvent(f"{what} {idx} out of slots 0..=7")
    return int(idx)


def check_bool(v: int, what: str) -> int:
    if not isinstance(v, int) or not (v == 0 or v == 1):
        raise BadEvent(f"{what} must be boolean 0/1, got {v!r}")
    return v


def check_utf8(data: bytes, what: str) -> str:
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as e:
        raise BadEvent(f"{what}: invalid UTF-8: {e}") from e


def parse_name16(raw: bytes) -> str:
    """QR/contact `name[16]`: valid UTF-8 before the first NUL; all remaining
    bytes must be NUL."""
    first_nul = raw.find(b"\x00")
    if first_nul < 0:
        raise BadEvent("name[16] missing NUL terminator")
    name = check_utf8(raw[:first_nul], "name")
    if any(raw[first_nul:]):
        raise BadEvent("name[16] trailing bytes after NUL are not NUL")
    return name


# ---------------------------------------------------------------------------
# Command frame encoders (host -> node). All 13 command types are covered.
# ---------------------------------------------------------------------------


def _end(t: int) -> bytes:
    return bytes([t])


def encode_ping() -> bytes:
    return _end(proto.CMD_PING)


def encode_get_info() -> bytes:
    return _end(proto.CMD_GET_INFO)


def encode_export_qr() -> bytes:
    return _end(proto.CMD_EXPORT_QR)


def encode_get_contacts() -> bytes:
    return _end(proto.CMD_GET_CONTACTS)


def encode_sync_time(epoch_ms: int) -> bytes:
    if not isinstance(epoch_ms, int) or isinstance(epoch_ms, bool):
        raise BadEvent("epoch_ms must be an integer")
    if not 0 <= epoch_ms <= 0xFFFF_FFFF_FFFF_FFFF:
        raise BadEvent("epoch_ms out of u64 range")
    return _end(proto.CMD_SYNC_TIME) + _u(epoch_ms, 8)


def encode_raw_tx(raw: bytes) -> bytes:
    if len(raw) > 255:
        raise BadEvent("CMD_RAW_TX payload > 255 bytes")
    return _end(proto.CMD_RAW_TX) + bytes(raw)


def encode_set_sniff(on: int) -> bytes:
    return _end(proto.CMD_SET_SNIFF) + bytes([check_bool(on, "sniff on")])


def encode_send_text(text: str, idx: int) -> bytes:
    body = text.encode("utf-8")
    if len(body) > proto.MAX_TEXT_BYTES:
        raise BadEvent(f"text is {len(body)} bytes > {proto.MAX_TEXT_BYTES}")
    return _end(proto.CMD_SEND_TEXT) + bytes([check_idx(idx)]) + body


def encode_import_qr(payload: bytes) -> bytes:
    if len(bytes(payload)) != proto.QR_PAYLOAD_LEN:
        raise BadEvent(f"QR payload must be {proto.QR_PAYLOAD_LEN} bytes, got {len(payload)}")
    return _end(proto.CMD_IMPORT_QR) + bytes(payload)


def encode_verify(idx: int) -> bytes:
    return _end(proto.CMD_VERIFY) + bytes([check_idx(idx)])


def encode_set_block(idx: int, blocked: int) -> bytes:
    return _end(proto.CMD_SET_BLOCK) + bytes([check_idx(idx)]) + bytes([check_bool(blocked, "blocked")])


def encode_remove_contact(idx: int) -> bytes:
    return _end(proto.CMD_REMOVE_CONTACT) + bytes([check_idx(idx)])


def encode_test_drop_acks(percent: int) -> bytes:
    if not isinstance(percent, int) or isinstance(percent, bool) or not 0 <= percent <= 100:
        raise BadEvent("percent must be integer 0..=100")
    return _end(proto.CMD_TEST_DROP_ACKS) + bytes([percent])


# ---------------------------------------------------------------------------
# Event parsers (node -> host). parse_event validates the exact §Host protocol
# shapes — sizes, boolean/enum ranges, UTF-8 — and returns dicts.
# ---------------------------------------------------------------------------




# ---------------------------------------------------------------------------
# NodeSession: the §Host protocol command lifecycle over a USB-CDC port
# (any two-way byte stream). This is the ONE lifecycle implementation the
# CLI, the sim bridge and the acceptance harness share.
# ---------------------------------------------------------------------------


class NodeSession:
    def __init__(self, port: str, baud: int = 115_200,
                 on_event: Callable[[dict], None] | None = None):
        import serial  # deferred: codec-only users never need pyserial
        ser = serial.Serial(port, baud, timeout=0.5)
        try:  # one CLI connection per node; TIOCEXCL is best-effort (PTy EIO)
            ser.exclusive = True
        except Exception:
            pass
        self._ser = ser
        self.on_event = on_event or (lambda ev: None)
        self.decoder = CobsDecoder()
        self.malformed_count = 0        # framing errors recovered on link
        self.unsolicited = deque(maxlen=256)  # bounded history (chat UI)
        self.unsolicited_count = 0      # total dispatched (bounded ret only)
        self.events = deque(maxlen=64)  # fresh-lifecycle match queue
        self.contact_cache: dict[int, dict] = {}
        self.needs_resync = False       # set after an unresolved send/verify
        self._pending = False           # one-unresolved rule

    @classmethod
    def open(cls, port: str, **kw) -> "NodeSession":
        return cls(port=port, **kw)

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    # -- low level ---------------------------------------------------------
    def write_frame(self, payload: bytes) -> None:
        """COBS-delimit one protocol frame and put it on the wire."""
        self._ser.write(encode_frame(payload))
        self._ser.flush()

    def drain(self) -> None:
        """Pull whatever the link has, advance decode state and dispatch.
        Malformed/overlong segments are bounded-discarded through their next
        delimiter; subsequent good frames still decode (self-recovery)."""
        try:
            raw = self._ser.read(4096)
        except OSError:
            return  # port closed/disconnected: stop quietly
        if not raw:
            return
        frames, malformed = feed_all(bytes(raw), self.decoder)
        self.malformed_count += malformed
        for fr in frames:
            try:
                ev = parse_event(fr)
            except BadEvent:
                self.bad_event_count = getattr(self, "bad_event_count", 0) + 1
                continue
            t = ev["type"]
            if t == "CONTACT":
                self.contact_cache[ev["idx"]] = ev
            self.unsolicited_count += 1 if t not in ("CMD_ACK", "ERROR") else 0
            if t not in ("CMD_ACK", "ERROR"):
                self.unsolicited.append(ev)
                self.on_event(ev)  # consumers (chat UI/acceptance) take it here
            self.events.append(ev)  # lifecycle match queue, bounded

    def drain_seconds(self, seconds: float) -> None:
        loop_until = time.monotonic() + seconds
        while time.monotonic() < loop_until:
            self._ser.timeout = max(0.05, min(0.5, loop_until - time.monotonic()))
            self.drain()

    def _wait_event(self, names: tuple[str, ...], seconds: float,
                    idx: int | None = None) -> dict | None:
        """Consume events until one of `names` arrives (removed from the
        queue) or the deadline expires; all other events stay queued."""
        deadline = time.monotonic() + seconds
        while True:
            for ev in tuple(self.events):
                if ev["type"] in names and (idx is None or ev.get("idx") == idx):
                    self.events.remove(ev)
                    return ev
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._ser.timeout = min(0.2, remaining)
            self.drain()
    # -- command lifecycle --------------------------------------------------
    def command(self, frame: bytes, timeout: float | None = None) -> dict:
        """Issue ONE serialized command fully per §Host protocol:
        1. CMD_ACK admission: ok=0 -> ERROR ends the command (refused).
        2. CMD_ACK ok=1, command terminal == the ACK itself -> done.
        3. CMD_ACK ok=1 with SEND_TEXT/VERIFY -> await the later async
           terminal (EVT_SEND_RESULT / EVT_VERIFY_RESULT), abandon after
           SEND_TIMEOUT_S / VERIFY_TIMEOUT_S with a KNOWN-UNKNOWN result;
           the command is never auto-resended (event stream is lossy)."""
        if self._pending:
            raise BusyError("one unresolved send/verify at a time")
        if self.needs_resync:
            raise SessionStale(
                "previous send/verify terminal was lost; reconnect the "
                "session before issuing more commands")
        cmd = frame[0]
        expected_idx = frame[1] if cmd in (proto.CMD_VERIFY,) else None
        self.events.clear()   # fresh lifecycle; no stale retro-accept
        self._pending = True
        try:
            self.write_frame(frame)
            admit = self._wait_event(("CMD_ACK",), 5.0)
            if admit is None:
                return {"ok": False, "error": {"type": "ERROR", "code": 0,
                        "name": "CMD_ACK_TIMEOUT",
                        "text": "no CMD_ACK admission within 5 s"}}
            if not admit["ok"]:
                err = self._wait_event(("ERROR",), 2.0)
                return {"ok": False, "error": err if err else {
                    "type": "ERROR", "code": 0, "name": "ACK_0_NO_ERROR",
                    "text": "CMD_ACK(ok=0) without a following ERROR"}}
            terminal_name = proto.TERMINAL_EVENT_NAME.get(cmd)
            if terminal_name and cmd not in (
                    proto.CMD_SEND_TEXT, proto.CMD_VERIFY):
                got = self._wait_event((terminal_name,), 5.0)
                return ({"ok": True, "result": got} if got else
                        {"ok": False, "error": {"type": "ERROR", "code": 0,
                            "name": "TERMINAL_TIMEOUT",
                            "text": f"no terminal {terminal_name} in 5 s"}})
            if cmd in (proto.CMD_SEND_TEXT, proto.CMD_VERIFY):
                name = ("SEND_RESULT" if cmd == proto.CMD_SEND_TEXT
                        else "VERIFY_RESULT")
                seconds = timeout if timeout is not None else (
                    proto.SEND_TIMEOUT_S if cmd == proto.CMD_SEND_TEXT
                    else proto.VERIFY_TIMEOUT_S)
                got = self._wait_event((name,), seconds, idx=expected_idx if cmd == proto.CMD_VERIFY else None)
                if got is None:
                    self.events.clear()
                    self.needs_resync = True   # session can't be trusted
                    return {"ok": None, "status": "unknown", "cmd": cmd,
                            "note": (f"terminal {name} not observed within "
                                     f"{seconds} s; outcome unknown; not "
                                     "re-sent; session marked stale")}
                return {"ok": True, "result": got}
            return {"ok": True}
        finally:
            self._pending = False

def parse_event(frame: bytes) -> dict:
    if not frame:
        raise BadEvent("empty event frame")
    t = frame[0]
    body = frame[1:]
    if t == proto.EVT_PONG:
        if body:
            raise BadEvent(f"EVT_PONG body not empty ({len(body)})")
        return {"type": "PONG"}
    if t == proto.EVT_INFO:
        if len(body) != proto.INFO_BODY_LEN:
            raise BadEvent(f"EVT_INFO body {len(body)} != {proto.INFO_BODY_LEN}")
        return {
            "type": "INFO",
            "serial": body[0:8],
            "pubkey": body[8:40],
            "fw_ver": int.from_bytes(body[40:42], "big"),
            "time_synced": bool(check_bool(body[42], "time_synced")),
            "tx_power_dbm": struct.unpack_from(">b", body, 43)[0],
        }
    if t == proto.EVT_LOG:
        if not body or body[0] not in proto.LOG_LEVELS:
            raise BadEvent(f"EVT_LOG bad level {body[0] if body else '(missing)'}")
        return {"type": "LOG", "level": proto.LOG_LEVELS[body[0]], "text": check_utf8(body[1:], "log")}
    if t == proto.EVT_RX_RAW:
        if len(body) < 3:
            raise BadEvent("EVT_RX_RAW body too short")
        return {
            "type": "RX_RAW",
            "rssi": struct.unpack_from(">h", body, 0)[0],
            "snr": struct.unpack_from(">b", body, 2)[0],
            "raw": body[3:],
        }
    if t == proto.EVT_TEXT_RX:
        # final v2 shape (secure cutover): contact_idx u8 || pkt_id u32 ||
        # rssi i16 || snr i8 || utf8(0..=200). Plaintext path is GONE.
        if len(body) < 8:
            raise BadEvent(f"EVT_TEXT_RX body {len(body)} < 8")
        text = check_utf8(body[8:], "EVT_TEXT_RX text")
        if len(text.encode("utf-8")) > proto.MAX_TEXT_BYTES:
            raise BadEvent("EVT_TEXT_RX text exceeds 200 bytes")
        return {
            "type": "TEXT_RX",
            "contact_idx": check_idx(body[0], "EVT_TEXT_RX idx"),
            "pkt_id": int.from_bytes(body[1:5], "big"),
            "rssi": struct.unpack_from(">h", body, 5)[0],
            "snr": struct.unpack_from(">b", body, 7)[0],
            "utf8": text,
        }
    if t == proto.EVT_SEND_RESULT:
        if len(body) != 6:
            raise BadEvent(f"EVT_SEND_RESULT body {len(body)} != 6")
        status = body[4]
        if status not in (0, 1):
            raise BadEvent(f"EVT_SEND_RESULT status {status} not 0/1")
        return {
            "type": "SEND_RESULT",
            "pkt_id": int.from_bytes(body[0:4], "big"),
            "status": "ack" if status == 0 else "timeout",
            "attempts": body[5],
        }
    if t == proto.EVT_QR:
        if len(body) != proto.QR_PAYLOAD_LEN:
            raise BadEvent(f"EVT_QR body {len(body)} != {proto.QR_PAYLOAD_LEN}")
        return {"type": "QR", "payload": bytes(body)}
    if t == proto.EVT_CONTACT:
        if len(body) != proto.CONTACT_BODY_LEN:
            raise BadEvent(f"EVT_CONTACT body {len(body)} != {proto.CONTACT_BODY_LEN}")
        return {
            "type": "CONTACT",
            "idx": check_idx(body[0], "EVT_CONTACT idx"),
            "serial": bytes(body[1:9]),
            "name": parse_name16(body[9:25]),
            "verified": bool(check_bool(body[25], "verified")),
            "blocked": bool(check_bool(body[26], "blocked")),
            "last_rssi": struct.unpack_from(">h", body, 27)[0],
        }
    if t == proto.EVT_CONTACTS_END:
        if body:
            raise BadEvent(f"EVT_CONTACTS_END body not empty ({len(body)})")
        return {"type": "CONTACTS_END"}
    if t == proto.EVT_VERIFY_RESULT:
        if len(body) != 2:
            raise BadEvent(f"EVT_VERIFY_RESULT body {len(body)} != 2")
        return {
            "type": "VERIFY_RESULT",
            "idx": check_idx(body[0], "EVT_VERIFY_RESULT idx"),
            "ok": bool(check_bool(body[1], "ok")),
        }
    if t == proto.EVT_RELAY:
        if len(body) != 15:
            raise BadEvent(f"EVT_RELAY body {len(body)} != 15")
        return {
            "type": "RELAY",
            "pkt_id": int.from_bytes(body[0:4], "big"),
            "hop_in": body[4],
            "hop_out": body[5],
            "to": int.from_bytes(body[6:10], "big"),
            "from": int.from_bytes(body[10:14], "big"),
            "pay_len": body[14],
        }
    if t == proto.EVT_CMD_ACK:
        if len(body) != 1 or body[0] not in (0, 1):
            raise BadEvent(f"EVT_CMD_ACK ok must be 0/1, body {len(body)}")
        return {"type": "CMD_ACK", "ok": bool(body[0])}
    if t == proto.EVT_ERROR:
        if not body or body[0] not in proto.ERROR_NAMES:
            raise BadEvent(f"EVT_ERROR bad code {body[0] if body else '(missing)'}")
        return {
            "type": "ERROR",
            "code": body[0],
            "name": proto.ERROR_NAMES[body[0]],
            "text": check_utf8(body[1:], "error text"),
        }
    raise BadEvent(f"unknown event type 0x{t:02X}")



