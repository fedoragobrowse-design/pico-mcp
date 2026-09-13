"""Pico MCP server — interact with, program, and flash a Raspberry Pi Pico over USB.

Local stdio MCP server (FastMCP). Designed for the LoRa Mesh v2 project
(see LORA_MESH_V2_PLAN.md): Pico 2 W (RP2350) nodes running Rust/Embassy
firmware flashed via picotool, with a USB-CDC COBS-framed host protocol.

Tool surface (one tool per action, <15 actions):
  pico_status        — detect BOOTSEL drive / running serial device / picotool
  pico_flash         — flash a .uf2 (drive copy or picotool) or .elf (picotool)
  pico_reboot        — reboot device (app <-> BOOTSEL) via picotool
  pico_serial        — write to and read from the USB-CDC serial port
  pico_exec          — run Python code on MicroPython via raw REPL
  pico_file_list     — list files on a MicroPython device
  pico_file_put      — upload a local file to a MicroPython device
"""

from __future__ import annotations

import base64
import glob
import json
import os
import shutil
import subprocess
import re
import time
import sys
from pathlib import Path

import serial  # pyserial
from fastmcp import FastMCP

# §Host protocol ONE framing implementation — vendored (serial_protocol.py,
# proto.py) so this repo is standalone; canonical source: tools/host-cli
sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial_protocol as hostproto  # noqa: E402

mcp = FastMCP("pico", instructions=(
    "Raspberry Pi Pico (RP2040/RP2350) flash + interaction over USB. "
    "Start with pico_status to learn the device state, then act accordingly."
))

BAUD = 115200
_PICOTOOL_CANDIDATES = ("picotool", str(Path.home() / ".local/bin/picotool"))


# ---------------------------------------------------------------- helpers

def _picotool() -> str | None:
    for cand in _PICOTOOL_CANDIDATES:
        p = shutil.which(cand) if "/" not in cand else cand
        if p and Path(p).is_file() and os.access(p, os.X_OK):
            return p
    return None


def _run(cmd: list[str], timeout: float = 30) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return 1, str(e)


def _bootsel_drives() -> list[dict]:
    """Mass-storage drives exposed by Pico BOOTSEL ROM (vendor RPI)."""
    out = _run(["lsblk", "-J", "-o", "NAME,PATH,VENDOR,MODEL,MOUNTPOINT"])[1]
    drives = []
    try:
        for dev in json.loads(out).get("blockdevices", []):
            for d in _flatten(dev):
                if (d.get("vendor") or "").upper().startswith("RPI") or \
                   (d.get("model") or "").upper().startswith("RP"):
                    for part in d.get("children", []) or [d]:
                        mp = part.get("mountpoint")
                        drives.append({
                            "path": part.get("path"),
                            "mountpoint": mp,
                            "info_uf2": _info_uf2(mp),
                        })
    except (json.JSONDecodeError, KeyError):
        pass
    return drives


def _flatten(dev: dict):
    yield dev
    for c in dev.get("children", []):
        yield from _flatten(c)


def _info_uf2(mountpoint: str | None) -> str | None:
    if not mountpoint:
        return None
    f = Path(mountpoint) / "INFO_UF2.TXT"
    if f.is_file():
        try:
            return f.read_text(errors="replace").strip()
        except OSError:
            pass
    return None


def _serial_ports() -> list[dict]:
    ports = []
    for pat in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for p in sorted(glob.glob(pat)):
            entry = {"port": p, "by_id": p.startswith("/dev/serial/by-id")}
            try:
                entry["target"] = os.path.realpath(p) if entry["by_id"] else None
                entry["readable"] = os.access(p, os.R_OK | os.W_OK)
            except OSError:
                pass
            ports.append(entry)
    # dedupe real paths when by-id and ttyACM both match
    seen, uniq = set(), []
    for e in ports:
        key = e.get("target") or e["port"]
        if key not in seen:
            seen.add(key)
            uniq.append(e)
    return uniq


def _resolve_port(port: str | None) -> str:
    if port:
        return port
    ports = _serial_ports()
    if not ports:
        raise RuntimeError(
            "No serial device found. Is the Pico running an application with USB CDC? "
            "In BOOTSEL mode there is no serial port; flash firmware first (pico_flash).")
    # prefer by-id entries (stable, per LORA_MESH_V2_PLAN serial_number addressing)
    for p in ports:
        if p.get("by_id"):
            return p["port"]
    return ports[0]["port"]


def _lsusb_picos() -> list[str]:
    _, out = _run(["lsusb"])
    return [l.strip() for l in out.splitlines()
            if any(t in l for t in ("2e8a:", "ID 2e8a", "Raspberry Pi", "MicroPython"))]


def _permission_hint(port: str, err: Exception) -> str:
    return (f"{err}\nCannot open {port}. Fix with one of:\n"
            f"  sudo usermod -aG dialout {os.environ.get('USER', '$USER')}  (re-login)\n"
            f"or a udev rule with MODE='0666' for this device.")


# ---------------------------------------------------------------- tools

@mcp.tool
def pico_status() -> dict:
    """Report the connected Pico state: BOOTSEL mass-storage drive, running
    serial (CDC) ports, USB devices, and picotool availability. Run this first."""
    bootsel = _bootsel_drives()
    return {
        "mode": "bootsel" if bootsel else ("serial" if _serial_ports() else "unknown"),
        "bootsel_drives": bootsel,
        "serial_ports": _serial_ports(),
        "usb_devices": _lsusb_picos(),
        "picotool": _picotool(),
    }


@mcp.tool
def pico_flash(file: str, wait_seconds: float = 15.0) -> str:
    """Flash firmware to the Pico. Accepts .uf2 or .elf (ELF requires picotool).

    Strategy: if a picotool binary is available use `picotool load -u -v -x`
    (works in both app and BOOTSEL mode). Otherwise, if the device is in
    BOOTSEL mode with a mounted drive, copy the .uf2 onto it (the device
    reboots into the new firmware automatically).
    """
    path = Path(file).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"No such file: {path}")
    kind = path.suffix.lower()

    pt = _picotool()
    if pt:
        flag = "-t elf" if kind == ".elf" else "-t uf2"
        cmd = [pt, "load", "-u", "-v", "-x", flag.split()[1], str(path)]
        code, out = _run(cmd, timeout=120)
        if code == 0:
            return f"Flashed {path.name} via picotool:\n{out}"
        if not bootsel_drive_mounted() and "No accessible device" in out:
            raise RuntimeError(
                f"picotool failed and no BOOTSEL drive found:\n{out}\n"
                "Put the device in BOOTSEL (hold BOOTSEL while plugging in) "
                "and retry with a .uf2 file.")
        # fall through to drive copy for .uf2
        if kind != ".uf2":
            raise RuntimeError(f"picotool failed:\n{out}")

    if kind != ".uf2":
        raise RuntimeError(
            f"{kind} files can only be flashed with picotool "
            "(install: ~/.local/bin/picotool, plus a udev rule granting USB access).")

    mp = bootsel_drive_mounted()
    if not mp:
        raise RuntimeError(
            "Device is not in BOOTSEL mode and picotool is unavailable. "
            "Hold BOOTSEL while plugging in USB, then retry (a drive named "
            "RP2/RP2350 appears). Or run pico_reboot(to_bootloader=true).")

    dest = Path(mp) / path.name
    shutil.copy2(path, dest)
    os.sync()
    # BOOTSEL reboots the device after the write: the drive disappears.
    deadline = time.time() + wait_seconds
    while time.time() < deadline:
        if not bootsel_drive_mounted():
            return f"Flashed {path.name} to {mp}; device rebooted into new firmware."
        time.sleep(0.5)
    return (f"Flashed {path.name} to {mp}. Drive still present after "
            f"{wait_seconds}s — check pico_status; it may need a moment or a re-plug.")


def bootsel_drive_mounted() -> str | None:
    for d in _bootsel_drives():
        if d.get("mountpoint"):
            return d["mountpoint"]
    # Drive present but unmounted: try a polkit-allowed udisks mount.
    for d in _bootsel_drives():
        if d.get("path") and d["path"].startswith("/dev/"):
            code, out = _run(["udisksctl", "mount", "-b", d["path"]], timeout=10)
            if code == 0:
                return _mounted_from(out) or bootsel_drive_mounted()
    return None


def _mounted_from(udisks_out: str) -> str | None:
    # "Mounted /dev/sdb1 at /run/media/user/RP2350"
    m = re.search(r" at (\S+)", udisks_out)
    return m.group(1) if m else None


@mcp.tool
def pico_reboot(to_bootloader: bool = False) -> str:
    """Reboot the Pico: into BOOTSEL (to_bootloader=true, needs firmware with
    picotool/MicroPython support) or into the flash application (default; also
    works from BOOTSEL mode). Requires picotool; without it, use
    machine.bootloader() via pico_exec on MicroPython, or re-plug."""
    out = ""
    pt = _picotool()
    if pt:
        flag = "-u" if to_bootloader else "-a"
        code, out = _run([pt, "reboot", "-f", flag], timeout=30)
        if code == 0:
            return out or ("Rebooted into " + ("BOOTSEL" if to_bootloader else "application") + ".")
    # picotool cannot reboot firmware lacking picotool support (e.g. MicroPython):
    # fall back to the REPL, which is exactly what its message tells us to do.
        try:
            pico_exec("import machine; machine.bootloader()")
        except Exception as e:
            if not any("Boot" in l for l in _lsusb_picos()):
                raise RuntimeError(
                    f"picotool reboot failed:\n{out}\nREPL fallback failed: {e}\n"
                    "Put the device in BOOTSEL manually (hold BOOTSEL while plugging in).")
        return "Rebooted into BOOTSEL via MicroPython machine.bootloader()."
    raise RuntimeError(f"picotool reboot failed:\n{out}")


@mcp.tool
def pico_serial(port: str | None = None, write: str | None = None,
                write_hex: str | None = None, frame: str = "raw",
                read_seconds: float = 2.0, baud: int = BAUD) -> dict:
    """Interact with the Pico over USB-CDC serial: optionally write a string
    (write) or raw bytes (write_hex, e.g. '50494e47'), then read output for
    read_seconds. frame='cobs' wraps the payload in COBS with a 0x00
    delimiter — the framing used by the LoRa mesh hostproto. Returns what
    the device sent back (hex + decoded text)."""
    port = _resolve_port(port)
    payload = bytes.fromhex(write_hex) if write_hex else (
        write.encode() if write is not None else b"")
    if frame == "cobs":
        payload = hostproto.encode_frame(payload)
    try:
        ser = serial.Serial(port, baud, timeout=0.5, exclusive=True)
    except (serial.SerialException, PermissionError) as e:
        raise RuntimeError(_permission_hint(port, e))
    try:
        ser.reset_input_buffer()
        if payload:
            ser.write(payload)
            ser.flush()
        buf = bytearray()
        deadline = time.time() + max(read_seconds, 0.1)
        while time.time() < deadline:
            buf += ser.read(4096)
        text = buf.decode("utf-8", errors="replace")
        result = {"port": port, "sent": payload.hex(),
                  "received_hex": bytes(buf).hex(), "received_text": text}
        if frame == "cobs":
            # decoded hostproto events + bounded framing-error count; raw
            # hex/text above preserved unchanged for raw/silicon sessions
            dec = hostproto.CobsDecoder()
            frames, malformed = hostproto.feed_all(bytes(buf), dec)
            try:
                result["decoded_events"] = [hostproto.parse_event(f) for f in frames]
            except hostproto.BadEvent:
                result["decoded_events"] = [
                    {"event_hex": f.hex(), "note": "frame did not match §"
                     "Host protocol shapes"}
                    for f in frames]
            result["framing_errors"] = malformed
        return result
    finally:
        ser.close()


@mcp.tool
def pico_exec(code: str, port: str | None = None, timeout: float = 10.0) -> str:
    """Run Python code on a MicroPython Pico via the raw REPL and return its
    output. Example: pico_exec('print(2+2)') or
    pico_exec('import machine; print(machine.freq())')."""
    port = _resolve_port(port)
    try:
        ser = serial.Serial(port, BAUD, timeout=1.0, exclusive=True)
    except (serial.SerialException, PermissionError) as e:
        raise RuntimeError(_permission_hint(port, e))
    try:
        ser.reset_input_buffer()
        ser.write(b"\x01")  # Ctrl-A: raw REPL
        if not _wait_for(ser, b">", 3):
            raise RuntimeError("No raw-REPL prompt; is MicroPython running on this port?")
        ser.write(code.encode() + b"\r\x04")
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            buf += ser.read(4096)
            if buf.endswith(b"\x04\x04>"):
                break
        body = bytes(buf)
        if body.endswith(b"\x04\x04>"):
            body = body[:-3]
        text = body.decode("utf-8", errors="replace")
        if text.startswith("OK"):  # raw REPL ack preceding execution output
            text = text[2:]
        # raw REPL: failures carry a traceback between the 0x04 markers
        if "Traceback" in text or "\x1bE" in text:
            raise RuntimeError(f"Device error:\n{text.strip(chr(4))}")
        return text.strip()
    finally:
        ser.write(b"\x02")  # Ctrl-B: back to normal REPL
        ser.close()


def _wait_for(ser: serial.Serial, token: bytes, timeout: float) -> bool:
    deadline, buf = time.time() + timeout, bytearray()
    while time.time() < deadline:
        buf += ser.read(256)
        if buf.endswith(token) or token in buf:
            return True
    return False


@mcp.tool
def pico_file_list(port: str | None = None) -> list[str]:
    """List files in the root of a MicroPython device's filesystem."""
    out = pico_exec("import os; print('\\n'.join(os.listdir()))", port)
    return [l for l in out.splitlines() if l and not l.startswith(("MPY:", "raw REPL"))]


@mcp.tool
def pico_file_put(local_path: str, dest: str, port: str | None = None) -> str:
    """Upload a local file to the MicroPython device's filesystem (e.g.
    dest='main.py' to run at boot). Transfers in 256-byte base64 chunks."""
    data = Path(local_path).expanduser().read_bytes()
    port = _resolve_port(port)
    chunk = 192  # bytes of binary per exec (base64 256 chars)
    pico_exec("import binascii, os", port)
    if pico_exec(f"print('{dest}' in os.listdir())", port).strip() == "True":
        pico_exec(f"os.remove('{dest}')", port)
    pico_exec(f"f = open('{dest}', 'wb')", port)
    for i in range(0, len(data), chunk):
        b64 = base64.b64encode(data[i:i + chunk]).decode()
        pico_exec(f"f.write(binascii.a2b_base64('{b64}'))", port)
    pico_exec("f.close()", port)
    size = pico_exec(f"print(os.stat('{dest}')[6])", port).strip()
    return f"Uploaded {local_path} -> {dest} ({size} bytes, {len(data)} expected)"


def main() -> None:
    mcp.run()  # stdio


if __name__ == "__main__":
    main()
