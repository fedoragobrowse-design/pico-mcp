"""§Host protocol constants — literal mirror of crates/mesh-core/src/hostproto.rs.

Every frame: ``type: u8 || body``, big-endian multi-byte integers, stable
``contact_idx`` slots 0..=7. Do not edit a side here without the same edit in
the Rust table (plan: "keep literal-for-literal").
"""

# Validation bounds
MAX_CONTACT_IDX = 7
MAX_TEXT_BYTES = 200          # CMD_SEND_TEXT payload / EVT_TEXT_RX body
QR_PAYLOAD_LEN = 56           # serial[8] || x25519_pub[32] || name[16]
QR_NAME_LEN = 16
QR_SERIAL_LEN = 8
SERIAL_LEN = 8                # node serial (EVT_INFO[0..8], EVT_CONTACT)
PUBKEY_LEN = 32
INFO_BODY_LEN = SERIAL_LEN + PUBKEY_LEN + 2 + 1 + 1   # 44
CONTACT_BODY_LEN = 1 + SERIAL_LEN + QR_NAME_LEN + 1 + 1 + 2  # 29
MIN_FRAME_LEN = 17                # 15-byte header + 2 CRC bytes
MAX_FRAME = 300               # decoded hostproto frame cap
MAX_RAW_TX = 255

# Host -> node commands
CMD_PING = 0x01
CMD_GET_INFO = 0x02
CMD_SYNC_TIME = 0x03
CMD_RAW_TX = 0x04
CMD_SET_SNIFF = 0x05
CMD_SEND_TEXT = 0x06
CMD_EXPORT_QR = 0x07
CMD_IMPORT_QR = 0x08
CMD_GET_CONTACTS = 0x09
CMD_VERIFY = 0x0A
CMD_SET_BLOCK = 0x0B
CMD_REMOVE_CONTACT = 0x0C
CMD_TEST_DROP_ACKS = 0x7F

# Node -> host events
EVT_PONG = 0x81
EVT_INFO = 0x82
EVT_LOG = 0x83
EVT_RX_RAW = 0x84
EVT_TEXT_RX = 0x85
EVT_SEND_RESULT = 0x86
EVT_QR = 0x87
EVT_CONTACT = 0x88
EVT_CONTACTS_END = 0x89
EVT_VERIFY_RESULT = 0x8A
EVT_RELAY = 0x8B
EVT_CMD_ACK = 0x8C
EVT_ERROR = 0x8D

ERROR_NAMES = {
    1: "BAD_CMD", 2: "NO_SUCH_CONTACT", 3: "UNVERIFIED", 4: "BLOCKED",
    5: "NO_TIME", 6: "BUSY", 7: "TOO_LONG", 8: "CONTACTS_FULL",
    9: "DUP_CONTACT", 10: "SELF_CONTACT", 11: "STORAGE",
    12: "COUNTER_EXHAUSTED",
}

LOG_LEVELS = {0: "err", 1: "warn", 2: "info", 3: "debug"}

# Per §Host protocol: one-unresolved rule. send/verify have a LATER
# asynchronous terminal event; their CMD_ACK(ok=1) is admission only.
ADMISSION = 0
SEND_TIMEOUT_S = 10.0     # abandon EVT_SEND_RESULT after 10 s
VERIFY_TIMEOUT_S = 15.0    # abandon EVT_VERIFY_RESULT after 15 s

TERMINAL_FOR = {
    CMD_PING: EVT_PONG,
    CMD_GET_INFO: EVT_INFO,
    CMD_EXPORT_QR: EVT_QR,
    CMD_GET_CONTACTS: EVT_CONTACTS_END,
    CMD_SEND_TEXT: EVT_SEND_RESULT,
    CMD_VERIFY: EVT_VERIFY_RESULT,
    # every other command: CMD_ACK(ok=1) itself is terminal
}

# Event-type NAMES for NodeSession to await terminals by name.
TERMINAL_EVENT_NAME = {
    CMD_PING: "PONG",
    CMD_GET_INFO: "INFO",
    CMD_EXPORT_QR: "QR",
    CMD_GET_CONTACTS: "CONTACTS_END",
    CMD_SEND_TEXT: "SEND_RESULT",
    CMD_VERIFY: "VERIFY_RESULT",
}
