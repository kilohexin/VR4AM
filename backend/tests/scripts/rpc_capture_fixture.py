"""Create artificial PCAP bytes only. No socket, SDK or capture APIs."""
import base64
import hashlib
import json
import struct
from pathlib import Path

EPOCH_NS = 1_700_000_000_000_000_000
CLIENT_IP = bytes([192, 0, 2, 1])
SERVER_IP = bytes([198, 51, 100, 2])


def checksum(data):
    data += b'\0' if len(data) % 2 else b''
    value = sum(struct.unpack('!' + 'H' * (len(data) // 2), data))
    while value >> 16:
        value = (value & 65535) + (value >> 16)
    return (~value) & 65535


def packet(client, seq, ack, flags, payload):
    source, dest = (CLIENT_IP, SERVER_IP) if client else (SERVER_IP, CLIENT_IP)
    sport, dport = (46464, 3031) if client else (3031, 46464)
    tcp = struct.pack('!HHIIBBHHH', sport, dport, seq, ack, 5 << 4, flags, 65535, 0, 0)
    pseudo = source + dest + struct.pack('!BBH', 0, 6, len(tcp) + len(payload))
    tcp = tcp[:16] + struct.pack('!H', checksum(pseudo + tcp + payload)) + tcp[18:]
    ip = struct.pack('!BBHHHBBH', 0x45, 0, 40 + len(payload), 0, 0x4000, 64, 6, 0) + source + dest
    ip = ip[:10] + struct.pack('!H', checksum(ip)) + ip[12:]
    return bytes.fromhex('0200000000020200000000010800') + ip + tcp + payload


def websocket(value, masked):
    raw = json.dumps(value, separators=(',', ':')).encode()
    assert len(raw) < 126
    if not masked:
        return bytes([0x81, len(raw)]) + raw
    key = bytes([1, 2, 3, 4])
    return bytes([0x81, 128 | len(raw)]) + key + bytes(x ^ key[i % 4] for i, x in enumerate(raw))


def build(root, *, trailing=False, coalesced=False, split_reply=False):
    root = Path(root)
    frames = []
    seq = [1000, 2000]
    def emit(ms, client, payload=b'', flags=0x18, original_seq=None):
        index = 0 if client else 1
        current_seq = seq[index] if original_seq is None else original_seq
        frames.append((EPOCH_NS + int(ms * 1_000_000), packet(client, current_seq, seq[1-index], flags, payload)))
        if original_seq is None:
            seq[index] += len(payload) + int(bool(flags & 2))

    emit(0, True, flags=2)
    emit(1, False, flags=0x12)
    emit(2, True, flags=0x10)
    key = 'dGhlIHNhbXBsZSBub25jZQ=='
    request = ('GET / HTTP/1.1\r\nHost: synthetic.invalid:3031\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode()
    accept = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
    reply = ('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
             f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode()
    emit(3, True, request)
    emit(4, False, reply)
    message = websocket({'jsonrpc': '2.0', 'id': 7, 'method': 'stop_move', 'params': [{}]}, True)
    request_seq = seq[0]
    emit(10, True, message[:24])
    emit(20, True, message[24:])
    emit(21, False, flags=0x10)
    emit(25, True, message[:24], original_seq=request_seq)
    emit(100, False, websocket({'jsonrpc': '2.0', 'id': 99, 'result': {}}, False))
    response = websocket({'jsonrpc': '2.0', 'id': 7, 'result': {}}, False)
    emit(620, False, response[:12])
    emit(630, False, response[12:])
    emit(700, True, websocket({'jsonrpc': '2.0', 'id': 8, 'method': 'get_robot_state', 'params': [{}]}, True))
    last_reply = websocket({'jsonrpc': '2.0', 'id': 8, 'result': 'IDLE'}, False)
    if coalesced:
        last_reply += websocket({'jsonrpc': '2.0', 'id': 9, 'result': {}}, False)[:12]
    if split_reply:
        emit(705, False, last_reply[:10])
        last_reply = last_reply[10:]
    emit(710, False, last_reply)
    if trailing:
        emit(720, True, websocket({'jsonrpc': '2.0', 'id': 9, 'method': 'get_robot_state', 'params': [{}]}, True)[:12])
    path = root / 'synthetic-rpc.pcap'
    with path.open('xb') as capture:
        capture.write(struct.pack('<IHHIIII', 0xa1b23c4d, 2, 4, 0, 0, 65535, 1))
        for timestamp, raw in frames:
            seconds, ns = divmod(timestamp, 1_000_000_000)
            capture.write(struct.pack('<IIII', seconds, ns, len(raw), len(raw)))
            capture.write(raw)
    manifest = dict(synthetic_only=True, network_used=False, frame_count=len(frames),
                    pcap_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    expected=[dict(id=7, method='stop_move', request_complete_ms=20,
                                   response_complete_ms=630, capture_completion_interval_ms=610),
                              dict(id=8, method='get_robot_state', request_complete_ms=700,
                                   response_complete_ms=710, capture_completion_interval_ms=10)],
                    orphan_response_ids=[99], retransmission_frame=9,
                    hardware_stop_verified=False)
    with (root / 'synthetic-manifest.json').open('x', encoding='utf-8') as output:
        json.dump(manifest, output, indent=2)
    print(json.dumps(manifest, indent=2))
