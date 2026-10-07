"""Offline TShark WebSocket/JSON-RPC extraction. Never opens a capture interface."""
import argparse
from collections import defaultdict
from decimal import Decimal, DecimalException
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def _field(node, name, attribute='show'):
    field = node.find(f".//field[@name='{name}']")
    return None if field is None else field.get(attribute)


def _integer(value):
    if value is None or not value.isdecimal():
        raise ValueError('invalid_integer')
    return int(value)


def _epoch_ns(value):
    try:
        timestamp = Decimal(value) * 1_000_000_000
        if (not timestamp.is_finite() or not 0 <= timestamp <= 2**63 - 1
                or timestamp != timestamp.to_integral_value()):
            raise ValueError('invalid_epoch')
        return int(timestamp)
    except (DecimalException, TypeError):
        raise ValueError('invalid_epoch') from None


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_json_key')
        result[key] = value
    return result


def _invalid_constant(_):
    raise ValueError('nonfinite_json')


def _rpc(raw):
    value = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                       parse_constant=_invalid_constant)
    if (not isinstance(value, dict) or value.get('jsonrpc') != '2.0'
            or type(value.get('id')) not in (int, str)):
        raise ValueError('invalid_rpc')
    if isinstance(value['id'], str) and len(value['id']) > 4096:
        raise ValueError('invalid_rpc')
    if 'method' in value:
        if (type(value['method']) is not str or not value['method']
                or len(value['method']) > 128 or 'result' in value or 'error' in value):
            raise ValueError('invalid_rpc')
        return value, 'request'
    if ('result' in value) == ('error' in value):
        raise ValueError('invalid_rpc')
    return value, 'response'


def analyze_pdml(document, *, port=3031):
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('invalid_port')
    if isinstance(document, str):
        document = document.encode('utf-8')
    if len(document) > 128 * 1024 * 1024 or b'<!DOCTYPE' in document.upper() or b'<!ENTITY' in document.upper():
        raise ValueError('unsupported_xml')
    try:
        root = ET.fromstring(document)
    except ET.ParseError:
        raise ValueError('invalid_pdml') from None
    if root.tag != 'pdml':
        raise ValueError('invalid_pdml')
    requests, responses = defaultdict(list), defaultdict(list)
    upgrades = {}
    issues, packets = [], root.findall('packet')
    payload_frames, accounted_frames = set(), set()
    pending_tails, later_reassembled = set(), set()
    decoded = 0
    for packet in packets:
        stream = frame = None
        try:
            frame = _integer(_field(packet, 'frame.number'))
            timestamp = _epoch_ns(_field(packet, 'frame.time_epoch'))
            if _field(packet, 'tcp.stream') is None:
                continue
            stream = _integer(_field(packet, 'tcp.stream'))
            source, destination = (_integer(_field(packet, name)) for name in ('tcp.srcport', 'tcp.dstport'))
            if port not in (source, destination) or source == destination:
                continue
        except ValueError:
            issues.append(dict(reason='invalid_packet_metadata', frame=frame, stream=stream))
            continue
        try:
            captured, length = _field(packet, 'frame.cap_len'), _field(packet, 'frame.len')
            if captured is not None and length is not None and _integer(captured) < _integer(length):
                issues.append(dict(reason='truncated_frame', frame=frame, stream=stream))
                continue
        except ValueError:
            issues.append(dict(reason='invalid_packet_metadata', frame=frame, stream=stream))
            continue
        if _field(packet, 'http.response.code') == '101' and source == port:
            upgrades[stream] = timestamp
        for flag in ('tcp.analysis.retransmission', 'tcp.analysis.lost_segment', 'tcp.analysis.out_of_order'):
            if packet.find(f".//field[@name='{flag}']") is not None:
                issues.append(dict(reason='transport_observation', flag=flag, frame=frame, stream=stream))
        try:
            length = _field(packet, 'tcp.len')
            if length is not None and _integer(length) > 0:
                payload_frames.add((stream, frame))
            applications = packet.findall(".//proto[@name='http']") + packet.findall(".//proto[@name='websocket']")
            if applications:
                accounted_frames.add((stream, frame))
                carriers = {_integer(field.get('show'))
                            for field in packet.findall(".//field[@name='tcp.segment']")}
                accounted_frames.update((stream, carrier) for carrier in carriers)
                later_reassembled.update((stream, carrier) for carrier in carriers if carrier < frame)
                if any(_integer(field.get('size')) > 0
                       for field in packet.findall(".//field[@name='tcp.segment_data']")):
                    # Reassembled PDUs also expose their current TCP segment as
                    # segment_data. Subtract bytes supplied by earlier carriers
                    # before deciding whether this packet has a pending tail.
                    pdu_sizes = packet.findall(".//field[@name='tcp.pdu.size']")
                    covered = sum(_integer(field.get('show')) for field in pdu_sizes)
                    covered -= sum(_integer(field.get('show')) for field in
                                   packet.findall(".//field[@name='tcp.reassembled.length']"))
                    covered += sum(_integer(field.get('size')) for field in
                                   packet.findall(".//field[@name='tcp.segment']")
                                   if _integer(field.get('show')) == frame)
                    if length is None or not pdu_sizes or covered < _integer(length):
                        pending_tails.add((stream, frame))
            if packet.find(".//field[@name='tcp.analysis.retransmission']") is not None:
                # Already explicit as a transport observation; not another RPC.
                accounted_frames.add((stream, frame))
        except ValueError:
            issues.append(dict(reason='invalid_packet_metadata', frame=frame, stream=stream))
        for websocket in packet.findall(".//proto[@name='websocket']"):
            if ((_field(websocket, 'websocket.fin', 'value') or _field(websocket, 'websocket.fin')) != '1'
                    or _field(websocket, 'websocket.opcode') != '1'):
                issues.append(dict(reason='unsupported_websocket_fragment', frame=frame, stream=stream))
                continue
            try:
                payload = _field(websocket, 'websocket.payload', 'value')
                if payload is None or len(payload) > 2 * 1024 * 1024:
                    raise ValueError('invalid_payload')
                rpc, kind = _rpc(bytes.fromhex(payload.replace(':', '')))
                if ((kind == 'request' and destination != port)
                        or (kind == 'response' and source != port)):
                    raise ValueError('wrong_direction')
                carriers = [_integer(field.get('show')) for field in packet.findall(".//field[@name='tcp.segment']")]
            except (ValueError, UnicodeError):
                issues.append(dict(reason='invalid_rpc', frame=frame, stream=stream))
                continue
            decoded += 1
            key = (stream, type(rpc['id']).__name__, rpc['id'])
            entry = dict(frame=frame, timestamp_ns=timestamp, carriers=carriers or [frame],
                         method=rpc.get('method'), return_kind='error' if 'error' in rpc else 'result')
            (requests if kind == 'request' else responses)[key].append(entry)
    for stream, frame in sorted(payload_frames - accounted_frames):
        issues.append(dict(reason='unaccounted_tcp_payload', frame=frame, stream=stream))
    for stream, frame in sorted(pending_tails - later_reassembled):
        issues.append(dict(reason='incomplete_application_payload', frame=frame, stream=stream))
    pairs = []
    for key in requests.keys() | responses.keys():
        stream, id_type, identity = key
        outgoing, incoming = requests[key], responses[key]
        detail = dict(stream=stream, id=identity, id_type=id_type)
        if len(outgoing) > 1 or len(incoming) > 1:
            issues.append(dict(reason='ambiguous_id', **detail))
            continue
        if not outgoing:
            issues.append(dict(reason='orphan_response', **detail))
            continue
        if not incoming:
            issues.append(dict(reason='missing_response', **detail))
            continue
        request, response = outgoing[0], incoming[0]
        if stream not in upgrades or upgrades[stream] > request['timestamp_ns']:
            issues.append(dict(reason='missing_upgrade', **detail))
            continue
        if response['timestamp_ns'] < request['timestamp_ns']:
            issues.append(dict(reason='response_before_request', **detail))
            continue
        pairs.append(dict(**detail, method=request['method'],
                          request_complete_frame=request['frame'], response_complete_frame=response['frame'],
                          request_complete_epoch_ns=request['timestamp_ns'],
                          response_complete_epoch_ns=response['timestamp_ns'],
                          request_carrier_frames=request['carriers'], response_carrier_frames=response['carriers'],
                          completion_interval_ns=response['timestamp_ns'] - request['timestamp_ns'],
                          rpc_return_kind=response['return_kind'], first_byte_epoch_ns=None))
    # Capture-wide missing upgrade is explicit even if truncated data decoded no RPC.
    if not upgrades:
        issues.append(dict(reason='missing_upgrade'))
    pairs.sort(key=lambda pair: (pair['request_complete_epoch_ns'], pair['stream'], pair['request_complete_frame']))
    return dict(schema_version=1, packet_count=len(packets), decoded_jsonrpc_messages=decoded,
                matched_pairs=pairs, issues=issues, hardware_stop_verified=False,
                app_clock_alignment_verified=False,
                scope='Dissector completion frames only; not controller receipt or physical stopping time')


def analyze_capture(path, *, tshark, port=3031):
    path, tshark = Path(path).resolve(strict=True), Path(tshark).resolve(strict=True)
    if not path.is_file() or path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError('unsupported_capture_size')
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('invalid_port')
    raw = path.read_bytes()
    command = [str(tshark), '-2', '-n', '-r', str(path), '-d', f'tcp.port=={port},http',
               '-o', 'tcp.desegment_tcp_streams:TRUE', '-o', 'tcp.reassemble_out_of_order:TRUE', '-T', 'pdml']
    with tempfile.TemporaryFile() as output:
        process = subprocess.run(command, stdout=output, stderr=subprocess.PIPE, timeout=30, shell=False)
        if process.returncode != 0:
            raise RuntimeError(f'offline_dissector_exit_{process.returncode}')
        if output.tell() > 128 * 1024 * 1024:
            raise ValueError('dissection_too_large')
        output.seek(0)
        report = analyze_pdml(output.read(), port=port)
    if path.read_bytes() != raw:
        raise ValueError('capture_changed_during_analysis')
    report.update(capture_sha256=hashlib.sha256(raw).hexdigest(),
                  tshark_sha256=hashlib.sha256(tshark.read_bytes()).hexdigest(), offline_only=True)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--tshark', type=Path, default=Path(shutil.which('tshark') or r'C:\Program Files\Wireshark\tshark.exe'))
    parser.add_argument('--port', type=int, default=3031)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve output before invoking any dissector; prior evidence never overwritten.
    with args.output.open('x', encoding='utf-8') as output:
        report = analyze_capture(args.capture, tshark=args.tshark, port=args.port)
        json.dump(report, output, ensure_ascii=False, allow_nan=False, indent=2)
        output.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
