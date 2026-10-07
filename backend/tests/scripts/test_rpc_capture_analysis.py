"""Synthetic dissector output only; no capture, SDK or network access."""
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest
from tests.scripts import rpc_capture_fixture

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('rpc_capture_analysis', ROOT / 'scripts/analyze_rpc_capture.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pdml(messages, *, handshake=True, truncate=False):
    root = ET.Element('pdml')
    if handshake:
        packet = ET.SubElement(root, 'packet')
        for name, value in [('frame.number', '1'), ('frame.time_epoch', '1700000000.0'),
                            ('tcp.stream', '0'), ('tcp.srcport', '3031'), ('tcp.dstport', '46464'),
                            ('http.response.code', '101')]:
            ET.SubElement(packet, 'field', name=name, show=value)
    for index, item in enumerate(messages, 2):
        packet = ET.SubElement(root, 'packet')
        fields = [('frame.number', str(index)), ('frame.time_epoch', item.get('time', '1700000000.020000000')),
                  ('tcp.stream', str(item.get('stream', 0))),
                  ('tcp.srcport', str(46464 if item.get('client', True) else 3031)),
                  ('tcp.dstport', str(3031 if item.get('client', True) else 46464)),
                  ('frame.cap_len', '80' if truncate else '100'), ('frame.len', '100')]
        for name, value in fields:
            ET.SubElement(packet, 'field', name=name, show=value)
        for segment in item.get('segments', []):
            ET.SubElement(packet, 'field', name='tcp.segment', show=str(segment))
        ws = ET.SubElement(packet, 'proto', name='websocket')
        for name, value in [('websocket.fin', str(item.get('fin', 1))),
                            ('websocket.opcode', str(item.get('opcode', 1))),
                            ('websocket.mask', '1' if item.get('client', True) else '0')]:
            ET.SubElement(ws, 'field', name=name, show=value)
        raw = item.get('raw', json.dumps(item.get('rpc')).encode())
        ET.SubElement(ws, 'field', name='websocket.payload', value=raw.hex())
    return ET.tostring(root)


def request(identity=7, **extra):
    return dict(rpc={'jsonrpc': '2.0', 'method': 'stop_move', 'params': [{}], 'id': identity}, **extra)


def response(identity=7, **extra):
    return dict(client=False, time='1700000000.630000000',
                rpc={'jsonrpc': '2.0', 'id': identity, 'result': {}}, **extra)


def test_pair_uses_stream_typed_id_and_integer_nanosecond_completion_times():
    report = module.analyze_pdml(pdml([request(segments=[1, 2]), response()]))
    pair, = report['matched_pairs']
    assert pair['completion_interval_ns'] == 610_000_000
    assert pair['request_complete_epoch_ns'] == 1_700_000_000_020_000_000
    assert pair['response_complete_epoch_ns'] == 1_700_000_000_630_000_000
    assert pair['request_carrier_frames'] == [1, 2]
    assert pair['rpc_return_kind'] == 'result'
    assert report['hardware_stop_verified'] is False
    assert report['app_clock_alignment_verified'] is False
    assert pair['first_byte_epoch_ns'] is None


def test_pdml_boolean_display_uses_raw_value_instead_of_display_label():
    root = ET.fromstring(pdml([request(), response()]))
    for field in root.findall(".//field[@name='websocket.fin']"):
        field.set('show', 'True')
        field.set('value', '1')
    report = module.analyze_pdml(ET.tostring(root))
    assert len(report['matched_pairs']) == 1


def test_orphan_response_and_missing_reply_are_reported_without_false_pairing():
    report = module.analyze_pdml(pdml([request(7), response(99)]))
    assert report['matched_pairs'] == []
    assert {i['reason'] for i in report['issues']} >= {'missing_response', 'orphan_response'}


@pytest.mark.parametrize('which', ['request', 'response'])
def test_duplicate_id_is_ambiguous_and_no_pair_is_selected(which):
    values = [request(), response()]
    values.append(request() if which == 'request' else response())
    report = module.analyze_pdml(pdml(values))
    assert report['matched_pairs'] == []
    assert 'ambiguous_id' in {i['reason'] for i in report['issues']}


@pytest.mark.parametrize('other', [request('7'), request(7, stream=1)])
def test_string_integer_ids_and_different_streams_are_not_conflated(other):
    report = module.analyze_pdml(pdml([request(), other, response()]))
    assert len(report['matched_pairs']) == 1
    assert report['matched_pairs'][0]['id'] == 7
    assert any(i['reason'] == 'missing_response' for i in report['issues'])


@pytest.mark.parametrize('identity', [True, None, 1.5, [], {}])
def test_unsupported_id_is_not_used_for_matching(identity):
    report = module.analyze_pdml(pdml([request(identity), response(identity)]))
    assert not report['matched_pairs']
    assert any(i['reason'] == 'invalid_rpc' for i in report['issues'])


@pytest.mark.parametrize('value', [dict(raw=b'{broken'), dict(fin=0), dict(opcode=0)])
def test_malformed_or_fragmented_messages_do_not_become_rpc_pairs(value):
    report = module.analyze_pdml(pdml([request(**value), response()]))
    assert not report['matched_pairs']
    assert report['issues']


def test_duplicate_json_keys_do_not_choose_an_arbitrary_id():
    raw = b'{"jsonrpc":"2.0","id":7,"id":8,"method":"stop_move"}'
    report = module.analyze_pdml(pdml([request(raw=raw), response()]))
    assert not report['matched_pairs']


def test_missing_handshake_and_truncated_frames_remain_explicit():
    report = module.analyze_pdml(pdml([request(), response()], handshake=False, truncate=True))
    assert report['matched_pairs'] == []
    assert {'missing_upgrade', 'truncated_frame'} <= {i['reason'] for i in report['issues']}


@pytest.mark.parametrize('timestamp', ['NaN', '-1', '1700000000.0000000001', 'bad'])
def test_invalid_or_subnanosecond_timestamp_is_rejected(timestamp):
    report = module.analyze_pdml(pdml([request(time=timestamp), response()]))
    assert not report['matched_pairs']
    assert any(i['reason'] == 'invalid_packet_metadata' for i in report['issues'])


def test_reply_earlier_than_request_is_not_latency_evidence():
    report = module.analyze_pdml(pdml([request(time='1700000001.0'), response()]))
    assert not report['matched_pairs']
    assert any(i['reason'] == 'response_before_request' for i in report['issues'])


def test_error_reply_is_not_reported_as_success():
    reply = response()
    reply['rpc'].pop('result')
    reply['rpc']['error'] = {'code': -1, 'message': 'synthetic error'}
    pair, = module.analyze_pdml(pdml([request(), reply]))['matched_pairs']
    assert pair['rpc_return_kind'] == 'error'
    assert 'synthetic error' not in json.dumps(pair)


@pytest.mark.parametrize('xml', [b'not xml', b'<other/>', b'<!DOCTYPE pdml><pdml/>'])
def test_invalid_dissection_document_is_rejected(xml):
    with pytest.raises(ValueError):
        module.analyze_pdml(xml)


def test_cli_refuses_existing_output_before_running_tshark(tmp_path, monkeypatch):
    output = tmp_path / 'result.json'
    output.write_text('original')
    def forbidden(*_args, **_kwargs):
        pytest.fail('dissector must not run')
    monkeypatch.setattr(module, 'analyze_capture', forbidden)
    with pytest.raises(FileExistsError):
        module.main([str(tmp_path / 'capture.pcapng'), '--output', str(output)])
    assert output.read_text() == 'original'


def test_actual_tshark_reassembles_generated_packets_without_duplicate_rpc(tmp_path):
    tshark = Path(shutil.which('tshark') or r'C:\Program Files\Wireshark\tshark.exe')
    if not tshark.is_file():
        pytest.skip('TShark not installed; remaining tests use synthetic PDML')
    rpc_capture_fixture.build(tmp_path)
    report = module.analyze_capture(tmp_path / 'synthetic-rpc.pcap', tshark=tshark)
    assert report['packet_count'] == 14
    assert report['decoded_jsonrpc_messages'] == 5
    assert [pair['completion_interval_ns'] for pair in report['matched_pairs']] == [610_000_000, 10_000_000]
    assert {issue['reason'] for issue in report['issues']} == {'transport_observation', 'orphan_response'}
    assert all(pair['first_byte_epoch_ns'] is None for pair in report['matched_pairs'])
    assert report['offline_only'] and not report['hardware_stop_verified']


def test_oversized_epoch_is_rejected_before_integer_allocation():
    report = module.analyze_pdml(pdml([request(time='1e999999'), response()]))
    assert not report['matched_pairs']


def test_undissected_trailing_tcp_payload_is_explicit_with_earlier_valid_pair():
    root = ET.fromstring(pdml([request(), response(), request(9)]))
    packet = root.findall('packet')[-1]
    packet.remove(packet.find("proto[@name='websocket']"))
    ET.SubElement(packet, 'field', name='tcp.len', show='12')
    report = module.analyze_pdml(ET.tostring(root))
    assert len(report['matched_pairs']) == 1
    assert dict(reason='unaccounted_tcp_payload', frame=4, stream=0) in report['issues']


def test_payload_carrier_reassembled_in_later_message_is_accounted_for():
    root = ET.fromstring(pdml([request(9), request(segments=[2, 3]), response()]))
    packet = root.findall('packet')[1]
    packet.remove(packet.find("proto[@name='websocket']"))
    ET.SubElement(packet, 'field', name='tcp.len', show='12')
    report = module.analyze_pdml(ET.tostring(root))
    assert len(report['matched_pairs']) == 1
    assert 'unaccounted_tcp_payload' not in {issue['reason'] for issue in report['issues']}


def test_actual_tshark_incomplete_trailing_message_is_reported(tmp_path):
    tshark = Path(shutil.which('tshark') or r'C:\Program Files\Wireshark\tshark.exe')
    if not tshark.is_file():
        pytest.skip('TShark not installed')
    rpc_capture_fixture.build(tmp_path, trailing=True)
    report = module.analyze_capture(tmp_path / 'synthetic-rpc.pcap', tshark=tshark)
    assert len(report['matched_pairs']) == 2
    assert dict(reason='unaccounted_tcp_payload', frame=15, stream=0) in report['issues']


@pytest.mark.parametrize('split_reply,final_frame', [(False, 14), (True, 15)])
def test_actual_tshark_complete_and_partial_messages_sharing_segment(tmp_path, split_reply, final_frame):
    tshark = Path(shutil.which('tshark') or r'C:\Program Files\Wireshark\tshark.exe')
    if not tshark.is_file():
        pytest.skip('TShark not installed')
    rpc_capture_fixture.build(tmp_path, coalesced=True, split_reply=split_reply)
    report = module.analyze_capture(tmp_path / 'synthetic-rpc.pcap', tshark=tshark)
    assert len(report['matched_pairs']) == 2
    assert any(issue['frame'] == final_frame and issue['reason'] == 'incomplete_application_payload'
               for issue in report['issues'] if 'frame' in issue)


def test_partial_tail_completed_in_later_carrier_is_not_reported_incomplete():
    root = ET.fromstring(pdml([request(), response(), request(9, segments=[3, 4]), response(9)]))
    packet = root.findall('packet')[2]
    ET.SubElement(packet, 'field', name='tcp.segment_data', size='12')
    report = module.analyze_pdml(ET.tostring(root))
    assert len(report['matched_pairs']) == 2
    assert 'incomplete_application_payload' not in {issue['reason'] for issue in report['issues']}
