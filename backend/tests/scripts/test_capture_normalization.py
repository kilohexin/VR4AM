"""Synthetic closed files only; no SDK, network or capture interface."""
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import shutil

import pytest
from tests.scripts import rpc_capture_fixture as rpc

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def module():
    path = ROOT/'scripts/normalize_rpc_capture.py'
    assert path.is_file(), 'bounded offline normalizer missing'
    spec = importlib.util.spec_from_file_location('capture_normalization', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def block(kind, body, endian='<'):
    body += b'\0' * (-len(body) % 4)
    size = len(body) + 12
    return struct.pack(endian+'II', kind, size) + body + struct.pack(endian+'I', size)


def option(code, value, endian='<'):
    return struct.pack(endian+'HH', code, len(value)) + value + b'\0' * (-len(value) % 4)


def pcapng(packets, *, linktype=1, endian='<', resolution=6, offset=0, interface=0,
           original_delta=0, extra_options=b''):
    header = block(0x0a0d0d0a, struct.pack(endian+'IHHq', 0x1a2b3c4d, 1, 0, -1), endian)
    opts = option(9, bytes([resolution]), endian)
    if offset:
        opts += option(14, struct.pack(endian+'q', offset), endian)
    opts += extra_options + option(0, b'', endian)
    data = header + block(1, struct.pack(endian+'HHI', linktype, 0, 65535)+opts, endian)
    for ticks, packet in packets:
        body = struct.pack(endian+'IIIII', interface, ticks >> 32, ticks & 0xffffffff,
                           len(packet), len(packet)+original_delta) + packet
        data += block(6, body, endian)
    return data


def ethernet():
    return rpc.packet(True, 1000, 2000, 0x18, b'synthetic')


def wlan(*, protected=False, aggregation=False, four_address=False, order=False, fragment=False):
    control = 0x0088 | (0x4000 if protected else 0) | (0x0300 if four_address else 0)
    control |= 0x8000 if order else 0
    header = struct.pack('<H', control)+b'\0'*20+struct.pack('<H', int(fragment))
    header += b'\0'*6 if four_address else b''
    header += bytes([0x80 if aggregation else 0, 0])
    header += b'\0'*4 if order else b''
    return header+bytes.fromhex('aaaa030000000800')+ethernet()[14:]


def source(tmp_path, raw):
    path = tmp_path/'closed capture.pcapng'
    path.write_bytes(raw)
    return path


def review(inspection):
    return dict(schema_version=1, source_sha256=inspection['source_sha256'],
                layout=inspection['layout'], port=inspection['port'],
                frame_mapping=inspection['frame_mapping'], reviewed_plaintext_ipv4=True)


def read_pcap(path):
    raw = path.read_bytes()
    magic, major, minor, _, _, snaplen, linktype = struct.unpack_from('<IHHIIII', raw)
    assert (magic, major, minor, snaplen, linktype) == (0xa1b23c4d, 2, 4, 65535, 228)
    records, cursor = [], 24
    while cursor < len(raw):
        seconds, ns, length, original = struct.unpack_from('<IIII', raw, cursor)
        assert length == original
        records.append((seconds*1_000_000_000+ns, raw[cursor+16:cursor+16+length]))
        cursor += 16+length
    assert cursor == len(raw)
    return records


@pytest.mark.parametrize('endian', ['<', '>'])
def test_declared_ethernet_preserves_bytes_time_and_source_without_review(module, tmp_path, endian):
    packet = ethernet()
    path = source(tmp_path, pcapng([(1_700_000_000_123456, packet)], endian=endian))
    before = path.read_bytes()
    result = module.normalize_capture(path, tmp_path/'derived')
    assert result['normalization_complete'] and not result['hardware_stop_verified']
    assert result['app_clock_alignment_verified'] is False
    assert read_pcap(tmp_path/'derived/ipv4.pcap') == [(1_700_000_000_123456000, packet[14:])]
    assert path.read_bytes() == before
    assert result['source_sha256'] == hashlib.sha256(before).hexdigest()
    assert result['no_new_timestamp_precision_claim'] and result['frame_mapping'][0]['source_units_per_second'] == 1_000_000
    assert '192.0.2.1' not in json.dumps(result)


def test_inspection_never_approves_mislabeled_or_protected_plaintext(module, tmp_path):
    path = source(tmp_path, pcapng([(12, wlan(protected=True, aggregation=True))]))
    report = module.inspect_capture(path, layout='pktmon_wlan_llc')
    assert report['review_required'] and report['interpretation_verified'] is False
    assert report['frame_mapping'][0]['protected_flag'] and report['frame_mapping'][0]['aggregation_flag']
    assert not (tmp_path/'derived').exists()
    with pytest.raises(ValueError, match='review_required'):
        module.normalize_capture(path, tmp_path/'derived', layout='pktmon_wlan_llc')
    assert not (tmp_path/'derived').exists()


@pytest.mark.parametrize('flags', [{}, {'protected':True}, {'aggregation':True},
                                  {'four_address':True, 'order':True}])
def test_bound_review_allows_only_the_exact_observed_wlan_bytes(module, tmp_path, flags):
    path = source(tmp_path, pcapng([(123456, wlan(**flags))]))
    inspection = module.inspect_capture(path, layout='pktmon_wlan_llc')
    result = module.normalize_capture(path, tmp_path/'derived', layout='pktmon_wlan_llc', review=review(inspection))
    assert result['review_used'] and result['normalization_complete']
    assert read_pcap(tmp_path/'derived/ipv4.pcap') == [(123456000, ethernet()[14:])]


@pytest.mark.parametrize('field,value', [('source_sha256', '0'*64), ('layout', 'declared'), ('port', True),
                                       ('reviewed_plaintext_ipv4', 1), ('schema_version', True), ('frame_mapping', [])])
def test_changed_review_or_non_boolean_attestation_rejects_without_output(module, tmp_path, field, value):
    path = source(tmp_path, pcapng([(12, wlan(protected=True))]))
    approved = review(module.inspect_capture(path, layout='pktmon_wlan_llc'))
    approved[field] = value
    with pytest.raises(ValueError, match='review'):
        module.normalize_capture(path, tmp_path/'derived', layout='pktmon_wlan_llc', review=approved)
    assert not (tmp_path/'derived').exists()


def test_review_cannot_ignore_a_changed_frame_or_grant_a_future_file(module, tmp_path):
    path = source(tmp_path, pcapng([(12, wlan(protected=True))]))
    approved = review(module.inspect_capture(path, layout='pktmon_wlan_llc'))
    approved['frame_mapping'][0]['ip_length'] += 1
    with pytest.raises(ValueError, match='review'):
        module.normalize_capture(path, tmp_path/'derived', layout='pktmon_wlan_llc', review=approved)
    assert not (tmp_path/'derived').exists()


def test_declared_wlan_protected_data_still_requires_exact_review(module, tmp_path):
    path = source(tmp_path, pcapng([(12, wlan(protected=True))], linktype=105))
    with pytest.raises(ValueError, match='review_required'):
        module.normalize_capture(path, tmp_path/'derived')
    inspection = module.inspect_capture(path)
    assert module.normalize_capture(path, tmp_path/'derived', review=review(inspection))['review_used']


@pytest.mark.parametrize('linktype,packet', [(228, ethernet()[14:]), (105, wlan())])
def test_correct_declared_raw_ip_or_wlan_needs_no_override(module, tmp_path, linktype, packet):
    path = source(tmp_path, pcapng([(12, packet)], linktype=linktype))
    assert module.normalize_capture(path, tmp_path/'derived')['normalization_complete']


@pytest.mark.parametrize('resolution,ticks,offset,epoch', [(9,123456789,2,2123456789), (0,2,3,5_000_000_000),
                                                       (0x8a,1024,0,1_000_000_000)])
def test_timestamp_units_and_offsets_are_parsed_not_assumed(module, tmp_path, resolution, ticks, offset, epoch):
    path = source(tmp_path, pcapng([(ticks, ethernet())], resolution=resolution, offset=offset))
    module.normalize_capture(path, tmp_path/'derived')
    assert read_pcap(tmp_path/'derived/ipv4.pcap')[0][0] == epoch


@pytest.mark.parametrize('resolution,ticks,offset', [(10,1,0), (0x8a,1,0), (6,1,-1), (0,2**32,0)])
def test_unrepresentable_timestamp_never_rounds_or_adds_precision(module, tmp_path, resolution, ticks, offset):
    path = source(tmp_path, pcapng([(ticks, ethernet())], resolution=resolution, offset=offset))
    with pytest.raises(ValueError, match='timestamp'):
        module.normalize_capture(path, tmp_path/'derived')
    assert not (tmp_path/'derived').exists()


@pytest.mark.parametrize('problem', ['trailer', 'truncated_capture', 'unknown_interface', 'missing_section',
                                   'duplicate_resolution', 'short_option', 'unsupported_linktype',
                                   'no_packets', 'simple_packet'])
def test_incomplete_or_unsupported_container_is_explicit_failure(module, tmp_path, problem):
    kwargs = {}
    if problem == 'truncated_capture': kwargs['original_delta'] = 1
    if problem == 'unknown_interface': kwargs['interface'] = 1
    if problem == 'duplicate_resolution': kwargs['extra_options'] = option(9, b'\x06')
    if problem == 'short_option': kwargs['extra_options'] = struct.pack('<HH', 42, 100)+b'xx'
    if problem == 'unsupported_linktype': kwargs['linktype'] = 127
    raw = pcapng([] if problem == 'no_packets' else [(12,ethernet())], **kwargs)
    if problem == 'trailer': raw = raw[:-4]+b'\0'*4
    if problem == 'missing_section': raw = raw[28:]
    if problem == 'simple_packet': raw += block(3, struct.pack('<I', 4)+b'xxxx')
    with pytest.raises(ValueError):
        module.normalize_capture(source(tmp_path, raw), tmp_path/'derived')
    assert not (tmp_path/'derived').exists()


@pytest.mark.parametrize('problem', ['wrong_ether_type', 'ip_truncated', 'ip_fragment', 'udp',
                                   'tcp_short', 'tcp_header', 'wrong_port', 'wlan_fragment', 'encrypted_bytes'])
def test_unknown_packet_encoding_never_becomes_an_ipv4_interpretation(module, tmp_path, problem):
    packet = bytearray(ethernet())
    layout = 'declared'
    if problem == 'wrong_ether_type': packet[12:14] = b'\x86\xdd'
    if problem == 'ip_truncated': packet = packet[:-5]
    if problem == 'ip_fragment': packet[20:22] = b'\x20\x00'
    if problem == 'udp': packet[23] = 17
    if problem == 'tcp_short': packet[16:18] = struct.pack('!H', 24)
    if problem == 'tcp_header': packet[46] = 0xf0
    if problem == 'wrong_port': packet[36:38] = struct.pack('!H', 9999)
    if problem in ('wlan_fragment', 'encrypted_bytes'):
        packet = bytearray(wlan(fragment=problem=='wlan_fragment', protected=problem=='encrypted_bytes'))
        layout = 'pktmon_wlan_llc'
        if problem == 'encrypted_bytes': packet[26:34] = b'not LLC!'
    with pytest.raises(ValueError):
        module.normalize_capture(source(tmp_path, pcapng([(12,bytes(packet))])), tmp_path/'derived', layout=layout)
    assert not (tmp_path/'derived').exists()


def test_new_output_directory_is_exclusive_and_source_cannot_be_overwritten(module, tmp_path):
    path = source(tmp_path, pcapng([(12, ethernet())]))
    output = tmp_path/'derived'
    output.mkdir()
    (output/'keep').write_text('original')
    with pytest.raises(FileExistsError): module.normalize_capture(path, output)
    assert (output/'keep').read_text() == 'original'
    with pytest.raises(FileExistsError): module.normalize_capture(path, tmp_path)


def test_cli_inspection_creates_only_an_unapproved_report(module, tmp_path):
    path = source(tmp_path, pcapng([(12, wlan(protected=True))]))
    output = tmp_path/'inspection'
    assert module.main([str(path), '--layout', 'pktmon_wlan_llc', '--inspect', '--output-directory', str(output)]) == 0
    assert list(p.name for p in output.iterdir()) == ['inspection.json']
    assert json.loads((output/'inspection.json').read_text())['interpretation_verified'] is False


@pytest.mark.parametrize('problem', ['bytes', 'timestamp', 'protected_flag', 'extra_field'])
def test_review_freezes_flags_times_and_entire_packet_not_only_ip(module, tmp_path, problem):
    path = source(tmp_path, pcapng([(12, wlan(protected=True))]))
    approved = review(module.inspect_capture(path, layout='pktmon_wlan_llc'))
    if problem == 'bytes':
        packet = bytearray(wlan(protected=True))
        packet[4] = 9  # MAC header changed; IP payload unchanged.
        path.write_bytes(pcapng([(12, bytes(packet))]))
        approved['source_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    elif problem == 'timestamp': approved['frame_mapping'][0]['epoch_ns'] += 1
    elif problem == 'protected_flag': approved['frame_mapping'][0]['protected_flag'] = False
    else: approved['new_unreviewed_field'] = True
    with pytest.raises(ValueError, match='review'):
        module.normalize_capture(path, tmp_path/'derived', layout='pktmon_wlan_llc', review=approved)
    assert not (tmp_path/'derived').exists()


def test_multiple_sections_reset_interfaces_and_preserve_individual_clocks(module, tmp_path):
    raw = pcapng([(12,ethernet())], resolution=6)
    raw += pcapng([(123,ethernet()[14:])], endian='>', resolution=9, offset=2, linktype=228)
    path = source(tmp_path, raw)
    result = module.normalize_capture(path, tmp_path/'derived')
    assert [m['section'] for m in result['frame_mapping']] == [0,1]
    assert [m['interface'] for m in result['frame_mapping']] == [0,0]
    assert [row[0] for row in read_pcap(tmp_path/'derived/ipv4.pcap')] == [12000,2000000123]


def test_packet_flags_and_unsupported_fcs_metadata_are_not_silently_dropped(module, tmp_path):
    raw = pcapng([(12,ethernet())], extra_options=option(13,b'\x20'))
    with pytest.raises(ValueError, match='fcs'):
        module.normalize_capture(source(tmp_path,raw),tmp_path/'derived')


def test_source_changes_during_export_leave_explicit_incomplete_manifest(module, tmp_path, monkeypatch):
    path = source(tmp_path, pcapng([(12,ethernet())]))
    original = module.digest
    calls = 0
    def changing(data):
        nonlocal calls
        calls += 1
        value = original(data)
        if calls == 4:  # Full derived file hash, after source/packet/IP hashes.
            path.write_bytes(b'changed synthetic source')
        return value
    monkeypatch.setattr(module,'digest',changing)
    with pytest.raises(ValueError, match='source_changed_during_output'):
        module.normalize_capture(path,tmp_path/'derived')
    report = json.loads((tmp_path/'derived/manifest.json').read_text())
    assert not report['normalization_complete'] and not report['interpretation_verified']


def test_normalized_wlan_trace_preserves_real_offline_dissector_stop_pairing(module, tmp_path):
    tshark = Path(shutil.which('tshark') or r'C:\Program Files\Wireshark\tshark.exe')
    if not tshark.is_file(): pytest.skip('offline TShark not installed')
    rpc.build(tmp_path)
    original = tmp_path/'synthetic-rpc.pcap'
    data = original.read_bytes()
    cursor, records = 24, []
    while cursor < len(data):
        seconds, ns, length, _ = struct.unpack_from('<IIII',data,cursor)
        ethernet_packet = data[cursor+16:cursor+16+length]
        wrapped = wlan(protected=True,aggregation=True)[:34] + ethernet_packet[14:]
        records.append((seconds*1_000_000_000+ns,wrapped))
        cursor += 16+length
    path = source(tmp_path,pcapng(records,resolution=9))
    inspection = module.inspect_capture(path,layout='pktmon_wlan_llc')
    module.normalize_capture(path,tmp_path/'derived',layout='pktmon_wlan_llc',review=review(inspection))
    spec = importlib.util.spec_from_file_location('normalizer_rpc_analysis',ROOT/'scripts/analyze_rpc_capture.py')
    analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyzer)
    baseline = analyzer.analyze_capture(original,tshark=tshark)
    normalized = analyzer.analyze_capture(tmp_path/'derived/ipv4.pcap',tshark=tshark)
    assert normalized['matched_pairs'] == baseline['matched_pairs']
    assert normalized['issues'] == baseline['issues']
    stop, = [p for p in normalized['matched_pairs'] if p['method']=='stop_move']
    assert stop['completion_interval_ns']==610_000_000 and stop['first_byte_epoch_ns'] is None
    assert not normalized['hardware_stop_verified'] and not normalized['app_clock_alignment_verified']
