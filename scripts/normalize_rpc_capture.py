"""Interpret closed PCAPNG files as IPv4 copies. No capture or network APIs.

Container fields follow draft-ietf-opsawg-pcapng-06; this is a deliberately
bounded decoder, not a general WLAN/decryption or aggregation implementation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

MAX_BYTES = 64 * 1024 * 1024
MAX_FRAMES = 100_000
LAYOUTS = ('declared', 'pktmon_wlan_llc')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    # JSON comparison distinguishes True from 1, unlike Python dict equality.
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def options(data, endian):
    cursor, result = 0, {}
    while cursor < len(data):
        if cursor + 4 > len(data):
            raise ValueError('truncated_option')
        code, size = struct.unpack_from(endian+'HH', data, cursor)
        cursor += 4
        if code == 0:
            if size or any(data[cursor:]):
                raise ValueError('invalid_end_option')
            return result
        padded = (size+3)//4*4
        if cursor + padded > len(data):
            raise ValueError('truncated_option')
        result.setdefault(code, []).append(data[cursor:cursor+size])
        cursor += padded
    if data:
        raise ValueError('missing_end_option')
    return result


def single_option(values, code, size, default):
    entries = values.get(code)
    if entries is None:
        return default
    if len(entries) != 1 or len(entries[0]) != size:
        raise ValueError('ambiguous_interface_option')
    return entries[0]


def packets(raw):
    cursor, section, frame, endian = 0, -1, 0, None
    interfaces = []
    skipped = []
    while cursor < len(raw):
        if cursor + 12 > len(raw):
            raise ValueError('truncated_block')
        if raw[cursor:cursor+4] == b'\x0a\x0d\x0d\x0a':
            magic = raw[cursor+8:cursor+12]
            if magic not in (b'\x4d\x3c\x2b\x1a', b'\x1a\x2b\x3c\x4d'):
                raise ValueError('invalid_byte_order_magic')
            endian = '<' if magic[0] == 0x4d else '>'
        if endian is None:
            raise ValueError('missing_section_header')
        kind, size = struct.unpack_from(endian+'II', raw, cursor)
        if size < 12 or size % 4 or cursor+size > len(raw):
            raise ValueError('invalid_block_length')
        if struct.unpack_from(endian+'I', raw, cursor+size-4)[0] != size:
            raise ValueError('block_trailer_mismatch')
        body = raw[cursor+8:cursor+size-4]
        if kind == 0x0a0d0d0a:
            if len(body) < 16 or struct.unpack_from(endian+'HH', body, 4) != (1, 0):
                raise ValueError('unsupported_section_version')
            # Finite section bounds are not implemented: reject rather than
            # silently ignore a contradictory declared section length.
            if struct.unpack_from(endian+'q', body, 8)[0] != -1:
                raise ValueError('unsupported_finite_section_length')
            options(body[16:], endian)
            section += 1
            interfaces = []
        elif kind == 1:
            if len(body) < 8:
                raise ValueError('short_interface')
            linktype, reserved, snaplen = struct.unpack_from(endian+'HHI', body)
            if reserved:
                raise ValueError('invalid_interface_reserved')
            values = options(body[8:], endian)
            resolution = single_option(values, 9, 1, b'\x06')[0]
            units = (2 if resolution & 128 else 10) ** (resolution & 127)
            offset = struct.unpack(endian+'q', single_option(values, 14, 8, b'\0'*8))[0]
            fcs_bits = single_option(values, 13, 1, b'\0')[0]
            if fcs_bits:
                raise ValueError('unsupported_interface_fcs')
            interfaces.append(dict(linktype=linktype, snaplen=snaplen, units=units, offset=offset))
        elif kind == 6:
            if len(body) < 20:
                raise ValueError('short_packet_block')
            identity, high, low, captured, original = struct.unpack_from(endian+'IIIII', body)
            if identity >= len(interfaces):
                raise ValueError('unknown_interface')
            interface = interfaces[identity]
            padded = (captured+3)//4*4
            if (captured != original or not captured or 20+padded > len(body)
                    or (interface['snaplen'] and captured > interface['snaplen'])):
                raise ValueError('truncated_capture_or_invalid_length')
            packet_options = options(body[20+padded:], endian)
            # FCS overrides/error bits and other packet flags need their own
            # interpretation; current offline evidence has no nonzero flags.
            flags = single_option(packet_options, 2, 4, b'\0'*4)
            if struct.unpack(endian+'I', flags)[0]:
                raise ValueError('unsupported_packet_flags')
            ticks = (high << 32) + low
            numerator = ticks*1_000_000_000
            quotient, remainder = divmod(numerator, interface['units'])
            timestamp = quotient + interface['offset']*1_000_000_000
            if remainder or not 0 <= timestamp < 2**32*1_000_000_000:
                raise ValueError('unrepresentable_timestamp')
            frame += 1
            if frame > MAX_FRAMES:
                raise ValueError('too_many_frames')
            yield body[20:20+captured], dict(frame=frame, section=section, interface=identity,
                declared_linktype=interface['linktype'], timestamp_ticks=ticks, epoch_ns=timestamp,
                source_units_per_second=interface['units'], source_offset_seconds=interface['offset'])
        elif kind in (2, 3):
            raise ValueError('unsupported_packet_block')
        elif kind in (4, 5):
            skipped.append(kind)  # known name-resolution/statistics metadata
        else:
            raise ValueError('unsupported_block_type')
        cursor += size
    if not frame:
        raise ValueError('no_packets')


def extract_ip(packet, mapping, layout):
    linktype = mapping['declared_linktype']
    protected = aggregation = False
    if layout == 'pktmon_wlan_llc' or linktype == 105:
        if len(packet) < 24:
            raise ValueError('short_wlan_header')
        control = struct.unpack_from('<H', packet)[0]
        subtype = (control >> 4) & 15
        if control & 3 or (control >> 2) & 3 != 2 or subtype not in (0, 8):
            raise ValueError('unsupported_wlan_frame')
        if control & 0x400 or struct.unpack_from('<H', packet, 22)[0] & 15:
            raise ValueError('unsupported_wlan_fragment')
        qos = subtype == 8
        start = 24 + (6 if control & 0x300 == 0x300 else 0)
        if qos:
            if len(packet) < start+2:
                raise ValueError('short_qos_header')
            aggregation = bool(packet[start] & 0x80)
            start += 2
        if control & 0x8000:
            if not qos:
                raise ValueError('unsupported_wlan_order')
            start += 4
        protected = bool(control & 0x4000)
        if packet[start:start+8] != bytes.fromhex('aaaa030000000800'):
            raise ValueError('unsupported_llc_or_encrypted_payload')
        start += 8
    elif linktype == 1:
        if len(packet) < 14 or packet[12:14] != b'\x08\x00':
            raise ValueError('unsupported_ethernet_type')
        start = 14
    elif linktype == 228:
        start = 0
    else:
        raise ValueError('unsupported_linktype')
    if len(packet) < start+20 or packet[start] >> 4 != 4:
        raise ValueError('invalid_ipv4_header')
    header = (packet[start] & 15)*4
    length = struct.unpack_from('!H', packet, start+2)[0]
    if header < 20 or length < header+20 or start+length > len(packet):
        raise ValueError('invalid_ipv4_length')
    if packet[start+9] != 6 or struct.unpack_from('!H', packet, start+6)[0] & 0x3fff:
        raise ValueError('unsupported_protocol_or_ip_fragment')
    tcp = start+header
    tcp_length = (packet[tcp+12] >> 4)*4
    if tcp_length < 20 or header+tcp_length > length:
        raise ValueError('invalid_tcp_length')
    if 3031 not in struct.unpack_from('!HH', packet, tcp):
        raise ValueError('packet_outside_verified_rpc_port')
    ip = packet[start:start+length]
    tail = packet[start+length:]
    ethernet_padding = (layout == 'declared' and linktype == 1 and length < 46
                        and len(tail) <= 46-length and not any(tail))
    if tail and not ethernet_padding:
        raise ValueError('unsupported_packet_trailer')
    mapping.update(original_packet_sha256=digest(packet), source_ip_offset=start, ip_length=length,
                   extracted_ip_sha256=digest(ip), protected_flag=protected, aggregation_flag=aggregation,
                   excluded_padding_bytes=len(tail))
    return ip


def decode(raw, layout):
    if layout not in LAYOUTS:
        raise ValueError('unsupported_layout')
    records = []
    for packet, mapping in packets(raw):
        records.append((extract_ip(packet, mapping, layout), mapping))
    report = dict(schema_version=1, source_sha256=digest(raw), layout=layout, port=3031,
                  packet_count=len(records), frame_mapping=[item[1] for item in records],
                  review_required=(layout != 'declared' or any(
                      mapping['protected_flag'] or mapping['aggregation_flag'] for _, mapping in records)),
                  interpretation_verified=False, offline_only=True, hardware_stop_verified=False,
                  app_clock_alignment_verified=False, no_new_timestamp_precision_claim=True)
    return records, report


def snapshot(path):
    path = Path(path).resolve(strict=True)
    if not path.is_file() or not 0 < path.stat().st_size <= MAX_BYTES:
        raise ValueError('unsupported_capture_size')
    raw = path.read_bytes()
    if not 0 < len(raw) <= MAX_BYTES:
        raise ValueError('unsupported_capture_size')
    return path, raw


def inspect_capture(path, *, layout='declared'):
    path, raw = snapshot(path)
    _, report = decode(raw, layout)
    if path.read_bytes() != raw:
        raise ValueError('source_changed_during_inspection')
    return report


def normalize_capture(path, output_directory, *, layout='declared', review=None):
    path, raw = snapshot(path)
    records, report = decode(raw, layout)
    if review is None:
        if report['review_required']:
            raise ValueError('exact_file_and_frame_review_required')
    else:
        expected = dict(schema_version=1, source_sha256=report['source_sha256'], layout=layout,
                        port=3031, frame_mapping=report['frame_mapping'], reviewed_plaintext_ipv4=True)
        if canonical(review) != canonical(expected):
            raise ValueError('review_not_bound_to_source_and_frames')
    if path.read_bytes() != raw:
        raise ValueError('source_changed_before_output')
    output = Path(output_directory).resolve()
    output.mkdir(parents=True, exist_ok=False)
    report.update(normalization_complete=False, review_used=review is not None,
                  interpretation_verified=True, extracted_ip_bytes_unchanged=True,
                  review_content_sha256=digest(canonical(review).encode()) if review is not None else None)
    try:
        with (output/'ipv4.pcap').open('xb') as file:
            file.write(struct.pack('<IHHIIII', 0xa1b23c4d, 2, 4, 0, 0, 65535, 228))
            for ip, mapping in records:
                seconds, ns = divmod(mapping['epoch_ns'], 1_000_000_000)
                file.write(struct.pack('<IIII', seconds, ns, len(ip), len(ip)))
                file.write(ip)
        report['derived_sha256'] = digest((output/'ipv4.pcap').read_bytes())
        if path.read_bytes() != raw:
            raise ValueError('source_changed_during_output')
        report.update(source_unchanged=True, normalization_complete=True)
    except Exception as error:
        report.update(error_type=type(error).__name__, interpretation_verified=False)
        write_json(output/'manifest.json', report)
        raise
    write_json(output/'manifest.json', report)
    return report


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as file:
        json.dump(value, file, ensure_ascii=False, allow_nan=False, indent=2)
        file.write('\n')


def unique_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('duplicate_review_key')
        value[key] = item
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--layout', choices=LAYOUTS, default='declared')
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--review', type=Path)
    parser.add_argument('--output-directory', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.inspect:
        if args.review:
            raise ValueError('inspection_is_not_review_approval')
        report = inspect_capture(args.capture, layout=args.layout)
        args.output_directory.mkdir(parents=True, exist_ok=False)
        write_json(args.output_directory/'inspection.json', report)
    else:
        review = json.loads(args.review.read_text(encoding='utf-8-sig'), object_pairs_hook=unique_object) if args.review else None
        report = normalize_capture(args.capture, args.output_directory, layout=args.layout, review=review)
    print(json.dumps({key: report[key] for key in ('packet_count', 'review_required',
          'interpretation_verified', 'hardware_stop_verified')}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
