"""Read a numeric local Windows table only; no SDK, sockets or DNS lookup."""
import ipaddress
import json
import os
import subprocess

STATES = {'CLOSED','LISTEN','LISTENING','SYN_SENT','SYN_SEND','SYN_RECEIVED',
          'ESTABLISHED','FIN_WAIT_1','FIN_WAIT_2','CLOSE_WAIT','CLOSING',
          'LAST_ACK','TIME_WAIT','TIMED_WAIT','DELETE_TCB'}
HEADERS = {
    ('Active','Connections'): ['Proto','Local','Address','Foreign','Address','State','PID'],
    ('活动连接',): ['协议','本地地址','外部地址','状态','PID'],
}


def port_of(endpoint):
    host, separator, port = endpoint.rpartition(':')
    if not separator or not port.isascii() or not port.isdecimal() or not 0 <= int(port) <= 65535:
        raise ValueError('invalid_local_table_endpoint')
    ipaddress.ip_address(host[1:-1] if host.startswith('[') and host.endswith(']') else host)
    return int(port)


def parse_rpc_connections(text):
    active = None
    header = False
    count = 0
    for line in text.splitlines():
        fields = line.split()
        if not fields:
            continue
        if tuple(fields) in HEADERS and active is None and not header:
            active = tuple(fields)
            continue
        if active is not None and fields == HEADERS[active] and not header:
            header = True
            continue
        if not header:
            raise ValueError('unrecognized_local_table_header')
        if fields[0] == 'TCP' and len(fields) == 5:
            port_of(fields[1])
            remote = port_of(fields[2])
            if fields[3] not in STATES or not fields[4].isascii() or not fields[4].isdecimal():
                raise ValueError('invalid_local_tcp_row')
            count += int(fields[3] == 'ESTABLISHED' and remote == 3031)
        elif fields[0] == 'UDP' and len(fields) == 4:
            port_of(fields[1])
            if fields[2] != '*:*':
                port_of(fields[2])
            if not fields[3].isascii() or not fields[3].isdecimal():
                raise ValueError('invalid_local_udp_row')
        else:
            raise ValueError('unknown_or_incomplete_local_table_row')
    if not header:
        raise ValueError('missing_local_connection_table')
    return count


def output_encoding():
    # CREATE_NO_WINDOW netstat uses the system OEM page on this Windows host.
    # Capture bytes so Python UTF8 mode cannot change native output decoding.
    if os.name == 'nt':
        import ctypes
        codepage = ctypes.windll.kernel32.GetOEMCP()
        if not codepage:
            raise ValueError('local_connection_codepage_unavailable')
        return f'cp{codepage}'
    return 'utf-8'


def read_rpc_connections():
    result = subprocess.run(['netstat.exe','-ano'],capture_output=True,
                            timeout=5,shell=False,
                            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if result.returncode or result.stderr.strip():
        raise ValueError('local_connection_table_unavailable')
    return parse_rpc_connections(result.stdout.decode(output_encoding(), errors='strict'))


if __name__ == '__main__':
    print(json.dumps(read_rpc_connections()))
