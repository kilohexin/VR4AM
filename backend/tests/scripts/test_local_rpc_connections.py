"""Local Windows connection-table reader; synthetic text, no sockets."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import subprocess

import pytest

ROOT=Path(__file__).resolve().parents[3]
HEADER='\nActive Connections\n\n  Proto  Local Address          Foreign Address        State           PID\n'
ZH_HEADER='\n\u6d3b\u52a8\u8fde\u63a5\n\n  \u534f\u8bae \u672c\u5730\u5730\u5740 \u5916\u90e8\u5730\u5740 \u72b6\u6001 PID\n'

@pytest.fixture
def module():
    spec=importlib.util.spec_from_file_location('local_rpc_table',ROOT/'scripts/local_rpc_connections.py')
    value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value

def test_exact_remote_port_and_ipv4_ipv6_are_counted(module):
    text=HEADER+'''  TCP 127.0.0.1:5000 192.0.2.1:3031 ESTABLISHED 10
  TCP [::1]:5001 [2001:db8::1]:3031 ESTABLISHED 11
  TCP [fe80::1%7]:5002 [fe80::2%7]:3031 ESTABLISHED 12
  TCP 127.0.0.1:3031 192.0.2.1:5000 ESTABLISHED 13
  TCP 127.0.0.1:5003 192.0.2.1:30311 ESTABLISHED 14
  TCP 0.0.0.0:3031 0.0.0.0:0 LISTENING 15
  TCP 127.0.0.1:5004 192.0.2.1:3031 TIME_WAIT 0
  UDP 0.0.0.0:3031 *:* 16
'''
    assert module.parse_rpc_connections(text)==3

def test_valid_empty_table_can_report_zero(module):
    assert module.parse_rpc_connections(HEADER)==0


def test_observed_hidden_windows_chinese_table(module):
    assert module.parse_rpc_connections(ZH_HEADER+'TCP 127.0.0.1:1 192.0.2.1:3031 ESTABLISHED 1')==1


@pytest.mark.parametrize('text', [ZH_HEADER.replace('PID', 'pid'),
                                 ZH_HEADER.replace('\u6d3b\u52a8\u8fde\u63a5', 'Active Connections')])
def test_mismatched_localized_header_is_rejected(module, text):
    with pytest.raises(ValueError): module.parse_rpc_connections(text)


def test_hidden_table_uses_explicit_oem_decoding_even_under_python_utf8(module, monkeypatch):
    monkeypatch.setattr(module, 'output_encoding', lambda: 'cp936', raising=False)
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=0, stdout=ZH_HEADER.encode('cp936'), stderr=b''))
    assert module.read_rpc_connections()==0


def test_invalid_encoded_bytes_never_report_zero(module, monkeypatch):
    monkeypatch.setattr(module, 'output_encoding', lambda: 'utf-8', raising=False)
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **k: SimpleNamespace(
        returncode=0, stdout=b'\xff', stderr=b''))
    with pytest.raises(UnicodeError): module.read_rpc_connections()


@pytest.mark.parametrize('remote', ['127.0.0.1:3031', '[::1]:3031'])
def test_connected_udp_row_is_valid_but_is_not_tcp_rpc(module, remote):
    assert module.parse_rpc_connections(HEADER+f'UDP 127.0.0.1:5000 {remote} 12\n')==0


def test_unknown_udp_remote_is_rejected(module):
    with pytest.raises(ValueError):
        module.parse_rpc_connections(HEADER+'UDP 127.0.0.1:5000 robot:3031 12\n')

@pytest.mark.parametrize('text',[
    '', 'Access denied', 'TCP 127.0.0.1:1 192.0.2.1:3031 ESTABLISHED 1',
    HEADER+'TCP 127.0.0.1:1 192.0.2.1:3031 ESTABLISHED',
    HEADER+'TCP 127.0.0.1:1 192.0.2.1:3031 UNKNOWN 1',
    HEADER+'TCP 127.0.0.1:1 robot:3031 ESTABLISHED 1',
    HEADER+'TCP 127.0.0.1:1 192.0.2.1:65536 ESTABLISHED 1',
    HEADER+'TCP 127.0.0.1:1 192.0.2.1:3031 ESTABLISHED badpid',
    HEADER+'unrecognized trailing output',
])
def test_unknown_or_incomplete_table_never_reports_zero(module,text):
    with pytest.raises(ValueError): module.parse_rpc_connections(text)

def test_query_is_numeric_local_table_only_and_has_no_powershell_or_dns(module,monkeypatch):
    calls=[]
    def run(args,**kwargs):
        calls.append((args,kwargs))
        return SimpleNamespace(returncode=0,stdout=HEADER.encode(),stderr=b'')
    monkeypatch.setattr(module.subprocess,'run',run)
    assert module.read_rpc_connections()==0
    assert calls[0][0]==['netstat.exe','-ano']
    assert calls[0][1]['timeout']==5 and calls[0][1]['shell'] is False

@pytest.mark.parametrize('problem',['exit','stderr','timeout'])
def test_query_failure_never_reports_zero(module,monkeypatch,problem):
    def run(*args,**kwargs):
        if problem=='timeout': raise subprocess.TimeoutExpired('synthetic-local-table',5)
        return SimpleNamespace(returncode=1 if problem=='exit' else 0,stdout=HEADER.encode(),
                               stderr=b'warning' if problem=='stderr' else b'')
    monkeypatch.setattr(module.subprocess,'run',run)
    with pytest.raises((ValueError,subprocess.TimeoutExpired)):
        module.read_rpc_connections()
