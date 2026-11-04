"""Lossless compact candidate wire format; not motion admission or an executor.

Two 120-word banks fit the native 256 int32 signals. Each describes one
80ms reference and a complete global-quintic stopping tail. A caller MUST
validate the expanded trajectory and binding before any actuator uses it.
CRC detects transmission damage; it is not authentication.
"""
from dataclasses import dataclass
import math
import re
import struct
import zlib

from app.control.cartesian_pvat_reference import JointReference,PvatSegment

BANK_SIZE=120
MAGIC=0x43504631
PERIOD=.08


def _integer(value,minimum=0,maximum=2**31-1):
    if type(value) is not int or not minimum<=value<=maximum:
        raise ValueError('cartesian_bundle_wire_integer')
    return value


def _signed(value):return value if value<2**31 else value-2**32


def _hash_words(value,count):
    if type(value) is not str or re.fullmatch('[0-9a-f]{'+str(count*8)+'}',value) is None:
        raise ValueError('cartesian_bundle_wire_identity')
    return struct.unpack('<'+'i'*count,bytes.fromhex(value))


def _float_words(values):
    result=[]
    for v in values:
        if (type(v) not in (float,int) or not math.isfinite(v) or abs(v)>1e6
                or 0<abs(v)<2**-1022):
            raise ValueError('cartesian_bundle_wire_float')
        result.extend(struct.unpack('<ii',struct.pack('<d',v)))
    return result


@dataclass(frozen=True)
class CompactBundle:
    session: str
    model_sha256: str
    generation: int
    sequence: int
    parent_tick: int
    due_ms: int  # Controller epoch-relative time, never host/wall time.
    captured_ms: int
    lead: PvatSegment
    terminal: JointReference
    braking_ticks: int

    def __post_init__(self):
        _hash_words(self.session,4);_hash_words(self.model_sha256,8)
        _integer(self.generation,1);_integer(self.sequence,1);_integer(self.parent_tick)
        _integer(self.due_ms);_integer(self.captured_ms)
        _integer(self.braking_ticks,1,60)
        if (not isinstance(self.lead,PvatSegment) or self.lead.duration!=PERIOD
                or not isinstance(self.terminal,JointReference)
                or any(x!=0. for x in self.terminal.v+self.terminal.a)
                or self.captured_ms>self.due_ms):
            raise ValueError('cartesian_bundle_wire_structure')
        _float_words(self.lead.start.q+self.lead.start.v+self.lead.start.a
            +self.lead.end.q+self.lead.end.v+self.lead.end.a+self.terminal.q)

    def expand_tail(self):
        whole=PvatSegment(self.lead.end,self.terminal,self.braking_ticks*PERIOD)
        result=[];previous=self.lead.end
        for tick in range(1,self.braking_ticks+1):
            end=self.terminal if tick==self.braking_ticks else whole.state(tick*PERIOD)
            result.append(PvatSegment(previous,end,PERIOD));previous=end
        return tuple(result)


def encode(bundle):
    if type(bundle) is not CompactBundle:raise ValueError('cartesian_bundle_wire_structure')
    fields=[MAGIC,bundle.generation,*_hash_words(bundle.session,4),*_hash_words(bundle.model_sha256,8),
        bundle.parent_tick,bundle.due_ms,bundle.captured_ms,bundle.braking_ticks]
    fields+=_float_words(bundle.lead.start.q+bundle.lead.start.v+bundle.lead.start.a
        +bundle.lead.end.q+bundle.lead.end.v+bundle.lead.end.a+bundle.terminal.q)
    words=[bundle.sequence]+fields
    words+=[0]*(BANK_SIZE-1-len(words))
    words.append(_signed(zlib.crc32(struct.pack('<'+'i'*(BANK_SIZE-1),*words))))
    return tuple(words)


def decode(words):
    if type(words) not in (tuple,list) or len(words)!=BANK_SIZE:
        raise ValueError('cartesian_bundle_wire_shape')
    for value in words:_integer(value,-2**31,2**31-1)
    _integer(words[0],1)
    checksum=_signed(zlib.crc32(struct.pack('<'+'i'*(BANK_SIZE-1),*words[:-1])))
    if checksum!=words[-1] or words[1]!=MAGIC or any(words[103:-1]):
        raise ValueError('cartesian_bundle_wire_integrity')
    values=[]
    for index in range(19,103,2):values.append(struct.unpack('<d',struct.pack('<ii',*words[index:index+2]))[0])
    _float_words(values)
    start=JointReference(tuple(values[:6]),tuple(values[6:12]),tuple(values[12:18]))
    end=JointReference(tuple(values[18:24]),tuple(values[24:30]),tuple(values[30:36]))
    return CompactBundle(session=struct.pack('<4i',*words[3:7]).hex(),
        model_sha256=struct.pack('<8i',*words[7:15]).hex(),generation=words[2],sequence=words[0],
        parent_tick=words[15],due_ms=words[16],captured_ms=words[17],braking_ticks=words[18],
        lead=PvatSegment(start,end,PERIOD),terminal=JointReference(tuple(values[36:42])))
