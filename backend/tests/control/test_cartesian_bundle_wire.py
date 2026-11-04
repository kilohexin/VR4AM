import struct
import zlib

import pytest

from app.control.cartesian_pvat_reference import JointReference,PvatSegment
from app.control.cartesian_bundle_wire import BANK_SIZE,CompactBundle,decode,encode


def packet():
    start=JointReference((0.,-.8,1.4,-.6,1.57,-0.))
    end=JointReference((.0001,-.8002,1.4001,-.6,1.57,.0001),(.001,)*6,(.002,)*6)
    return CompactBundle('01'*16,'a9'*32,1,23,4,400,340,PvatSegment(start,end,.08),JointReference(end.q),10)


def test_wire_is_lossless_binary64_and_fits_two_native_banks():
    p=packet();words=encode(p);r=decode(words)
    assert len(words)==BANK_SIZE==120 and 2*BANK_SIZE==240
    assert r==p and encode(r)==words
    assert struct.pack('<d',r.lead.start.q[-1])==struct.pack('<d',-0.)
    tail=r.expand_tail()
    assert tail[0].start==r.lead.end and tail[-1].end==r.terminal
    assert all(a.end==b.start for a,b in zip(tail,tail[1:]))


def test_damage_in_every_word_is_rejected_even_when_header_was_not_torn():
    good=encode(packet())
    for index in range(len(good)):
        bad=list(good);bad[index]^=1
        with pytest.raises(ValueError):decode(bad)


def checksum(words):
    crc=zlib.crc32(struct.pack('<119i',*words[:-1]));words[-1]=crc if crc<2**31 else crc-2**32
    return words


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf'),1e9,5e-324])
def test_forged_crc_cannot_admit_invalid_floats(value):
    words=list(encode(packet()));words[19:21]=struct.unpack('<ii',struct.pack('<d',value))
    with pytest.raises(ValueError):decode(checksum(words))


def test_reserved_words_and_bad_structure_reject_even_with_valid_crc():
    good=list(encode(packet()))
    for index,value in ((0,0),(2,0),(16,0),(18,0),(18,61),(103,1)):
        words=good.copy();words[index]=value
        with pytest.raises(ValueError):decode(checksum(words))


def test_native_int32_contract_rejects_bool_and_out_of_range_values():
    words=list(encode(packet()))
    for value in (True,2**31,-2**31-1,0.5):
        words[40]=value
        with pytest.raises(ValueError):decode(words)
