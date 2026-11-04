"""Static loopback-only native3010 pipe for an already admitted Docker.

No shell command interpolation; no real port/address option. TCP_NODELAY
and a quick ACK after each read avoid small-packet delayed-ACK stalls.
Failure to provide these socket options rejects the pipe, never falls back.
"""
BRIDGE=r'''
use strict;
use warnings;
use IO::Socket::INET;
use IO::Select;
use Socket qw(IPPROTO_TCP TCP_NODELAY TCP_QUICKACK);
my $marker=$ARGV[0] // '';
my $sock=IO::Socket::INET->new(PeerAddr=>'127.0.0.1',PeerPort=>3010,Proto=>'tcp',Timeout=>3)
    or die "native_pipe_connect";
setsockopt($sock,IPPROTO_TCP,TCP_NODELAY,pack('i',1)) or die "native_pipe_nodelay";
setsockopt($sock,IPPROTO_TCP,TCP_QUICKACK,pack('i',1)) or die "native_pipe_quickack";
binmode STDIN;binmode STDOUT;binmode $sock;
my $select=IO::Select->new($sock,\*STDIN);
my $scan='';
sub write_all {
    my ($handle,$data)=@_;my $offset=0;
    while ($offset<length($data)) {
        my $count=syswrite($handle,$data,length($data)-$offset,$offset);
        defined($count) && $count>0 or die "native_pipe_write";$offset+=$count;
    }
}
while (my @ready=$select->can_read(20)) {
    for my $input (@ready) {
        my $data='';my $count=sysread($input,$data,16384);
        defined($count) or die "native_pipe_read";
        if (fileno($input)==fileno(STDIN)) {
            if ($count==0) {$select->remove(\*STDIN);next;}
            write_all($sock,$data);
        } else {
            exit 0 if $count==0;
            setsockopt($sock,IPPROTO_TCP,TCP_QUICKACK,pack('i',1)) or die "native_pipe_quickack";
            write_all(\*STDOUT,$data);
            $scan=substr($scan.$data,-8192);
            exit 0 if length($marker) && $scan=~/(?:^|\n)\Q$marker\E\t[^\n]*\n/;
        }
    }
}
die "native_pipe_timeout";
'''


def arguments(admission,marker=''):
    if marker and marker not in ('CART_DONE',):raise ValueError('native_pipe_marker')
    return ['docker','exec','-i',admission['container_id'],'perl','-e',BRIDGE,marker]
