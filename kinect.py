import socket
import time
import sys
import json

def status( e ):
    print >>sys.stderr, e
    sys.stderr.flush()

# this comes from the network
def data(**kwargs):

    ip = kwargs['ip']
    port = kwargs.get('port', 9000)
    size = kwargs.get('size', 1024)
    # timeout = kwargs.get('timeout', None)

    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        status('connecting to source...' )
        sock.connect( (ip, port) )
        status('ok')

        try:
            stream = sock.makefile(mode = 'rw')

            for line in stream:
                try:
                    obj = json.loads(line)
                    stream.write('ack\n');
                    stream.flush()
                    yield obj
                except ValueError, e:
                    status('JSON failed: {}'.format(e) )
                    status(line)

        except socket.error:
            status('source diconnected')
        finally:
            sock.close()

