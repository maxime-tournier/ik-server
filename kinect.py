import socket
import time
import sys
import json
import gc

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
        
            content = sock.makefile()

            for line in content:
                try:
                    yield json.loads(line)
                except ValueError, e:
                    status('JSON failed: {}'.format(e) )
                    status(line)

                gc.collect()
        except socket.error:
            status('source diconnected')
        finally:
            sock.close()

