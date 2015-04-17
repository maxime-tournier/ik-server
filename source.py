
import sys
import socket

import time

ip = '127.0.0.1'
port = 9000

filename = sys.argv[1]


while True:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:

        sock.bind((ip, port))
        sock.listen(1)

        print 'waiting for client...'
        conn, addr = sock.accept()
        print 'ok'

        while True:
            with open(filename) as f:
                for i, line in enumerate(f):
                    time.sleep(5e-2)
                    print 'frame', i
                    conn.send( line )
    except socket.error, e:
        print 'client disconnected', e
    finally:
        conn.close()
        sock.close()



