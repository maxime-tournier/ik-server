
from tool import *
import kinect
import target
import threading
import skeleton
import solver
import time
import model
import sys
import constraint
import json
import socket

ip = '172.22.15.162'
port = 9000

# source thread
def fetch(event, result):

    for data in kinect.data(ip = ip, port = port):
        if data: event.set()
        result['data'] = data


source = {}
ready = threading.Event()

thread = threading.Thread( target = fetch, args = (ready, source) )
thread.daemon = True
thread.start()


definition = skeleton.load( 'skeleton.json')
body, joint = skeleton.create( definition )
dofs = Rigid3.array( len(body) )

adaptor = model.kinect_adaptor(body)
world = target.World()

with open('calibration.json') as f:
    world.load( json.loads( f.read() ) )

def inertia(dofs):
    return skeleton.inertia(body, dofs)

def skeleton_constraints(dofs, **kwargs):
    return skeleton.constraints(joint, dofs, compliance = 1e-8)


dt = 1e-1
eps = 1e-2

print 'building skeleton...'
for d, mu in solver.step(dofs, inertia,
                         skeleton_constraints,
                         dt = dt):
    error = norm(mu) / dt
    # print error
    if error <= eps: break

print 'ok'

print 'waiting for kinect data...'
while not ready.wait(1): pass
print 'ok'

def server(**kwargs):
    ip = ''
    port = kwargs.get('port', 9000)

    while True:

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind((ip, port))
            sock.listen(1)

            conn, addr = sock.accept()

            print 'client connected'
            yield conn.makefile()

        finally:
            print 'client disconnected'
            # conn.close()
            sock.close()


ex = basis(0, 3)                        
ey = basis(1, 3)            
ez = basis(2, 3)

# ones = np.ones(3)
# ones /= norm(ones)

def flip( d ):
    axis = np.array( map(float, d) )
    axis /= norm(axis)

    return Quaternion.exp( math.pi * axis)

system_left = flip( [1, -1, 0] )
system_right = flip( [1, 1, 0] )
system_spine = flip( [0, 0, 1] )

shoulder_origin = Quaternion.exp( -math.pi / 2.0 * ez )
id = Quaternion()

info = {
    'LeftElbow': ('elbow_left', system_left, id ),
    'LeftShoulder': ('shoulder_left', system_left, shoulder_origin ),
    
    'RightElbow': ('elbow_right', system_right, id),
    'RightShoulder': ('shoulder_right', system_right, shoulder_origin.conj()),

    'Spine1': ('spine', system_spine, id),
}

def target_constraints(dofs, **kwargs):
    t = target.create( source['data'], adaptor, world )
    return target.constraints(t, body, dofs, compliance = 1e-4)

print 'waiting for occulus client...'

def send(event, result):
    
    for client in server():

        while True:
            event.wait()
            dofs = result['dofs']
            event.clear()

            data = [ ]

            data.append( {'position': 'Hips',
                          'x': '0',
                          'y': '1',
                          'z': '0' } )

            for k, (v, ref, origin) in info.iteritems():
                state = origin.conj() * ref.conj() * joint[v].state(dofs) * ref 

                # order = [1, 2, 0]
                order = [1, 0, 2]

                # print 'log:', state.log()

                # angles = -ey * state.angle() * Quaternion.deg
                angles = state.euler(order) * Quaternion.deg

                print k, angles
                
                chunk = {
                    'rotation' : k,
                    'x': str(angles[0]),
                    'y': str(angles[1]),
                    'z': str(angles[2]),
                }

                data.append( chunk )

            msg = json.dumps( data )
            client.write( msg + '\r\n' )

occulus = threading.Thread(target = send, args = (ready, source) )
occulus.daemon = True
occulus.start()

print 'ik started'
for d, mu in solver.step(dofs, inertia,
                         constraint.merge(skeleton_constraints,
                                          target_constraints),
                         dt = dt):
    source['dofs'] = np.copy(d).view(dtype = Rigid3)
    ready.set()
        

    
