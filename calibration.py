
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


try:
    ip = sys.argv[1]
except:
    ip = '127.0.0.1'
    
# source thread
def fetch(event, result):

    for data in kinect.data(ip = ip):
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

print 'waiting for kinect frames...'
ready.wait()
# print 'ok'

print 'kinect ready, press return to start calibration'
raw_input()

print 'stay still in front of the kinect'
for i in reversed( xrange(3) ):
    print i
    time.sleep(1)

frame = source['data']    
print 'starting calibration'

world = target.World()
adaptor = model.kinect_adaptor(body)

def target_constraints(dofs, **kwargs):

    out = kwargs['out_data']
    t = target.create( frame, adaptor, world )
    out['targets'] = t
    return target.constraints(t, body, dofs, compliance = 1e-4)

dt = 5e-3
eps = 1e-2

for d, t in solver.calibration(world, dofs, inertia,
                               constraint.merge(skeleton_constraints,
                                                target_constraints),
                               dt = dt,
                               eps = eps):
    pass

calibration_filename = 'calibration.json'
with open(calibration_filename, 'w') as f:
    f.write( json.dumps(world, default = lambda o: o.dump()))
    print 'wrote', calibration_filename
