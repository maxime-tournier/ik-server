

from tool import *
import json


try:
    definition = json.loads( open('skeleton.json').read() )
except:
    import sys
    print 'error loading skeleton, generate one first'
    sys.exit(0)


import skeleton


body, joint = skeleton.create( definition )
dofs = Rigid3.array( len(body) )

import pyqglviewer
import solver
import gui

import constraint
import target
import threading

import kinect
import json

payload = [None]

def fetch():

    while True:
        for data in kinect.data(ip = '127.0.0.1',
                                port = 9000):
            payload[0] = data


        
thread = threading.Thread(target = fetch)
thread.daemon = True
thread.start()



def vec(v): return map(float, [ v['x'],
                                v['y'],
                                v['z'] ])



               
def make_chunk(name, local, **kwargs):

    compliance = kwargs.get('compliance', 1e-4)
    
    def res(world):
        return {
            'body': name,
            'local': local,
            'desired': world,
            'compliance': compliance
        }

    return res


def end_point(name, **kwargs):
    return make_chunk(name, -basis(1, 3) * body[name].dim / 2, **kwargs)


def start_point(name, **kwargs):
    return make_chunk(name, basis(1, 3) * body[name].dim / 2, **kwargs)


occulus = {
    'LeftHand': end_point('forearm_left'),
    'LeftElbow': end_point('arm_left'),    
    'RightHand': end_point('forearm_right'),
    'RightElbow': end_point('arm_right'),

    'Hips': end_point('trunk'),
    'Head': start_point('head'),
    'Neck': end_point('head'),
}

kinect = {
    'HandLeft': end_point('forearm_left'),
    'ElbowLeft': end_point('arm_left'),    

    'HandRight': end_point('forearm_right'),
    'ElbowRight': end_point('arm_right'),

    'HipsLeft': start_point('femur_left'),
    'HipsRight': start_point('femur_right'),    

    'KneeLeft': start_point('tibia_left'),
    'KneeRight': start_point('tibia_right'),

    'AnkleLeft': end_point('tibia_left'),
    'AnkleRight': end_point('tibia_right'),
    
    'Head': start_point('head'),
    'Neck': end_point('head'),
}


adaptor = kinect


def targets():
    data = payload[0]

    res = []
    if not data: return res

    for chunk in data:
        pos = chunk.get('position', None)

        if pos:
            adapt = adaptor.get(pos)
            if adapt:
                res.append( adapt( vec(chunk)) )

    return res




with pyqglviewer.app():    

    w = gui.Viewer()

    w.body = body
    w.joint = joint


    def inertia(dofs):
        return skeleton.inertia(body, dofs)


    def skeleton_constraints(dofs):
        return skeleton.constraints(joint, dofs, compliance = 0)


    # TODO these need world 
    def target_constraints(dofs):
        return target.constraints(targets(), body, dofs, compliance = 1e-5)

    
  

    def build( *args ):
        def res(dofs):
            full = sum( (f(dofs) for f in args), [] )
            return constraint.merge( full )
        return res
                            

    def source(**kwargs):

        dt = kwargs.get('dt', 1e-1)
        eps = kwargs.get('eps', 1e-2)
        
        # don't enable targets just yet to get skeleton in shape
        print 'assembling skeleton...'
        for d, mu in solver.step(dofs, inertia,
                                 build(skeleton_constraints),
                                 dt = dt):
            yield dofs
            error = norm(mu)
            # print error
            if error <= eps: break

        print 'calibrating...'
        # now fix the target constraint and calibrate
        init_targets = targets()
        init_dofs = np.copy( dofs ).view( dtype = Rigid3 )

        w.targets = init_targets
        def init_target_constraints(dofs):
            return target.constraints(init_targets, body, dofs, compliance = 1e-5)

        for d, mu in solver.step(dofs, inertia,
                                 build(skeleton_constraints,
                                       init_target_constraints),
                                 dt = dt):
            # to stuff with mu
            yield init_dofs
            dofs[:] = init_dofs

        # then regular ik
        print 'ik started'
        for d, mu in solver.step(dofs, inertia,
                                    build(skeleton_constraints,
                                          target_constraints),
                                    dt = dt):
            # TODO hack
            w.targets = targets()
            yield dofs

    w.source = source(dt = 1e-1)
    
    w.setSceneRadius( 10 )
    w.show()
    w.startAnimation()

# TODO:

# limits
# - network
# - external forces

