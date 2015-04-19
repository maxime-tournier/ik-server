

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
import math

payload = [None]

def fetch():

    while True:
        for data in kinect.data(ip = '127.0.0.1',
                                port = 9000):
            payload[0] = data


        
thread = threading.Thread(target = fetch)
thread.daemon = True
thread.start()



def vec(v): return np.array(map(float, [ v['x'],
                                         v['y'],
                                         v['z'] ]) )



               
def make_chunk(name, local, **kwargs):

    compliance = kwargs.get('compliance', 1e-5)
    
    def res(world):
        return {
            'body': name,
            'local': local,
            'desired': world,
            # 'compliance': compliance
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
    'HandRight': end_point('forearm_right'),


    'HipLeft': start_point('femur_left'),
    'HipRight': start_point('femur_right'),    

    'AnkleLeft': end_point('tibia_left'),
    'AnkleRight': end_point('tibia_right'),
    
    'Head': start_point('head'),

    'KneeLeft': start_point('tibia_left'),
    'KneeRight': start_point('tibia_right'),

    'ElbowLeft': end_point('arm_left'),    
    'ElbowRight': end_point('arm_right'),

    'ShoulderLeft': start_point('arm_left'),    
    'ShoulderRight': start_point('arm_right'),

    
    'Neck': end_point('head'),
}


adaptor = kinect


class World:

    def __init__(self):
        self.frame = Rigid3()
        self.scale = 1.0
    
    def __call__(self, x):
        return self.frame( self.scale * x )


    def inv(self):
        res = World()
        res.frame = self.frame.inv()
        res.frame.center /= self.scale
        res.scale = 1.0 / self.scale
        return res
    
    
world = World()
    
def targets( data ):

    res = []
    if not data: return res

    for chunk in data:
        pos = chunk.get('position', None)

        if pos:
            adapt = adaptor.get(pos)
            if adapt:
                p = world( vec(chunk) )
                
                res.append( adapt( p ) )
        
    return res



with pyqglviewer.app():    

    w = gui.Viewer()

    w.body = body
    w.joint = joint


    def inertia(dofs):
        return skeleton.inertia(body, dofs)


    def skeleton_constraints(dofs):
        return skeleton.constraints(joint, dofs, compliance = 1e-8)


    # TODO these need world 
    def target_constraints(dofs):
        return target.constraints(targets( payload[0] ), body, dofs, compliance = 1e-4)

    
  

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
            error = norm(mu) / dt
            print error
            if error <= eps: break

        print 'calibrating...'
        
        # now fix the target constraint and calibrate
        init_data = payload[0]
        init_dofs = np.copy( dofs ).view( dtype = Rigid3 )

        def init_target_constraints(dofs):
            return target.constraints( targets( init_data ), body, dofs, compliance = 1e-4)

        h = 1e-2
        eps = 1e-3
        for d, mu in solver.step(dofs, inertia,
                                 build(skeleton_constraints,
                                       init_target_constraints),
                                 dt = h):
            # TODO hack
            w.targets = targets(init_data)

            alpha = 16.0
            mt = alpha
            mr = alpha
            ms = alpha

            vt = np.zeros(3)
            vr = np.zeros(3)            
            vs = 0

            ft = np.zeros(3)
            fr = np.zeros(3)            
            fs = 0
            
            damping = 2.0
            
            for i, t in enumerate(w.targets):
                start = len(mu) - 3 * len(w.targets) + 3 * i

                
                desired = t['desired']                
                local = world.inv()(desired)

                f = -mu[start: start+3]

                ft += f
                fr += world.scale * np.cross( local, world.frame.orient.conj()(f) )
                fs += world.frame.orient(local).dot(f)

            net = math.sqrt( norm2(ft) + norm2(fr) + fs * fs )

            if net <= eps: break 
            
            vt[:] = ft / mt
            vr[:] = fr / mr 
            vs = fs / ms
            
            # integrate
            world.frame.center += h * vt
            world.frame.orient = world.frame.orient * Quaternion.exp( h * vr )
            world.scale += h * vs

            yield init_dofs
            dofs[:] = init_dofs

        # then regular ik
        print 'ik started'
        for d, mu in solver.step(dofs, inertia,
                                    build(skeleton_constraints,
                                          target_constraints),
                                    dt = dt):
            # TODO hack
            w.targets = targets(payload[0])
            yield dofs

    w.source = source(dt = 1e-1)
    
    w.setSceneRadius( 10 )
    w.show()
    w.startAnimation()

# TODO:

# limits
# - network
# - external forces

