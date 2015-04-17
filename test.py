

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
            'world': world,
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

    def constraints(dofs):

        # print payload[0]
        t = targets()

        # if t: print t

        # hack
        w.targets = t
        
        full = (skeleton.constraints(joint, dofs, compliance = 0) +
                target.constraints(t, body, dofs, compliance = 1e-5)
        )
        
        return constraint.merge( full )


    w.source = solver.step(dofs, inertia, constraints, dt = 1e-1)

    w.setSceneRadius( 10 )
    w.show()
    w.startAnimation()

# TODO:

# limits
# - network
# - external forces

