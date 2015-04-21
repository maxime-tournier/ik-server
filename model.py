# default skeleton model

import numpy as np

from tool import *
import json


# TODO make this easy to customize
from numpy import array as vec


head = {
    'mass': 5,
    'dim': vec([0.25, 0.25, 0.25]),
}

trunk =  {
    'mass': 20,
    'dim': vec([0.4, 0.65, 0.25])
}


femur =  {
    'mass': 7,
    'dim': vec([0.2, 0.5, 0.2]),
}


tibia =  {
    'mass': 5,
    'dim': [0.10, 0.4, 0.1]
}

arm = {
    'mass': 4,
    'dim': [0.1, 0.4, 0.10]
}

forearm =  {
    'mass': 3,
    'dim': [0.1, 0.4, 0.1]
}

foot = {
    'mass': 2,
    'dim': [0.1, 0.3, 0.1]
}

stiffness = 1e2
compliance = 1 / stiffness
hard = 1e-8

def hip(side):

    sign = 1 if side == 'left' else -1
    
    return {
        "coords": [['trunk', [sign * trunk['dim'][0] / 2,
                              - trunk['dim'][1] / 2,
                              0]],
                   ['femur_{}'.format(side), [0,
                                              femur['dim'][1] / 2,
                                              0]]],
        "rest": Quaternion(),
        "compliance": compliance
    }


def shoulder(side):
    sign = 1 if side == 'left' else -1
    
    return {
        "coords": [['trunk', [sign * (1.2 * trunk['dim'][0]) / 2,
                              trunk['dim'][1] / 2,
                              0]],
                   ['arm_{}'.format(side), [0, arm['dim'][1] / 2, 0]]],
        
        "rest": Quaternion.exp( sign * math.pi / 6.0 * basis(2, 3)),
        "compliance": compliance
    }


def elbow(side):
    return  {
        "coords": [['arm_{}'.format(side), [0,
                                            -arm['dim'][1] / 2,
                                            0]],
                   ['forearm_{}'.format(side), [0,
                                                forearm['dim'][1] / 2,
                                                0]]],
        "rest": Quaternion.exp( -math.pi / 6 * basis(0, 3) ),
        "compliance": [compliance, hard, hard]
    }

def knee(side):
    return {
        "coords": [['femur_{}'.format(side), [0,
                                              -femur['dim'][1] / 2,
                                              0]],
                   ['tibia_{}'.format(side), [0,
                                              tibia['dim'][1] / 2,
                                              0]]],
        "rest": Quaternion.exp( math.pi / 6 * basis(0, 3) ),
        "compliance": [compliance, hard, hard]
    }

# TODO finer ?
def ankle(side):
    return {
        "coords": [['tibia_{}'.format(side), [0,
                                              -tibia['dim'][1] / 2,
                                              0]],
                   ['foot_{}'.format(side), [0,
                                             foot['dim'][1] / 2,
                                             0]]],
        "rest": Quaternion.exp( -math.pi /2  * basis(0, 3) ),
        "compliance": compliance
    }


skeleton = {
    
    'body': {

        'head': head,
        'trunk': trunk,

        'femur_left': femur,
        'femur_right': femur,

        'tibia_left': tibia,
        'tibia_right': tibia,

        'arm_left': arm,
        'arm_right': arm,
        
        'forearm_left': forearm,
        'forearm_right': forearm,

        'foot_left': foot,
        'foot_right': foot
    },
    

    'joint': {

        'neck': {
            "coords": [['head', [0, -0.15, 0]],
                       ['trunk', [0, 0.35, 0]]],
            "rest": Quaternion(),
            "compliance": compliance
        },

        'shoulder_left': shoulder('left'),
        'shoulder_right': shoulder('right'),

        'elbow_left': elbow('left'),
        'elbow_right': elbow('right'),

        'hip_left': hip('left'),
        'hip_right': hip('right'),
        
        'knee_left': knee('left'),
        'knee_right': knee('right'),

        'ankle_left': ankle('left'),
        'ankle_right': ankle('right'),
        
    }
}


def cleanup(x):
    '''make skeleton definition json-friendly'''
    
    if type(x) is dict:
        return { k:cleanup(v) for k, v in x.iteritems() }
    if type(x) is list:
        return [ cleanup(i) for i in x ]

    try:
        return x.tolist()
    except AttributeError:
        return x



# target mapping



def kinect_adaptor(body):

    import target
    
    def end_point(name, **kwargs):
        return target.chunk(name, -basis(1, 3) * body[name].dim / 2, **kwargs)


    def start_point(name, **kwargs):
        return target.chunk(name, basis(1, 3) * body[name].dim / 2, **kwargs)

    
    return {
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
    

if __name__ == '__main__':
    print json.dumps( cleanup(skeleton) )

