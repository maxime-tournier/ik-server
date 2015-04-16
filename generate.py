import numpy as np

from tool import *
import json


# TODO make this easy to customize
from numpy import array as vec


head = {
    'mass': 5,
    'dim': vec([0.3, 0.3, 0.3]),
}

trunk =  {
    'mass': 20,
    'dim': vec([0.4, 0.7, 0.25])
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

stiffness = 1e3
compliance = 1 / stiffness

def hip(side):

    sign = -1 if side == 'left' else 1
    
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
        
        "rest": Quaternion.exp( sign * math.pi / 2.0 * basis(2, 3)),
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
        "rest": Quaternion(),
        "compliance": [compliance, 0, 0]
    }

def knee(side):
    return {
        "coords": [['femur_{}'.format(side), [0,
                                              -femur['dim'][1] / 2,
                                              0]],
                   ['tibia_{}'.format(side), [0,
                                              tibia['dim'][1] / 2,
                                              0]]],
        "rest": Quaternion(),
        "compliance": [compliance, 0, 0]
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
    '''make skeleton defintion json-friendly'''
    
    if type(x) is dict:
        return { k:cleanup(v) for k, v in x.iteritems() }
    if type(x) is list:
        return [ cleanup(i) for i in x ]

    try:
        return x.tolist()
    except AttributeError:
        return x

print json.dumps( cleanup(skeleton) )


