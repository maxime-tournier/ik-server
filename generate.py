import numpy as np

from tool import *
import json


# TODO make this easy to customize
from numpy import array as vec

class Info : pass





head = {
    'mass': 5,
    'dim': vec([0.3, 0.3, 0.3]),
}

trunk =  {
    'mass': 20,
    'dim': vec([0.4, 0.7, 0.3])
}


femur =  {
    'mass': 7,
    'dim': vec([0.2, 0.5, 0.3]),
}


tibia =  {
    'mass': 5,
    'dim': [0.20, 0.4, 0.30]
}

arm = {
    'mass': 4,
    'dim': [0.10, 0.4, 0.10]
}

forearm =  {
    'mass': 3,
    'dim': [0.10, 0.4, 0.10]
}

foot = {
    'mass': 2,
    'dim': [0.10, 0.3, 0.10]
}

stiffness = 1e2
compliance = 1 / stiffness

def hip(side):

    sign = -1 if side is 'left' else 1
    
    return {
        "coords": [['trunk', [sign * 0.2, -0.35, 0]],
                   ['femur_{}'.format(side), [0, 0.25, 0]]],
        "rest": Quaternion(),
        "compliance": compliance
    }


def shoulder(side):
    sign = -1 if side is 'left' else 1
    
    return {
        "coords": [['trunk', [-0.25, 0.35, 0]],
                   ['arm_{}'.format(side), [0, 0.2, 0]]],
        
        "rest": Quaternion.exp( sign * math.pi / 2 * basis(2, 3)),
        "compliance": compliance
    }


def elbow(side):
    return  {
        "coords": [['arm_{}'.format(side), [0, -0.2, 0]],
                   ['forearm_{}'.format(side), [0, 0.2, 0]]],
        "rest": Quaternion(),
        "compliance": compliance
    }

def knee(side):
    return {
        "coords": [['femur_{}'.format(side), [0, -0.2, 0]],
                   ['tibia_{}'.format(side), [0, 0.2, 0]]],
        "rest": Quaternion(),
        "compliance": compliance
    }

def ankle(side):
    return {
        "coords": [['tibia_{}'.format(side), [0, -0.2, 0]],
                   ['foot_{}'.format(side), [0, 0.15, 0]]],
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

        'hip_left': hip('left'),
        'hip_right': hip('right'),

        'shoulder_left': shoulder('left'),
        'shoulder_right': shoulder('right'),

        'elbow_left': elbow('left'),
        'elbow_right': elbow('right'),

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


