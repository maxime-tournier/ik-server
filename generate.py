import numpy as np

from tool import *
import json


# TODO make this easy to customize

skeleton = {
    
    'body': {

        'head': {
            'mass': 5,
            'dim': [0.3, 0.3, 0.3]
        },

        'trunk': {
            'mass': 20,
            'dim': [0.40, 0.70, 0.30]
        },

        'femur_left': {
            'mass': 7,
            'dim': [0.20, 0.5, 0.30]
        },

        'femur_right': {
            'mass': 7,
            'dim': [0.20, 0.5, 0.30]
        },

        'tibia_left': {
            'mass': 5,
            'dim': [0.20, 0.4, 0.30]
        },

        'tibia_right': {
            'mass': 5,
            'dim': [0.20, 0.4, 0.30]
        },

        'arm_left': {
            'mass': 4,
            'dim': [0.10, 0.4, 0.10]
        },

        'forearm_left': {
            'mass': 3,
            'dim': [0.10, 0.4, 0.10]
        },

        
        'arm_right': {
            'mass': 4,
            'dim': [0.10, 0.4, 0.10]
        },

        'forearm_right': {
            'mass': 3,
            'dim': [0.10, 0.4, 0.10]
        },

        'foot_left': {
            'mass': 2,
            'dim': [0.10, 0.3, 0.10]
        },

        'foot_right': {
            'mass': 2,
            'dim': [0.10, 0.3, 0.10]
        },

        
    },
    

    'joint': {

        'neck': {
            "coords": [['head', [0, -0.15, 0]],
                       ['trunk', [0, 0.35, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-1
        },

        'hip_left': {
            "coords": [['trunk', [-0.2, -0.35, 0]],
                       ['femur_left', [0, 0.25, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-2
        },

        'hip_right': {
            "coords": [['trunk', [0.2, -0.35, 0]],
                        ['femur_right', [0, 0.25, 0]]],
            
            "rest": Quaternion().tolist(),
            "compliance": 1e-2
        },

        'shoulder_left': {
             "coords": [['trunk', [-0.25, 0.35, 0]],
                        ['arm_left', [0, 0.2, 0]]],
             
             "rest": Quaternion.exp( -math.pi / 2 * basis(2, 3)).tolist(),
             "compliance": 1e-2
        },

        'shoulder_right': {
            "coords": [['trunk', [0.25, 0.35, 0]],
                       ['arm_right', [0, 0.2, 0]]],

            "rest": Quaternion.exp( math.pi / 2 * basis(2, 3)).tolist(),
             "compliance": 1e-2
        },

        'elbow_left': {
            "coords": [['arm_left', [0, -0.2, 0]],
                       ['forearm_left', [0, 0.2, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-1
        },

        'elbow_right': {
            "coords": [['arm_right', [0, -0.2, 0]],
                       ['forearm_right', [0, 0.2, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-1
        },

        'knee_left': {
            "coords": [['femur_left', [0, -0.2, 0]],
                       ['tibia_left', [0, 0.2, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-1
            
        },

        'knee_right': {
            "coords": [['femur_right', [0, -0.2, 0]],
                       ['tibia_right', [0, 0.2, 0]]],
            "rest": Quaternion().tolist(),
            "compliance": 1e-1
            
        },

        'ankle_left': {
            "coords": [['tibia_left', [0, -0.2, 0]],
                       ['foot_left', [0, 0.15, 0]]],
            "rest": Quaternion.exp( -math.pi /2  * basis(0, 3) ).tolist(),
            "compliance": 1e-1
            
        },

        'ankle_right': {
            "coords": [['tibia_right', [0, -0.2, 0]],
                       ['foot_right', [0, 0.15, 0]]],
            "rest": Quaternion.exp( -math.pi /2  * basis(0, 3) ).tolist(),
            "compliance": 1e-1
            
        },
        
       
        
    }
}


print json.dumps( skeleton )


