
import numpy as np
from tool import *

import mapping

# definition = [

#     { 'body': 'forearm_left',
#       'local': [0, -0.1, 0],
#       'world': [0, 1, 1],
#       'compliance': 1e-4 },

#     { 'body': 'forearm_right',
#       'local': [0, -0.1, 0],
#       'world': [0, 1, 1],
#       'compliance': 1e-4 }

# ]



class World:

        def __init__(self):
            self.frame = Rigid3()
            self.scale = 1.0

        def __call__(self, x):
            return self.frame( self.scale * x )


        def dump(self):
            return self.__dict__

        def load(self, x):
            self.scale = x['scale']
            self.frame.load(x['frame'])
            
        def inv(self):
            res = World()
            res.frame = self.frame.inv()
            res.frame.center /= self.scale
            res.scale = 1.0 / self.scale
            return res


def chunk(name, local, **kwargs):
    '''produces a target info from desired coordinates'''
    
    compliance = kwargs.get('compliance', 1e-5)

    def res(desired):
        return {
            'body': name,
            'local': local,
            'desired': desired,
            # 'compliance': compliance
        }

    return res

def create( data, adaptor, world ):
    '''create targets from json objects.

     adaptor maps between json targets and current body
    targets.

    '''

    def desired(v): return np.array(map(float, [ v['x'],
                                                 v['y'],
                                                 v['z'] ]) )
    
    res = []
    if not data: return res

    for d in data:
        pos = d.get('position', None)

        if pos:
            adapt = adaptor.get(pos)
            if adapt:
                p = world( desired(d) )

                res.append( adapt( p ) )

    return res



def constraints(info, body, dofs, **kwargs):
    rows = 3
    cols = 6 * len(body)

    compliance = kwargs.get('compliance', 0)
    world = kwargs.get('world', lambda x: x)
    
    res = []
    
    for i, t in enumerate(info):
        k = body[ t['body'] ].index
        
        local = np.array(t['local'])
        desired = world( np.array(t['desired']) )

        J = np.zeros( (rows, cols) )
        J[:, 6*k: 6*k+6], current = mapping.rigid(dofs[k], local)

        phi = current - desired

        c = t.get('compliance', compliance) * np.ones( 3 )
        
        res.append( (J, phi, c) )
        
    return res


        
        
