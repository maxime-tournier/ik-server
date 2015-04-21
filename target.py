
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


        
        
