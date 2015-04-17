
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



def constraints(info, body, dofs, **kwargs):
    rows = 3
    cols = 6 * len(body)

    compliance = kwargs.get('compliance', 0)

    res = []
    
    for i, c in enumerate(info):
        k = body[ c['body'] ].index
        
        local = np.array(c['local'])
        desired = np.array(c['world'])

        J = np.zeros( (rows, cols) )
        J[:, 6*k: 6*k+6], world = mapping.rigid(dofs[k], local)

        phi = world - desired

        c = c.get('compliance', compliance) * np.ones( 3 )
        
        res.append( (J, phi, c) )
        
    return res


        
        
