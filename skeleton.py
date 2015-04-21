
import numpy as np
from tool import *
import json

def load(filename):
    try:
        return json.loads( open('skeleton.json').read() )
    except:
        import sys
        print 'error loading skeleton, generate one first'
        raise

def create( skeleton ):

    class Body: pass

    body = {}

    for i, (k, v) in enumerate( skeleton['body'].iteritems() ):
        b = Body()

        b.index = i
        b.name = k

        b.mass = float(v.get('mass', 1))
        b.dim = np.array( v.get('dim', [1.0, 1.0, 1.0] ) )

        dim2 = b.dim * b.dim
        b.inertia = b.mass / 12.0 * (sum(dim2) - dim2)

        body[k] = b


    class Joint:

        class Coords: pass

    joint = {}

    for i, (k, v) in enumerate( skeleton['joint'].iteritems() ):
        j = Joint()

        j.index = i
        j.name = k

        coords = v['coords']

        parent = coords[0]
        child = coords[1]
        
        # TODO info[] ?
        j.parent = Joint.Coords()
        j.parent.body = body[ parent[0] ]
        j.parent.frame = parent[1]

        j.child = Joint.Coords()
        j.child.body = body[ child[0] ]
        j.child.frame = child[1]
        
        j.rest = v.get('rest', None)
        j.compliance = v.get('compliance', None)

        # np arrays
        if j.rest is not None:
            j.rest = np.array(j.rest).view( Quaternion )

        if j.compliance is not None:
            j.compliance = np.array(j.compliance)
            
        joint[k] = j

    return body, joint




def inertia(body, dofs):
    dim = 6 * len(body)
    res = np.zeros( (dim, dim) )

    block = np.zeros( (6, 6) )
    
    for b in body.itervalues():
        i = b.index
        block[:3, :3] = b.mass * np.identity(3)
        block[3:, 3:] = np.diag( b.inertia )
        
        res[6*i: 6*i + 6,
            6*i: 6*i + 6] = block
        
    return res


import constraint

def constraints(joint, dofs, **kwargs):
    
    sub = [constraint.attach, constraint.rest
    ]
    return sum( (c(joint, dofs, **kwargs) for c in sub), [] )



    
