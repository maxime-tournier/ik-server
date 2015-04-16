
import numpy as np
import mapping

def attach(joint, dofs, **kwargs):
    
    rows = 3 * len(joint)
    cols = 6 * len(dofs)

    c = kwargs.get('compliance', 0)
    
    matrix = np.zeros( (rows, cols) )
    value = np.zeros( rows )
    compliance = np.zeros( rows )
    
    for j in joint.itervalues():
        i = j.index
        pi = j.parent.body.index
        ci = j.child.body.index

        Jp = mapping.rigid(dofs[pi], j.parent.frame )
        Jc = mapping.rigid(dofs[ci], j.child.frame )

        matrix[ 3*i: 3*i + 3, 6 * pi: 6 * pi + 6 ] = Jp
        matrix[ 3*i: 3*i + 3, 6 * ci: 6 * ci + 6 ] = -Jc

        value[ 3*i: 3*i + 3] = dofs[pi](j.parent.frame) - dofs[ci](j.child.frame)

        compliance[ 3*i: 3*i + 3] = c * np.ones( 3 )
        
    return matrix, value, compliance


def rest(joint, dofs, **kwargs):

    num = len( [j for j in joint.itervalues() if j.rest is not None] )

    if num == 0: return None
    
    rows = 3 * num
    cols = 6 * len(dofs)
    
    J = np.zeros( (rows, cols) )
    b = np.zeros( rows )
    C = np.zeros( rows )

    i = 0
    for j in joint.itervalues():
        if j.rest is None: continue
        
        pi = j.parent.body.index
        ci = j.child.body.index
        
        Jp, Jc, bi = mapping.relative(dofs[pi], dofs[ci], j.rest)

        J[ 3*i: 3*i + 3, 6 * pi: 6 * pi + 6 ] = Jp
        J[ 3*i: 3*i + 3, 6 * ci: 6 * ci + 6 ] = Jc

        b[ 3*i: 3*i + 3 ] = bi

        C[ 3*i: 3*i + 3] = j.compliance * np.ones( 3 )
        i += 1
        
    return J, b, C



