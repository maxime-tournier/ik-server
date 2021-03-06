
import numpy as np
import mapping


# TODO not quite sure attach/rest belong here, should probably be in
# skeleton


# a constraint should return a list of tuple of the form: (J, phi, c)
# where J is the constraint jacobian, phi is the constraint error and
# c is the constraint compliance. you may add other info to the tuple
# as well for you own purpose.

def attach(joint, dofs, **kwargs):
    
    rows = 3
    cols = 6 * len(dofs)

    compliance = kwargs.get('compliance', 0)

    res = []
    
    for j in joint.itervalues():
        i = j.index
        pi = j.parent.body.index
        ci = j.child.body.index

        Jp, phip = mapping.rigid(dofs[pi], j.parent.frame )
        Jc, phic = mapping.rigid(dofs[ci], j.child.frame )

        J = np.zeros( (rows, cols) )
        
        J[ :, 6 * pi: 6 * pi + 6 ] = Jp
        J[ :, 6 * ci: 6 * ci + 6 ] = -Jc

        phi = phip - phic

        c = compliance * np.ones( 3 )

        res.append( (J, phi, c) )

    return res


def rest(joint, dofs, **kwargs):

    rows = 3
    cols = 6 * len(dofs)
    
    res = []

    for j in joint.itervalues():
        if j.rest is None: continue
        
        pi = j.parent.body.index
        ci = j.child.body.index
        
        Jp, Jc, phi = mapping.relative(dofs[pi], dofs[ci], j.rest)

        J = np.zeros( (rows, cols) )
        
        J[ :, 6 * pi: 6 * pi + 6 ] = Jp
        J[ :, 6 * ci: 6 * ci + 6 ] = Jc

        c = j.compliance * np.ones( 3 )

        res.append( (J, phi, c) )
        
    return res




def assemble( chunks ):

    J = np.vstack( (c[0] for c in chunks ) )
    phi = np.concatenate( tuple(c[1] for c in chunks ) )
    c = np.concatenate( tuple(c[2] for c in chunks ) )

    return J, phi, c


def merge( *args ):
    def res(dofs, **kwargs):
        return  sum( (f(dofs, **kwargs) for f in args), [] )
    return res


