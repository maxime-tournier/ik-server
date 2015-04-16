import numpy as np
from tool import *

def rigid(dofs, local):
    res = np.zeros( (3, 6) )

    res[:, :3] = np.identity(3)

    R = dofs.orient.matrix()
    hat = Quaternion.hat(local)

    res[:, 3:] = -R.dot( hat )
    
    return res

# TODO: could be group-generic
def relative(p, c, rest):
    Jp = np.zeros( (3, 6) )
    Jc = np.zeros( (3, 6) )

    relative = p.orient.conj() * c.orient
    
    Jp[:, 3:] = -relative.matrix()
    Jc[:, 3:] = np.identity(3) 

    b = (rest.conj() * relative).flip().log()

    return Jp, Jc, b
    
