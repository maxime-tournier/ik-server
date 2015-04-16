import numpy as np
from tool import *

def rigid(dofs, local):
    '''map a point from local to world coordinates in a rigid frame'''
    
    res = np.zeros( (3, 6) )

    res[:, :3] = np.identity(3)

    R = dofs.orient.matrix()
    hat = Quaternion.hat(local)

    res[:, 3:] = -R.dot( hat )
    
    return res, dofs(local)


# TODO: could be group-generic
def relative(p, c, rest):
    Jp = np.zeros( (3, 6) )
    Jc = np.zeros( (3, 6) )

    relative = p.orient.inv() * c.orient
    
    Jp[:, 3:] = -relative.inv().matrix()
    Jc[:, 3:] = np.identity(3) 

    b = (rest.inv() * relative).log()

    return Jp, Jc, b
    
