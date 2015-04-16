
from tool import *
import numpy as np

import skeleton

def integrate(dofs, x, dt):
    for gi, xi in zip(dofs, x):
        gi.center += dt * xi.linear
        gi.orient = gi.orient * Quaternion.exp( dt * xi.angular )


def step(dofs, body, joint, dt, **kwargs):

    M = skeleton.inertia(body, dofs)

    L = np.linalg.cholesky(M)

    # TODO better ?
    Linv = np.linalg.inv(L)
    
    while True:

        J, phi, C = skeleton.constraints(joint, dofs, **kwargs)

        # Minv = Linv.transpose().dot(Linv)

        LinvJT = Linv.dot(J.transpose())

        S = LinvJT.transpose().dot(LinvJT) + np.diag(C / (dt * dt))
        Sinv = np.linalg.inv(S)

        # TODO external forces
        f = Rigid3.Deriv.array(len(dofs))

        p = dt * f.flatten()
        
        Linvp = Linv.dot(p)
        b = -phi / dt - LinvJT.transpose().dot(Linvp)

        mu = Sinv.dot(b)

        x = Rigid3.Deriv.view( Linv.transpose().dot(Linvp + LinvJT.dot(mu)) )
        
        integrate(dofs, x, dt)

        yield dofs
