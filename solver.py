
from tool import *
import numpy as np

import skeleton

def integrate(dofs, x, dt):
    for gi, xi in zip(dofs, x):
        gi.center += dt * xi.linear
        gi.orient = gi.orient * Quaternion.exp( dt * xi.angular )

import scipy as sp
from scipy import linalg

def step(dofs, inertia, constraints, **kwargs):

    dt = kwargs.get('dt', 1)
    M = inertia(dofs)
    
    L = np.linalg.cholesky(M)
    LT = L.transpose()
    
    # TODO better ?
    # Linv = np.linalg.inv(L)
    class Matrix:

        def dot(self, other):
            return sp.linalg.solve_triangular(L, other, lower = True, check_finite = False)


    class MatrixT:

        def dot(self, other):
            return sp.linalg.solve_triangular(LT, other, lower = False, check_finite = False)
        
    Linv = Matrix()
    LinvT = MatrixT()
    
    while True:

        J, phi, c = constraints(dofs)

        # Minv = Linv.transpose().dot(Linv)

        LinvJT = Linv.dot(J.transpose())

        S = LinvJT.transpose().dot(LinvJT) + np.diag(c / (dt * dt))

        # TODO arg
        Sinv = np.linalg.inv(S)

        # TODO external forces
        f = Rigid3.Deriv.array(len(dofs))

        p = dt * f.flatten()
        
        Linvp = Linv.dot(p)
        b = -phi / dt - LinvJT.transpose().dot(Linvp)

        mu = Sinv.dot(b)

        x = Rigid3.Deriv.view( LinvT.dot(Linvp + LinvJT.dot(mu)) )
        
        integrate(dofs, x, dt)

        yield dofs
