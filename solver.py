
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

        J, phi, c = constraints(dofs, **kwargs)

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

        yield dofs, mu




def calibration(world, dofs, inertia, constraints, **kwargs):

    init_dofs = np.copy( dofs ).view( dtype = Rigid3 )

    dt = kwargs.get('dt', 1e-2)
    eps = kwargs.get('eps', 1e-1)

    out_data = {}
    
    for d, mu in step(dofs, inertia, constraints, dt = dt, out_data = out_data):

        alpha = 20.0
        mt = alpha
        mr = alpha
        ms = alpha

        vt = np.zeros(3)
        vr = np.zeros(3)            
        vs = 0

        ft = np.zeros(3)
        fr = np.zeros(3)            
        fs = 0

        damping = 2.0

        targets = out_data['targets']

        for i, t in enumerate(targets):
            start = len(mu) - 3 * len(targets) + 3 * i

            desired = t['desired']                
            local = world.inv()(desired)

            f = -mu[start: start+3]

            ft += f
            fr += world.scale * np.cross( local, world.frame.orient.conj()(f) )
            fs += world.frame.orient(local).dot(f)

        net = math.sqrt( norm2(ft) + norm2(fr) + fs * fs )
        print 'calibration error:', net

        if net <= eps: break 

        vt[:] = ft / mt
        vr[:] = fr / mr 
        vs = fs / ms

        # integrate
        world.frame.center += dt * vt
        world.frame.orient = world.frame.orient * Quaternion.exp( dt * vr )
        world.scale += dt * vs

        yield init_dofs, targets
        dofs[:] = init_dofs

