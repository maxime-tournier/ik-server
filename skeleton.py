

import numpy as np

from tool import *
import json

try:
    skeleton = json.loads( open('skeleton.json').read() )
except:
    import sus
    print 'error loading skeleton, generate one first'
    sys.exit(0)
    
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



body, joint = create( skeleton )
    
dofs = Rigid3.array( len(body) )

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


def attach_constraints(joint, dofs, **kwargs):
    
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

        Jp = rigid_mapping(dofs[pi], j.parent.frame )
        Jc = rigid_mapping(dofs[ci], j.child.frame )

        matrix[ 3*i: 3*i + 3, 6 * pi: 6 * pi + 6 ] = Jp
        matrix[ 3*i: 3*i + 3, 6 * ci: 6 * ci + 6 ] = -Jc

        value[ 3*i: 3*i + 3] = dofs[pi](j.parent.frame) - dofs[ci](j.child.frame)

        compliance[ 3*i: 3*i + 3] = c * np.ones( 3 )
        
    return matrix, value, compliance


def rest_constraints(joint, dofs, **kwargs):

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
        
        Jp, Jc, bi = relative_mapping(dofs[pi], dofs[ci], j.rest)

        J[ 3*i: 3*i + 3, 6 * pi: 6 * pi + 6 ] = Jp
        J[ 3*i: 3*i + 3, 6 * ci: 6 * ci + 6 ] = Jc

        b[ 3*i: 3*i + 3 ] = bi

        C[ 3*i: 3*i + 3] = j.compliance * np.ones( 3 )
        i += 1
        
    return J, b, C

def constraints(joint, dofs, **kwargs):

    sub = (attach_constraints, rest_constraints)
    chunks = [c(joint, dofs, **kwargs) for c in sub]
    
    J = np.vstack( (c[0] for c in chunks if c ) )
    b = np.concatenate( tuple(c[1] for c in chunks if c ) )
    C = np.concatenate( tuple(c[2] for c in chunks if c ) )

    return J, b, C
        

def rigid_mapping(dofs, local):
    res = np.zeros( (3, 6) )

    res[:, :3] = np.identity(3)

    R = dofs.orient.matrix()
    hat = Quaternion.hat(local)

    res[:, 3:] = -R.dot( hat )
    
    return res

# TODO: could be group-generic
def relative_mapping(p, c, rest):
    Jp = np.zeros( (3, 6) )
    Jc = np.zeros( (3, 6) )

    relative = p.orient.conj() * c.orient
    
    Jp[:, 3:] = -relative.matrix()
    Jc[:, 3:] = np.identity(3) 

    b = (rest.conj() * relative).flip().log()

    return Jp, Jc, b
    
def integrate(dofs, x, dt):
    for gi, xi in zip(dofs, x):
        gi.center += dt * xi.linear
        gi.orient = gi.orient * Quaternion.exp( dt * xi.angular )


def step(dt, **kwargs):

    M = inertia(body, dofs)

    L = np.linalg.cholesky(M)

    # TODO better ?
    Linv = np.linalg.inv(L)
    
    while True:

        J, phi, C = constraints(joint, dofs, **kwargs)

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


import OpenGL
from OpenGL.GL import *


import pyqglviewer
class Viewer(pyqglviewer.QGLViewer):

    def animate(self):
        try:
            self.dofs = next(self.source)
        except StopIteration:
            import sys
            sys.exit(0)

    def draw(self):

        if 'dofs' not in self.__dict__: return
        
        glDisable(GL_LIGHTING)

        glColor(1, 1, 1)
        glBegin(GL_LINES)
        
        for b in self.body.itervalues():
            g = self.dofs[b.index]
            c = g.center
            
            for i in xrange(3):
                ei = basis(i, 3)
                vi = g.orient( ei * b.dim / 2 )
                
                glColor(*ei)
                glVertex(c - vi)
                glVertex(c + vi)
            
        glEnd()

        glColor(1, 1, 1)
        glPointSize(4)
        glBegin(GL_POINTS)
        
        for j in self.joint.itervalues():
            p = j.parent.body.index
            c = j.child.body.index
            
            glVertex(self.dofs[p].center + self.dofs[p].orient(j.parent.frame) )
            glVertex(self.dofs[c].center + self.dofs[c].orient(j.child.frame) )
            
        glEnd()
        
        glEnable(GL_LIGHTING)


    def keyPressEvent(self, e):
        if e.text() == 'r':
            for i in self.dofs:
                i.center = [0, 0, 0]
                i.orient = Quaternion()
import json
json.dumps( skeleton )

with pyqglviewer.app():    

    w = Viewer()

    w.body = body
    w.joint = joint

    w.source = step(1e-1, compliance = 0)

    w.show()
    w.startAnimation()


    
# TODO:

# - JSON
# - network
# - external forces

