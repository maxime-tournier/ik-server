
import OpenGL
from OpenGL.GL import *

from tool import *

import pyqglviewer

class Viewer(pyqglviewer.QGLViewer):

    def __init__(self):
        pyqglviewer.QGLViewer.__init__(self)

        self.dofs = None
        self.targets = None
    
    def animate(self):
        try:
            self.dofs, self.targets = next(self.source)
        except StopIteration:
            import sys
            sys.exit(0)

    def draw(self):

        if self.dofs is None: return
        
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
        


        if self.targets:
            glColor(1, 0.5, 0)
            glBegin(GL_LINES)

            for t in self.targets:
                bi = self.body[t['body']].index
                b = self.dofs[ bi ]

                current = b( t['local'] )
                desired = t['desired']
                
                glVertex(current)
                glVertex(desired)
            
            glEnd()


                
        glEnable(GL_LIGHTING)

        
    def keyPressEvent(self, e):
        if e.text() == 'r':
            for i in self.dofs:
                i.center = [0, 0, 0]
                i.orient = Quaternion()

                
