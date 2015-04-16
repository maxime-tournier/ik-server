
import OpenGL
from OpenGL.GL import *

from tool import *

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

                
