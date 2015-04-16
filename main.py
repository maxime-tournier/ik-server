

from tool import *
import json


try:
    definition = json.loads( open('skeleton.json').read() )
except:
    import sys
    print 'error loading skeleton, generate one first'
    sys.exit(0)


import skeleton


body, joint = skeleton.create( definition )
dofs = Rigid3.array( len(body) )

import pyqglviewer
import solver
import gui

import constraint
import target

with pyqglviewer.app():    

    w = gui.Viewer()

    w.body = body
    w.joint = joint


    def inertia(dofs):
        return skeleton.inertia(body, dofs)

    def constraints(dofs):

        full = (skeleton.constraints(joint, dofs, compliance = 0) +
                target.constraints(target.definition, body, dofs )
        )
        
        return constraint.merge( full )


    w.source = solver.step(dofs, inertia, constraints, dt = 1e-1)

    w.setSceneRadius( 10 )
    w.show()
    w.startAnimation()

# TODO:

# limits
# - network
# - external forces

