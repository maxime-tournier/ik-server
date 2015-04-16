

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

with pyqglviewer.app():    

    w = gui.Viewer()

    w.body = body
    w.joint = joint

    w.source = solver.step(dofs, body, joint, dt = 1e-1, compliance = 0)

    w.show()
    w.startAnimation()

# TODO:

# - network
# - external forces

