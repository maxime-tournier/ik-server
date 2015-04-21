

from tool import *
import json


import pyqglviewer
import solver
import gui

import constraint
import target
import threading

import kinect
import json
import math
import time
import skeleton

payload = [None]

flags = {}

def fetch():

    while 'exit' not in flags:
        for data in kinect.data(ip = sys.argv[1],
                                port = 9000):
            if data:
                # print 'got frame yo'
                payload[0] = data


        
thread = threading.Thread(target = fetch)
thread.daemon = True

try:
    thread.start()

    try:
        definition = skeleton.load( 'skeleton.json')
    except Exception, e:
        print e
        print 'error loading skeleton, did you create one ?'
        sys.exit(1)

     
    body, joint = skeleton.create( definition )
    dofs = Rigid3.array( len(body) )
    

    def end_point(name, **kwargs):
        return target.chunk(name, -basis(1, 3) * body[name].dim / 2, **kwargs)


    def start_point(name, **kwargs):
        return target.chunk(name, basis(1, 3) * body[name].dim / 2, **kwargs)
    
    
    occulus = {
        'LeftHand': end_point('forearm_left'),
        'LeftElbow': end_point('arm_left'),    
        'RightHand': end_point('forearm_right'),
        'RightElbow': end_point('arm_right'),

        'Hips': end_point('trunk'),
        'Head': start_point('head'),
        'Neck': end_point('head'),
    }

    kinect = {
        'HandLeft': end_point('forearm_left'),
        'HandRight': end_point('forearm_right'),


        'HipLeft': start_point('femur_left'),
        'HipRight': start_point('femur_right'),    

        'AnkleLeft': end_point('tibia_left'),
        'AnkleRight': end_point('tibia_right'),

        'Head': start_point('head'),

        'KneeLeft': start_point('tibia_left'),
        'KneeRight': start_point('tibia_right'),

        'ElbowLeft': end_point('arm_left'),    
        'ElbowRight': end_point('arm_right'),

        'ShoulderLeft': start_point('arm_left'),    
        'ShoulderRight': start_point('arm_right'),
        
        'Neck': end_point('head'),
    }



    def desired(v): return np.array(map(float, [ v['x'],
                                             v['y'],
                                             v['z'] ]) )
    

    def targets( data, adaptor, world ):
        '''adaptor maps target names to chunks'''

        res = []
        if not data: return res

        for d in data:
            pos = d.get('position', None)

            if pos:
                adapt = adaptor.get(pos)
                if adapt:
                    p = world( desired(d) )
                    
                    res.append( adapt( p ) )

        return res

    adaptor = kinect


    class World:

        def __init__(self):
            self.frame = Rigid3()
            self.scale = 1.0

        def __call__(self, x):
            return self.frame( self.scale * x )


        def dump(self):
            return self.__dict__

        def load(self, x):
            self.scale = x['scale']
            self.frame.load(x['frame'])
            
        def inv(self):
            res = World()
            res.frame = self.frame.inv()
            res.frame.center /= self.scale
            res.scale = 1.0 / self.scale
            return res


    world = World()

   

    with pyqglviewer.app():    

        w = gui.Viewer()

        w.body = body
        w.joint = joint


        def inertia(dofs):
            return skeleton.inertia(body, dofs)


        def skeleton_constraints(dofs, **kwargs):
            return skeleton.constraints(joint, dofs, compliance = 1e-8)


        def target_constraints(dofs, **kwargs):
            out_data = kwargs.get('out_data', {})
            t = targets( payload[0], adaptor, world )
            out_data['targets'] = t
            return target.constraints(t, body, dofs, compliance = 1e-4)


        def build( *args ):
            def res(dofs, **kwargs):
                full = sum( (f(dofs, **kwargs) for f in args), [] )
                return constraint.merge( full )
            return res

        def source(**kwargs):

            dofs = kwargs.get('dofs')
            
            dt = kwargs.get('dt', 1e-1)
            eps = kwargs.get('eps', 1e-2)

            # don't enable targets just yet to get skeleton in shape
            print 'assembling skeleton...'
            for d, mu in solver.step(dofs, inertia,
                                     build(skeleton_constraints),
                                     dt = dt):
                yield dofs, None
                
                error = norm(mu) / dt
                print error
                if error <= eps: break

           
            def calibrate():
                print 'calibrating...'
                print 'please stand still'

                time.sleep(2)
                
                # wait until we have a skeleton frame
                while True:
                    init_data = payload[0]
                    if init_data: break
                    sys.stdout.write('.'); sys.stdout.flush()
                    time.sleep(1)
                print 'ok'

                # where to get targets from during calibration
                def get_data():
                    # return payload[0]
                    return init_data

                # calibration target constraints
                def init_target_constraints(dofs, **kwargs):

                    out_data = kwargs.get('out_data', {})
                    t = targets( get_data(), adaptor, world )
                    out_data['targets'] = t

                    return target.constraints(t, body, dofs, compliance = 1e-4)
                
                h = 1e-2
                eps = 1e-2

                for d, t in solver.calibration(world, dofs, inertia,
                                                 build(skeleton_constraints,
                                                       init_target_constraints),
                                                 dt = h,
                                                 eps = eps):
                    w.targets = t
                    yield d, t


            try:
                calibration_filename = 'calibration.json'
                with open(calibration_filename) as f:
                    world.load( json.loads(f.read()) )
            except:
                for d, t in calibrate():
                    yield d, t
            with open(calibration_filename, 'w') as f:
                f.write( json.dumps(world, default = lambda o: o.dump()))
                print 'wrote', calibration_filename
                    
            # then regular ik
            print 'ik started'
            out_data = {}
            
            for d, mu in solver.step(dofs, inertia,
                                     build(skeleton_constraints,
                                           target_constraints),
                                     dt = dt,
                                     out_data = out_data):
                t = out_data['targets']
                yield d, t

        w.source = source(dt = 1e-1, dofs = dofs)

        w.setSceneRadius( 10 )
        w.show()
        w.startAnimation()



except Exception, e:
    print e
    raise
        
finally:
    raise Exception()
    print 'waiting for network thread to exit'
    flags['exit'] = True
    thread.join()
    thread.stop()


    # TODO:

    # limits
    # - network
    # - external forces

