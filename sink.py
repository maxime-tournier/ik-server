
import kinect
import sys
import json

while True:
    for i, data in enumerate(kinect.data(ip = sys.argv[1])):
        print json.dumps(data)
        

    
