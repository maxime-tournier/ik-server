# ik-server

this is a simple ik solver and server that fits a kinematic model to
ik targets coming from the network.

it is meant to be a filtering bridge between kinectv2 and
occulus/unity demos since the kinectv2 sdk orientations are really
noisy

# usage

1. Generate a skeleton definition:

```sh
$ python model.py > skeleton.json
```

you may tweak either `model.py` or the resulting file for your needs.

2. calibrate the scene

```sh
$ python calibrate.py <ip>
```

where `<ip>` is the target source server, with default port 9000.

the mapping between source targets and model anchors is performed in
`target.py`, so tweak it as needed.

the default skeleton definition/target mapping works for JSON targets
such as:

```js
[{"y": -0.4968303, "position": "SpineBase", "z": 1.70303965, "x": -0.0443293042}, {"y": -0.178456277, "position": "SpineMid", "z": 1.6178565, "x": -0.0205915235}, {"y": 0.130871385, "position": "Neck", "z": 1.51529324, "x": 0.00289628538}, {"y": 0.262803853, "position": "Head", "z": 1.45563316, "x": 0.0410893}, {"y": 0.0461130179, "position": "ShoulderLeft", "z": 1.51446247, "x": -0.164839}, {"y": -0.230310664, "position": "ElbowLeft", "z": 1.54731178, "x": -0.255745471}, {"y": -0.4476466, "position": "WristLeft", "z": 1.494228, "x": -0.277606815}, {"y": -0.499036223, "position": "HandLeft", "z": 1.4880296, "x": -0.2682711}, {"y": 0.0364198051, "position": "ShoulderRight", "z": 1.56479108, "x": 0.166294485}, {"y": -0.2141666, "position": "ElbowRight", "z": 1.63231707, "x": 0.193412572}, {"y": -0.448038846, "position": "WristRight", "z": 1.50331831, "x": 0.16579701}, {"y": -0.490654945, "position": "HandRight", "z": 1.50715077, "x": 0.135519311}, {"y": -0.480613261, "position": "HipLeft", "z": 1.65557861, "x": -0.127024859}, {"y": -0.67532146, "position": "KneeLeft", "z": 1.30374789, "x": -0.141852066}, {"y": -1.00822043, "position": "AnkleLeft", "z": 1.18105888, "x": -0.141313359}, {"y": -1.03534722, "position": "FootLeft", "z": 1.04560268, "x": -0.123352721}, {"y": -0.490618467, "position": "HipRight", "z": 1.67325032, "x": 0.0403243974}, {"y": -0.7525866, "position": "KneeRight", "z": 1.29755676, "x": 0.124624327}, {"y": -0.8104322, "position": "AnkleRight", "z": 1.67638087, "x": 0.009866156}, {"y": -0.789838, "position": "FootRight", "z": 1.50924647, "x": 0.0458623022}, {"y": 0.0550712757, "position": "SpineShoulder", "z": 1.54378343, "x": -0.00290112034}, {"y": -0.566544235, "position": "HandTipLeft", "z": 1.47139823, "x": -0.270116746}, {"y": -0.4969024, "position": "ThumbLeft", "z": 1.4606638, "x": -0.23709558}, {"y": -0.545468, "position": "HandTipRight", "z": 1.49029338, "x": 0.111190423}, {"y": -0.490386277, "position": "ThumbRight", "z": 1.54815781, "x": 0.1521317}]
```

(see my other repository [kinect-server]() for the corresponding
kinectv2 target source server)

3. start the server

(this part is highly specific to my needs. if you need something
generic please wait until I make this part customizable)

```sh
$ python server.py <ip>
```

this will connect to a target server `<ip` on port `9000`, and serve
joint angles for the occulus demo on port `9000` as well.



