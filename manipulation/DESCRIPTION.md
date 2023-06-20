## Robot Manipulation of transparent objects

- manipulation_ws - workspace used for object manipulation

## Setting robot for object detection

- in order to detect objects we need to set our robotic manipulator in some position from which the camera attached to the tool can see the scene
- this is a hardcoded position implemented in method <code>setStartingPosition()</code> whit the next coordinates and quaternion:
    - x = 0.12,
    - y = 0.044,
    - z = 0.66,
    - qx = 0,
    - qy = -0.94,
    - qz = -0.342,
    - qw = 0

## Setting robot for object manipulation


