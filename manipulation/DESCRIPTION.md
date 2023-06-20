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

- there are 3 steps needed in order to position our tool so it can grasp the object:
1. Align the tool with the center of the object at a distance of ca 30cm
2. Rotate the tool so the object can be grasped
3. Move the tool to the object center (reduce the distance from 30cm to 5-10cm)

- after these steps are done we can close the gripper and move the object

