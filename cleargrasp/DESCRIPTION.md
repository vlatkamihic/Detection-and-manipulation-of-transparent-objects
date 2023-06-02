## ClearGrasp live demo modifications

- live_demo_ws - ROS workspace for capturing images from Lidar camera and saving cleargrasp network output
- config - configuration file needed for running live_demo.py file

### Running the programm

1. Clone repo: https://github.com/Shreeyak/cleargrasp
2. Settup environment so you can run live demo on your local machine
3. Make changes to live_demo folder so they match this repos live_demo folder
4. Open terminal.
5. Run <code>roscore</code>
6. In another terminal start the camera: 
<code>roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 align_depth:=true</code>
7. Open terminal in src folder(...cleargrasp/live_demo/live_demo_ws/src/cleargrasp/src) of live_demo and run these commands:
```
source devel/setup.bash
export PYTHONPATH="$PYTHONPATH:/home/robot/cleargrasp"
rosrun cleargrasp live_demo.py
```
8. 