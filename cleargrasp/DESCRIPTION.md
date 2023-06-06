## ClearGrasp api modifications

- depth_completion_api.py - Exposes class containing API for running depth completion using depth2depth module (external C++ executable).
depth2depth is taken from the project DeepCompletion. See: [http://deepcompletion.cs.princeton.edu/](http://deepcompletion.cs.princeton.edu/)
- inference_models.py - Class to run Inference of the Outlines Prediction Model
- utils.py - Misc functions like functions for reading and saving EXR images using OpenEXR, saving pointclouds, etc.

These scripts are used in live_demo.py file!

## ClearGrasp live demo modifications

- live_demo_ws - ROS workspace for capturing images from Lidar camera and saving cleargrasp network output
- config - configuration file needed for running live_demo.py file

### Running the programm manually

1. Clone repo: https://github.com/Shreeyak/cleargrasp
2. Settup environment so you can run live demo on your local machine
3. Make changes to live_demo folder so they match this repos live_demo folder
4. Open terminal.
5. Run <code>roscore</code>
6. In another terminal start the camera: 
<code>roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true depth_width:=640 depth_height:=480 depth_fps:=15 color_width:=640 color_height:=480 color_fps:=15 align_depth:=true</code>
7. Open terminal in workspace(...cleargrasp/live_demo/live_demo_ws/) of live_demo and run these commands:
```
source devel/setup.bash
export PYTHONPATH="$PYTHONPATH:/home/robot/cleargrasp"
rosrun cleargrasp live_demo.py
```
8. Once you run the programm you have to press key 'c' to capture image and for it to be processed. 

![image](https://github.com/vlatkamihic/Detekcija-i-manipulacija-transparentnih-objekata/assets/78767436/17235897-6f3d-41c3-966f-99a6609e9211)

<p align="center">
  Live Demo - RGB, depth
</p>

   As a result you get a grid image containing images of:
      - First row:
        - input_image,
        - surface_normals_rgb,
        - outlines_rgb,
        - occlusion_weight_rgb,
        - masked_img
      - Second row:
        - orig_input_depth_rgb,
        - input_depth_rgb,
        - output_depth_rgb, 
        - mask_rgb, 
        - mask_valid_region_3d

![image](https://github.com/vlatkamihic/Detekcija-i-manipulacija-transparentnih-objekata/assets/78767436/0f0ddc08-fefb-42b6-98bd-1bed3d3e7d27)

<p align="center">
  Result image
</p>

9. For the whole process to run once again you have to press any key on any opened image. After this you can once again repeat the step number 8.

### Running the programm with launch file
