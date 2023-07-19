# Detection-and-manipulation-of-transparent-objects

Determining the three-dimensional shape and position of transparent objects for the purpose of robotic manipulation

To clone this repo:
<code>git clone https://PersonalAccessToken@github.com/vlatkamihic/Detekcija-i-manipulacija-transparentnih-objekata</code>

### [ClearGrasp](https://github.com/Shreeyak/cleargrasp) neural network-based approach for 3D object detection and segmentation from RGB-D images

The network is used for 3D transparent object detection which will be used for their manipulation.
ClearGrasp network model is updated and changed so it works with RealSense Lidar L515 camera and ROS.
Changes made to the ClearGrasp repo are placed into to [cleargrasp](https://github.com/vlatkamihic/Detekcija-i-manipulacija-transparentnih-objekata/tree/main/cleargrasp/live_demo) folder.

TO DO:
- dodati obradu rezultata detekcije i manipulaciju u .launch file
- kalibrirati kameru
- dovrsiti skriptu za manipulaciju

RUNNING PROGRAMM:
```
cd cleargrasp/live_demo/live_demo_ws/
source devel/setup.bash
export PYTHONPATH="$PYTHONPATH:/home/robot/cleargrasp"
source /home/robot/catkin_ws/devel/setup.bash --extend
roslaunch cleargrasp detect.launch
```
