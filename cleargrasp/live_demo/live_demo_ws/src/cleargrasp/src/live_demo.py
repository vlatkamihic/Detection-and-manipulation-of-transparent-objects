#!/usr/bin/env python3
'''Live demo of ClearGrasp
Will predict depth for all transparent objects on images streaming from a realsense camera using our API.
'''
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
print(dir_path)

import errno
import argparse
import glob
import os
import shutil
import sys
import time

import cv2
import h5py
import numpy as np
import numpy.ma as ma
import termcolor
import yaml
from attrdict import AttrDict
from PIL import Image

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from api import utils, depth_completion_api
# from realsense import camera
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

bridge = CvBridge()


class ClearGraspNode:
    def __init__(self):
        # Load Config File
        # CONFIG_FILE_PATH = args.configFile
        CONFIG_FILE_PATH = '/home/robot/cleargrasp/live_demo/config/config.yaml'
        with open(CONFIG_FILE_PATH) as fd:
            config_yaml = yaml.safe_load(fd)
        self.config = AttrDict(config_yaml)


        self.captures_dir = '/home/robot/cleargrasp/data/captures/exp-000'
        rospy.init_node('live_demo_node')
        rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)

        # parser = argparse.ArgumentParser(
        #     description='Run live demo of depth completion on realsense camera')
        # parser.add_argument('-c', '--configFile', required=True, help='Path to config yaml file', metavar='path/to/config.yaml')
        # args = parser.parse_args()

        # Initialize Camera
        print('Running live demo of depth completion. Make sure realsense camera is streaming.\n')

        # print(camera_info_msg.K)

        # rcamera = camera.Camera()
        # camera_intrinsics = rcamera.color_intr
        # realsense_fx = camera_intrinsics[0, 0]
        # realsense_fy = camera_intrinsics[1, 1]
        # realsense_cx = camera_intrinsics[0, 2]
        # realsense_cy = camera_intrinsics[1, 2]
        # time.sleep(1)  # Give camera some time to load data

        
        # # Create directory to save captures
        runs = sorted(glob.glob(os.path.join(self.config.captures_dir, 'exp-*')))
        prev_run_id = int(runs[-1].split('-')[-1]) if runs else 0
        self.captures_dir = os.path.join(self.config.captures_dir, 'exp-{:03d}'.format(prev_run_id))
        if os.path.isdir(self.captures_dir):
            if len(os.listdir(self.captures_dir)) > 5:
                # Min 1 file always in folder: copy of config file
                self.captures_dir = os.path.join(self.config.captures_dir, 'exp-{:03d}'.format(prev_run_id + 1))
                os.makedirs(self.captures_dir)
        else:
            os.makedirs(self.captures_dir)

        # # Save a copy of config file in the logs
        shutil.copy(CONFIG_FILE_PATH, os.path.join(self.captures_dir, 'config.yaml'))

        print('Saving captured images to folder: ' + termcolor.colored('"{}"'.format(self.captures_dir), 'blue'))
        print('\n Press "c" to capture and save image, press "q" to quit\n')

        # Initialize Depth Completion API
        outputImgHeight = int(self.config.depth2depth.yres)
        outputImgWidth = int(self.config.depth2depth.xres)
        self.depthcomplete = depth_completion_api.DepthToDepthCompletion(normalsWeightsFile=self.config.normals.pathWeightsFile,
                                                                    outlinesWeightsFile=self.config.outlines.pathWeightsFile,
                                                                    masksWeightsFile=self.config.masks.pathWeightsFile,
                                                                    normalsModel=self.config.normals.model,
                                                                    outlinesModel=self.config.outlines.model,
                                                                    masksModel=self.config.masks.model,
                                                                    depth2depthExecutable=self.config.depth2depth.pathExecutable,
                                                                    outputImgHeight=outputImgHeight,
                                                                    outputImgWidth=outputImgWidth,
                                                                    fx=int(self.config.depth2depth.fx),
                                                                    fy=int(self.config.depth2depth.fy),
                                                                    cx=int(self.config.depth2depth.cx),
                                                                    cy=int(self.config.depth2depth.cy),
                                                                    filter_d=self.config.outputDepthFilter.d,
                                                                    filter_sigmaColor=self.config.outputDepthFilter.sigmaColor,
                                                                    filter_sigmaSpace=self.config.outputDepthFilter.sigmaSpace,
                                                                    maskinferenceHeight=self.config.masks.inferenceHeight,
                                                                    maskinferenceWidth=self.config.masks.inferenceWidth,
                                                                    normalsInferenceHeight=self.config.normals.inferenceHeight,
                                                                    normalsInferenceWidth=self.config.normals.inferenceWidth,
                                                                    outlinesInferenceHeight=self.config.normals.inferenceHeight,
                                                                    outlinesInferenceWidth=self.config.normals.inferenceWidth,
                                                                    min_depth=self.config.depthVisualization.minDepth,
                                                                    max_depth=self.config.depthVisualization.maxDepth,
                                                                    tmp_dir=self.captures_dir)
        self.capture_num = 0
        

        ts = message_filters.TimeSynchronizer([rgb_image_sub, depth_image_sub, info_sub], 10)
        ts.registerCallback(self.callback)
        cv2.destroyAllWindows()

        rospy.spin()


    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        self.main(rgb_msg, depth_msg, camera_info_msg)


    def main(self, rgb_msg, depth_msg, camera_info_msg):
        # Get Frame. Expected format: ColorImg -> (H, W, 3) uint8, DepthImg -> (H, W) float64
        # color_img, input_depth = rcamera.get_data()
        # input_depth = input_depth.astype(np.float32)
        rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough') # desired_encoding='passthrough')
        input_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        # rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        # rgb_image = cv2.imread('/home/robot/Downloads/real-test/d435/000000000-transparent-rgb-img.jpg')
        # input_depth = cv2.imread('/home/robot/Downloads/real-test/d435/000000000-transparent-depth-img.exr', cv2.IMREAD_ANYDEPTH)
        
        # rgb_image = cv2.imread('/home/robot/cleargrasp/eval_depth_completion/results/exp-000/input-image/000000000-rgb.png')
        # input_depth = cv2.imread('/home/robot/cleargrasp/eval_depth_completion/results/exp-000/input-depth/000000000-modified-input-depth-rgb.png', cv2.IMREAD_ANYDEPTH).astype(np.float32)
        
        # ratio = np.amax(input_depth) / 256
        # input_depth = (input_depth / ratio).astype('uint8')
        # input_depth = input_depth.astype(np.float32)

        # max_pixel = (np.asarray(input_depth)).astype(np.float32).max()
        # input_depth /= max_pixel
        # print(type(input_depth[0][0]))
        input_depth /= 1000.0
        
        
        
        # Normalize depth_image to the range 0-1
        # input_depth_origin = input_depth.copy()

       

        # Set invalid depth pixels to zero
        # input_depth[np.isnan(input_depth)] = 0.0
        # input_depth[np.isinf(input_depth)] = 0.0

        # print(max_pixel)
        color_img = rgb_image
        
        
        # examples_dir = 'Example' + str(self.capture_num)
        # path = os.path.join('/home/robot/Detekcija-i-manipulacija-transparentnih-objekata/Examples', examples_dir)
        # try:
        #         os.mkdir(path)
        # except OSError as exc:
        #     if exc.errno != errno.EEXIST:
        #         raise
        #     pass

        # # Saving images
        # cv2.imwrite(os.path.join(path , 'input_rgb.jpg'), rgb_image)
        # cv2.imwrite(os.path.join(path , 'input_depth.jpg'), input_depth)

        cv2.imshow('Live Demo', rgb_image)
        # cv2.imshow('Live Demo Depth', input_depth)
        
        

        # rospy.sleep(25.) 
        # cv2.destroyAllWindows()
        keypress = cv2.waitKey(10) & 0xFF
        print("HELLO")
        if keypress == ord('q'):
            pass
        elif keypress == ord('c'):
            print("I am here!")

            try:
                output_depth, filtered_output_depth = self.depthcomplete.depth_completion(
                    color_img,
                    input_depth,
                    inertia_weight=float(self.config.depth2depth.inertia_weight),
                    smoothness_weight=float(self.config.depth2depth.smoothness_weight),
                    tangent_weight=float(self.config.depth2depth.tangent_weight),
                    mode_modify_input_depth=self.config.modifyInputDepth.mode)
            except depth_completion_api.DepthCompletionError as e:
                print('Depth Completion Failed:\n  {}\n  ...skipping image {}'.format(e, i))
                pass
            

            color_img = self.depthcomplete.input_image
            # input_depth = self.depthcomplete.input_depth
            surface_normals = self.depthcomplete.surface_normals
            surface_normals_rgb = self.depthcomplete.surface_normals_rgb
            occlusion_weight = self.depthcomplete.occlusion_weight
            occlusion_weight_rgb = self.depthcomplete.occlusion_weight_rgb
            outlines_rgb = self.depthcomplete.outlines_rgb
            mask = self.depthcomplete.mask_predicted
            mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
            # Display Results in Window
            input_depth_mapped = utils.depth2rgb(input_depth, min_depth=self.config.depthVisualization.minDepth,
                                                max_depth=self.config.depthVisualization.maxDepth,
                                                color_mode=cv2.COLORMAP_JET, reverse_scale=True)
            output_depth_mapped = utils.depth2rgb(output_depth, min_depth=self.config.depthVisualization.minDepth,
                                                max_depth=self.config.depthVisualization.maxDepth,
                                                color_mode=cv2.COLORMAP_JET, reverse_scale=True)
            filtered_output_depth_mapped = utils.depth2rgb(filtered_output_depth,
                                                        min_depth=self.config.depthVisualization.minDepth,
                                                        max_depth=self.config.depthVisualization.maxDepth,
                                                        color_mode=cv2.COLORMAP_JET, reverse_scale=True)
            color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
            surface_normals_rgb = cv2.cvtColor(surface_normals_rgb, cv2.COLOR_RGB2BGR)
            outlines_rgb = cv2.cvtColor(outlines_rgb, cv2.COLOR_RGB2BGR)
            occlusion_weight_rgb = cv2.cvtColor(occlusion_weight_rgb, cv2.COLOR_RGB2BGR)

            grid_image1 = np.concatenate((color_img, surface_normals_rgb, outlines_rgb, occlusion_weight_rgb), 1)
            grid_image2 = np.concatenate((input_depth_mapped, output_depth_mapped, filtered_output_depth_mapped,
                                        mask_rgb), 1)
            grid_image = np.concatenate((grid_image1, grid_image2), 0)
            
            # Saving grid
            # cv2.imwrite(os.path.join(path , 'result_grid.jpg'), grid_image)

            cv2.imshow('Live Demo Capture', grid_image)
            cv2.waitKey(0)
            # Save captured data to config.captures_dir
            self.depthcomplete.store_depth_completion_outputs(self.captures_dir,
                                                         self.capture_num,
                                                         min_depth=self.config.depthVisualization.minDepth,
                                                         max_depth=self.config.depthVisualization.maxDepth)
            print('captured image {0:06d}'.format(self.capture_num))
            self.capture_num += 1



if __name__ == '__main__':

    cgn = ClearGraspNode()

