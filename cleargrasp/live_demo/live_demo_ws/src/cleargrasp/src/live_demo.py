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
from cleargrasp.srv import CheckCurrentPhase, CheckCurrentPhaseResponse


os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"

bridge = CvBridge()
isFinished = False
isMyTurn =  False

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


        # Initialize Camera
        print('Running live demo of depth completion. Make sure realsense camera is streaming.\n')

        
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
        global isFinished
        global isMyTurn
        rospy.wait_for_service('check_current_phase')
        current_phase = rospy.ServiceProxy('check_current_phase', CheckCurrentPhase)
        isMyTurn = (current_phase(0, isFinished)).isMyTurn
        # print("My turn: "+str(isMyTurn)+" , finished: "+str(isFinished)+", data type: "+str(type(isMyTurn))+", "+str(type(isFinished)))
        if(isMyTurn == True and isFinished == False):
            print("Usao u main")
            self.main(rgb_msg, depth_msg, camera_info_msg)
        


    def main(self, rgb_msg, depth_msg, camera_info_msg):
        global isFinished

        rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough') # desired_encoding='passthrough')
        input_depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)

        input_depth /= 1000.0
        color_img = rgb_image
        print("Trebam prikazati sliku")
        cv2.imshow('Live Demo', rgb_image)

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
            input_depth = self.depthcomplete.input_depth
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
            isFinished = True



if __name__ == '__main__':

    cgn = ClearGraspNode()

