#! /bin/python3

import rospy
import readline
import traceback

import numpy as np

from CameraReader import CameraReader
from ArucoDetector import ArucoDetector
from RobotComms import RobotComms

from CameraCommands import CameraCommands
from RobotCommands import RobotCommands
from GripperCommands import GripperCommands
from ImageCommands import ImageCommands
from MarkerCommands import MarkerCommands

import izracun


def printHelp():
    print("calibration program commands are:")
    print("camera - commands used to show current image from camera")
    print("robot - commands used to move robot and read position from robot")
    print("gripper - commands use to control gripper")
    print("image - commands used to capture images for calibration")
    print("marker - commands used to capture marker positions for calibration")
    print("calculate - perform calculation")
    print("help - prints this message")

if __name__ == '__main__':
    print("Running robot-camera calibration program")
    print("use commands camera, robot and gripper to work with specified component")
    print("use commands image and marker to capture values for calculation")
    print("after values are in memory use calculate command to perform calcluation")
    print("use quit or q to exit program")
    
    np.set_printoptions(precision=20, linewidth=100)
    
    rospy.init_node("calibration_program", anonymous=True)

    camerareader = CameraReader()
    arucoDetector = ArucoDetector()
    robotComms = RobotComms()

    cameraCommands = CameraCommands(camerareader, arucoDetector, robotComms)
    robotCommands = RobotCommands(robotComms)
    gripperCommands = GripperCommands()
    imageCommands = ImageCommands(camerareader, arucoDetector, robotComms)
    markerCommands = MarkerCommands(robotComms)
    
    while True:
        try:
            text = input(">>")
            input_split = text.split(' ', 1)
            command = input_split[0]
            rest = ""
            if(len(input_split) == 2):
                rest = input_split[1]
            if(command == "quit" or command == "q"):
                break
            elif(command == "camera"):
                cameraCommands.eval(rest)
            elif(command == "robot"):
                robotCommands.eval(rest)
            elif(command == "gripper"):
                gripperCommands.eval(rest)
            elif(command == "image"):
                imageCommands.eval(rest)
            elif(command == "marker"):
                markerCommands.eval(rest)
            elif(command == "calculate"):
                tool_E = np.array([0.0, 0.0, 0.25, 1.0])
                #calculate(tool_E, markerCommands.markers, imageCommands.images)
                num_iterations = 50
                E_T_C, tool_E = izracun.calculate2(tool_E, markerCommands.markers, imageCommands.images, num_iterations)
                #E_T_C = np.array([
                #    [rotation[0][0], rotation[0][1], rotation[0][2], pose.position.x],
                #    [rotation[1][0], rotation[1][1], rotation[1][2], pose.position.y],
                #    [rotation[2][0], rotation[2][1], rotation[2][2], pose.position.z],
                #    [0             , 0             , 0             , 1]
                #])
                #tool_E = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
                print("E_T_C")
                print(E_T_C)
                print("tool_E")
                print(tool_E)
                imageCommands.E_T_C = E_T_C
                imageCommands.tool_E = tool_E
                imageCommands.markers = markerCommands.markers
                cameraCommands.E_T_C = E_T_C
                cameraCommands.tool_E = tool_E
            elif(command == "help"):
                printHelp()
            else:
                print("Unknown command!")
                printHelp()
        except Exception as error:
            traceback.print_exc()
