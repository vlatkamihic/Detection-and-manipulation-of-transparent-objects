#!/usr/bin/env python

import moveit_commander, geometry_msgs.msg, moveit_msgs.msg, rospy, sys
import roslib
from time import time
from trac_ik_python.trac_ik import IK 
import os
import glob
import numpy as np
roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput
from cleargrasp.srv import CheckCurrentPhase

isFinished = False
isMyTurn =  False

class RobotControl:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        # definirati instancu
        self.robot = moveit_commander.RobotCommander()

        # definirati scenu
        self.scene = moveit_commander.PlanningSceneInterface()

        # stvoriti objekt koji pruza sucelje za planiranje pomaka za definiranu grupu u setup assistant-u
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # gripper publisher
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)
        
        #INVERSE KINEMATICS
        self.ik = IK('base_link', 'tool0', solve_type="Distance")



    def setPosition(self, x, y, z, qx, qy, qz, qw):

        # postavljanje orijentacije zadnjeg clanka
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 0.66259
        # pose_goal.orientation.y = 0.24732
        # pose_goal.orientation.z = -0.6532
        # pose_goal.orientation.w = -0.27055

        # postavljanje pozicije
        # pose_goal.position.x = float(x)
        # pose_goal.position.y = float(y)
        # pose_goal.position.z = float(z)
        
        # pose_goal.position.x = -0.2
        # pose_goal.position.y = 0.5
        # pose_goal.position.z = 0.5
        
        

        # planiranje trajektorije
        #self.move_group.set_pose_target(pose_goal)
        #plan = self.move_group.plan()[1]
        #return plan

        x = float(x)
        y = float(y)
        z = float(z)

        # x = 0.2
        # y = 0.4
        # z = 0.7

        #gripper faces down
        # qx = 0
        # qy = 1
        # qz = 0
        # qw = 0

        


        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)

        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]

            
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        
    

    def openGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 0
        
        
        # Delay od 3 sekunde da se saka otvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 100
        
        # Delay od 3 sekunde da se saka zatvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break
    
    def executeTrajectory(self, plan):
        self.move_group.execute(plan, wait=True)
    
    def setStartingPosition(self):
        # Euler Angles:
        # x: -140 deg
        # y: 0 deg
        # z: -180 deg
        # Quaternion
        # qx = 0
        # qy = -0.94
        # qz = -0.342
        # qw = 0
        self.setPosition(0.12, 0.044, 0.66, 0, -0.94, -0.342, 0)
        print("Position set to starting point!")

def wait_for_step_2():
    global isFinished
    global isMyTurn
    
    while(isMyTurn == False):
        current_phase = rospy.ServiceProxy('check_current_phase', CheckCurrentPhase)
        isMyTurn = (current_phase(2, isFinished)).isMyTurn

def transform(point):
    transformation_matrix = np.matrix([[0.03337431515055133,  -0.4408604975735485,    0.8715525306984567,   -0.8894262002271308],
    [-0.9758393828489554,    0.021754876424220235,  0.048372112132777374,  0.5908410028678681],
    [-0.04122245709138153,  -0.8719199412964584,   -0.43946781679706426,   0.5256948658953127],
    [ 0.0, 0.0, 0.0, 1.0]])

    P = np.append(point, 1)
    transformed = np.dot(transformation_matrix, P)
    # Transformacija točke pomoću transformacijske matrice
    xT = transformed[0, 0]
    yT = transformed[0, 1]
    zT = transformed[0, 2]

    return xT, yT, zT
    
if __name__ == '__main__':
    
    wait_for_step_2()

    # instanca upravljaca robotom
    RC = RobotControl()

    # plan=RC.setPosition(-0.4, -0.4, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.4, 0.0, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.4, 0.4, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.2, 0.55, -0.6)
    # RC.executeTrajectory(plan)


    # Tocka 1
    
    #RC.executeTrajectory(plan)

    RC.setStartingPosition()

    dir_path = "/home/robot/cleargrasp/data/captures/"
    runs = sorted(glob.glob(os.path.join(dir_path, 'exp-*')))
    prev_run_id = int(runs[-1].split('-')[-1]) if runs else 0
    
    if prev_run_id < 10:
        path = dir_path + "exp-00" + str(prev_run_id) + "/"
    elif prev_run_id > 10 and prev_run_id < 100:
        path = dir_path + "exp-0" + str(prev_run_id) + "/"
    else:
        path = dir_path + "exp-" + str(prev_run_id) + "/"
    path = dir_path + "exp-007"  + "/"
    result_folder_path = path + "result-center-and-dimensions/"

    center_file = open(result_folder_path + "center.txt", "r")
    dimensions_file = open(result_folder_path + "dimensions.txt", "r")

    # Remove the square brackets from the string
    center = (center_file.read()).strip('[]')

    # Convert the cleaned string to a NumPy array
    center = np.fromstring(center, sep=' ')

    # Centar objekta (u odnosu na bazu)

    x = float(center[0]) #m
    y = float(center[1]) #m
    z = float(center[2]) #m
    print("x: " + str(x))
    print("y: " + str(y))
    print("z: " + str(z))
    point = np.array([x, y, z])

    print("point: " + str(point))

    xT, yT, zT = transform(point)
    print(xT)
    print(yT)
    print(zT)
    
    # Dimenzije objekta

    l = float(dimensions_file.readline()) #m
    w = float(dimensions_file.readline()) #m
    h = float(dimensions_file.readline()) #m

    # Hvatanje objekta
    # Sastoji se od 3 faze:
    # 1. Poravnati alata sa središtem objekta tako da je  
    # 2. Okrenuti alat tako da se objekt može uhvatiti 
    # 3. Pomaknuti alat u središte objekta (smanjiti udaljenost s 30 cm na 5-10 cm pogodnih za hvatanje)

    # Hvatanje uspravne boce - udaljavanje po x i z osi

    RC.setPosition(x, y, z, 0, 1, 0, 0)
    

    # RC.setPosition(0.195, 0.575, 0.302, 0.721, 0.381, -0.518, 0.26)
    
    print("Sequence done!")

    rospy.spin()

    isFinished = True
    current_phase = rospy.ServiceProxy('check_current_phase', CheckCurrentPhase)
    isMyTurn = (current_phase(2, isFinished)).isMyTurn
