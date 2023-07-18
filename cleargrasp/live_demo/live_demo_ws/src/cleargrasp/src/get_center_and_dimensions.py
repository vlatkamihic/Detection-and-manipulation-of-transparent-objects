#!/usr/bin/env python3
import sys
import os
import open3d as o3d
import numpy as np
import pyransac3d as pyrsc
import cv2
import glob
from PIL import Image
import rospy
from cleargrasp.srv import CheckCurrentPhase

isFinished = False
isMyTurn =  False


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

def wait_for_live_demo():
    global isFinished
    global isMyTurn
    
    while(isMyTurn == False):
        current_phase = rospy.ServiceProxy('check_current_phase', CheckCurrentPhase)
        isMyTurn = (current_phase(1, isFinished)).isMyTurn

def get_object_center_and_dimensions(path):

    point_cloud_og = o3d.io.read_point_cloud(path + "output-point-cloud/output-point-cloud-object.ply")

    surface_ptc = o3d.io.read_point_cloud(path + "input-point-cloud/000000000-input-pointcloud.ply")

    # Apply plane fitting using RANSAC
    plane_model, inliers = surface_ptc.segment_plane(distance_threshold=0.01,
                                                    ransac_n=3,
                                                    num_iterations=1000)
    normal = plane_model[0:3]
    yaxis = np.array([0, 1, 0])
    unit_normal= normal / np.linalg.norm(normal)
    unit_yaxis = yaxis / np.linalg.norm(yaxis)
    dot_product = np.dot(unit_normal, unit_yaxis)
    angle = -np.arccos(dot_product)

    rotation_matrix = np.matrix([[1, 0, 0],
                   [0, np.cos(angle),-np.sin(angle)],
                   [0, np.sin(angle), np.cos(angle)]])

    # point_cloud = o3d.geometry.PointCloud(point_cloud_og)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh = mesh.rotate(rotation_matrix, mesh.get_center())
    # Extract the dominant surface points
    dominant_surface = surface_ptc.select_by_index(inliers)

    # Get the x, y, and z coordinates of the dominant surface
    coordinates = np.asarray(dominant_surface.points)
    # Create an Open3D point cloud from the coordinates
    dominant_surface_cloud = o3d.geometry.PointCloud()
    dominant_surface_cloud.points = o3d.utility.Vector3dVector(coordinates)


    _, ind = point_cloud_og.remove_statistical_outlier(30, 7.0)
    display_inlier_outlier(point_cloud_og, ind)
    point_cloud = point_cloud_og.select_by_index(ind)
    point_cloud_2 = o3d.geometry.PointCloud(point_cloud)
    point_cloud = point_cloud.rotate(rotation_matrix, mesh.get_center())
    dominant_surface = dominant_surface.rotate(rotation_matrix, mesh.get_center())
    o3d.io.write_point_cloud(path + "output-point-cloud/copy_of_fragment.ply", point_cloud)
    bounding_box = point_cloud.get_axis_aligned_bounding_box()


    # Accessing the center of the bounding box
    center = bounding_box.get_center()
    # print(np.linalg.norm(center))
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01, resolution=20)
    mesh_sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.01, resolution=20)
    mesh_sphere.translate(center)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    # Accessing the dimensions of the bounding box
    length, width, height = bounding_box.get_extent()

    print("Bounding box center:", center)
    print("Bounding box dimensions (L x W x H):", length, width, height)

    o3d.visualization.draw_geometries([bounding_box, point_cloud, point_cloud_2, dominant_surface, mesh, mesh_sphere, mesh_frame])

    return center, length, width, height

if __name__ == '__main__':

    dir_path = "/home/robot/cleargrasp/data/captures/"

    rospy.init_node('get_center_and_dimensions')
    rospy.wait_for_service('check_current_phase')
    wait_for_live_demo()

    runs = sorted(glob.glob(os.path.join(dir_path, 'exp-*')))
    prev_run_id = int(runs[-1].split('-')[-1]) if runs else 0
    
    path = dir_path + "exp-0" + str(prev_run_id) + "/"

    center, length, width, height = get_object_center_and_dimensions(path)

    result_folder_path = path + "result-center-and-dimensions/"
    os.makedirs(result_folder_path)

    center_file = open(result_folder_path + "center.txt", "w")
    dimensions_file = open(result_folder_path + "dimensions.txt", "w")

    center_file.write(str(center))
    dimensions_file.write(str(length) + "\n" + str(width) + "\n" + str(height))

    isFinished = True
    current_phase = rospy.ServiceProxy('check_current_phase', CheckCurrentPhase)
    isMyTurn = (current_phase(1, isFinished)).isMyTurn
