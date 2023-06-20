import open3d as o3d
import numpy as np
import pyransac3d as pyrsc
import cv2
from PIL import Image

FX_DEPTH = 597.9033203125
FY_DEPTH = 598.47998046875
CX_DEPTH = 323.8436584472656
CY_DEPTH = 236.32774353027344

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

if __name__ == '__main__':

    path = "/home/robot/cleargrasp/data/captures/exp-025/"

    pil_image = Image.open(path+"output-depth/000000000-output-depth-rgb.png")
    np_array = np.array(pil_image)
    open3d_image = o3d.geometry.Image(np_array.astype(np.uint8))
    # Convert the Open3D image to a compatible format
    open3d_image = o3d.create_compatible(o3d.geometry.RGBDImage)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()

    width = 640
    height = 480
    fx = 597.9033203125
    fy = 598.47998046875
    cx = 323.8436584472656
    cy = 236.32774353027344

    intrinsic.set_intrinsics(width, height, fx, fy, cx, cy)
    point_cloud = o3d.geometry.PointCloud.create_from_depth_image(open3d_image, intrinsic, depth_scale=1000.0, depth_trunc=1000.0, stride=1)

    o3d.io.write_point_cloud(path+"my_point_cloud.ply", point_cloud)

    # point_cloud_og = o3d.io.read_point_cloud(path + "output-point-cloud/output-point-cloud-object.ply")
    # # point_cloud_og = o3d.io.read_point_cloud(path + "output-point-cloud/copy_of_fragment.ply")
    
    # _, ind = point_cloud_og.remove_statistical_outlier(2, 7.0)
    # display_inlier_outlier(point_cloud_og, ind)
    # point_cloud_og = point_cloud_og.select_by_index(ind)
    # o3d.io.write_point_cloud(path + "output-point-cloud/copy_of_fragment.ply", point_cloud_og)
    # bounding_box_og = point_cloud_og.get_axis_aligned_bounding_box()

    # surface_ptc = o3d.io.read_point_cloud(path + "input-point-cloud/000000000-input-pointcloud.ply")

    # # Apply plane fitting using RANSAC
    # plane_model, inliers = surface_ptc.segment_plane(distance_threshold=0.01,
    #                                                 ransac_n=3,
    #                                                 num_iterations=1000)
    # normal = plane_model[0:3]
    # yaxis = np.array([0, 1, 0])
    # unit_normal= normal / np.linalg.norm(normal)
    # unit_yaxis = yaxis / np.linalg.norm(yaxis)
    # dot_product = np.dot(unit_normal, unit_yaxis)
    # angle = np.arccos(dot_product)
    # # angle = 1.57079632679
    # print(angle)
    # rotation_matrix = np.matrix([[1, 0, 0],
    #                [0, np.cos(angle),-np.sin(angle)],
    #                [0, np.sin(angle), np.cos(angle)]])

    # # point_cloud = o3d.geometry.PointCloud(point_cloud_og)
    # # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # # mesh = mesh.rotate(rotation_matrix, mesh.get_center())
    # # Extract the dominant surface points
    # dominant_surface = surface_ptc.select_by_index(inliers)

    # # # Get the x, y, and z coordinates of the dominant surface
    # # coordinates = np.asarray(dominant_surface.points)
    # # # Create an Open3D point cloud from the coordinates
    # # dominant_surface_cloud = o3d.geometry.PointCloud()
    # # dominant_surface_cloud.points = o3d.utility.Vector3dVector(coordinates)
    # # # Visualize the dominant surface
    # # o3d.visualization.draw_geometries([surface_ptc])
    # # o3d.visualization.draw_geometries([dominant_surface_cloud])

    # # _, ind = point_cloud.remove_statistical_outlier(2, 7.0)
    # # display_inlier_outlier(point_cloud, ind)
    # # point_cloud = point_cloud.select_by_index(ind)
    # bounding_box = point_cloud_og.get_oriented_bounding_box()


    # # Accessing the center of the bounding box
    # center = bounding_box.get_center()
    # print(np.linalg.norm(center))
    # mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01, resolution=20)
    # mesh_sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.01, resolution=20)
    # mesh_sphere.translate(center)
    # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    # # Accessing the dimensions of the bounding box
    # length, width, height = bounding_box.extent

    # print("Bounding box center:", center)
    # print("Bounding box dimensions (L x W x H):", length, width, height)

    # length, width, height = bounding_box_og.get_extent()
    # center = bounding_box_og.get_center()

    # print("Bounding box OG center:", center)
    # print("Bounding box OG dimensions (L x W x H):", length, width, height)

    # o3d.visualization.draw_geometries([bounding_box, bounding_box_og, point_cloud_og, surface_ptc, mesh_sphere, mesh_sphere2, mesh_frame])



    

    