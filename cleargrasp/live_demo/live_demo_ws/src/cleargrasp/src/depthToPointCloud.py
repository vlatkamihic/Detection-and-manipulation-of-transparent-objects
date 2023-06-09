import open3d as o3d
import cv2
import numpy as np
import copy

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

    point_cloud = o3d.io.read_point_cloud(r"/home/robot/cleargrasp/data/captures/exp-010/output-point-cloud/000000000-output-pointcloud.ply")
    print(type(point_cloud))
    _, ind = point_cloud.remove_statistical_outlier(20, 2.0)
    display_inlier_outlier(point_cloud, ind)
    point_cloud = point_cloud.select_by_index(ind)
    bounding_box = point_cloud.get_oriented_bounding_box()

    o3d.visualization.draw_geometries([bounding_box, point_cloud])
    