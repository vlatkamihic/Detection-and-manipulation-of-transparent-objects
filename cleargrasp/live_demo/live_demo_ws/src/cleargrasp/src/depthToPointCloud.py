import open3d as o3d
import cv2
import numpy as np
import copy

FX_DEPTH = 570
FY_DEPTH = 314
CX_DEPTH = 320
CY_DEPTH = 240

def convert_to_point_cloud(rgbd_image):
    global FX_DEPTH, FY_DEPTH, CX_DEPTH, CY_DEPTH
    # Konvertujte dubinsku sliku u oblak tačaka
    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, FX_DEPTH, FY_DEPTH, CX_DEPTH, CY_DEPTH)
    # Convert the RGBD image to an Open3D RGBDImage object
    rgbd_image_o3d = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbd_image, rgbd_image)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1000.0, convert_rgb_to_intensity=False)
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image_o3d, camera_intrinsic)

    return point_cloud

if __name__ == '__main__':

    # # Read color image
    # color_image = cv2.imread("/home/vlatka/Desktop/Diplomski/exp-004/input-image/000000000-rgb.png")

    # Read depth image
    # rgbd_image = cv2.imread(r"C:\Users\davor\OneDrive\Desktop\VLATKA\exp\exp-004\output-depth\000000000-output-depth-rgb.png", cv2.IMREAD_UNCHANGED)
    # Load the RGBD image from a PNG file
    rgbd_image = o3d.io.read_image(r"C:\Users\davor\OneDrive\Desktop\VLATKA\exp\exp-004\output-depth\000000000-output-depth-rgb.png")



    # Read segmentation image
    # segmentation_image = o3d.io.read_image(r"C:\Users\davor\OneDrive\Desktop\VLATKA\exp\exp-004\result-viz\mask.png")

    point_cloud = convert_to_point_cloud(rgbd_image)

    # Save the point cloud to a PLY file
    o3d.io.write_point_cloud(r"C:\Users\davor\OneDrive\Desktop\VLATKA\exp\PointCloud\pc1.ply", point_cloud)
