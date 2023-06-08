import open3d as o3d
import cv2
import numpy as np
import copy

FX_DEPTH = 570
FY_DEPTH = 314
CX_DEPTH = 320
CY_DEPTH = 240

def convert_to_point_cloud(rgbd_image, segmentation_image):
    # Konvertujte dubinsku sliku u oblak tačaka
    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, FX_DEPTH, FY_DEPTH, CX_DEPTH, CY_DEPTH)
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1000.0, convert_rgb_to_intensity=False)
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsic)

    # Konvertujte sliku segmentacije u boje oblaka tačaka
    colors = segmentation_image.astype(np.float) / 255.0
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    return point_cloud

if __name__ == '__main__':

    # # Read color image
    # color_image = cv2.imread("/home/vlatka/Desktop/Diplomski/exp-004/input-image/000000000-rgb.png")

    # Read depth image
    rgbd_image = cv2.imread("/home/vlatka/Desktop/Diplomski/exp-004/output-depth/000000000-output-depth-rgb.png", cv2.IMREAD_UNCHANGED)

    # Read segmentation image
    segmentation_image = cv2.imread("/home/vlatka/Desktop/Diplomski/exp-004/result-viz/segmentation.png")

    point_cloud = convert_to_point_cloud(rgbd_image, segmentation_image)

    # Save the point cloud to a PLY file
    o3d.io.write_point_cloud("/home/vlatka/Desktop/Diplomski/Detekcija-i-manipulacija-transparentnih-objekata/PointCloud/pc1.ply", point_cloud)