import numpy as np
import open3d as o3d
import cv2 as cv
import os
import matplotlib.pyplot as plt
from PointCloud.src.imgequalization.imgequ import *

# https://github.com/isl-org/Open3D/issues/2596

dir = os.path.realpath(os.path.dirname(__file__))

if __name__ == "__main__":
    # READING DEPTH IMAGE
    #004373465147amp0
    depth_raw = o3d.io.read_image(dir + "/007086770647amp0.png")

    # # EQUALIZING IMAGE
    # img_eq = equalizethis(depth_raw)
    # cv.imshow("img_norm", img_eq)
    # cv.waitKey()
    # cv.destroyAllWindows()

    # CAM PARAMETERS
    intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.Kinect2DepthCameraDefault)
    # if create_from_rgbd_image is used white points will be ommited
    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_raw, intrinsic)
    #pcd.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([pcd])

    fx = intrinsic.intrinsic_matrix[0][0]
    fy = intrinsic.intrinsic_matrix[1][1]
    cx = intrinsic.intrinsic_matrix[0][2]
    cy = intrinsic.intrinsic_matrix[1][2]
    print(f"fx={fx}\n"
          f"fy={fy}\n"
          f"cx={cx}\n"
          f"cy={cy}\n")

    # CREATING DEPTH IMAGE FROM POINT CLOUD
    depth_raw_np = np.asarray(depth_raw)
    (h, w) = depth_raw_np.shape
    depthoutputimg = np.zeros((h, w, 1), np.uint16)

    #pcd0 = o3d.io.read_point_cloud(dir + "/018408745047amp0.ply")
    #o3d.visualization.draw_geometries([pcd0])

    for (x, y, z) in pcd.points:
        d = round(z * 1000)
        u = round(x * fx / (z ) + cx)
        v = round(y * fy / (z) + cy)
        depthoutputimg[v][u] = d
        # if(d != depth_raw_np[v][u]):
        #     print("diff"+str(u)+" "+str(v))

    # Flip it, otherwise the pointcloud will be upside down
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    output = o3d.geometry.Image(depthoutputimg)
    o3d.io.write_image(dir + "/outputprova.png", output)

    # Equalizing image
    img_new = equalizethis(output)
    cv.imshow("img_from_ply", img_new)
    cv.waitKey()
    cv.destroyAllWindows()
    # cv2.imwrite(dir + "/000027_bigdepth_out_cv.png", depthoutputimg)