import argparse
import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from PointCloud.src.imgequalization.imgequ import *

CURDIR = os.path.realpath(os.path.dirname(__file__))
DATASETS = ["Calibration20220922_14_23_47_135", "Calibration20220922_14_25_02_597"]


def display_inlier_outlier(cloud, ind):
    """
    Function that show the point cloud inlier and outlier
     (that will be removed)

    :param cloud: point cloud data
    :param ind: point cloud indexes
    :return: null, it
    """
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


def main(
        dataset: int,
        plot: bool
) -> None:
    ## DATASET CHOICE
    print("DATASET CHOICE")
    # Directory of datasets
    datasets = [CURDIR + "/Datasets/" + DATASETS[0],
                CURDIR + "/Datasets/" + DATASETS[1]
                ]
    if dataset == 1:
        mainFolder = datasets[dataset - 1]
        print("ok, dataset 1 choosen")
    elif dataset == 2:
        mainFolder = datasets[dataset - 1]
        print("ok, dataset 2 choosen")
    else:
        raise TypeError(f'Dataset not allowed, only {len(datasets)} available')

    ## EQUALIZE CAM IMAGES
    # https://github.com/torywalker/histogram-equalizer/blob/master/HistogramEqualization.ipynb
    print("EQUALIZING CAM IMAGES and saving them in /Out/")
    # Get the list of all png files in current directory
    filelist = os.listdir(mainFolder)
    for file in filelist:
        if not (file.endswith(".png")):
            filelist.remove(file)
    if plot: print(filelist)

    for i in range(len(filelist)):
        img = cv.imread(mainFolder + "/" + filelist[i], cv.IMREAD_ANYDEPTH)
        img_new = equalizethis(img)
        # Save the equalized images
        cv.imwrite(f"Out/Cam{i}norm.png", img_new)
        if plot:
            cv.imshow("img_norm", img_new)
            cv.waitKey()
            cv.destroyAllWindows()

    ## POINT CLOUD
    # http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html
    print("POINT CLOUD OPERATIONS")
    # Read point cloud and colouring them
    pcd0 = o3d.io.read_point_cloud(mainFolder + "/" + "004373465147cloud0.ply")
    pcd0.paint_uniform_color([1, 0, 0])
    pcd1 = o3d.io.read_point_cloud(mainFolder + "/" + "007086770647cloud0.ply")
    pcd1.paint_uniform_color([0, 1, 0])
    pcd2 = o3d.io.read_point_cloud(mainFolder + "/" + "018408745047cloud0.ply")
    pcd2.paint_uniform_color([0, 0, 1])

    # Add mesh coordinate frame
    # x->red; y->green; z->blue
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

    # Performing Voxel Downsampling
    voxel_size = 0.03
    pcd0_down = pcd0.voxel_down_sample(voxel_size=voxel_size)
    pcd1_down = pcd1.voxel_down_sample(voxel_size=voxel_size)
    pcd2_down = pcd2.voxel_down_sample(voxel_size=voxel_size)

    pcd_all = [pcd0_down, pcd1_down, pcd2_down]

    # ?? Hnorm
    Hnorm = [[0, 1, 0, 0],
             [0, 0, 1, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 1]]

    # Load the transformation matrices
    H0 = np.loadtxt(CURDIR + "/H0.txt", dtype=float)
    print(H0)
    H1 = np.loadtxt(CURDIR + "/H1.txt", dtype=float)
    H2 = np.loadtxt(CURDIR + "/H2.txt", dtype=float)

    # Transform the point clouds
    pcd_all[1].transform(np.matmul(np.linalg.inv(H0), H1))
    pcd_all[2].transform(np.matmul(np.linalg.inv(H0), H2))

    # Make a combined point cloud
    pcd_combined = o3d.geometry.PointCloud()
    for i in range(len(pcd_all)):
        pcd_combined += pcd_all[i]

    # Uniformly resampling using voxel. Recommended post-process after
    # merging since it can relieve duplicated or over-densified points
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)

    ## STATISTICAL OUTLIER REMOVAL
    # removes points that are further away from their neighbors
    # compared to the average for the point cloud
    pcd, ind = pcd_combined_down.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.7)
    # Display points removed
    if plot: display_inlier_outlier(pcd_combined_down, ind)

    ## PLANE SEGMENTATION
    # Used in order to remove ground and walls
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                             ransac_n=3,
                                             num_iterations=10000)
    [a, b, c, d] = plane_model
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # Display plane segmented
    if plot: o3d.visualization.draw_geometries([outlier_cloud, mesh_frame])
    pcd = outlier_cloud

    ## DBSCAN CLUSTERING
    # Clustering the cloud points
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=0.1, min_points=20, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # Display clusters by colors
    if plot: o3d.visualization.draw_geometries([pcd, mesh_frame])

    # obtain the more repeated index different of -1
    class_idx = np.bincount(labels[labels != -1]).argmax()
    # Find indexes of interest cluster
    indexes = np.where(labels == class_idx)[0]

    # extract the point cloud of the current cluster
    pcd = pcd.select_by_index(indexes)
    # Save the final point cloud
    o3d.io.write_point_cloud("Out/out.ply", pcd)
    # Visualize the point cloud
    if plot: o3d.visualization.draw_geometries([pcd, mesh_frame])

    ## ARUCO DETECTION
    # https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html
    print("ARUCO DETECTION")
    camera_matrix =[[388.198, 0.0, 253.270], [0.0, 389.033, 213.934], [0.0, 0.0, 1.0]]
    dist_coeff = [[0.126, -0.329, 0.111, -0.001, -0.002]]

    frame = cv.imread(CURDIR + "/Out/Cam0norm.png")
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    if plot:
        plt.figure()
        plt.imshow(frame_markers)
        for i in range(len(ids)):
            c = corners[i][0]
            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label="id={0}".format(ids[i]))
        plt.legend()
        plt.show()

    # Extrapolating aruco coordinate and useful data
    corners_all = np.array(c[0] for c in corners)

    corner = corners[0][0][0]
    x = corner[0]
    y = corner[1]
    print(f"First corner coordinates: [{x}, {y}]")

    # Build KDTree from point cloud
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    [k, idx, _] = pcd_tree.search_knn_vector_3d([x, y, 0], 10)
    np.asarray(pcd.colors)[idx[1:], :] = [1, 0, 0]
    o3d.visualization.draw_geometries([pcd, mesh_frame])
    # http: // www.open3d.org / docs / latest / tutorial / Basic / kdtree.html


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--dataset", required=True, help="The dataset to choose from", type=int)
    parser.add_argument("-p", required=False, help="View all the plots", default=False, action="store_true")

    args = parser.parse_args()

    main(
        dataset=args.dataset,
        plot=args.p
    )
