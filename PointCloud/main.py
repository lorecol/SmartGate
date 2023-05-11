import argparse
import os

import cv2 as cv
import matplotlib.pyplot as plt
import open3d as o3d
import json
from scipy.spatial.transform import Rotation

from PointCloud.src.imgequalization.imgequ import *

CURDIR = os.path.realpath(os.path.dirname(__file__))
DATASETS = ["Calibration20220922_14_23_47_135", "Calibration20220922_14_25_02_597"]


def display_inlier_outlier(cloud, ind):
    """
    Function that show the point cloud inlier and outlier
     (that will be removed)

    :param cloud: point cloud data
    :param ind: point cloud indexes
    :return: null
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
    """
    Main function that create the point cloud from the depth image
    :param dataset: dataset number
    :param plot: if True, plot the images
    :return: null
    """

    ## DATASET CHOICE
    print("=============DATASET CHOICE=============")
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
    print("=============EQUALIZING CAM IMAGES and saving them in /output/=============")
    # Get the list of all png files in current directory
    filelist = os.listdir(mainFolder)
    for file in filelist:
        if not (file.endswith(".png")):
            filelist.remove(file)
    if plot: print(filelist)

    for i in range(len(filelist)):
        img = cv.imread(mainFolder + "/" + filelist[i], cv.IMREAD_ANYDEPTH)
        h, w = img.shape
        img_new = equalizethis(img)
        # Save the equalized images
        cv.imwrite(f"output/Cam{i}norm.png", img_new)
        if plot:
            cv.imshow("img_norm", img_new)
            cv.waitKey()
            cv.destroyAllWindows()

    ## POINT CLOUD
    # http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html
    print("=============POINT CLOUD OPERATIONS=============")
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
    o3d.io.write_point_cloud("output/out.ply", pcd)
    # Visualize the point cloud
    if plot: o3d.visualization.draw_geometries([pcd, mesh_frame])

    ## ARUCO DETECTION
    # https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html
    print("=============ARUCO DETECTION=============")

    # Read cam0 point cloud and colouring them
    pcd0 = o3d.io.read_point_cloud(mainFolder + "/" + "004373465147cloud0.ply")

    # Extracting points from point cloud
    all_points = np.asarray(pcd0.points)

    # Initializing the matrix
    img_matrix_x = np.zeros((h, w))
    img_matrix_y = np.zeros((h, w))
    img_matrix_z = np.zeros((h, w))

    # Filling the matrix with the points
    z = 0
    for i in range(h):
        for j in range(w):
            img_matrix_x[i, j] = all_points[z, 0]
            img_matrix_y[i, j] = all_points[z, 1]
            img_matrix_z[i, j] = all_points[z, 2]
            z = z + 1
    if plot:
        plt.imshow(img_matrix_z, interpolation='nearest')
        plt.title('depth image from point cloud')
        plt.show()
    # Saving the array in a text file
    # np.set_printoptions(threshold=np.inf)
    # file = open("img_matrix_x.txt", "w+")
    # np.set_printoptions(threshold=np.inf)
    # content = str(img_matrix_x)
    # file.write(content)
    # file.close()

    # Detecting the aruco markers from  the normalized image
    frame = cv.imread(CURDIR + "/output/Cam0norm.png")
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    print(f"Detected {len(corners)} aruco markers.")

    # Show the image with the markers
    if plot:
        plt.figure()
        plt.imshow(frame_markers)
        for i in range(len(ids)):
            c = corners[i][0]
            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label="id={0}".format(ids[i]))
        plt.legend()
        plt.show()

    # Extracting the coordinates of the aruco marker
    aruco_xyz = np.zeros((4, 3))
    for i in range(4):
        aruco_xyz[i, 0] = img_matrix_x[int(corners[0][0][i][1]), int(corners[0][0][i][0])]
        aruco_xyz[i, 1] = img_matrix_y[int(corners[0][0][i][1]), int(corners[0][0][i][0])]
        aruco_xyz[i, 2] = img_matrix_z[int(corners[0][0][i][1]), int(corners[0][0][i][0])]

    # Find the coordinates of the square's center
    square_center = np.mean(aruco_xyz, axis=0)

    # define the global reference frame that will coincide with cam 0
    global_origin = np.array([0, 0, 0])
    global_x_axis = np.array([1, 0, 0])
    global_y_axis = np.array([0, 1, 0])
    global_z_axis = np.array([0, 0, 1])

    # find the translation vector
    translation_vector = square_center - global_origin

    # Find the rotation matrix
    x_axis = (aruco_xyz[1] - aruco_xyz[0]) / np.linalg.norm(aruco_xyz[1] - aruco_xyz[0])
    y_axis = (aruco_xyz[3] - aruco_xyz[0]) / np.linalg.norm(aruco_xyz[3] - aruco_xyz[0])
    z_axis = np.cross(x_axis, y_axis)
    rotation_matrix = np.array([x_axis, y_axis, z_axis])
    # From rotation matrix to euler angles
    # create a Rotation object from the rotation matrix
    r = Rotation.from_matrix(rotation_matrix)
    # get the Euler angles in radians
    euler = r.as_euler('xyz')

    # Form the pose matrix
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = translation_vector

    # Print the results
    print("Translation vector: \n", translation_vector)
    print("Rotation matrix: \n", rotation_matrix)
    print("Euler angles in degrees: \n", np.degrees(euler))
    print("Pose matrix: \n ", pose_matrix)

    # Saving the relative position into a json file
    data = {"translation vector [m]": translation_vector.tolist(),
            "rotation matrix": rotation_matrix.tolist(),
            "euler angles deg [degree]": np.degrees(euler).tolist()}
    jname = f"output/Aruco_pose_relative_to_cam_0.json"
    with open(jname, "w") as f:
        json.dump(data, f)

    # Rotate of 90 degrees around z axis
    theta_deg = 90
    theta = np.deg2rad(theta_deg)
    Rz = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])
    translation_vector_rotated = np.dot(Rz, translation_vector)
    rotation_matrix_rotated = np.dot(rotation_matrix, Rz)
    print(f"Translation vector after rotation of {theta_deg} degrees: \n", translation_vector_rotated)
    print(f"Rotation matrix after rotation of {theta_deg} degrees: \n", rotation_matrix_rotated)

    # From rotation matrix to euler angles
    r = Rotation.from_matrix(rotation_matrix_rotated)
    # get the Euler angles in radians
    euler = r.as_euler('xyz')
    print("Euler angles in degrees of rotated matrix:\n", np.degrees(euler))

    # Saving the relative position into a json file
    data = {"translation vector [m]": translation_vector_rotated.tolist(),
            "rotation matrix ": rotation_matrix_rotated.tolist(),
            "euler angles deg [degree]": np.degrees(euler).tolist()}
    jname = f"output/Aruco_pose_relative_to_cam_0_rotated{theta_deg}.json"
    with open(jname, "w") as f:
        json.dump(data, f)



if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser()

    parser.add_argument("--dataset", required=True, help="The dataset to choose from", type=int)
    parser.add_argument("-p", required=False, help="View all the plots", default=False, action="store_true")

    args = parser.parse_args()

    # Call the main function
    main(
        dataset=args.dataset,
        plot=args.p
    )
    # You can execute the script from the terminal with the following command:
    # python main.py --dataset 0 -p
