import os
import cv2 as cv
import matplotlib.pyplot as plt
import open3d as o3d
from PointCloud.src.imgequalization.imgequ import *

PLOT = 1 # Display PLOT
CURDIR = os.path.realpath(os.path.dirname(__file__))
DATASETS = ["Calibration20220922_14_23_47_135", "Calibration20220922_14_25_02_597"]

## DATASET CHOICE
print("DATASET CHOICE")
#Directory of datasets
datasets = [CURDIR + "/Datasets/" + DATASETS[0],
            CURDIR + "/Datasets/" + DATASETS[1]
            ]
# Choose from which dataset import the data
data_choice = int(input(
    "Which data set you want to evaluate?\n - 1 for the 1st" +
    " dataset\n - 2 for the 2nd dataset\n\n Choice: "))
if data_choice == 1:
    mainFolder = datasets[data_choice - 1]
    print("ok, dataset 1 choosen")
elif data_choice == 2:
    mainFolder = datasets[data_choice - 1]
    print("ok, dataset 2 choosen")
else:
    raise TypeError('Dataset not allowed')


## EQUALIZE CAM IMAGES
#https://github.com/torywalker/histogram-equalizer/blob/master/HistogramEqualization.ipynb
print("EQUALIZING CAM IMAGES and saving them in /Out/")
# Get the list of all png files in current directory
filelist = os.listdir(mainFolder)
for file in filelist:
    if not(file.endswith(".png")):
        filelist.remove(file)
if PLOT: print(filelist)

for i in range(len(filelist)):
    img = cv.imread(mainFolder + "/" + filelist[i], cv.IMREAD_ANYDEPTH)
    img_new = equalizethis(img)
    # Save the equalized images
    cv.imwrite(f"Out/Cam{i}norm.png", img_new)
    if PLOT:
        cv.imshow("img_norm",img_new)
        cv.waitKey()
        cv.destroyAllWindows()


## POINT CLOUD
# http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html
print("POINT CLOUD OPERATIONS")
# Read point cloud
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

# Load the transoformation matrices
H0 = np.loadtxt(CURDIR + "/H0.txt", dtype=float)
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
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

pcd, ind = pcd_combined.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.7)
# Display points removed
if PLOT: display_inlier_outlier(pcd_combined, ind)


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
if PLOT: o3d.visualization.draw_geometries([outlier_cloud, mesh_frame])
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
if PLOT: o3d.visualization.draw_geometries([pcd, mesh_frame])

# obtain the more repeated index different of -1
class_idx = np.bincount(labels[labels != -1]).argmax()
# Find indexes of interest cluster
indexes = np.where(labels == class_idx)[0]

# extract the point cloud of the current cluster
pcd = pcd.select_by_index(indexes)
o3d.visualization.draw_geometries([pcd, mesh_frame])
o3d.io.write_point_cloud("Out/out.ply", pcd)

## ARUCO DETECTION
print("ARUCO DETECTION")
frame = cv.imread(CURDIR + "/Out/Cam0norm.png")
cv.imshow('img', frame)
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
frame_markers = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

if PLOT:
    plt.figure()
    plt.imshow(frame_markers)
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label="id={0}".format(ids[i]))
    plt.legend()
    plt.show()

