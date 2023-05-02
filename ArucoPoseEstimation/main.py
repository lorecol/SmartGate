import argparse
import json

import cv2 as cv
import numpy as np


def inversePerspective(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv.Rodrigues(R)
    return invRvec, invTvec


def relativePosition(rvec1, tvec1, rvec2, tvec2):
    """
    Get relative position for rvec2 & tvec2.
    :param rvec1: rotation vector of marker 1
    :param tvec1: translation vector of marker 1
    :param rvec2: rotation vector of marker 2
    :param tvec2: translation vector of marker 2
    :return: composed rvec2 & tvec2 relative to rvec1 & tvec1
    """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    info = cv.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec


def main(
        calib_data_path: str,
        marker_size: int,
        image: str,
        from_id: int,
        to_id: int
) -> None:
    """
    :param calib_data_path:  Calibration data path
    :param marker_size:  Markers size [cm]
    :param image: Image path to which perform the aruco pose
    :param from_id: Marker id relative to which is detected the second marker
    :param to_id: Marker id which we have to calculate the pose relative to the first marker id
    :return:
    """

    # LOAD CAMERA CALIBRATION PARAMS
    with open(calib_data_path, "r") as read_file:
        calib_data = json.load(read_file)
    cam_mat = np.array(calib_data["camera_matrix"])
    dist_coef = np.array(calib_data["dist_coeff"])


    # SET COORDINATE SYSTEM in the middle of the aruco
    objPoints = np.array([[-marker_size / 2, marker_size / 2, 0],
                          [marker_size / 2, marker_size / 2, 0],
                          [marker_size / 2, -marker_size / 2, 0],
                          [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    # LOADING IMAGE
    frame = cv.imread(image)

    # ARUCO DETECTION INITIALIZATION
    param_markers = cv.aruco.DetectorParameters()
    # Marker dictionary, change if necessary
    marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
    # Initialize detector
    detector = cv.aruco.ArucoDetector(marker_dict, param_markers)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)

    # Check if there are at least to markers
    if marker_IDs.size < 2:
        raise TypeError(f'There are no 2 markers, only {marker_IDs.size} available')

    if marker_corners:
        cv.aruco.drawDetectedMarkers(frame, marker_corners, marker_IDs)
        nada = []
        rvec = []
        tvec = []
        for c in marker_corners:
            n, r, t = cv.solvePnP(objPoints, c, cam_mat, dist_coef, False, cv.SOLVEPNP_IPPE_SQUARE)
            rvec.append(r)
            tvec.append(t)
            nada.append(n)
        total_markers = range(0, marker_IDs.size)

        from_index = np.inf
        to_index = np.inf
        # Drawing axes frame
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.drawFrameAxes(frame, cam_mat, dist_coef, rvec[i], tvec[i], 3)
            if ids == from_id:
                from_index = i
            elif ids == to_id:
                to_index = i
        # Check if markers indexes as been found
        if from_index == np.inf or to_index == np.inf:
            TypeError(f"The two markers has not been found!! Check id and dictionary")
        else:
            print("The markers has been found")

        # Relative position of marker 2 with respect to marker 1
        composedRvec, composedTvec = relativePosition(rvec[from_index],
                                                      tvec[from_index],
                                                      rvec[to_index],
                                                      tvec[to_index])
        print(f"composedRvec [degree]:\n {np.degrees(composedRvec)}\n"
              f"composedTvec [cm]:\n {composedTvec}")

        # Distance of marker 2 from marker 1
        distance = np.sqrt(composedTvec[0, 0] ** 2 + composedTvec[1, 0] ** 2 + composedTvec[2, 0] ** 2)
        print(
            f"The distance of marker {marker_IDs[to_index, 0]} from marker {marker_IDs[from_index, 0]} is: {distance}cm")

        # Saving the relative position into json file
        data = {"composedRvec [degree]": np.degrees(composedRvec).tolist(), "composedTvec [cm]": composedTvec.tolist()}
        jname = f"output/Marker{marker_IDs[to_index, 0]}_relative_to{marker_IDs[from_index, 0]}.json"
        with open(jname, "w") as f:
            json.dump(data, f)

    # Showing the resulting image and exit procedures
    frame = cv.putText(frame,
                       f"Distance: {round(distance, 2)} cm",
                       (50, 50),
                       cv.FONT_HERSHEY_PLAIN,
                       2,
                       (0, 0, 255),
                       2,
                       cv.LINE_AA)
    frame = cv.resize(frame, (1080, 720))
    cv.imwrite("output/out_img.png", frame)
    cv.imshow("frame", frame)
    cv.waitKey()
    cv.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib-path", required=True, help="The camera calibration path", type=str)
    parser.add_argument("--marker-size", required=False, help="The marker size [cm]", default=8, type=int)
    parser.add_argument("--img-path", required=True, help="The image path to which perform the aruco pose", type=str)
    parser.add_argument("--from-id", required=False, help="The 1st marker ", default=555, type=int),
    parser.add_argument("--to-id", required=False, help="The 2nd marker ", default=444, type=int)

    args = parser.parse_args()

    main(
        calib_data_path=args.calib_path,
        marker_size=args.marker_size,
        image=args.img_path,
        from_id=args.from_id,
        to_id=args.to_id
    )
    # --calib-path calib_data/data.json --marker-size 8 --img-path Hololens_calib_photo.jpg --from-id 555 --to-id 444
