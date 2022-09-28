import cv2
from cv2 import aruco
import numpy as np
import os
import pickle

files = os.listdir("images")
total_detections = 0

"""open file dialog and path selection"""
calib_pth = "..//src//calib_aruco_dict_original_01.pickle"
print(str(calib_pth))
# Check for camera calibration data
if not os.path.exists(calib_pth):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open(calib_pth, "rb")
    (cameraMatrix, distCoeffs) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print(
            "Calibration issue. Remove ./calibration/CameraCalibration.pckl and recalibrate your camera with calibration_ChAruco.py.")
        exit()

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

# Create grid board object we're using in our stream
board = aruco.GridBoard_create(
    markersX=1,
    markersY=1,
    markerLength=0.05,
    markerSeparation=0.01,
    dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rotation_vectors, translation_vectors = None, None
axis = np.float32([[-.5, -.5, 0], [-.5, .5, 0], [.5, .5, 0], [.5, -.5, 0],
                   [-.5, -.5, 1], [-.5, .5, 1], [.5, .5, 1], [.5, -.5, 1]])

for i in files:
    colorImage = cv2.imread(f"images//{i}")

    try:
        gray = cv2.cvtColor(colorImage, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT,
                                                              parameters=ARUCO_PARAMETERS)
        corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
            image=gray,
            board=board,
            detectedCorners=corners,
            detectedIds=ids,
            rejectedCorners=rejectedImgPoints,
            cameraMatrix=cameraMatrix,
            distCoeffs=distCoeffs)

        if ids is not None and len(ids) > 0:
            # Estimate the posture per each Aruco marker
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, 0.04,
                cameraMatrix,
                distCoeffs)
            total_detections += 1
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                vals = tvec[0]


    except:
        pass

    print(total_detections)
print("finished")
