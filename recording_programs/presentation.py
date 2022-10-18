import os
import pickle
import sys
import time

import cv2
import matplotlib.pyplot as plt
import mediapipe as mp
import numpy as np
import pandas as pd
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from cv2 import aruco

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


_pth = r"C:\Users\Sujith\Desktop\ARMBO.mp4"
cap = cv2.VideoCapture(_pth)

"""open file dialog and path selection"""
calib_pth = ".//src//calib_aruco_dict_original_01.pickle"
print(str(calib_pth))
# Check for camera calibration data


vid_save = r"C:\Users\Sujith\Desktop\test.avi"
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

# We need to set resolutions.
# so, convert them from float to integer.


result = cv2.VideoWriter(vid_save, 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, (1920, 1080))
while True:
    ret, frame = cap.read()
    if ret:
        img = frame
        # img = cv2.flip(img, 1)
        w, h, _ = img.shape
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        try:
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
                    corners, 0.05,
                    cameraMatrix,
                    distCoeffs)
                for rvec, tvec in zip(rotation_vectors, translation_vectors):
                    colorImage = aruco.drawAxis(img, cameraMatrix, distCoeffs, rvec,
                                                        tvec, 0.05)
                    
                result.write(colorImage)

            
            else:
                result.write(img)
            
        except:
            pass
            
    else:
        break

result.release()
