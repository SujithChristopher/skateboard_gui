import pandas as pd
import numpy as np
import cv2
import msgpack as mp
import msgpack_numpy as mpn
import glob
import os
import sys
from cv2 import aruco


"""these are default data, you can change it by calling the functions below"""

_pth = r"C:\Users\CMC\Dropbox\mira\mira_vellore\splitVideos\SUJIXXXXXXXXU010120000000XXXXXXXXX\calibration"

calib_pth = os.path.join(_pth, "AR_CALIBRATION.msgpack")
_calib_file = open(calib_pth, "rb")
unpacker = mp.Unpacker(_calib_file, object_hook=mpn.decode)
_calib = []
for unpacked in unpacker:
    _calib.append(unpacked)

cameraMatrix = _calib[0][0]
distCoeffs = _calib[0][1]


def camera_parameters(ar_parameters = None, ar_dictionary = None, markerLength = 0.05, markerSeparation = 0.01):

    """
    ar_parameters: aruco camera parameters using 'aruco.DetectorParameters_create()'
    ar_dictionary: dictionary of aruco markers
    markerLength: length of marker in meters
    markerSeparation: separation between markers in meters
    """

    ARUCO_PARAMETERS = ar_parameters
    ARUCO_DICT = ar_dictionary

    if ar_parameters is None:
        ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    if ar_dictionary is None:
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

    
    # Create grid board object we're using in our stream
    board = aruco.GridBoard_create(
            markersX=1,
            markersY=1,
            markerLength=markerLength,
            markerSeparation=markerSeparation,
            dictionary=ARUCO_DICT)

    return ARUCO_PARAMETERS, ARUCO_DICT, board


def estimate_ar_pose(frame, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs):

    """
    frame: frame to be processed
    cameraMatrix: camera matrix from calibration
    distCoeffs: distortion coefficients from calibration file
    """

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ARUCO_PARAMETERS, ARUCO_DICT, board = camera_parameters()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

    # Refine detected markers
    # Eliminates markers not part of our board, adds missing markers to the board
    corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
            image = gray,
            board = board,
            detectedCorners = corners,
            detectedIds = ids,
            rejectedCorners = rejectedImgPoints,
            cameraMatrix = cameraMatrix,
            distCoeffs = distCoeffs)

    rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
    return rotation_vectors, translation_vectors, _objPoints


def get_ar_pose_data(_pth, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs, process_raw = False, _pth_to_save=""):

    """
    _pth: path to video (msgpack) files containing calibration data
    """
    df = pd.DataFrame(columns=["frame_id", "x", "y", "z", "yaw", "pitch", "roll"])
    rotation_vectors, translation_vectors = None, None


    if process_raw:
        targetPattern = f"{_pth}\\COLOUR*"
        color_file_list = glob.glob(targetPattern)


        for fname in color_file_list:
            cfile = open(fname, "rb") #colour file
            unpacker = mp.Unpacker(cfile, object_hook=mpn.decode)
            for frame in unpacker:

                rotation_vectors, translation_vectors, _ = estimate_ar_pose(frame, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
                data = [5]
                if rotation_vectors is not None:
                    data.extend(translation_vectors[0][0])
                    data.extend(rotation_vectors[0][0])
                    df.loc[len(df)] = data
                
            cfile.close()
    else:

        vid_pth = os.path.join(_pth, "Video.avi")
        cap = cv2.VideoCapture(vid_pth)

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                rotation_vectors, translation_vectors, _ = estimate_ar_pose(frame, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

                data = [5]
                if rotation_vectors is not None:
                    data.extend(translation_vectors[0][0])
                    data.extend(rotation_vectors[0][0])
                    df.loc[len(df)] = data
                
            else:
                break
        cap.release()
        print("returning dataframe")
        print(df)
    return df



if __name__ == '__main__':
    # print("hi")
    get_ar_pose_data(r"C:\Users\CMC\Dropbox\mira\mira_vellore\splitVideos\SUJIXXXXXXXXU010120000000XXXXXXXXX\test_trial_0", process_raw=True)