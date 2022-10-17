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

from scipy import signal

from support.pymf import get_MF_devices as get_camera_list

from gui_box_v1 import Ui_MainWindow
from support.support_mp4 import generate_pdf


mp_pose = mp.solutions.pose

"""open file dialog and path selection"""
calib_pth = ".//src//calib_aruco_dict_original_01.pickle"
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


def find_peaks(data, threshold):
    count = 0
    trigger = True
    for i in data:
        if i > threshold and trigger:
            count += 1
            trigger = False
        if i < threshold and not trigger:
            trigger = True
    return count


class WorkerSignals(QObject):
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(QImage)
    progress = pyqtSignal(QImage)
    changePixmap = pyqtSignal(QImage)


class Worker(QRunnable):

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            pass
        else:
            pass
        finally:
            self.signals.finished.emit()  # Done


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    xyRectPos = pyqtSignal(int)
    imageStatus = pyqtSignal(str)
    progress_callback = pyqtSignal(QImage)
    newPixmap = pyqtSignal(QImage)

    prev_frame_time = 0
    new_frame_time = 0

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.threadpool = QThreadPool()
        self.cam = cv2.VideoCapture(0)
        self.yRes = 640
        self.xRes = 480

        self.t_vec = []
        self.r_vec = []

        self.xPos = 0
        self.yPos = 0
        self.initUi
        self.pose = False
        self.data_points = np.nan
        self.data_points = np.array(self.data_points)

        self.disp_pose.stateChanged.connect(self.checkedcpp)
        self.start_button.clicked.connect(self.readImage)
        self.start_program.clicked.connect(self.start_process)
        self.set_orgin.clicked.connect(self.orign_set)

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.start_p = False
        # self.start_process = False
        self.cur_time = 0
        self.time_count = 0
        self.res = []
        self.cl_names = ["time", "X", "Y", "Z"]

        # selecting the webcam
        self.device_list = get_camera_list()
        print(self.device_list)
        self.webcam_id = self.device_list.index("e2eSoft iVCam")
        self.capture_device = cv2.VideoCapture(self.webcam_id)
        

        self.cam_space = pd.DataFrame(columns=self.cl_names)
        self.cam_space = self.cam_space.astype(np.float32)
        self.gen_report.clicked.connect(self.run_analysis)

    def orign_set(self):

        yPos = 0
        xPos = 0
        yRes = self.yRes
        xRes = self.xRes

        org_file = open(".//src//orgin.pickle", "wb")
        # Capture frame-by-frame
        ret, frame = self.capture_device.read()
        if ret:
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img = cv2.flip(img, 1)
            gray = img[yPos * 2:yPos * 2 + yRes, xPos * 2:xPos * 2 + xRes].copy()

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
                        vals = tvec[0]
                        pickle.dump(vals)
                        org_file.close()

            except:
                pass
        print("orgin saved")

    def checkedcpp(self, checked):
        if checked:
            self.pose = True
        else:
            self.pose = False

    def initUi(self):
        self.initCam()

    def readImage(self):
        worker = Worker(self.runCam)
        worker.signals.progress.connect(self.set_image)
        worker.signals.finished.connect(self.thread_complete)
        worker.signals.result.connect(self.thread_complete)
        self.threadpool.start(worker)

    def thread2(self):
        worker1 = Worker(self.run_analysis)
        worker1.signals.progress.connect(self.do_nothing)
        worker1.signals.finished.connect(self.thread_complete)
        worker1.signals.result.connect(self.thread_complete)
        self.threadpool.start(worker1)

    def start_process(self):
        self.start_p = True

    def do_nothing(self):
        pass

    def run_analysis(self):
        self.cam_space = self.cam_space[:-1]
        self.cam_space = self.cam_space.interpolate(method='linear', limit_direction='forward')
        print(self.cam_space)
        Nmedf = 5
        camfdf = self.cam_space
        for _col in camfdf:
            camfdf[_col] = signal.medfilt(camfdf[_col], kernel_size=Nmedf)

        plt.title("ROM")
        plt.scatter(self.cam_space["X"], self.cam_space["Z"], c="r")
        plt.savefig(".//src//figure_scatter.png")
        plt.clf()

        plt.subplot(3, 1, 1)
        plt.title("Coordinates")
        plt.plot(self.cam_space["time"], self.cam_space["X"], label="X", c="r")
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(self.cam_space["time"], self.cam_space["Y"], label="Y", c="y")

        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(self.cam_space["time"], self.cam_space["Z"], label="Z", c="b")

        plt.legend()

        plt.savefig(".//src//figure.png")
        a = generate_pdf(self)
        print(a)

    def print_progress(self):
        pass

    def thread_complete(self):
        print("thread completed")
        self.cam.release()

    @pyqtSlot(QImage)
    def set_image(self, image):
        self.video_disp.setPixmap(QPixmap.fromImage(image))

    def initCam(self):
        pass

    def runCam(self, progress_callback):
        mp_pose = mp.solutions.pose
        pose = mp_pose.PoseLandmark

        yPos = 0
        xPos = 0
        yRes = self.yRes
        xRes = self.xRes

        tr = True
        self.peaks = 0

        count = 0
        ln = len(self.cam_space.columns)
        empt_list = []
        for i in range(ln):
            empt_list.append(np.nan)

        time_tr = True

        while True:

            # Capture frame-by-frame
            ret, frame = self.capture_device.read()
            if ret:
                # img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                img = frame
                img = cv2.flip(img, 1)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                image = img[yPos * 2:yPos * 2 + yRes, xPos * 2:xPos * 2 + xRes].copy()

                self.colorImage = image

                if self.start_p and time_tr:
                    self.st_time = time.time()

                    time_tr = False

                if self.start_p:
                    self.cur_time = time.time()
                    self.time_count = int(self.cur_time - self.st_time)
                    self.time_float = self.cur_time - self.st_time

                if self.pose:
                    try:

                        gray = cv2.cvtColor(self.colorImage, cv2.COLOR_BGR2GRAY)
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

                        self.colorImage = aruco.drawDetectedMarkers(self.colorImage, corners, borderColor=(0, 0, 255))

                        # results = self.holistic.process(self.colorImage)
                        if self.start_p:
                            if ids is not None and len(ids) > 0:
                                # Estimate the posture per each Aruco marker
                                rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(
                                    corners, 0.05,
                                    cameraMatrix,
                                    distCoeffs)
                                for rvec, tvec in zip(rotation_vectors, translation_vectors):
                                    self.colorImage = aruco.drawAxis(self.colorImage, cameraMatrix, distCoeffs, rvec,
                                                                     tvec, 0.05)
                                    self.tvec_dist = tvec

                                    self.cam_space.loc[count, self.cl_names[0]] = self.time_float
                                    self.cam_space.loc[count, self.cl_names[1]] = tvec[0][0]
                                    self.cam_space.loc[count, self.cl_names[2]] = tvec[0][1]
                                    self.cam_space.loc[count, self.cl_names[3]] = tvec[0][2]
                                    count += 1

                            else:
                                pass

                    except:
                        pass
                flpimg = self.colorImage
                flpimg = cv2.resize(flpimg, (640, 480))
                h1, w1, ch = flpimg.shape
                bytesPerLine = ch * w1
                convertToQtFormat = QImage(flpimg.data.tobytes(), w1, h1, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(960, 540, Qt.KeepAspectRatio)

                if self.start_p and tr:
                    img12 = cv2.cvtColor(self.colorImage, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(".//src//color.jpeg", img12)
                    tr = False

                if self.time_count >= 500 and self.start_p:
                    self.run_analysis()
                    self.start_p = False

                if self.start_p:
                    try:
                        self.label_x.setText(str(f"X {round(self.tvec_dist[0][0], 2)} m"))
                        self.label_y.setText(str(f"Y {round(self.tvec_dist[0][1], 2)} m"))
                        self.label_z.setText(str(f"Z {round(self.tvec_dist[0][2], 2)} m"))
                    except:
                        pass

                progress_callback.emit(p)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    # ImageUpdate()
    w.show()
    sys.exit(app.exec_())
