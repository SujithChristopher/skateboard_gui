""" 

this program records video from two cameras simultaneously and saves the
files locally, in message pack format

this is designed to run in Kinect and Realsense

This also records IMU, and data from mecanum wheels, and saves in respective directory

This code is written by Sujith, 13-09-2022
"""

import os
import sys
import datetime
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import msgpack as mp
import msgpack_numpy as mpn
import numpy as np
import pyrealsense2 as rs

import cv2
import fpstimer
import multiprocessing
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from mecanum_wheel.encoder_stream_test import SerialPort
from support.pymf import get_MF_devices as get_camera_list

from threading import Thread
import keyboard

class MultiSensorRecorder:
    def __init__(self, _pth, display= False, record = False):

        """kinect parameters for recording"""
        self.yRes = 736
        self.xRes = 864

        self.xPos = 274 # fixed parameters
        self.yPos = 112

        self.fps_val = 15
        self._pth = _pth

        self.kill_signal = False
        self.display = display
        self.record = record

        """realsense parameters for recording"""
        self.yResRs = 670
        self.xResRs = 750

        self.device_list = get_camera_list()

        self.start_recording = False

        self.webcam_id = self.device_list.index("e2eSoft iVCam")

    
    def kill_thread(self):
        """kill the thread"""
        # kill the thread
        self.kill_signal = True

    
    def kinect_capture_frame(self):
        """kinect capture frame"""
        # kinect capture frame
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color)


        if self.record:
            _save_pth = os.path.join(self._pth, "kinect_color.msgpack")
            _save_file = open(_save_pth, "wb")
            _timestamp_file = open(os.path.join(self._pth, "kinect_timestamp.msgpack"), "wb")

        
        while True:
            if self.kinect.has_new_color_frame():
                frame = self.kinect.get_last_color_frame()
                frame = frame.reshape((1080, 1920, 4))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
                frame = frame[self.yPos * 2:self.yPos * 2 + self.yRes, self.xPos * 2:self.xPos * 2 + self.xRes].copy()

                if self.record and self.start_recording:
                    _packed_file = mp.packb(frame, default=mpn.encode)
                    _save_file.write(_packed_file)
                    _time_stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    _packed_timestamp = mp.packb(_time_stamp)
                    _timestamp_file.write(_packed_timestamp)


                fpstimer.FPSTimer(self.fps_val)

                if self.display:
                    cv2.imshow('kinect', frame)
                    cv2.waitKey(1)

                if keyboard.is_pressed('q'):  # if key 'q' is pressed 
                    print('You Pressed A Key!, ending kinect')
                    
                    self.kill_thread()  # finishing the loop
                    cv2.destroyAllWindows()
                
                if keyboard.is_pressed('s'):  # if key 's' is pressed
                    print('You Pressed A Key!, started recording from kinect')
                    self.start_recording = True

                if self.kill_signal:
                    break

        if self.record:
            _save_file.close()
            _timestamp_file.close()


    def rs_capture_frame(self):

        """capture frame from realsense"""

        # realsense capture frame

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8 , 15)
        pipeline.start(config)

        if self.record:
            _save_pth = os.path.join(self._pth, "realsense_color.msgpack")
            _save_file = open(_save_pth, "wb")
            _timestamp_file = open(os.path.join(self._pth, "realsense_timestamp.msgpack"), "wb")
        
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            color_image = color_image[self.yPos:self.yPos + self.yResRs, self.xPos:self.xPos + self.xResRs].copy()
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            if self.record and self.start_recording:
                _packed_file = mp.packb(gray_image, default=mpn.encode)
                _save_file.write(_packed_file)
                _time_stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                _packed_timestamp = mp.packb(_time_stamp)
                _timestamp_file.write(_packed_timestamp)

            fpstimer.FPSTimer(self.fps_val)

            if self.display:
                cv2.imshow('realsense', gray_image)
                cv2.waitKey(1)

            if not color_frame:
                continue

            if self.kill_signal:
                break

            if keyboard.is_pressed('s'):  # if key 's' is pressed
                print('You Pressed A Key!, started recording from realsense')
                self.start_recording = True
                
            if keyboard.is_pressed('q'):  # if key 'q' is pressed 
                print('You Pressed A Key!, ending realsense')
                self.kill_thread()  # finishing the loop

                cv2.destroyAllWindows()
                break

        if self.record:
            _save_file.close()
            _timestamp_file.close()

    def capture_webcam(self):
        """capture webcam"""

        #list available webcam
        cap = cv2.VideoCapture(self.webcam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 15)

        if self.record:
            _save_pth = os.path.join(self._pth, "webcam_color.msgpack")
            _save_file = open(_save_pth, "wb")
            _timestamp_file = open(os.path.join(self._pth, "webcam_timestamp.msgpack"), "wb")

        while True:
            ret, frame = cap.read()
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_image = gray_image[self.yPos:self.yPos + self.yResRs, self.xPos:self.xPos + self.xResRs].copy()

            if self.record and self.start_recording:
                _packed_file = mp.packb(gray_image, default=mpn.encode)
                _save_file.write(_packed_file)
                _time_stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                _packed_timestamp = mp.packb(_time_stamp)
                _timestamp_file.write(_packed_timestamp)

            fpstimer.FPSTimer(self.fps_val)

            if self.display:
                cv2.imshow('webcam', gray_image)
                cv2.waitKey(1)

            if keyboard.is_pressed('q'):  # if key 'q' is pressed 
                print('You Pressed A Key!, ending webcam')
                cap.release()
                cv2.destroyAllWindows()                
                self.kill_thread()  # finishing the loop
                if self.record:
                    _save_file.close()
                    _timestamp_file.close()
                break
            
            if keyboard.is_pressed('s'):  # if key 's' is pressed
                print('You Pressed A Key!, started recording from webcam')
                self.start_recording = True
                
    def record_for_calibration(self):
        """
        record data for calibration
        this does not include imu and mecanum wheel data
        """
        # record for calibration
        pass

    def run(self, cart_sensors):
        """run the program"""
        # run the program

        if not cart_sensors:
            kinect_capture_frame = multiprocessing.Process(target=self.kinect_capture_frame)
            rs_capture_frame = multiprocessing.Process(target=self.rs_capture_frame)
            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)

            kinect_capture_frame.start()
            rs_capture_frame.start()
            webcam_capture_frame.start()

            kinect_capture_frame.join()
            rs_capture_frame.join()
            webcam_capture_frame.join()


            if self.kill_signal:
                # kinect_capture_frame.terminate()
                # rs_capture_frame.terminate()
                print("killing the process")

        if cart_sensors:

            myport = SerialPort("COM4", 115200, csv_path=self._pth, csv_enable=True, single_file_protocol=True)
            cart_sensors = Thread(target=myport.run_program)
            kinect_capture_frame = multiprocessing.Process(target=self.kinect_capture_frame)
            rs_capture_frame = multiprocessing.Process(target=self.rs_capture_frame)
            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)
            
            cart_sensors.start()
            kinect_capture_frame.start()
            rs_capture_frame.start()
            webcam_capture_frame.start()
            
            cart_sensors.join()
            kinect_capture_frame.join()
            rs_capture_frame.join()
            webcam_capture_frame.join()
    

            if self.kill_signal:
                # kinect_capture_frame.terminate()
                # rs_capture_frame.terminate()

                # self._imu_p.kill()
                # cart_sensors.terminate()
                print("killing the process")

if __name__ == "__main__":
    # main program

    """Enter the respective parameters"""
    record = True
    if record:
        _name = input("Enter the name of the recording: ")
    display = True
    _pth = None # this is default do not change, path gets updated by your input

    if record:
        _pth = os.path.join(os.path.dirname(__file__), "test_data", _name)
        print(_pth)
        if not os.path.exists(_pth):
            os.makedirs(_pth)

    recorder = MultiSensorRecorder(_pth, display=display, record=record)
    recorder.run(cart_sensors=False)

    print("done recording")
            

        

    