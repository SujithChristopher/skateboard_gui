"""
this program records data from webcam and teensy controller
"""
import cv2
import os
import sys
import datetime
import keyboard
import msgpack as mp
import msgpack_numpy as mpn
import fpstimer
import multiprocessing
from threading import Thread
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from mecanum_wheel.encoder_stream_test import SerialPort
from support.pymf import get_MF_devices as get_camera_list

class RecordData:
    def __init__(self, _pth = None, record = True):

        self.device_list = get_camera_list()
        self.cam_device = self.device_list.index("e2eSoft iVCam")

        """webcam parameters for recording"""
        self.yResRs = 670
        self.xResRs = 750

        self.xPos = 274 # fixed parameters
        self.yPos = 112
        self.record = record
        self.start_recording = False
        self._pth = _pth
        self.kill_signal = False
        self.fps_val = 15
        self.display = True
    
    def capture_webcam(self):
        """capture webcam"""

        #list available webcam
        cap = cv2.VideoCapture(self.cam_device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 15)

        if self.record:
            _save_pth = os.path.join(self._pth, "webcam_color.msgpack")
            _save_file = open(_save_pth, "wb")
            _timestamp_file = open(os.path.join(self._pth, "webcam_timestamp.msgpack"), "wb")

        while True:
            ret, frame = cap.read()
            if ret:
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
                    # self.kill_thread()  # finishing the loop
                    if self.record:
                        _save_file.close()
                        _timestamp_file.close()
                    break
                
                if keyboard.is_pressed('s'):  # if key 's' is pressed
                    print('You Pressed A Key!, started recording from webcam')
                    self.start_recording = True


    def run(self, cart_sensors):
        """run the program"""
        # run the program

        if not cart_sensors:

            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)
            webcam_capture_frame.start()
            webcam_capture_frame.join()

            if self.kill_signal:
                print("killing the process")
            

        if cart_sensors and self.record:

            myport = SerialPort("COM6", 115200, csv_path=self._pth, csv_enable=True, single_file_protocol=True)
            cart_sensors = Thread(target=myport.run_program)
            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)
            
            cart_sensors.start()
            webcam_capture_frame.start()
            
            cart_sensors.join()
            webcam_capture_frame.join()
    

            if self.kill_signal:
                print("killing the process")

if __name__ == "__main__":

    """Enter the respective parameters"""
    record = True
    if record:
        _name = input("Enter the name of the recording: ")
    display = True
    _pth = None # this is default do not change, path gets updated by your input

    if record:
        _pth = os.path.join(os.path.dirname(__file__), "test_data","single_cam_oct_8", _name)
        print(_pth)
        if not os.path.exists(_pth):
            os.makedirs(_pth)

    record_data = RecordData(_pth=_pth)
    record_data.run(cart_sensors=False)