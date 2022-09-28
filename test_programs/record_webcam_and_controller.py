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
    def __init__(self, _pth, cart_sensors=False):

        self.device_list = get_camera_list()
        self.cam_device = self.device_list.index("e2eSoft iVCam")

        """webcam parameters for recording"""
        self.yResRs = 670
        self.xResRs = 750
        self.cart_sensors = cart_sensors

        self._pth = _pth
    
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


    def run(self, cart_sensors):
        """run the program"""
        # run the program

        if not cart_sensors:

            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)
            webcam_capture_frame.start()
            webcam_capture_frame.join()

            if self.kill_signal:
                print("killing the process")

        if cart_sensors:

            myport = SerialPort("COM4", 115200, csv_path=self._pth, csv_enable=True, single_file_protocol=True)
            cart_sensors = Thread(target=myport.run_program)
            webcam_capture_frame = multiprocessing.Process(target=self.capture_webcam)
            
            cart_sensors.start()
            webcam_capture_frame.start()
            
            cart_sensors.join()
            webcam_capture_frame.join()
    

            if self.kill_signal:
                print("killing the process")

if __name__ == "__main__":

    record_data = RecordData()
    _pth = os.path.join(os.path.dirname(__file__), "test_data")
    record_data.run(_pth,cart_sensors=True)