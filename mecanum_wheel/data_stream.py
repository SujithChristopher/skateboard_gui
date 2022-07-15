"""This program is for recording IMU data, camera data, and wheels data through HC05 bluetooth module"""

from turtle import st
import serial
import struct
import keyboard
import csv
from datetime import datetime
import sys
from sys import stdout
import getopt
import time
import cv2
import threading
import os
from pymf import get_MF_devices

class SerialPort(object):
    # Contains functions that enable communication between the docking station and the IMU watches

    def __init__(self, params, _foldername):
        # Initialise serial payload

        self.params = params

        self.csv_enabled = self.params["csv_enabled"]
        serialport = self.params["serialport"]
        serialrate = self.params["serialrate"]
        self.camera = self.params["camera"]
        _current_pth = os.getcwd()
        print(_current_pth)
        # self._foldername = os.path.dirname(_current_pth)
        self._foldername = _current_pth
        self._foldername = os.path.join(self._foldername,"data" ,_foldername)

        if os.path.isdir(self._foldername):
            print("folder exists, please delete the folder")
        else:
            os.mkdir(self._foldername)
        
        self.csv_path = os.path.join(self._foldername, "data.csv")
        
        self.count = 0
        self.plSz = 0
        self.payload = bytearray()

        self.serialport = serialport
        self.ser_port = serial.Serial(serialport, serialrate)

        if self.csv_enabled:

            self.csv_file = open(self.csv_path, "w")
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(["sys_time", "e_fr", "e_fl", "e_rr", "e_rl", "rtc", "mils", "sync", "ax", "ay", "az", "gx", "gy", "gz"])
        self.triggered = True
        self.connected = False

        """finding the camera"""

        device_list = get_MF_devices()
        for i, device_name in enumerate(device_list):
            print(f"opencv_index: {i}, device_name: {device_name}")
            if device_name.startswith("Kinect"):
                self.camera_device = i
                print("found camera")
                break

        stdout.write("Initializing imu program\n")

    """capture video frames and save it to a file"""
    def capture_video(self):

        # define a video capture object
        vid = cv2.VideoCapture(self.camera_device)
        
        while(True):
            
            # Capture the video frame
            # by frame
            ret, self.frame = vid.read()

            #name of the camera


            if ret:
        
                # Display the resulting frame
                cv2.imshow('frame', self.frame)
                
                # the 'q' button is set as the
                # quitting button you may use any
                # desired button of your choice
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
  
        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()
        
        # After the loop release the cap object
        vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()

    def serial_write(self, payload):

        # Format:
        # | 255 | 255 | no. of bytes | payload | checksum |

        header = [255, 255]
        chksum = 254
        payload_size = len(payload) + 1
        chksum += payload_size

        self.ser_port.write(bytes([header[0]]))
        self.ser_port.write(bytes([header[1]]))
        self.ser_port.write(bytes([payload_size]))
        self.ser_port.write(bytes([payload]))
        self.ser_port.write(bytes([chksum % 256]))

    def serial_read(self):
        """returns bool for valid read, also returns the data read"""
        # print(self.ser_port.read())

        if (self.ser_port.read() == b'\xff') and (self.ser_port.read() == b'\xff'):
            self.connected = True
            chksum = 255 + 255
            self.plSz = self.ser_port.read()[0]
            chksum += self.plSz
            self.payload = self.ser_port.read(self.plSz - 1)

            chksum += sum(self.payload)
            chksum = bytes([chksum % 256])
            _chksum = self.ser_port.read()

            return _chksum == chksum

        return False
    
    def serial_read_loop(self):

        while True:

            # print(self.ser_port.read())

            if self.serial_read():
                # print(len(self.payload))

                val = struct.unpack("4l", self.payload[:16])    # encoder values
                _rtc = struct.unpack("Q", self.payload[16:24])    # rtc values time delta
                mils = struct.unpack("L", self.payload[24:28])
                _sync = struct.unpack("c", self.payload[28:29])[0].decode("utf-8")
                _imu_data = struct.unpack("6f", self.payload[29:])

                # print(mils)

                _rtcval = datetime.fromtimestamp(_rtc[0]).strftime("%Y-%m-%d %I.%M.%S.%f %p")

                # # time_delta = struct.unpack("3H", self.payload[24:30])


                # print(val, _rtc)
                nw = None

                if not nw:
                    nw = datetime.now()     # datetime

                if self.csv_enabled:
                    self.csv.writerow([str(nw), val[0], val[1], val[2], val[3], _rtcval, mils[0], _sync, _imu_data[0], _imu_data[1], _imu_data[2], _imu_data[3], _imu_data[4], _imu_data[5]])
                if keyboard.is_pressed("e"):
                    self.csv_file.close()
                    break
            if keyboard.is_pressed("a"):
                print("closing")
                break



    def disconnect(self):
        stdout.write("disconnected\n")

    def run_program(self):
        """threading two funcitons"""
        t1 = threading.Thread(target=self.serial_read_loop)
        t2 = threading.Thread(target=self.capture_video)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        stdout.write("program started\n")


        


if __name__ == '__main__':
    # opts, args = getopt.getopt(sys.argv[1:], "p:", ["path"])

    # print(opts[0])
    # _filepath = opts[0][1]

    # myport = SerialPort("COM15", 115200, csv_path=_filepath, csv_enable=True)

    params = {
        'camera': True, 
        "imu": True, 
        "wheels": True, 
        "csv_enabled": True,
        "serialport": "COM4",
        "serialrate": 115200}

    myport = SerialPort(params, _foldername="testing5")
    myport.run_program()

