# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
import time
import numpy as np
from sys import platform
import argparse
import matplotlib.pyplot as plt
from numpy.linalg import norm
from matplotlib import pyplot, transforms, style
from mpl_toolkits import mplot3d

try:
    # Import Openpose (Windows/Ubuntu/OSX)
    # dir_path = r'C:\Users\BioRehab\Desktop\Sathya\posetrack\openpose-1.6.0'##python-3.8
    dir_path = r'C:\Users\BioRehab\Desktop\Sathya\posetrack\openpose-1.6.0-new\openpose-1.6.0'  ##python-3.7
    # print 'dir_path'
    try:
        # Windows Import
        if platform == "win32":
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append(dir_path + r'\build\python\openpose\Release')
            os.environ['PATH'] = os.environ[
                                     'PATH'] + ';' + dir_path + r'\build\x64\Release;' + dir_path + r'\build\bin;'
            import pyopenpose as op
        else:
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append('../../python')
            # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the
            # OpenPose/python module from there. This will install OpenPose and the python library at your desired
            # installation path. Ensure that this is in your python path in order to use it.

            # sys.path.append('/usr/local/python')
            from openpose import pyopenpose as op
    except ImportError as e:
        print(
            'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python '
            'script in the right folder?')
        raise e

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = dir_path + "/models/"
    # params["disable_blending"] = True
    params["number_people_max"] = 1
    # params["net_resolution"] = "-1x560"
    # params["camera"] = 1
    # params["body"] = 1
    # params["face"] = True
    # params["hand"] = True
    # params[""] = 
    # params[""] =
    # params[""] =

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1]) - 1:
            next_item = args[1][i + 1]
        else:
            next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-', '')
            if key not in params:  params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-', '')
            if key not in params: params[key] = next_item

    # Construct it from system arguments
    # op.init_argv(args[1])
    # oppython = op.OpenposePython()

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    ## Set Camera
    cam0 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cam1 = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    # cam0 = cv2.VideoCapture(r'C:\Users\BioRehab\Desktop\Sathya\posev.mp4')
    # cam1 = cv2.VideoCapture(r'C:\Users\BioRehab\Desktop\Sathya\posev.mp4')
    # [x, y] = (1280, 720)#(480, 640)
    # cam0.set(3, x)#default size 480x640, 1280x720
    # cam0.set(4, y)
    # cam1.set(3, x)
    # cam1.set(4, y)

    # Read Calibration files from directory
    dir = r'C:\Users\BioRehab\Desktop\Sathya\outrec\data6'

    # N camera projection matrices
    cv_file = cv2.FileStorage(dir + '/cam0cam1.yml', cv2.FILE_STORAGE_READ)
    K1 = cv_file.getNode("K1").mat()
    K2 = cv_file.getNode("K2").mat()
    R = cv_file.getNode("R").mat()
    T = cv_file.getNode("T").mat()

    # P1=cv2.hconcat([np.eye(3), np.zeros((3,1))])
    # P1 = cv_file.getNode("P1").mat()
    # P2 = cv_file.getNode("P2").mat()
    # print(P1,'\n', P2)
    P1 = np.hstack((np.dot(K1, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.dot(K1, [[0], [0], [0]])))
    P2 = np.hstack((np.dot(K2, R), np.dot(K2, T)))
    print(P1, '\n', P2)

    print(T.T)
    l2 = norm(T)
    print('cam distance ', l2)

    ## OutPut Initializations
    start = time.time()
    fps_time = 0
    fig = plt.gcf()
    ax = plt.axes(projection='3d')
    ax.view_init(-90, -90)


    #### Joint Angle Function
    def angle_between(V1, V2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::
        """
        v1_u = V1 / np.linalg.norm(V1)
        v2_u = V2 / np.linalg.norm(V2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u.T), -1.0, 1.0)), np.degrees(
            np.arccos(np.clip(np.dot(v1_u, v2_u.T), -1.0, 1.0)))

    while True:
        ## Get Camera Frames
        ret0, img0 = cam0.read()
        ret1, img1 = cam1.read()

        # img0 = cv2.rotate(img0, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # img1 = cv2.rotate(img1, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Process Image
        datum0 = op.Datum()
        datum0.cvInputData = img0
        opWrapper.emplaceAndPop([datum0])

        datum1 = op.Datum()
        datum1.cvInputData = img1
        opWrapper.emplaceAndPop([datum1])

        # print("Body keypoints0: \n" ,datum0.poseKeypoints, datum0.poseKeypoints.shape)
        # print("Body keypoints1: \n" ,datum1.poseKeypoints, datum1.poseKeypoints.shape)

        # a0 = (datum0.poseKeypoints)[:,:,0:2]
        # # print("Body keypoints0: \n" ,a0,a0.shape)

        # a1 = (datum1.poseKeypoints)[:,:,0:2]
        # # print("Body keypoints1: \n" ,a1,a0.shape)

        # ## 3D keypoints
        # p = cv2.triangulatePoints(P1, P2, a0, a1)
        # p /= p[3]
        # # print("Body keypoints3D: \n" ,'       x','          y','          z','          w')
        # # print(p.T,p.shape)
        # [x, y, z, w] = p
        # # print(x)

        try:
            a0 = (datum0.poseKeypoints)[:, :, 0:2]
            # print("Body keypoints0: \n" ,a0,a0.shape)

            a1 = (datum1.poseKeypoints)[:, :, 0:2]
            # print("Body keypoints1: \n" ,a1,a0.shape)

            ## 3D keypoints
            p = cv2.triangulatePoints(P1, P2, a0, a1)
            p /= p[3]
            # print("Body keypoints3D: \n" ,'       x','          y','          z','          w')
            # print(p.T,p.shape)
            [x, y, z, w] = p

            ### Body Joint Keypoints
            N = np.array([x[1], y[1], z[1]])
            RS = np.array([x[2], y[2], z[2]])
            RE = np.array([x[3], y[3], z[3]])
            RW = np.array([x[4], y[4], z[4]])

            LS = np.array([x[5], y[5], z[5]])
            LE = np.array([x[6], y[6], z[6]])
            LW = np.array([x[7], y[7], z[7]])
            ### BodyPart Vectors
            RBiceps = RS - RE
            RForearm = RE - RW
            LBiceps = LS - LE
            LForearm = LE - LW

            RShoulder = N - RS
            LShoulder = N - LS

            ## Elbow flexion and extension
            REangle = angle_between(RBiceps, RForearm)
            LEangle = angle_between(LBiceps, LForearm)
            ## Shoulder Adduction and Abduction
            RSAangle = angle_between(RBiceps, RShoulder)
            LSAangle = angle_between(LBiceps, LShoulder)




            if not args[0].no_display:
                datum0.cvOutputData = cv2.putText(datum0.cvOutputData, 'Cam0 %f' % (1.0 / (time.time() - fps_time)),
                                                  (30, 30), 2, 1, ([0, 0, 255]), 2)
                cv2.imshow("OpenPose 1.6.0 - Cam0", datum0.cvOutputData)
                # print((1.0 / (time.time() - fps_time)))
                datum1.cvOutputData = cv2.putText(datum1.cvOutputData, 'Cam1 %f' % (1.0 / (time.time() - fps_time)),
                                                  (30, 30), 2, 1, ([0, 0, 255]), 2)
                cv2.imshow("OpenPose 1.6.0 - Cam1", datum1.cvOutputData)

                plt.gca().cla()
                # ax.set_xlim(-1,1)
                # ax.set_ylim(-1,1)
                # ax.set_zlim(0,5)

                # ax.scatter3D(0,0,0)
                # ax.scatter3D(x[0:8],y[0:8],z[0:8], c='k')
                # ax.scatter3D(x[0:15],y[0:15],z[0:15], c='black')
                # ax.scatter3D(x[15:19],y[15:19],z[15:19], c='black')

                # ax.scatter3D(x[19:26],y[19:26],z[19:26], c='black')
                # ax.scatter3D(x,y,z, c='k')
                ax.scatter3D(x[0], y[0], z[0], s=100, c='r')
                ax.scatter3D(x[7], y[7], z[7], s=100, c='g')
                ax.scatter3D(x[4], y[4], z[4], s=100, c='b')
                # ax.scatter3D(x[47],y[47],z[47], s=100, c='y')

                # ax.plot3D(x,y,z)
                ax.plot3D([x[0], x[1]], [y[0], y[1]], [z[0], z[1]], c='k')
                ax.plot3D([x[2], x[1]], [y[2], y[1]], [z[2], z[1]], c='r')
                ax.plot3D([x[2], x[3]], [y[2], y[3]], [z[2], z[3]], c='r')
                ax.plot3D([x[4], x[3]], [y[4], y[3]], [z[4], z[3]], c='r')
                ax.plot3D([x[5], x[1]], [y[5], y[1]], [z[5], z[1]], c='k')
                ax.plot3D([x[5], x[6]], [y[5], y[6]], [z[5], z[6]], c='k')
                ax.plot3D([x[7], x[6]], [y[7], y[6]], [z[7], z[6]], c='k')

                # ax.plot3D([x[8],x[1]], [y[8],y[1]], [z[8],z[1]], c='k')
                # ax.plot3D([x[8],x[9]], [y[8],y[9]], [z[8],z[9]], c='r')
                # ax.plot3D([x[8],x[12]], [y[8],y[12]], [z[8],z[12]], c='k')
                # ax.plot3D([x[10],x[9]], [y[10],y[9]], [z[10],z[9]], c='r')
                # ax.plot3D([x[13],x[12]], [y[13],y[12]], [z[13],z[12]], c='k')
                # ax.plot3D([x[10],x[11]], [y[10],y[11]], [z[10],z[11]], c='r')
                # ax.plot3D([x[13],x[14]], [y[13],y[14]], [z[13],z[14]], c='k')

                # ax.plot3D([x[22],x[11]], [y[22],y[11]], [z[22],z[11]], c='r')
                # ax.plot3D([x[19],x[14]], [y[19],y[14]], [z[19],z[14]], c='k')
                # ax.plot3D([x[22],x[23]], [y[22],y[23]], [z[22],z[23]], c='r')
                # ax.plot3D([x[19],x[20]], [y[19],y[20]], [z[19],z[20]], c='k')
                # ax.plot3D([x[24],x[23]], [y[24],y[23]], [z[24],z[23]], c='r')
                # ax.plot3D([x[21],x[20]], [y[21],y[20]], [z[21],z[20]], c='k')

                # Get rid of the panes
                ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
                ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
                ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

                # Get rid of the spines
                ax.w_xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
                ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
                ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))

                # Get rid of the ticks
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_zticks([])

                # ax.set_zlim(-200,200)
                # ax.grid(False)
                # ax.view_init(-90,-90)
                ax.text2D(0, 0.9, "RIGHT", transform=ax.transAxes, color='red')
                ax.text2D(1, 0.9, "LEFT", transform=ax.transAxes, color='black')

                ax.text2D(0, 0.6, "RSA JOINT %d \N{DEGREE SIGN}" % (180-RSAangle[1]), transform=ax.transAxes)
                ax.text2D(1, 0.6, "LSA JOINT %d \N{DEGREE SIGN}" % (90-LSAangle[1]), transform=ax.transAxes)
                ax.text2D(0, 0.5, "RE JOINT %d \N{DEGREE SIGN}" % (REangle[1]), transform=ax.transAxes)
                ax.text2D(1, 0.5, "LE JOINT %d \N{DEGREE SIGN}" % LEangle[1], transform=ax.transAxes)

                ax.set_xlabel('X - Width'), ax.set_ylabel('Y - Height'), ax.set_zlabel('Z - Depth')
                # ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
                ax.set_title('FPS %.2f' % (1.0 / (time.time() - fps_time)), loc='left')

                fig.canvas.draw()
                plt.pause(0.1)

                fps_time = time.time()
                k = cv2.waitKey(1)
                if k % 256 == 27:
                    break

        except:
            continue

    plt.show()
    end = time.time()
    print("OpenPose demo successfully finished. Total time: " + str(end - start) + " seconds")

except Exception as e:
    print(e)
    sys.exit(-1)
