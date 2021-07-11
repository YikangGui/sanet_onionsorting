#!/usr/bin/env python3
# coding: utf8
import sys
from time import time

# append py2 in order to import rospy
sys.path.append('/usr/lib/python2.7/dist-packages')
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image, CameraInfo
# in order to import yolov5 under python3
sys.path.remove('/usr/lib/python2.7/dist-packages')
from sanet_onionsorting.srv import yolo_srv
import numpy as np
import copy
import rospkg

rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
path = rospack.get_path('sanet_onionsorting')   # get the file path for sanet_onionsorting
sys.path.append(path + '/thirdparty/')
from yolov5.detect import YOLO

same_flag = 0
rgb_mem = None
depth_mem = None
weights = None
camera = None

def grabrgb(msg):

    global rgb_mem
    if msg is not None:
        rgb_mem = copy.copy(msg)
    else:
        return

def getpred(msg):
    global weights, rgb_mem, depth_mem
    # print("Entered getpred func")
    start_time = time()
    centxs = []
    centys = []
    colors = []
    y = YOLO(weights, conf_thres = 0.8)
    if rgb_mem is not None: 
        # thisimage = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).astype('float32')
        # print("\nThis image shape: \n",np.shape(thisimage))
        # print('output:   ',output)

        if (camera == "realsense"):
            # print("Switching to bgr!")
            rgb_origin = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).copy()
            rgb = np.swapaxes(rgb_origin, 0, 2)
            bgr = np.array([rgb[2], rgb[1], rgb[0]])
            bgr = np.swapaxes(bgr, 0, 2)
            bgr = bgr.reshape(-1).tobytes()
            rgb_mem.data = bgr

        output = y.detect(rgb_mem)
        if output is not None:
            if len(output) > 0:   
                for det in output:
                    for *xyxy, conf, cls in det:
                        ''' 
                        NOTE: Useful link: https://miro.medium.com/max/597/1*85uPFWLrdVejJkWeie7cGw.png
                        Kinect image resolution is (1920,1080)
                        But numpy image shape is (1080,1920) becasue np takes image in the order height x width.
                        '''
                        tlx, tly, brx, bry = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                        centx, centy = int((tlx+brx)/2), int((tly+bry)/2)
                        if (int(cls) == 0 or int(cls) == 1):
                            # print("\ntlx, tly, brx, bry, cls: ",tlx, tly, brx, bry, int(cls))
                            print(f"\nCentroid: {centx}, {centy}")
                            centxs.append(centx)
                            centys.append(centy)
                            colors.append(cls)
                        else:   pass
            else: pass   
        else: print("\nNo output from yolo received yet\n")
        rgb_mem = None
        print("\nTime taken by yolo is: ", time() - start_time)
        if len(centxs) > 0:
            # print(f"\nFound {len(centxs)} onions\n")
            return centxs,centys,colors
        else:
            print("\nNo onions detected in frame\n")
            return [-1], [-1], [-1]
    else:
        print("\nNo RGB image received yet\n")
        return None, None, None


def main():
    global weights, camera
    try:
        rospy.init_node("yolo_service")
        rospy.loginfo("Yolo service started")

        import argparse

        desc = "A ROS service to communicate with YOLOv5 and obtain the bounding boxes."
        # create parser
        parser = argparse.ArgumentParser(description = desc)

        # add arguments to the parser
        parser.add_argument('--choice', dest='choice', default= 'real',
                    help='Choose between real/gazebo for camera')
        parser.add_argument('--cam', dest='camera_name', default= 'kinect',
                    help='Choose between kinect/realsense camera')

        # parse the arguments
        args = parser.parse_args()

        if (args.choice == "real"):
            weights = "best_realkinect.pt"
            # for kinect v2
            print(f"{weights} weights selected with real {args.camera_name} camera")
            if (args.camera_name == "kinect"):
                rospy.Subscriber("/kinect2/hd/image_color_rect", Image, grabrgb)
            elif (args.camera_name == "realsense"):
                rospy.Subscriber("/camera/color/image_raw", Image, grabrgb)
            else:
                raise ValueError("Wrong camera name")
            # for kinect v2
            # rospy.Subscriber("/kinect2/hd/points", Image, grabdepth)
        elif (args.choice == "gazebo"):
            weights = "best_gazebokinect.pt"
            # for kinect gazebo
            print(f"{weights} weights selected with gazebo")
            if (args.camera_name == "kinect"):
                rospy.Subscriber("/kinect_V2/rgb/image_raw", Image, grabrgb)
            elif (args.camera_name == "realsense"):
                rospy.Subscriber("/camera/color/image_raw", Image, grabrgb)
            else:
                raise ValueError("Wrong camera name")
            # for kinect gazebo
            # rospy.Subscriber("/kinect_V2/depth/points", Image, grabdepth)
        else:
            print(f"Unknown choice: {args.choice}. Please choose between real and gazebo.")
            
        camera = args.camera_name
        service = rospy.Service("/get_predictions", yolo_srv, getpred)

    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        return
    except KeyboardInterrupt:
        return
    rospy.spin()



if __name__ == '__main__':    
    main()
