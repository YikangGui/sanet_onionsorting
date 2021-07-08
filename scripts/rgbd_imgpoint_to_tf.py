#!/usr/bin/env python
'''
Author: Prasanth Suresh (ps32611@uga.edu)
Owner: THINC Lab @ CS UGA

Please make sure you provide credit if you are using this code.

'''

import os
# import tf
import sys
import cv2
import time
import rospy
import random
import pprint
import image_geometry
import message_filters
import numpy as np
from itertools import chain
from rospy.core import rospyinfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
# from tf import TransformListener, transformations
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from sawyer_irl_project.msg import OBlobs
from sanet_onionsorting.srv import yolo_srv


class Camera():
    def __init__(self, camera_name, rgb_topic, depth_topic, camera_info_topic, choice=None):
        """
        @brief      A class to obtain time synchronized RGB and Depth frames from a camera topic and find the
                    3D position of the point wrt the required frame of reference.

        @param      camera_name        Just a relevant name for the camera being used.
        @param      rgb_topic          The topic that provides the rgb image information. 
        @param      depth_topic        The topic that provides the depth image information. 
        @param      camera_info_topic  The topic that provides the camera information. 
        @param      choice             If the camera used is a real or simulated camera based on commandline arg.
        """
        self.camera_name = camera_name
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.choice = choice

        self.poses = []
        self.rays = []
        self.OBlobs_x = []
        self.OBlobs_y = []
        self.OBlobs_z = []
        self.colors = []
        self.is_updated = False
        self.found_objects = False
        self.latest_rgb = None
        self.latest_depth_32FC1 = None

        tfBuffer = tf2_ros.Buffer()
        self.br = tf2_ros.TransformBroadcaster()
        self.lis = tf2_ros.TransformListener(tfBuffer)

        self.bridge = CvBridge()

        self.camera_model = image_geometry.PinholeCameraModel()


        # self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
        # cv2.setMouseCallback("Image window", self.mouse_callback)
        # self.br = tf.TransformBroadcaster()
        # self.lis = tf.TransformListener()
        # # Have we processed the camera_info and image yet?
        # self.ready_ = False
        q = 1

        self.pose3D_pub = rospy.Publisher('object_location', OBlobs, queue_size=q)

        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image, queue_size=q)

        self.sub_depth = message_filters.Subscriber(depth_topic, Image, queue_size=q)

        self.sub_camera_info = message_filters.Subscriber(camera_info_topic, CameraInfo, queue_size=q)
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size=15, slop=0.4)
        self.tss = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_camera_info], queue_size = q, slop = 0.5)
        #self.tss = message_filters.TimeSynchronizer([sub_rgb], 10)
        self.tss.registerCallback(self.callback)

        rospy.loginfo('Camera {} initialised, {}, {}, {}'.format(self.camera_name, rgb_topic, depth_topic, camera_info_topic))

    def save_response(self, response):
        '''
        @brief      A method that saves the latest yolo info.

        @param      response           The response message from the classifier, containing bounding box info.
        '''

        if len(response.centx) == 1 and response.centx[0] == -1:
            # If no objects are found in the image
            ob = OBlobs()
            ob.x = [-100]
            ob.y = [-100]
            ob.z = [-100]
            ob.color = [-100]
            self.pose3D_pub.publish(ob)
            rospy.sleep(0.1)
            print '\nNo onions found in the frame\n'
            return
        else:
            print '\nProcessing {0} onions info\n'.format(len(response.centx))
            self.xs = response.centx
            self.ys = response.centy
            self.colors = response.color
            self.found_objects = True


    def callback(self, rgb_msg, depth_msg, camera_info_msg):
        """
        @brief  Callback function for the ROS Subscriber that takes time synchronized 
                RGB and Depth image and publishes 3D poses wrt required reference frame.
        """

        # print "\nWe're in Callback!!\n"
        # img =  np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1).astype('float32')
        # img = img/255

        self.camera_model.fromCameraInfo(camera_info_msg)
        img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.latest_rgb = img
        depth_32FC1 = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        self.latest_depth_32FC1 = depth_32FC1.copy()
        self.is_updated = True

        # res = kinect_utils.filter_depth_noise(depth_32FC1)
        # depth_display = kinect_utils.normalize_depth_to_uint8(depth_32FC1.copy())
        # depth_display = cv2.normalize(depth_32FC1.copy(), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # depth_32FC1[depth_32FC1 < 0.1] = np.finfo(np.float32).max

        # if depth_32FC1.any() != 0:
        #     print "\nNot all zeros in {} depth image\n".format(np.shape(depth_32FC1))
        #     num_zeros = np.count_nonzero(depth_32FC1==0)
        #     print "\n{} zeros in {} sized array".format(num_zeros,np.shape(depth_32FC1)[0]*np.shape(depth_32FC1)[1])
        # else: print "\nAll zeros\n"

        # cv2.imshow("Image window", img)
        # cv2.imshow("depth", depth_32FC1)

        # cv2.setMouseCallback("Image window", self.mouse_callback)
        # cv2.waitKey(1)



    def OblobsPublisher(self):

        # print "\nWe're in OblobsPublisher!!\n"

        self.convertto3D()

        # print "\nLen of poses is: ",len(self.poses)

        if len(self.poses) > 0:

            self.getCam2Worldtf()

            # self.br.sendTransform(self.pose,(0,0,0,1),rospy.Time.now(),"clicked_object",self.camera_model.tfFrame())
            # self.marker_pub.publish(self.generate_marker(rospy.Time(0), self.get_tf_frame(), self.pose))
            ob = OBlobs()
            ob.x = self.OBlobs_x
            ob.y = self.OBlobs_y
            ob.z = self.OBlobs_z
            ob.color = self.colors
            # print '\nHere are the 3D locations: \n', ob
            self.pose3D_pub.publish(ob)
            rospy.sleep(0.1)
            self.is_updated = False
            # print '\nCamera is_updated inside callback: ', self.is_updated
            self.poses = []
            self.rays = []
            self.OBlobs_x = []
            self.OBlobs_y = []
            self.OBlobs_z = []
            ob, self.sub_camera_info, self.latest_rgb, self.latest_depth_32FC1 = None, None, None, None
            return

    def getCam2Worldtf(self):
        """
        @brief  Transforms 3D point in the camera frame to world frame 
                by listening to static transform between camera and world.
        """

        # print "\n Camera frame is: ",self.get_tf_frame()
        for i in range(len(self.poses)):
            camerapoint = tf2_geometry_msgs.tf2_geometry_msgs.PoseStamped()
            camerapoint.header.frame_id = self.get_tf_frame()
            camerapoint.header.stamp = rospy.Time(0)
            camerapoint.pose.position.x = self.poses[i][0]
            camerapoint.pose.position.y = self.poses[i][1]
            camerapoint.pose.position.z = self.poses[i][2]
            # print "\nCamerapoint: \n", camerapoint
            tf_buffer = tf2_ros.Buffer(
                rospy.Duration(1200.0))  # tf buffer length
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            cam_to_root_tf = tf_buffer.lookup_transform("root",  # target frame
                                                        self.get_tf_frame(),  # source frame
                                                        # get the tf at first available time
                                                        rospy.Time(0),
                                                        rospy.Duration(1.0))  # wait for 1 second for target frame to become available
            tf_point = tf2_geometry_msgs.do_transform_pose(
                camerapoint, cam_to_root_tf)
            # print '\n3D pose wrt world: ', tf_point

            self.OBlobs_x.append(tf_point.pose.position.x)
            self.OBlobs_y.append(tf_point.pose.position.y)
            self.OBlobs_z.append(tf_point.pose.position.z)
        return

    def get_current_rect_image(self):
        """
        @brief  Takes in a raw image

        @return Rectified image.
        """
        output_img = np.ndarray(self.get_current_raw_image().shape)
        self.camera_model.rectifyImage(
            self.get_current_raw_image(), output_img)
        return output_img

    def get_tf_frame(self):
        """
        @return camera transform frame.
        """
        return self.camera_model.tfFrame()

    def is_ready(self):
        """
        @return status.
        """
        return self.ready_

    def get_ray(self, uv_rect):
        """
        @brief  Takes in a 2D point on RGB image.

        @return Orthogonal projection.
        """
        return self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint(uv_rect))

    def mkmat(self, rows, cols, L):
        mat = np.matrix(L, dtype='float64')
        mat.resize((rows,cols))
        return mat

    def get_position_from_ray(self, ray, depth):
        """
        @brief      The 3D position of the object (in the camera frame) from a camera ray and depth value

        @param      ray    The ray (unit vector) from the camera centre point to the object point
        @param      depth  The norm (crow-flies) distance of the object from the camera

        @return     The 3D position of the object in the camera coordinate frame
        """

        # [ray_x * depth / ray_z, ray_y * depth / ray_z, ray_z * depth / ray_z]
        return [(i * depth) / ray[2] for i in ray]

    def generate_marker(self, stamp, frame_id, pose_3D):
        """
        @brief      Generates a marker of a given shape around the point on the image.

        @param      stamp    Header stamp of the marker message
        @param      frame_id  Header frame id
        @param      pose_3D   The 3D position of the point considered

        @return     Marker message to be published.
        """
        # marker_msg = Marker()
        # marker_msg.header.stamp = stamp
        # marker_msg.header.frame_id = frame_id
        # marker_msg.id = 0 #Marker unique ID

        # ARROW:0, CUBE:1, SPHERE:2, CYLINDER:3, LINE_STRIP:4, LINE_LIST:5, CUBE_LIST:6, SPHERE_LIST:7, POINTS:8, TEXT_VIEW_FACING:9, MESH_RESOURCE:10, TRIANGLE_LIST:11
        # marker_msg.type = 2
        # marker_msg.lifetime = 1
        # marker_msg.pose.position = pose_3D

        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id
        marker_msg.type = marker_msg.SPHERE
        marker_msg.action = marker_msg.ADD
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.pose.orientation.w = 1.0
        magicval_1 = 1.7
        marker_msg.pose.position.x = pose_3D[0]
        marker_msg.pose.position.y = pose_3D[1]
        marker_msg.pose.position.z = pose_3D[2]
        marker_msg.id = 1

        return marker_msg

    def process_ray(self, uv_rect, depth):
        """
        @param      uv_rect   Rectified x,y pose from RGB Image.
        @param      depth     Corresponding depth value.

        @return     Orthogonal ray and 3D pose wrt camera.
        """
        ray = self.get_ray(uv_rect)
        pose = self.get_position_from_ray(ray, depth)
        return ray, pose

    def mouse_callback(self, event, x, y, flags, param):
        """
        @brief  Callback function for mouse left click. Calculates 3D
                pose for the clicked point.

        @param  event   Here the event would be a mouse left click.
        @param  x,y     The corresponding x,y values from the image.
        @param  flags   Any flags raised about the event.
        @param  param   Any params passed with it.
        """

        if event == cv2.EVENT_LBUTTONDOWN:

            # clamp a number to be within a specified range
            def clamp(n, minn, maxn): return max(min(maxn, n), minn)

            # Small ROI around clicked point grows larger if no depth value found
            for bbox_width in range(20, int(self.latest_depth_32FC1.shape[0]/3), 5):
                tl_x = clamp(x-bbox_width/2, 0,
                             self.latest_depth_32FC1.shape[1])
                br_x = clamp(x+bbox_width/2, 0,
                             self.latest_depth_32FC1.shape[1])
                tl_y = clamp(y-bbox_width/2, 0,
                             self.latest_depth_32FC1.shape[0])
                br_y = clamp(y+bbox_width/2, 0,
                             self.latest_depth_32FC1.shape[0])
                # print('\n x, y, tl_x, tl_y, br_x, br_y: ',(x, y), (tl_x, tl_y, br_x, br_y))
                roi = self.latest_depth_32FC1[tl_y:br_y, tl_x:br_x]
                depth_distance = np.median(roi)

                if not np.isnan(depth_distance):
                    break

            # print('distance (crowflies) from camera to point: {:.2f}m'.format(depth_distance))
            self.ray, self.pose = self.process_ray((x, y), depth_distance)
            # print "\n3D pose wrt camera: \n", self.pose
            if self.choice == "real":
                ''' NOTE: The Real Kinect produces values in mm while ROS operates in m. '''
                self.poses.append(np.array(self.pose)/1000)
            else:
                self.poses.append(self.pose)

    def convertto3D(self):
        """
        @brief  Converts the point(s) provided as input into 3D coordinates wrt camera.
        """
        # print "\nWe're in convertto3D!!\n"
        # clamp a number to be within a specified range
        def clamp(n, minn, maxn): return max(min(maxn, n), minn)
        depth_distances = []
        # Small ROI around clicked point grows larger if no depth value found
        for i in range(len(self.xs)):
            roi = []
            for bbox_width in range(5, int(self.latest_depth_32FC1.shape[0]/3), 5):
                tl_x = int(clamp(self.xs[i]-bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[1]))
                br_x = int(clamp(self.xs[i]+bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[1]))
                tl_y = int(clamp(self.ys[i]-bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[0]))
                br_y = int(clamp(self.ys[i]+bbox_width/2,
                                 0, self.latest_depth_32FC1.shape[0]))
                print ('\n x, y, tl_x, tl_y, br_x, br_y: ',self.xs[i], self.ys[i], tl_x, tl_y, br_x, br_y)
                roi = (self.latest_depth_32FC1[tl_y:br_y, tl_x:br_x]).copy()
		
                roi = np.ma.masked_equal(roi, 0)

                # print '\nroi: \n', roi
                # rospy.sleep(100)
                if np.any(roi):
                    print(np.max(roi), np.min(roi))
                    depth_distances.append((np.max(roi) + np.min(roi)) / 2.0)
                    # print "\nGot Depth\n"

                if self.camera_name == "realsense":
                    if not np.isnan(depth_distances).any():
                        break
                else:
                    if np.min(roi) > 0.0:
                        # print "\nNo Nan values in depth values\n"
                        break

        # print('distance (crowflies) from camera to point: {}m'.format(depth_distances))
        for i in range(len(depth_distances)):
            # print "\nTotal depths: ",len(depth_distances)
            # print "\ni = ",i
            ray, pose = self.process_ray((self.xs[i], self.ys[i]), depth_distances[i])
            if self.choice == "real":
                ''' NOTE: The Real Kinect produces values in mm while ROS operates in m. '''
                self.rays.append(np.array(ray)/1000)
                self.poses.append(np.array(pose)/1000)
            else:
                if (self.camera_name == "kinect"):
                    self.rays.append(ray)
                    self.poses.append(pose)
                else:
                    self.rays.append(np.array(ray)/1000)
                    self.poses.append(np.array(pose)/1000)
        # print '\n(x,y): ',self.xs,self.ys
        # print '\n3D pose: ', self.poses


def main():
    """
    @brief  A ROS Node to transform a 2D point(s) on an image to its 3D world coordinates
            using RGB and Depth values obtained from an RGBD camera.
    """

    try:

        rospy.init_node('depth_from_object', anonymous=True)
        rate = rospy.Rate(25)
        # Contains the centroids of the obj bounding boxes
        rospy.wait_for_service("/get_predictions")

        if len(sys.argv) < 2:
            print "Default choice real kinect chosen"
            choice = "real"
        else:
            choice = sys.argv[1]
            camera_name = sys.argv[2]
            print "\n{} {} chosen".format(choice, camera_name)

        if (choice == "real"):
            if (camera_name == "kinect"):
                rgbtopic = '/kinect2/hd/image_color_rect'
                depthtopic = '/kinect2/hd/image_depth_rect'
                camerainfo = '/kinect2/hd/camera_info'
            elif (camera_name == "realsense"):
                rgbtopic = '/camera/color/image_raw'
                depthtopic = '/camera/aligned_depth_to_color/image_raw'
                camerainfo = '/camera/color/camera_info'

        elif (choice == "gazebo"):
            if (camera_name == "kinect"):
                rgbtopic = '/kinect_V2/rgb/image_raw'
                depthtopic = '/kinect_V2/depth/image_raw'
                camerainfo = '/kinect_V2/rgb/camera_info'
            elif (camera_name == "realsense"):
                rgbtopic = '/camera/color/image_raw'
                depthtopic = '/camera/depth/image_raw'
                camerainfo = '/camera/color/camera_info'
            else:
                raise ValueError("Wrong camera name.")


        camera = Camera(camera_name, rgbtopic, depthtopic, camerainfo, choice)

        while not rospy.is_shutdown():

            print '\nUpdating YOLO predictions...\n'
            gip_service = rospy.ServiceProxy("/get_predictions", yolo_srv)
            response = gip_service()
            # print '\nCentroid of onions: [x1,x2...],[y1,y2...] \n', response.centx, response.centy
            camera.save_response(response)
            # print "\nIs updated: ",camera.is_updated,"\tFound objects: ", camera.found_objects
            if camera.is_updated and camera.found_objects:    
                camera.OblobsPublisher()
            rospy.sleep(1)
            # rate.sleep()
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
