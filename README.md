# sanet_onionsorting
Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

This package uses Kinect V2 and YOLO to detect and obtain 3D coordinates of objects in the real world.

## The following instructions are written for Ubuntu 18.04, ROS Melodic.

If you need to install the packages for Kinect V2 with Ubuntu, check out this [link](https://github.com/thinclab/sawyer_irl_project/blob/master/Kinect_install_readme.md).

If you're using a Realsense D435 camera instead, follow [these](https://github.com/thinclab/RealSense2-D435/blob/main/README.md) instructions.

The following are the steps to be followed to get this package working:

  Assuming you have a working version of Ubuntu (This package has been built and tested on Ubuntu 18.04)
  
  1.) Install ROS (This package was built on ROS Melodic, for other versions you might need to make appropriate changes)
  
   [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
      
   [Catkin Workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
   
  2.) Clone this package and catkin_make within catkin_ws dir.
  
  3.) For the location of the camera used in this package, check [this link](https://github.com/thinclab/iai_kinect2/blob/master/kinect2_bridge/launch/rviz_tf.launch) if you're using a real kinect or [this link](https://github.com/thinclab/kinect_v2_udrf/blob/master/kinect_v2/launch/gazebo.launch) if you're using [this kinect_v2 package](https://github.com/thinclab/kinect_v2_udrf) with Gazebo.
  
   - Check sanet_onionsorting/thirdparty/yolov5/requirements.txt and make sure you have the latest version of all those dependencies installed correctly for yolo to work. **Do not worry about the package versions mentioned in the requirements file. pip installing the latest versions works just fine.**
   - Also, you need weights files to run yolo. Make sure you get the right weights file and put it in yolov5/weights.
  ### Note: If you plan to use your camera elsewhere, you have to change the coordinates accordingly in the static_transform publisher and the kinect_v2_standalong_physical.urdf.xacro file or the d435_camera_standalong.urdf.xacro file in case of Realsense camera.
  
  4.) 
  For the real camera, replace the word CAMERA with kinect or realsense in the following command arguments:
  
      roslaunch kinect2_bridge kinect2_bridge.launch
      
      roslaunch kinect2_bridge rviz_tf.launch
      
      rosrun sanet_onionsorting yolo_service.py real CAMERA
      
      rosrun sanet_onionsorting rgbd_imgpoint_to_tf.py real CAMERA
  For Gazebo kinect:
      After launching your required simulation setup on Gazebo:
      
        roslaunch kinect_v2 kinect_v2_full.launch
        
        rosrun sanet_onionsorting yolo_service.py gazebo CAMERA

        rosrun sanet_onionsorting rgbd_imgpoint_to_tf.py gazebo CAMERA
        
  5.) Now if you listen to /object_location topic, you get the 3D world coordinates of objects recognized by YOLO.
  
      
