#!/usr/bin/env python
'''
Author: Prasanth Suresh (ps32611@uga.edu)
Owner: THINC Lab @ CS UGA

@brief: This is a basic script for testing. Just gets the service results and prints it out.

Please make sure you provide credit if you are using this code.

'''
import rospy
from sanet_onionsorting.srv import frcnn_srv


if __name__ == "__main__":
    rospy.init_node("frcnn_client")
    rospy.wait_for_service("/get_predictions")
    rate=rospy.Rate(10)

    try:
        gip_service = rospy.ServiceProxy("/get_predictions", frcnn_srv)
        while not rospy.is_shutdown():
            response = gip_service()
            print("Centroid of leftmost onion: ", response.centx, response.centy)  

    except rospy.ServiceException as e:
        rospy.logwarn("Service frcnn Failed"+str(e))