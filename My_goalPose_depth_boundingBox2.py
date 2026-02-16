#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import messages
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import cos, sin, sqrt, atan2

# import the library
import rospy
import pyrealsense2 as rs
import numpy as np
import threading
import time
import actionlib


# global variable
person_X_center = 0.0
person_Y_center = 0.0
dist = 1

FOV = 1.39
divided_theta = FOV/16
target_coordinate_X = 0
target_coordinate_Y = 0
Num_degree = 0

#-----------------------------------------------------------------------------------------
# Setting pipeline for realsense D435
# Create a context object. This object owns the handles to all connected realsense devices
device_id="017322071787"
pipeline = rs.pipeline()

config = rs.config()
config.enable_device(device_id) if device_id is not None else 0
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)


#-----------------------------------------------------------------------------------------
# Opening server for publish goal messages

rospy.init_node('My_goalPose_depth_boundingBox2')

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()

goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()


#-----------------------------------------------------------------------------------------
# Define functions

def DepthFromD435():

    global pipeline
    global dist

    try:
        while True:
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            #if not depth: continue

            dist = depth.get_distance(int(person_X_center), int(person_Y_center))
            print("========================================")
            print("distance :", dist) #exit(0)
            print("========================================")
            print("\n\n")

            calculation()

            time.sleep(0.2)

    finally:
        pipeline.stop()



def callback(data):

    global person_X_center
    global person_Y_center

    for box in data.bounding_boxes:
        if box.Class == 'person':

            #rospy.loginfo(rospy.get_caller_id() + " I see\n %s", data.bounding_boxes)

            rospy.loginfo("\n\nClass: {},\n Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}\n".format(
            box.Class, box.xmin, box.xmax, box.ymin, box.ymax))

            person_X_center = (box.xmin + box.xmax)/2
            person_Y_center = (box.ymin + box.ymax)/2
            '''
            print("person_X_center:", person_X_center)
            print("person_Y_center:", person_Y_center)
            print("\n\n")
            '''



def calculation():

    global FOV
    global divided_theta
    global target_coordinate_X
    global target_coordinate_Y
    global Num_degree
    global person_X_center
    global person_Y_center
    global dist
    global client
    global goal

    Num_degree = int(person_X_center // 40)

    if Num_degree >= 0 and Num_degree < 8:
        target_coordinate_X = 3 - dist*sin(divided_theta*(8-Num_degree))
        target_coordinate_Y = 1 + dist*cos(divided_theta*(8-Num_degree))

    elif Num_degree >= 8 and Num_degree <= 16:
        target_coordinate_X = 3 + dist*sin(divided_theta*(Num_degree-7))
        target_coordinate_Y = 1 + dist*cos(divided_theta*(Num_degree-7))

    print("\n\ngoal : ", target_coordinate_X,target_coordinate_Y)

    goal.target_pose.pose.position.x = target_coordinate_X
    goal.target_pose.pose.position.y = target_coordinate_Y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)

    '''
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    '''



def reCoordinates():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    #rospy.init_node('reCoordinates', anonymous=True)

    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':

    Depth_thread = threading.Thread(target=DepthFromD435)
    Depth_thread.daemon=True
    Depth_thread.start()

    reCoordinates()

