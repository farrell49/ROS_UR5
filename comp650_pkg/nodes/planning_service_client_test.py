#!/usr/bin/env python

import sys
import rospy
from comp650_pkg.srv import *
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import moveit_commander
import moveit_msgs.msg
import rosparam
import rospkg
from yaml import load
from rospy_message_converter import message_converter


def request_plan():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "plan_gripper"
        TRAJECTORY = JointTrajectory()

        response = plan_request(REQUEST_TYPE, TRAJECTORY)

        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return False

def request_show():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "show_gripper"
        #TRAJECTORY = JointTrajectory()
        f = open(rospack.get_path("comp650_pkg")+"/traj_gripper.yaml")
        TRAJECTORY = load(f)
        f.close()
        rospy.loginfo("traj_yaml = %s", TRAJECTORY)
        TRAJECTORY = message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectory', TRAJECTORY)
        response = plan_request(REQUEST_TYPE, TRAJECTORY)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return False

def request_execute():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "execute_gripper"
        f = open(rospack.get_path("comp650_pkg")+"/traj.yaml")
        TRAJECTORY = load(f)
        f.close()
        rospy.loginfo("traj_yaml = %s", TRAJECTORY)
        TRAJECTORY = message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectory', TRAJECTORY)

        rospy.loginfo("Requesting Execute")
        response = plan_request(REQUEST_TYPE, TRAJECTORY)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True
'''
def request_pose():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "pose_gripper"
        TRAJECTORY = JointTrajectory()
        rospy.loginfo('requesting pose')
        resp = plan_request(REQUEST_TYPE, TRAJECTORY)
        rospy.loginfo('pose = {}'.format(resp))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True
'''
def request_joint_states():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "joint_states_gripper"
        TRAJECTORY = JointTrajectory()
        rospy.loginfo('requesting pose')
        resp = plan_request(REQUEST_TYPE, TRAJECTORY)
        rospy.loginfo('pose = {}'.format(resp))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True

if __name__ == "__main__":
    rospy.init_node("planning_service_client_test")

    rospy.loginfo("planning_service_client_test()")

    rospy.loginfo("-------------------")
    rospy.loginfo("Testing a plan call")

    rospack = rospkg.RosPack()
    if request_execute():
        rospy.loginfo("Plan call success")
    else:
        rospy.loginfo("Plan call FAIL")
