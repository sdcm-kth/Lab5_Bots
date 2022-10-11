#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy import linalg as LA

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseWithCovarianceStamped
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
 
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

CHECK_CUBE = False
DETECTED_CUBE = False


class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1
        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            rospy.loginfo("Tiago's head moved "+str(self.direction)+" successfully !")
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            rospy.loginfo("Tiago's head not moved "+str(self.direction)+" successfully...")
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class pickaruco(pt.behaviour.Behaviour):
    
    """
    Makes the robot pick the aruco cube on table A thanks to its camera.
    Returns running whilst awaiting the result, success if the action was
    succesful and v.v...
    """
    
    def __init__(self):
        
        rospy.loginfo("Initialising pick aruco behaviour.")
        
        # Call the pick service client which now knows the aruco pose
        pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_aruco_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
        rospy.wait_for_service(pick_srv_nm, timeout=30)
        
        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pickaruco, self).__init__("Pick aruco!")
        
    def update(self):
        
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.pick_aruco_req = self.pick_aruco_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_aruco_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_aruco_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING
        
class placearuco(pt.behaviour.Behaviour):
    
    """
    Makes the robot place the aruco cube on table B thanks to its camera.
    Returns running whilst awaiting the result, success if the action was
    succesful and v.v...
    """
    
    def __init__(self):
        
        rospy.loginfo("Initialising place aruco behaviour.")
        
        # Call the pick service client which now knows the aruco pose
        place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_aruco_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
        rospy.wait_for_service(place_srv_nm, timeout=30)
        
        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(placearuco, self).__init__("Place aruco!")
        
    def update(self):
        
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.place_aruco_req = self.place_aruco_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_aruco_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_aruco_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class checkaruco(pt.behaviour.Behaviour):
    
    """Make the robot check if the aruco cube is well placed on table B.
    Returns success if the cube is detected and fail if the cube isn't"""
    
    def __init__(self):
        
        rospy.loginfo("Initialising check aruco behaviour.")
        
        #self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        self.aruco_pose_top = "/robotics_intro/aruco_single/position"
        self.aruco_pose_subs =rospy.Subscriber(self.aruco_pose_top, Vector3Stamped, self.aruco_pose_cb)
        
        # become a behaviour
        super(checkaruco, self).__init__("Check aruco!")
    
    def update(self):
        global CHECK_CUBE, DETECTED_CUBE
        if CHECK_CUBE == False and DETECTED_CUBE == False:
            try:
                rospy.wait_for_message(self.aruco_pose_top, Vector3Stamped, timeout=5)
                rospy.loginfo("Aruco detected on table B yasss")
                CHECK_CUBE = True
                DETECTED_CUBE = True
                return pt.common.Status.SUCCESS
            
            except:
                rospy.loginfo("Aruco not detected on table B :( ")
                CHECK_CUBE = True
                return pt.common.Status.FAILURE
        elif DETECTED_CUBE == True :
            return pt.common.Status.SUCCESS
        else :
            return pt.common.Status.FAILURE
    
    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose_rcv = True
        pass

