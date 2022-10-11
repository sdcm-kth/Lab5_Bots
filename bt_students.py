#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# Move down Tiago's head to analyse the lower environment
		b0 = movehead("down")

		# Tuck Tiago's arm to detect Table A and aruco cube
		b1 = tuckarm()

		# Pick the aruco cube using Tiago's camera
		b2 = pickaruco()

		# go to table
		b3 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# move to chair
		b4 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		)

		# Place the aruco cube using Tiago's camera
		b5 = placearuco()
        
        # 
        b_succ = pt.composites.Sequence(
            name="Success sequence",
            children=[checkaruco(),tuckarm(),movehead("up")])
        
        #
        b_fail = pt.composites.Sequence(
            name="Failure sequence",
            children=[b3,b4])

        #
        b6 = pt.composites.Selector(
			name="Check the success fallback",
			children=[b_succ, b_fail])
           c c
		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

# class pickaruco(pt.behaviour.Behaviour):
    
#     """
#     Makes the robot pick the aruco cube on table A thanks to its camera.
#     Returns running whilst awaiting the result, success if the action was
#     succesful and v.v...
#     """
    
#     def __init__(self):
        
#         rospy.loginfo("Initialising pick aruco behaviour.")
        
#         # Call the pick service client which now knows the aruco pose
#         pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
#         self.pick_aruco_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
#         rospy.wait_for_service(pick_srv_nm, timeout=30)
        
#         # execution checker
#         self.tried = False
#         self.done = False

#         # become a behaviour
#         super(pickaruco, self).__init__("Pick aruco!")
        
#     def update(self):
        
#         # success if done
#         if self.done:
#             return pt.common.Status.SUCCESS

#         # try if not tried
#         elif not self.tried:

#             # command
#             self.pick_aruco_req = self.pick_aruco_srv()
#             self.tried = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if succesful
#         elif self.pick_aruco_req.success:
#             self.done = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.pick_aruco_req.success:
#             return pt.common.Status.FAILURE

#         # if still trying
#         else:
#             return pt.common.Status.RUNNING
        
# class placearuco(pt.behaviour.Behaviour):
    
#     """
#     Makes the robot place the aruco cube on table B thanks to its camera.
#     Returns running whilst awaiting the result, success if the action was
#     succesful and v.v...
#     """
    
#     def __init__(self):
        
#         rospy.loginfo("Initialising place aruco behaviour.")
        
#         # Call the pick service client which now knows the aruco pose
#         place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
#         self.place_aruco_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
#         rospy.wait_for_service(place_srv_nm, timeout=30)
        
#         # execution checker
#         self.tried = False
#         self.done = False

#         # become a behaviour
#         super(placearuco, self).__init__("Place aruco!")
        
#     def update(self):
        
#         # success if done
#         if self.done:
#             return pt.common.Status.SUCCESS

#         # try if not tried
#         elif not self.tried:

#             # command
#             self.place_aruco_req = self.place_aruco_srv()
#             self.tried = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if succesful
#         elif self.place_aruco_req.success:
#             self.done = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.place_aruco_req.success:
#             return pt.common.Status.FAILURE

#         # if still trying
#         else:
#             return pt.common.Status.RUNNING

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()