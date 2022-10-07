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
        
        # Turn Tiago's base towards Table B
        b3 = pt.composites.Selector(
			name="Turn to Table B fallback", 
			children=[counter(30, "Towards Table B?"), go("Turn towards Table B!", 0, -0.5)]
            # Change the counter and angular values to adjust the motion
        
        # Move Tiago's base to Table B
        b3 = pt.composites.Selector(
			name="Move to Table B fallback", 
			children=[counter(30, "To Table B?"), go("Move to Table B!", 1, 0)]
            # Change the counter and angular values to adjust the motion

        # Place the aruco cube using Tiago's camera
		b4 = placearuco()

        # Tuck Tiago's arm
        b5 = tuckarm()

		# Upper head
		b6 = movehead("up")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()