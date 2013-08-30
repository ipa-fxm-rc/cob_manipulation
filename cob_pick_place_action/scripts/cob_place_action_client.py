#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import actionlib
import manipulation_msgs.msg
import geometry_msgs.msg
# Brings in the messages used by the CobPlace action, including the
# goal message and the result message.
import cob_pick_place_action.msg 

def cob_place_action_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (CobPlaceAction) to the constructor.
	place_action_client = actionlib.SimpleActionClient('cob_place_action', cob_pick_place_action.msg.CobPlaceAction)

	# Waits until the action server has started up and started
	# listening for goals.
	place_action_client.wait_for_server()
	
	# Creates a goal to send to the action server.
	goal = cob_pick_place_action.msg.CobPlaceGoal()
	goal.object_id = 18;
	goal.object_name = "yellowsaltcube";
	g= manipulation_msgs.msg.PlaceLocation()
	p= geometry_msgs.msg.PoseStamped()
	#~ manipulation_msgs::PlaceLocation g;
	#~ geometry_msgs::PoseStamped p; 
	p.header.frame_id = "base_footprint";
	p.pose.position.x = -0.5;
	p.pose.position.y = -0.5;  
	p.pose.position.z =  0.6;
	p.pose.orientation.w = 1.0;
	p.pose.orientation.x = 0.0;
	p.pose.orientation.y = 0.0;
	p.pose.orientation.z = 0.0;
	goal.destination=p;
	# Sends the goal to the action server.
	place_action_client.send_goal(goal)

	# Waits for the server to finish performing the action.
	finished_before_timeout=place_action_client.wait_for_result(rospy.Duration(300, 0))

	if finished_before_timeout:
		state=place_action_client.get_state()
		print "Action finished: %s"%state
	# Prints out the result of executing the action
	return state  # State after waiting for CobPlaceAction

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobPlaceAction_client_py')
		result = cob_place_action_client()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
