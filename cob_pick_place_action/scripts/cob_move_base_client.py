#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import actionlib
import moveit_msgs.msg 

def cob_move_base_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (MoveGroupAction) to the constructor.
	move_base_action_client = actionlib.SimpleActionClient('MoveAction', moveit_msgs.msg.MoveGroupAction)

	# Waits until the action server has started up and started
	# listening for goals.
	move_base_action_client.wait_for_server()
	
	# Creates a goal to send to the action server.
	goal = moveit_msgs.msg.MoveGroupGoal()
	goal.planning_options.plan_only=True

	goal.request.group_name="base"
	goal.request.workspace_parameters.header.stamp = rospy.Time.now()
	goal.request.workspace_parameters.header.frame_id = "base_footprint"
	goal.request.workspace_parameters.min_corner.x=-2;
	goal.request.workspace_parameters.min_corner.y=-2;
	goal.request.workspace_parameters.min_corner.z=-2;
	goal.request.workspace_parameters.max_corner.x=2;
	goal.request.workspace_parameters.max_corner.y=2;
	goal.request.workspace_parameters.max_corner.z=2;
	print goal.request
	goal.request.goal_constraints.name="joint_constraints[]"
	goal.request.goal_constraints.joint_constraints[0].joint_name="world_joint/theta"
	goal.request.goal_constraints.joint_constraints[0].position=0
	goal.request.goal_constraints.joint_constraints[0].tolerance_above=0.0001
	goal.request.goal_constraints.joint_constraints[0].tolerance_below=0.0001
	goal.request.goal_constraints.joint_constraints[0].weight=1
	
	goal.request.goal_constraints.joint_constraints[1].joint_name="world_joint/x"
	goal.request.goal_constraints.joint_constraints[1].position=1
	goal.request.goal_constraints.joint_constraints[1].tolerance_above=0.0001
	goal.request.goal_constraints.joint_constraints[1].tolerance_below=0.0001
	goal.request.goal_constraints.joint_constraints[1].weight=1
	
	goal.request.goal_constraints.joint_constraints[2].joint_name="world_joint/y"
	goal.request.goal_constraints.joint_constraints[2].position=2
	goal.request.goal_constraints.joint_constraints[2].tolerance_above=0.0001
	goal.request.goal_constraints.joint_constraints[2].tolerance_below=0.0001
	goal.request.goal_constraints.joint_constraints[2].weight=1
	
	
	
	
	#~ goal.object_id = 18
	#~ goal.object_name = "yellowsaltcube"
	#~ goal.object_pose.header.stamp = rospy.Time.now()
	#~ goal.object_pose.header.frame_id = "base_footprint"
	#~ goal.object_pose.pose.position.x = -0.5
	#~ goal.object_pose.pose.position.y = -0.5  
	#~ goal.object_pose.pose.position.z =  0.6
	#~ goal.object_pose.pose.orientation.w = 1.0
	#~ goal.object_pose.pose.orientation.x = 0.0
	#~ goal.object_pose.pose.orientation.y = 0.0
	#~ goal.object_pose.pose.orientation.z = 0.0


	# Sends the goal to the action server.
	move_base_action_client.send_goal(goal)

	# Waits for the server to finish performing the action.
	finished_before_timeout=move_base_action_client.wait_for_result(rospy.Duration(300, 0))

	if finished_before_timeout:
		state=move_base_action_client.get_state()
		print "Action finished: %s"%state
	# Prints out the result of executing the action
	return state  # State after waiting for CobMoveBaseAction

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		# publish and subscribe over ROS.
		rospy.init_node('CobMoveBaseAction_client_py')
		result = cob_move_base_client()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
