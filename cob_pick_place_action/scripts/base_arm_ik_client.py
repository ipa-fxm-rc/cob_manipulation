#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_pick_place_action')
import rospy
import actionlib
import cob_pick_place_action.msg 
import sys
import tf
import math
import geometry_msgs.msg
from  geometry_msgs.msg import PoseStamped
from cob_pick_place_action.srv import *

if __name__ == "__main__":
		rospy.init_node('base_pose')
		rospy.wait_for_service('get_base_grasp_position_service')
		try:
				get_base_grasp_position = rospy.ServiceProxy('get_base_grasp_position_service', GraspBasePose)
				req=GraspBasePoseRequest()
				req.ik_link_name="arm_7_link"
				req.avoid_collisions=True
				pose = PoseStamped()
				pose.header.frame_id="/odom_combined"
				pose.header.stamp=rospy.Time.now()
				pose.pose.position.x = -1.0
				pose.pose.position.y = 2.0
				pose.pose.position.z = 0.70
				req.timeout = rospy.Duration(2)
				req.pose_stamped = pose
				#~ q = tf.transformations.quaternion_from_euler(-math.pi/2,-math.pi/2 ,0 )
				q = tf.transformations.quaternion_from_euler(0*math.pi/2,-math.pi/2 ,0 )
				req.pose_stamped.pose.orientation = geometry_msgs.msg.Quaternion(q[0], q[1], q[2], q[3])
				resp = get_base_grasp_position(req)
				#~ return resp1.sum
		except rospy.ServiceException, e:
				print "Service call failed: %s"%e

		print "Out of get_base_grasp_position_service"
		rospy.wait_for_service('get_motion_plan_trajectory')
		try:
				print "Inside get_motion_plan_trajectory"
				get_mobile_grasp_plan = rospy.ServiceProxy('get_motion_plan_trajectory', MobileGraspPlan)
				req_motion_plan = MobileGraspPlanRequest()
				req_motion_plan.joint_goal_pose=resp.solution
				req_motion_plan.goal_pose=pose
				#~ req_motion_plan.goal_pose.pose.position.x = 0.0
				#~ req_motion_plan.goal_pose.pose.position.y = 0.0
				req_motion_plan.ik_link_name="arm_7_link"
				res_motion_plan = get_mobile_grasp_plan(req_motion_plan)
				print "Motion Plan Result is: %s" %res_motion_plan.success
				#~ return resp1.sum
		except rospy.ServiceException, e:
				print "Service call failed: %s"%e
