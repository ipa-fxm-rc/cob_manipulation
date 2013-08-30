/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_manipulation_sandbox
 * \note
 *   ROS package name: cob_grasp_validity_checker
 *
 * \author
 *   Author: Rohit Chandra, email:rohit.chandra@ipa.fhg.de
 *
 * \date Date of creation: March, 2013
 *
 * \brief
 *	 This package provides service for generating collision free grasps.
 *   Takes information from the grasping request(object_ID & -- ). 
 *	 Obtains corresponding grasping possibilities from pre-generated grasps.
 *   Check all of them for collision in the planning scene. 
 *   Generates a new database of the collision-free grasps.
 *
 ****************************************************************/
#ifndef COB_GET_GRASP_BASE_POSITION_H
#define COB_GET_GRASP_BASE_POSITION_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cob_pick_place_action/GraspBasePose.h>
#include "moveit_msgs/GetPositionIK.h"
#include <cob_pick_place_action/cob_pick_place_action.h>

class COBGetGraspBasePosition
{
private:
	sensor_msgs::JointState current_joint_states;
	std::string JOINT_STATE_TOPIC;
	tf::TransformListener listener;
	ros::ServiceClient ik_solver_client;
	ros::ServiceServer grasp_base_position_server;
	ros::NodeHandle node_handle_;
	ros::Subscriber sub;
	
public:
	COBGetGraspBasePosition();
	void GetBasePosition(sensor_msgs::JointState &base_joint_states);
	bool GraspBasePositionCallback(cob_pick_place_action::GraspBasePose::Request &req, cob_pick_place_action::GraspBasePose::Response &res);
	void run();
	void Init();
	sensor_msgs::JointState InitialJointState();
	void currentJointStatesCallback(const sensor_msgs::JointStateConstPtr &response);
	void fillIKRequest(cob_pick_place_action::GraspBasePose::Request &req, moveit_msgs::GetPositionIK::Request  &request_ik_solver);
	
};
#endif
