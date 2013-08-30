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
 * \date Date of creation: April, 2013
 *
 * \brief
 *   
 *   This package gets the object information from COB Grasp Manager and 
 *   provides COB Grasp Generator with the object ID and requests for
 *   next grasp information from the Grasp Table already generated.
 *   Then it provides COB Grasp Validity Checker with the grasp information
 *   and object information and request for the validtiy information.
 *   Then the grasp information for valid grasp(if obtained) is passed to
 *   COB Grasp Manager.
 *
 ****************************************************************/
 
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <cob_pick_place_action/cob_grasp_base_position_server.h>

COBGetGraspBasePosition::COBGetGraspBasePosition(){}

//~ void BaseArmIKSolver::currentJointStatesCallback(const sensor_msgs::JointStateConstPtr &response, sensor_msgs::JointState &current_joint_states)
void COBGetGraspBasePosition::currentJointStatesCallback(const sensor_msgs::JointStateConstPtr &response)
{
	//~ ROS_INFO("response->name.size: %d", response->name.size());
	for(int i=0; i<response->name.size();i++)
	{
		if ((std::find(current_joint_states.name.begin(), current_joint_states.name.end(), response->name[i].c_str()) == current_joint_states.name.end())||!current_joint_states.name.size())
		{
			current_joint_states.name.push_back(response->name[i]);
			current_joint_states.position.push_back(response->position[i]);
		}
	}
}


sensor_msgs::JointState COBGetGraspBasePosition::InitialJointState()
{
	//~ sensor_msgs::JointState current_joint_states;
	current_joint_states.name.clear();
	current_joint_states.position.clear();
	sub = node_handle_.subscribe(JOINT_STATE_TOPIC, 10, &COBGetGraspBasePosition::currentJointStatesCallback, this);

	//~ sub = node_handle_.subscribe<sensor_msgs::JointState> (JOINT_STATE_TOPIC, 10, boost::bind(currentJointStatesCallback, _1, boost::ref(current_joint_states))); 
	ros::Duration(.5).sleep();
	ROS_INFO("before spinOnce");
	ros::spinOnce();
	ROS_INFO("after spinOnce");
	GetBasePosition(current_joint_states);
	ROS_INFO("current_joint_states.size: %d", current_joint_states.name.size());
	ROS_INFO_STREAM("current_joint_states: " <<current_joint_states);
	return current_joint_states;
}

//~ BaseArmIKSolver::GetBasePosition(sensor_msgs::JointState &base_joint_states)
void COBGetGraspBasePosition::GetBasePosition(sensor_msgs::JointState &base_joint_states)
{
	tf::StampedTransform transform_footprint_odom;
	ros::Time abhi = ros::Time::now();
	try
	{
		ros::Duration(2.0).sleep();
		listener.waitForTransform("/odom_combined" , "/base_footprint", abhi, ros::Duration(1.0));
		listener.lookupTransform("/odom_combined", "/base_footprint", abhi, transform_footprint_odom);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	geometry_msgs::Transform msg;
	tf::transformTFToMsg(transform_footprint_odom, msg);
	//~ ROS_INFO_STREAM("transform_footprint_odom" << transform_footprint_odom);
	//~ tf::Quaternion q;
	ROS_INFO_STREAM("geometry_msgs_transform: "<< msg);
	//~ double roll, pitch, yaw;
	double yaw = tf::getYaw(msg.rotation);
	//~ tf::quaternionMsgToTF(msg.rotation, q);
	//~ 
	//~ tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//~ ROS_INFO("roll, pitch, yaw: %f   :%f    :%F", roll, pitch, yaw);
	base_joint_states.name.push_back("base_placement_x_joint");
	base_joint_states.name.push_back("base_placement_y_joint");
	base_joint_states.name.push_back("base_placement_theta_joint");
	base_joint_states.position.push_back(msg.translation.x);
	base_joint_states.position.push_back(msg.translation.y);
	base_joint_states.position.push_back(yaw);
}
	
//~ BaseArmIKSolver::fillIKRequest()
void COBGetGraspBasePosition::fillIKRequest(cob_pick_place_action::GraspBasePose::Request &req, moveit_msgs::GetPositionIK::Request  &request_ik_solver)
{
	
	
	request_ik_solver.ik_request.robot_state.joint_state=InitialJointState();
	request_ik_solver.ik_request.ik_link_name = req.ik_link_name;
	request_ik_solver.ik_request.avoid_collisions = req.avoid_collisions;
	request_ik_solver.ik_request.pose_stamped = req.pose_stamped;
	request_ik_solver.ik_request.timeout = req.timeout;
}

//~ BaseArmIKSolver::baseArmIKCallback()
bool COBGetGraspBasePosition::GraspBasePositionCallback(cob_pick_place_action::GraspBasePose::Request &req, cob_pick_place_action::GraspBasePose::Response &res )
{
	moveit_msgs::GetPositionIK::Request  request_ik_solver;
	moveit_msgs::GetPositionIK::Response response_ik_solver; 
	fillIKRequest(req, request_ik_solver);
	
	if (ik_solver_client.call(request_ik_solver, response_ik_solver))
	{
		ROS_INFO_STREAM("Solution is: " << response_ik_solver.solution);
		res.solution=response_ik_solver.solution;
	}
	else 
	{
		ROS_ERROR("Grasp Base Position Server call failed");
	}
	return true;
}
	
//~ void BaseArmIKSolver::run()
void COBGetGraspBasePosition::run()
{
	ROS_INFO("cob_grasp_base_position_server...spinning");
	ros::spin();
}

void COBGetGraspBasePosition::Init()
{
	JOINT_STATE_TOPIC = "/joint_states";
	ik_solver_client = node_handle_.serviceClient<moveit_msgs::GetPositionIK>("get_ik");
	ros::service::waitForService("get_ik");
	grasp_base_position_server = node_handle_.advertiseService("get_base_grasp_position_service", &COBGetGraspBasePosition::GraspBasePositionCallback, this);
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "grasp_base_position");
	COBGetGraspBasePosition *base_arm_IKSolver = new COBGetGraspBasePosition();
	
	base_arm_IKSolver->Init();
	base_arm_IKSolver->run();
	
	return 0;
}

	
	
	
