/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <cob_pick_place_action/cob_pick_place_action.h>
#include <cob_pick_place_action/MobileGraspPlan.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

ros::Publisher pub_co;

geometry_msgs::PoseStamped newEEPose(cob_pick_place_action::MobileGraspPlan::Request &req_mobile_grasp_plan)
{
	tf::Quaternion quatBase = tf::createQuaternionFromYaw(req_mobile_grasp_plan.joint_goal_pose.joint_state.position[2]);
	tf::Vector3 vecBase= tf::Vector3(req_mobile_grasp_plan.joint_goal_pose.joint_state.position[0], req_mobile_grasp_plan.joint_goal_pose.joint_state.position[1], 0);
	tf::Transform odomTbase = tf::Transform(quatBase, vecBase);
	tf::Transform baseTodom = odomTbase.inverse();
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(req_mobile_grasp_plan.goal_pose.pose.orientation, quat);
	tf::Vector3 vec= tf::Vector3(req_mobile_grasp_plan.goal_pose.pose.position.x, req_mobile_grasp_plan.goal_pose.pose.position.y, req_mobile_grasp_plan.goal_pose.pose.position.z);
	tf::Transform OdomTeef = tf::Transform(quat, vec);
	tf::Transform BaseTeef = baseTodom.operator*(OdomTeef);
	
	geometry_msgs::Transform msg;
	tf::transformTFToMsg(BaseTeef, msg);
	
	geometry_msgs::PoseStamped eefinBase;
	eefinBase.header.frame_id=req_mobile_grasp_plan.goal_pose.header.frame_id;
	eefinBase.pose.position.x = msg.translation.x;
	eefinBase.pose.position.y = msg.translation.y;
	eefinBase.pose.position.z = msg.translation.z;
	eefinBase.pose.orientation = msg.rotation;
	ROS_INFO_STREAM("eefinBase" << eefinBase);
	return eefinBase;
}
 
bool mobileGraspPlanCallback(cob_pick_place_action::MobileGraspPlan::Request &req_mobile_grasp_plan, cob_pick_place_action::MobileGraspPlan::Response &res_mobile_grasp_plan)
{
	ros::NodeHandle node_handle("~");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	ROS_INFO_STREAM("req_mobile_grasp_plan.goal_pose:" << req_mobile_grasp_plan.goal_pose);
	ROS_INFO_STREAM("req_mobile_grasp_plan.joint_goal_pose" << req_mobile_grasp_plan.joint_goal_pose);
  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  /* Construct a planning scene - NOTE: this is for illustration purposes only.
     The recommended way to construct a planning scene is to use the planning_scene_monitor 
     to construct it for you.*/
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  /* SETUP THE PLANNER*/
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name= "ompl_interface/OMPLPlanner";

  /* Get the name of the planner we want to use */
  //~ if (!node_handle.getParam("/planning_plugin", planner_plugin_name))
    //~ ROS_FATAL_STREAM("Could not find planner plugin name");  

  /* Make sure to catch all exceptions */
  
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");    
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(8.0);
  sleep_time.sleep();

  /* CREATE A MOTION PLAN REQUEST FOR THE RIGHT ARM OF THE PR2 */
  /* We will ask the end-effector of the PR2 to go to a desired location */
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

	req.group_name = "base";
	req.workspace_parameters.header.frame_id="odom_combined";
  req.workspace_parameters.min_corner.x=-5;
  req.workspace_parameters.min_corner.y=-5;
  req.workspace_parameters.min_corner.z=-2.5;
  req.workspace_parameters.max_corner.x=5;
  req.workspace_parameters.max_corner.y=5;
  req.workspace_parameters.max_corner.z=2.5;
  
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup("base");  
  
  /* Now, setup a joint space goal*/
	robot_state::RobotState goal_state(robot_model);
  robot_state::JointStateGroup* goal_group = goal_state.getJointStateGroup("base");

    
  std::vector<double> joint_values(3, 0.0);  
  joint_values[0] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[0];
  joint_values[1] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[1];
  joint_values[2] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[2];  
  //~ joint_values[0] = -2.0;
  //~ joint_values[1] = -0.2;
  //~ joint_values[2] = -0.15;
  goal_group->setVariableValues(joint_values);  
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_group);
  
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  /* CALL THE PLANNER */
  context->solve(res);
  /* Call the Planner */


  /* Check that the planning was successful */
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  //~ /* Visualize the generated plan */
  //~ /* Publisher for display */  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);  
  moveit_msgs::DisplayTrajectory display_trajectory;  

  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
	
	
	//~ geometry_msgs::PoseStamped pose;
  //~ pose.header.frame_id = "base_footprint";  
  //~ pose.pose.position.x = 0.75;
  //~ pose.pose.position.y = 0.0;
  //~ pose.pose.position.z = 0.0;  
  //~ pose.pose.orientation.w = 1.0;
  //~ pose = req_mobile_grasp_plan.goal_pose;
  //~ pose = newEEPose(req_mobile_grasp_plan);
  //~ /* A desired tolerance */
  //~ std::vector<double> tolerance_pose(3, 0.01);
  //~ std::vector<double> tolerance_angle(3, 0.01);
  //~ 
  //~ /*Create the request */
  req.group_name = "arm";
  //~ moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(req_mobile_grasp_plan.ik_link_name, pose, tolerance_pose, tolerance_angle);  
  //~ req.goal_constraints.push_back(pose_goal);     

	planning_interface::PlannerManagerPtr planner_instance_arm;
	try
  {
    planner_instance_arm.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance_arm->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance arm");    
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_arm->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }
	//~ 
	robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  joint_state_group = robot_state.getJointStateGroup("arm");  

  robot_state::RobotState goal_state1(robot_model);
  robot_state::JointStateGroup* goal_group1 = goal_state1.getJointStateGroup("arm"); 

  std::vector<double> joint_values1(7, 0.0);  
  joint_values1[0] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[3];
  joint_values1[1] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[4];
  joint_values1[2] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[5];
  joint_values1[3] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[6];
  joint_values1[4] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[7];
  joint_values1[5] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[8];
  joint_values1[6] = req_mobile_grasp_plan.joint_goal_pose.joint_state.position[9];
  //~ 
  //~ /*joint_values1[0] = -2.0;
  //~ joint_values1[3] = -0.2;
  //~ joint_values1[5] = -0.15;  */
  goal_group1->setVariableValues(joint_values1);  

  moveit_msgs::Constraints joint_goal1 = kinematic_constraints::constructGoalConstraints(goal_group1);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal1);
  ROS_INFO_STREAM("MotionPlanRequest \n" << req);

  planning_interface::PlanningContextPtr context_arm = planner_instance_arm->getPlanningContext(planning_scene, req, res.error_code_);

  /* CALL THE PLANNER */
  context_arm->solve(res);
  /* Call the Planner */


  //~ /* Call the Planner */
  //~ planner_instance_arm->solve(planning_scene, req, res);

  /* Check that the planning was successful */
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  res_mobile_grasp_plan.success=true;
	ROS_INFO("Visualizing the trajectory");  
  res.getMessage(response);  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  //Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);

	for(int i=0; i< display_trajectory.trajectory.size(); i++)
  res_mobile_grasp_plan.trajectory.push_back(display_trajectory.trajectory[i]);
  ROS_INFO("Done");  
  planner_instance.reset();
  planner_instance_arm.reset();
}
 
void setupEnvironment()
{
	ros::NodeHandle node_;
	ros::Duration(1.0).sleep();
	ROS_INFO("Setting up initial environment..");
	//~ ros::Publisher pub_co;
	//~ pub_co = node_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_footprint";
	
	// remove pole
	co.id = "pole";
	co.operation = co.REMOVE;
	pub_co.publish(co);

	// add pole
	co.id = "pole";
	co.operation = co.ADD;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = -0.7;
	co.primitive_poses[0].position.y = 0.4;  
	co.primitive_poses[0].position.z = 0.85;
	co.primitive_poses[0].orientation.w = 1.0;
	pub_co.publish(co);

	// remove table
	co.id = "table";
	co.operation = co.REMOVE;
	pub_co.publish(co);

	// add table
	co.id = "table";
	co.operation = co.ADD;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
	co.primitive_poses[0].position.x = -0.7;
	co.primitive_poses[0].position.y = -0.2;  
	co.primitive_poses[0].position.z = 0.175;
	pub_co.publish(co);
	ros::Duration(1.0).sleep();
}
 
void run()
{
	ROS_INFO("get_motion_plan_trajectory...spinning");
	ros::spin();
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "get_motion_plan");
  ros::NodeHandle node_handle_;
  pub_co = node_handle_.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  //~ CobPickPlaceActionServer* cob_pick_place_action_server;
  //~ cob_pick_place_action_server= new CobPickPlaceActionServer();
  //~ cob_pick_place_action_server->setupEnvironment();
  setupEnvironment();
  //~ CobPickPlaceActionServerPtr cob_pick_place_action_server;
  //~ cob_pick_place_action_server->setupEnvironment();
  ros::ServiceServer grasp_base_position_server = node_handle_.advertiseService("get_motion_plan_trajectory", mobileGraspPlanCallback);
	run();
	
  /* SETUP A PLANNING SCENE*/
  /* Load the robot model */
  
  
  return 0;  
}


