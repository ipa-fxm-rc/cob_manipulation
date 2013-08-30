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

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
 
int main(int argc, char **argv)
{
  ros::init (argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");  

  /* SETUP A PLANNING SCENE*/
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  /* Construct a planning scene - NOTE: this is for illustration purposes only.
     The recommended way to construct a planning scene is to use the planning_scene_monitor 
     to construct it for you.*/
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  /* SETUP THE PLANNER*/
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader;
  planning_interface::PlannerPtr planner_instance;
  std::string planner_plugin_name= "ompl_interface/OMPLPlanner";

  /* Get the name of the planner we want to use */
  //~ if (!node_handle.getParam("/planning_plugin", planner_plugin_name))
    //~ ROS_FATAL_STREAM("Could not find planner plugin name");  

  /* Make sure to catch all exceptions */
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::Planner>("moveit_core", "planning_interface::Planner"));
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

	req.group_name = "base_arm";
	req.workspace_parameters.header.frame_id="odom_combined";
  req.workspace_parameters.min_corner.x=-2.5;
  req.workspace_parameters.min_corner.y=-2.5;
  req.workspace_parameters.min_corner.z=-2.5;
  req.workspace_parameters.max_corner.x=2.5;
  req.workspace_parameters.max_corner.y=2.5;
  req.workspace_parameters.max_corner.z=2.5;
  
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  //~ planning_scene->setCurrentState(response.trajectory_start);
  robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup("base_arm");  
  //~ joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  

  /* Now, setup a joint space goal*/
	robot_state::RobotState goal_state(robot_model);
  robot_state::JointStateGroup* goal_group = goal_state.getJointStateGroup("base_arm");

    
  std::vector<double> joint_values(10, 0.0);  
  joint_values[0] = -2.0;
  joint_values[1] = -0.2;
  joint_values[2] = -0.15;  
  joint_values[4] = -2.0;
  joint_values[5] = -0.2;
  joint_values[6] = -0.15;  
  goal_group->setVariableValues(joint_values);  
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_group);
  
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  
  /* Call the Planner */
  planner_instance->solve(planning_scene, req, res);

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
//~ 
  //~ /* Visualize the trajectory */
  //~ ROS_INFO("Visualizing the trajectory");  
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();

 //~ ROS_INFO("OK 1");
	//~ robot_state = planning_scene->getCurrentStateNonConst();
  //~ planning_scene->setCurrentState(response.trajectory_start);
   //~ ROS_INFO("OK 2");
  //~ joint_state_group = robot_state.getJointStateGroup("arm");  
  //joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  
  //~ ROS_INFO("OK 3");
 //~ 
 //~ 
//~ 
  //~ robot_state::RobotState goal_state1(robot_model);
  //~ robot_state::JointStateGroup* goal_group1 = goal_state1.getJointStateGroup("arm"); 
   //~ ROS_INFO("OK 4"); 
  //~ std::vector<double> joint_values1(7, 0.0);  
  //~ joint_values1[0] = -2.0;
  //~ joint_values1[3] = -0.2;
  //~ joint_values1[5] = -0.15;  
  //~ goal_group1->setVariableValues(joint_values1);  
   //~ ROS_INFO("OK 5");
  //~ moveit_msgs::Constraints joint_goal1 = kinematic_constraints::constructGoalConstraints(goal_group1);
   //~ ROS_INFO("OK 6");
  //~ req.goal_constraints.clear();
  //~ req.goal_constraints.push_back(joint_goal1);
   //~ ROS_INFO("OK 7");
  //~ /* Call the Planner */
  //~ planner_instance->solve(planning_scene, req, res);
   //~ ROS_INFO("OK 8");
  //~ /* Check that the planning was successful */
  //~ if(res.error_code_.val != res.error_code_.SUCCESS)
  //~ {
    //~ ROS_ERROR("Could not compute plan successfully");
    //~ return 0;
  //~ }
	//~ ROS_INFO("Visualizing the trajectory");  
  //~ res.getMessage(response);  
//~ 
  //~ display_trajectory.trajectory_start = response.trajectory_start;
  //~ display_trajectory.trajectory.push_back(response.trajectory);  
  //~ //Now you should see two planned trajectories in series
  //~ display_publisher.publish(display_trajectory);
//~ 
 //~ 
  ROS_INFO("Done");  
  planner_instance.reset();
  
  return 0;  
}
