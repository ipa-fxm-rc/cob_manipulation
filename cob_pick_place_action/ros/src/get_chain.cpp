

#include "pr2_arm_kinematics/pr2_arm_kinematics_plugin.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

#include "ros/ros.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/GetPositionFK.h"
#include "moveit_msgs/GetConstraintAwarePositionIK.h"
#include "moveit_msgs/GetKinematicSolverInfo.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <string>




int main (int argc, char **argv)
{
	moveit_msgs::KinematicSolverInfo chain_info;
	
	KDL::Tree robot_kin;
	std::string xml_string= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/model1.urdf");
	kdl_parser::treeFromFile(xml_string, robot_kin);

	KDL::Chain chain;
	robot_kin.getChain("base_placement_odom_link", "arm_7_link", chain);

	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();
	ROS_INFO("nj: %d  nl:%d", nj, nl);

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nj; i++)
  {
		if(segment.getJoint().getType()!=Joint::None)
		chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
		ROS_INFO("chain_info.joint_name[%d]: %s", i, chain_info.joint_names[i].c_str());
  }
	//link_names	
	for(unsigned int i=0; i<nl; i++)
  {
		chain_info.link_names.push_back(chain.getSegment(i).getName());
		ROS_INFO("chain_info.link_names[%d]: %s", i, chain_info.link_names[i].c_str());
  }
   ROS_INFO("chain found");
  return 0;
}

