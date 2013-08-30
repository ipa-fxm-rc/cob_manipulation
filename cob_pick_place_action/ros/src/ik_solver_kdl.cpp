#include "pr2_arm_kinematics/pr2_arm_kinematics_plugin.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>
//~ #include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware/kdl_arm_kinematics_plugin.h>
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

using namespace std;
using namespace KDL;
KDL::Chain chain;
//~ std::string xml_string= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/model1.urdf");
//~ std::string xml_string1= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/cob3-3-base-planning.urdf.xacro");


void getKDLChainInfo(moveit_msgs::KinematicSolverInfo &chain_info)
{
	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();
	ROS_INFO("nj: %d  nl:%d", nj, nl);

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nl; i++)
  {
		//~ if(chain.getSegment(i).getJoint().getType()!=Joint::None)
		//~ {
			chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
			//~ ROS_INFO("chain_info.joint_name[%d]: %s", i, chain_info.joint_names[i].c_str());
		//~ }
  }
	//link_names	
	for(unsigned int i=0; i<nl; i++)
  {
		chain_info.link_names.push_back(chain.getSegment(i).getName());
		ROS_INFO("chain_info.link_names[%d]: %s", i, chain_info.link_names[i].c_str());
  }
}

int getJointIndex(const std::string &name,
                  const moveit_msgs::KinematicSolverInfo &chain_info)
  {
    for(unsigned int i=0; i < chain_info.joint_names.size(); i++)
    {
      if(chain_info.joint_names[i] == name)
      {
          return i;
      }
    }
    return -1;
  }

void map_reqJoint_chainJoint(moveit_msgs::GetPositionIK::Request  &req, moveit_msgs::KinematicSolverInfo chain_info)
{
	moveit_msgs::GetPositionIK::Request req2;
	
	for(int i=0; i<chain_info.joint_names.size(); i++)
	{
		for(int j=0; j<req.ik_request.robot_state.joint_state.name.size(); j++)
		{
			if (!strcmp(chain_info.joint_names[i].c_str(), req.ik_request.robot_state.joint_state.name[j].c_str()))
			{
			req2.ik_request.robot_state.joint_state.name.push_back(req.ik_request.robot_state.joint_state.name[j]);
			req2.ik_request.robot_state.joint_state.position.push_back(req.ik_request.robot_state.joint_state.position[j]);
			ROS_INFO("req2.joint_state.name.[%d]: %s", i, req.ik_request.robot_state.joint_state.name[j].c_str());
			}
		}
	}
	
	ROS_INFO_STREAM("Got_Match_req2_joint_state_info: "<<req2.ik_request.robot_state.joint_state);
	
	req.ik_request.robot_state.joint_state.name.clear();
	req.ik_request.robot_state.joint_state.position.clear();
	for(int k=0; k<req2.ik_request.robot_state.joint_state.name.size(); k++)
	{
		req.ik_request.robot_state.joint_state.name.push_back(req2.ik_request.robot_state.joint_state.name[k]);
		req.ik_request.robot_state.joint_state.position.push_back(req2.ik_request.robot_state.joint_state.position[k]);
	}
		ROS_INFO_STREAM("Got_Match_req_joint_state_info: "<<req.ik_request.robot_state.joint_state);
}

bool readJointLimits(moveit_msgs::GetPositionIK::Request  &req, JntArray &q_min, JntArray &q_max)
{
	urdf::Model robot_model;
	//~ if (!robot_model.initParam(xml_string)) {
	if (!robot_model.initParam("virtual_base_joint_robot_description")) {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }
    moveit_msgs::JointConstraint joint_constraint;
	//~ boost::shared_ptr<const urdf::Joint> joint;
	for(int i=0; i<req.ik_request.robot_state.joint_state.name.size(); i++)
	{
		
		ROS_INFO("Joint Name: %s", req.ik_request.robot_state.joint_state.name[i].c_str());
		//~ joint = robot_model.getJoint(chain_info.joint_names[i].c_str());
		boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(req.ik_request.robot_state.joint_state.name[i].c_str());
		
		//~ For the first 3 "virtual Joints" there is no soft joint limits########################
		//~ For rest of the joints real joints soft joint limits has to be taken into account###########################
		if(i<3)
		{
			q_min.data[i] = joint->limits->lower;
			q_max.data[i] = joint->limits->upper;
		}
		else
		{
			q_min.data[i] = joint->safety->soft_lower_limit;
			q_max.data[i] = joint->safety->soft_upper_limit;
		}
		joint_constraint.position=(q_min.data[i]+q_max.data[i])/2;
		joint_constraint.tolerance_above=q_max.data[i]-0.3;
		joint_constraint.tolerance_below=q_min.data[i]+0.3;
		joint_constraint.joint_name=req.ik_request.robot_state.joint_state.name[i];
		
		moveit_msgs::PositionConstraint position_constraint;
		position_constraint.header.frame_id="base_link";
		position_constraint.link_name="arm_7_link";
		shape_msgs::SolidPrimitive primitive ;
		primitive.type=primitive.BOX;  primitive.dimensions.resize(3);
		primitive.dimensions[0] = 0.5;
		primitive.dimensions[1] = 0.5;
		primitive.dimensions[2] = 0.5;  
		position_constraint.constraint_region.primitives.push_back(primitive);
		geometry_msgs::Pose primitive_pose;
		primitive_pose.position.x = -0.5;
		primitive_pose.position.y = -0.5;
		primitive_pose.position.z = 0.5;
		primitive_pose.orientation.w = 1;
		position_constraint.constraint_region.primitive_poses.push_back(primitive_pose);
		req.ik_request.constraints.position_constraints.push_back(position_constraint);
		
		
		
		//~ req.ik_request.constraints.joint_constraints.push_back(joint_constraint);
	}
	return true;

}

bool ik_solve(moveit_msgs::GetPositionIK::Request  &req,
         moveit_msgs::GetPositionIK::Response &res )
{
	moveit_msgs::KinematicSolverInfo chain_info;
	getKDLChainInfo(chain_info);
	
	map_reqJoint_chainJoint(req, chain_info);
	
	ROS_INFO("get_ik_service has been called!");

	unsigned int nj = chain.getNrOfJoints();

	JntArray q_min(nj);
	JntArray q_max(nj);

	if(readJointLimits(req, q_min, q_max)==true)
	ROS_INFO_STREAM("readJointLimits successful");
	else
	{ 
		for(int i = 0; i < nj; i+=2)
		{
			q_min(i) = -6.0;
			q_max(i) = 6.0;
		}
		for(int i = 1; i < nj; i+=2)
		{
			q_min(i) = -2.0;
			q_max(i) = 2.0;
		}
	}
	
	ROS_INFO_STREAM("GetPositionIK::Request:" << req.ik_request);

//~ lower = joint->limits->lower;
                //~ upper = joint->limits->upper;
                //~ hasLimits = 1;


	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	ChainIkSolverPos_NR_JL iksolverpos(chain, q_min, q_max, fksolver1,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	JntArray q(nj);
	JntArray q_init(nj);
	for(int i = 0; i < nj; i++)
	//~ moveit_msgs/getPositionIK.srv   **req.ik_request.ik_seed_state is replaced by req.ik_request.robot_state** 
		//~ q_init(i) = req.ik_request.ik_seed_state.joint_state.position[i];
		
		//~ ++++++++++++++++Mapping has to be done here++++++++++++++++++++++++++++++++++++++++
		
		
		q_init(i) = req.ik_request.robot_state.joint_state.position[i];
	Frame F_dest;
	Frame F_ist;
	Frame F_test;
	fksolver1.JntToCart(q_init, F_ist);
	
	//~ req.ik_request.pose_stamped is replaced by req.ik_request.pose_stamped** 
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	std::cout << "Getting Goal\n";
	std::cout << F_dest <<"\n";
	//~ std::cout << "Calculated Position out of Configuration:\n";
	std::cout << F_ist <<"\n";

	//uhr-fm: here comes the actual IK-solver-call -> could be replaced by analytical-IK-solver (cob)
	int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
	res.solution.joint_state.name = req.ik_request.robot_state.joint_state.name;
	res.solution.joint_state.position.resize(nj);
	if(ret < 0)
	{
		res.error_code.val = -1;
		ROS_INFO("Inverse Kinematic found no solution");
		std::cout << "RET: " << ret << std::endl;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q_init(i);
	}
	else
	{
		ROS_INFO("Inverse Kinematic found a solution");
		res.error_code.val = 1;
		for(int i = 0; i < nj; i++)	
			res.solution.joint_state.position[i] = q(i);
	}
	
	fksolver1.JntToCart(q, F_test);
	ROS_INFO("position found is: %f  %f  %f", F_test.p[0], F_test.p[1], F_test.p[2]);
	//std::cout << "q_init\n";
	ROS_DEBUG("q_init: %f %f %f %f %f %f %f", q_init(0), q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6));
	ROS_DEBUG("q_out: %f %f %f %f %f %f %f", q(0), q(1), q(2), q(3), q(4), q(5), q(6));		
	//std::cout << "Solved with " << ret << " as return\n";
	//std::cout << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	

	return true;
}

bool constraint_aware_ik_solve(moveit_msgs::GetConstraintAwarePositionIK::Request  &req,
         moveit_msgs::GetConstraintAwarePositionIK::Response &res )
{
	moveit_msgs::GetPositionIK::Request request;
	moveit_msgs::GetPositionIK::Response response;

	//transform GetConstraintAwarePositionIK-msgs to GetPositionIK-msgs
	request.ik_request=req.ik_request;
	bool success = ik_solve(request, response);

	res.solution=response.solution;
	res.error_code=response.error_code;

	return true;
}


bool getIKSolverInfo(moveit_msgs::GetKinematicSolverInfo::Request  &req,
         moveit_msgs::GetKinematicSolverInfo::Response &res )
{
	ROS_INFO("[TESTING]: get_ik_solver_info_service has been called!");

	getKDLChainInfo(res.kinematic_solver_info);

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_ik_solver");
	ros::NodeHandle n;
	//~ KDL::Tree my_tree;
	ros::NodeHandle node;

	KDL::Tree robot_kin;
	//~ kdl_parser::treeFromFile(xml_string, robot_kin);
	kdl_parser::treeFromParam("virtual_base_joint_robot_description", robot_kin);
	//~ KDL::Chain chain;
	robot_kin.getChain("base_placement_odom_link", "arm_7_link", chain);
	
	ros::ServiceServer get_ik_service = n.advertiseService("get_ik", ik_solve);
	ros::ServiceServer get_constraint_aware_ik_service = n.advertiseService("get_constraint_aware_ik", constraint_aware_ik_solve);
	ros::ServiceServer get_ik_solver_info_service = n.advertiseService("get_ik_solver_info", getIKSolverInfo);
	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}


