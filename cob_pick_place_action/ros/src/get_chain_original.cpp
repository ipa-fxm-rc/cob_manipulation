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

using namespace std;
using namespace KDL;
KDL::Chain chain;



void getKDLChainInfo(moveit_msgs::KinematicSolverInfo &chain_info)
{
	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();
	ROS_INFO("nj: %d  nl:%d", nj, nl);

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nj; i++)
  {
		chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
		ROS_INFO("chain_info.joint_name[%d]: %s", i, chain_info.joint_names[i].c_str());
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

void map_reqJoint_chainJoint(moveit_msgs::GetPositionIK::Request  &req)
{
	moveit_msgs::GetPositionIK::Request req2;
	moveit_msgs::KinematicSolverInfo chain_info;
	getKDLChainInfo(chain_info);
	
	for(int i=0; i<chain_info.joint_names.size(); i++)
	{
		for(int j=0; j<req.ik_request.robot_state.joint_state.name.size(); j++)
		{
			if (!strcmp(chain_info.joint_names[i].c_str(), req.ik_request.robot_state.joint_state.name[j].c_str()))
			{
			req2.ik_request.robot_state.joint_state.name.push_back(req.ik_request.robot_state.joint_state.name[j]);
			req2.ik_request.robot_state.joint_state.position.push_back(req.ik_request.robot_state.joint_state.position[j]);
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

bool ik_solve(moveit_msgs::GetPositionIK::Request  &req,
         moveit_msgs::GetPositionIK::Response &res )
{
	map_reqJoint_chainJoint(req);
	ROS_INFO_STREAM("GetPositionIK::Request:" << req.ik_request);
	ROS_INFO("get_ik_service has been called!");

	unsigned int nj = chain.getNrOfJoints();

	JntArray q_min(nj);
	JntArray q_max(nj);
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
	fksolver1.JntToCart(q_init, F_ist);
	
	//~ req.ik_request.pose_stamped is replaced by req.ik_request.pose_stamped** 
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	std::cout << "Getting Goal\n";
	std::cout << F_dest <<"\n";
	std::cout << "Calculated Position out of Configuration:\n";
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
	std::string xml_string= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/model1.urdf");
	kdl_parser::treeFromFile(xml_string, robot_kin);

	//~ KDL::Chain chain;
	robot_kin.getChain("base_placement_odom_link", "arm_7_link", chain);
	
	ros::ServiceServer get_ik_service = n.advertiseService("get_ik", ik_solve);
	ros::ServiceServer get_constraint_aware_ik_service = n.advertiseService("get_constraint_aware_ik", constraint_aware_ik_solve);
	ros::ServiceServer get_ik_solver_info_service = n.advertiseService("get_ik_solver_info", getIKSolverInfo);
	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}




/*
#include "pr2_arm_kinematics/pr2_arm_kinematics_plugin.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <kdl_parser/kdl_parser.hpp>

#include "ros/ros.h"
//~ #include "moveit_msgs/GetPositionIK.h"
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


using namespace std;
using namespace KDL;
KDL::Chain chain;



void getKDLChainInfo(moveit_msgs::KinematicSolverInfo &chain_info)
{
	unsigned int nj = chain.getNrOfJoints();
	unsigned int nl = chain.getNrOfSegments();

	//---setting up response

	//joint_names
	for(unsigned int i=0; i<nj; i++)
  {
		chain_info.joint_names.push_back(chain.getSegment(i).getJoint().getName());
  }
	//link_names	
	for(unsigned int i=0; i<nl; i++)
  {
		chain_info.link_names.push_back(chain.getSegment(i).getName());
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


bool ik_solve(moveit_msgs::GetPositionIK::Request  &req,
         moveit_msgs::GetPositionIK::Response &res )
{
	ROS_INFO("get_ik_service has been called!");

	unsigned int nj = chain.getNrOfJoints();

	JntArray q_min(nj);
	JntArray q_max(nj);
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


	ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	ChainIkSolverPos_NR_JL iksolverpos(chain, q_min, q_max, fksolver1,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	JntArray q(nj);
	JntArray q_init(nj);
	for(int i = 0; i < nj; i++)
	//~ moveit_msgs/getPositionIK.srv   **req.ik_request.ik_seed_state is replaced by req.ik_request.robot_state** 
		//~ q_init(i) = req.ik_request.ik_seed_state.joint_state.position[i];
		q_init(i) = req.ik_request.robot_state.joint_state.position[i];
	Frame F_dest;
	Frame F_ist;
	fksolver1.JntToCart(q_init, F_ist);
	
	//~ req.ik_request.pose_stamped is replaced by req.ik_request.pose_stamped** 
	tf::PoseMsgToKDL(req.ik_request.pose_stamped.pose, F_dest);
	std::cout << "Getting Goal\n";
	std::cout << F_dest <<"\n";
	std::cout << "Calculated Position out of Configuration:\n";
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
	std::string xml_string= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/model.urdf");
	kdl_parser::treeFromFile(xml_string, robot_kin);

	//~ KDL::Chain chain;
	robot_kin.getChain("base_footprint", "arm_7_link", chain);
	
	ros::ServiceServer get_ik_service = n.advertiseService("get_ik", ik_solve);
	ros::ServiceServer get_constraint_aware_ik_service = n.advertiseService("get_constraint_aware_ik", constraint_aware_ik_solve);
	ros::ServiceServer get_ik_solver_info_service = n.advertiseService("get_ik_solver_info", getIKSolverInfo);
	//~ ros::ServiceServer get_fk_service = n.advertiseService("get_fk", fk_solve);
	//~ ros::ServiceServer get_fk_tcp_service = n.advertiseService("get_fk_tcp", fk_solve_TCP);
	//~ ros::ServiceServer get_fk_all_service = n.advertiseService("get_fk_all", fk_solve_all);
	//~ ros::ServiceServer get_fk_solver_info_service = n.advertiseService("get_fk_solver_info", getFKSolverInfo);


	ROS_INFO("IK Server Running.");
	ros::spin();

	return 0;
}








/*
int main (int argc, char **argv)
{
	
	
	KDL::Tree robot_kin;
	std::string xml_string= ros::package::getPath("cob_hardware_config")+std::string("/cob3-3-base-planning/urdf/model.urdf");
	kdl_parser::treeFromFile(xml_string, robot_kin);

	KDL::Chain chain;
	robot_kin.getChain("base_footprint", "arm_7_link", chain);

	unsigned int nj = chain.getNrOfJoints();

//~ KDL::JntArray joint_pos(chain.getNrOfJoints());

	KDL::JntArray q_min(nj);
	KDL::JntArray q_max(nj);
	
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
	
	//~ ChainIkSolverPos_NR_JL iksolverpos(chain, q_min, q_max, fksolver1,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
//~ KDL::Frame cart_pos;
//~ KDL::ChainFkSolverPos_recursive fk_solver(chain);
//~ fk_solver.JntToCart(joint_pos, cart_pos);
//~ 
//~ std::string root_name = "base_footprint";
//~ std::string tip_name= "cob3-3.urdf";
//~ KDL::Chain kdl_chain;

//~ if(pr2_arm_kinematics::getKDLChain(xml_string, root_name, tip_name, kdl_chain)== true)
   ROS_INFO("chain found");
  return 0;
}
* 
* */
