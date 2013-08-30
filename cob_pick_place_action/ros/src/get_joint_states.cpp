#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <string>
//~ KDL::Chain chain;
ros::Subscriber sub;
void chatterCallback(const sensor_msgs::JointStateConstPtr &response, sensor_msgs::JointState &j1)
{
	ROS_INFO("response->name: %d", response->name.size());

	for(int i=0; i<response->name.size();i++)
	{
	if ((std::find(j1.name.begin(), j1.name.end(), response->name[i].c_str()) == j1.name.end())||!j1.name.size())
	{
		j1.name.push_back(response->name[i]);
		j1.position.push_back(response->position[i]);
	}

	//~ for(int i=0; i<response->name.size();i++)
	//~ {
  //~ ROS_INFO_STREAM("Joint States1: " <<response->name[i]);
  //~ ROS_INFO_STREAM("Joint States1: " <<response->position [i]);
  //~ ROS_INFO_STREAM("Joint States1: " << response->getVariableNames());
  //~ j1.name.push_back(response->name[i]);
  //~ j1.position.push_back(response->position[i]);
  //~ j1.name[i]=response->position[i];
  //~ ROS_INFO_STREAM("Joint StatesName1: " <<j1.name[i]);
  //~ ROS_INFO_STREAM("Joint States1Position: " <<j1.position[i]);
	}
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "get_joint_states");
	ros::NodeHandle node_handle_;
	std::string PLANNING_SCENE_TOPIC = "/joint_states";
	//~ ros::Subscriber sub;
sensor_msgs::JointState j1, j2;
	//~ ROS_INFO("sub already avaiblale");

	//~ sub = node_handle_.subscribe(PLANNING_SCENE_TOPIC, 1000, 
	//~ sub = node_handle_.subscribe(PLANNING_SCENE_TOPIC, 1000, boost::bind(chatterCallback, _1, boost::ref(j1)) );
	
	 sub = node_handle_.subscribe<sensor_msgs::JointState> (PLANNING_SCENE_TOPIC, 10, boost::bind(chatterCallback, _1, boost::ref(j1) )); 
	
	
	//~ ROS_INFO_STREAM("Joint States: " );
	//~ ros::spinOnce();
	//~ ros::ServiceClient planning_scene_service_;
	//~ std::string GET_PLANNING_SCENE_SERVICE_NAME = "get_planning_scene";
	//~ moveit_msgs::GetPlanningScene::Request request;
    //~ moveit_msgs::GetPlanningScene::Response response;
	//~ planning_scene_service_ = node_handle_.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE_NAME);
	//~ request.components.components = request.components.ROBOT_STATE;
	//~ if (!planning_scene_service_.call(request, response))
		//~ ROS_INFO("planning_scene_not_recieved");
	//~ else 
		//~ ROS_INFO_STREAM("Joint States: " << response.scene.robot_state.joint_state);
			ros::Duration(.5).sleep();
	ros::spinOnce();
	ROS_INFO("sizeji: %d", j1.name.size());
	
	ROS_INFO_STREAM("Joint StatesName1: " <<j1);
	//~ for(int i=0; i<j1.name.size();i++)
	//~ {
		//~ if(j2.name.size())
		//~ for(int j=0; j<j2.name.size();j++)
			//~ {
				//~ if(j1.name[i]!=j1.name[j])
				//~ {
				//~ j2.name.push_back(j1.name[i]);
				//~ j2.position.push_back(j1.position[i]);
	     		//~ }
			//~ }
	//~ }
	//~ ROS_INFO("sizeji: %d", j2.name.size());

//~ while(!sub);
//~ 
	//~ ros::Duration(.5).sleep();
	return 0;
}

