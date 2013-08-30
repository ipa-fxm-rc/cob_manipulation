
//~ void BaseArmIKSolver::currentJointStatesCallback(const sensor_msgs::JointStateConstPtr &response, sensor_msgs::JointState &current_joint_states)
void currentJointStatesCallback(const sensor_msgs::JointStateConstPtr &response, sensor_msgs::JointState &current_joint_states)
{
	ROS_INFO("response->name: %d", response->name.size());
	for(int i=0; i<response->name.size();i++)
	{
		if ((std::find(current_joint_states.name.begin(), current_joint_states.name.end(), response->name[i].c_str()) == current_joint_states.name.end())||!current_joint_states.name.size())
		{
			current_joint_states.name.push_back(response->name[i]);
			current_joint_states.position.push_back(response->position[i]);
		}
	}
}


//~ sensor_msgs::JointState BaseArmIKSolver::InitialJointState("Do your required joints position filling here")
//~ sensor_msgs::JointState InitialJointState("Do your required joints position filling here")
sensor_msgs::JointState InitialJointState()
{
	std::string PLANNING_SCENE_TOPIC = "/joint_states";
	sensor_msgs::JointState current_joint_states;
	current_joint_states.name.clear();
	current_joint_states.position.clear();
	sub = node_handle_.subscribe<sensor_msgs::JointState> (PLANNING_SCENE_TOPIC, 10, boost::bind(currentJointStatesCallback, _1, boost::ref(current_joint_states) )); 
	ros::Duration(.5).sleep();
	ros::spinOnce();
	GetBasePosition(current_joint_states);
	ROS_INFO("sizeji: %d", current_joint_states.name.size());
	ROS_INFO_STREAM("Joint StatesName1: " <<current_joint_states);
}

//~ BaseArmIKSolver::GetBasePosition(sensor_msgs::JointState &base_joint_states)
void GetBasePosition(sensor_msgs::JointState &base_joint_states)
{
	tf::TransformListener listener;
	tf::StampedTransform transform_footprint_odom;
	try
	{
		listener.lookupTransform("/odom_combined", "/base_footprint", ros::Time::now(), transform_footprint_odom);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	geometry_msgs::Transform msg;
	tf::transformTFToMsg(transform_footprint_odom, msg);
	
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(msg.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	
	base_joint_states.name.pushback("base_placement_x_joint");
	base_joint_states.name.pushback("base_placement_y_joint");
	base_joint_states.name.pushback("base_placement_theta_joint");
	base_joint_states.position.push_back(msg.translation.x);
	base_joint_states.position.push_back(msg.translation.y);
	base_joint_states.position.push_back(yaw);
}
	
//~ BaseArmIKSolver::fillIKRequest()
void fillIKRequest()
{
	string group_name="base_arm";  //Maybe get it from request as well+++++++++You can give the joint names in request
	moveit_msgs/RobotState robot_state.jointstates=InitialJointState()
	//~ Constraints constraints
	bool avoid_collisions= true;
	string ik_link_name = "arm_7_link"//Maybe get it from request as well
	geometry_msgs/PoseStamped pose_stamped = "get from request";//Maybe get it from request as well
	duration timeout = "PUT TIMEOUT OR GET IT FROM REQUEST AS WELL";//Maybe get it from request as well
	int32 attempts = "PUT attempts OR GET IT FROM REQUEST AS WELL";//Maybe get it from request as well
}

//~ BaseArmIKSolver::baseArmIKCallback()
void baseArmIKCallback()
{
	fillIKRequest();
	callIKSolver();
	fillResponse();
}
	
//~ void BaseArmIKSolver::run()
void run()
{
	ROS_INFO("cob_pick_action...spinning");
	ros::spin();
}

void Init()
{
	std::string JOINT_STATE_TOPIC = "/joint_states";
	detect_object_client = gm.serviceClient<cob_object_detection_msgs::DetectObjects>("detect_object");
	grasp_select_client = gm.serviceClient<cob_grasp_selector::COBGetValidGrasp>("get_valid_grasp_server");
	ros::service::waitForService("detect_object");
	ros::service::waitForService("get_valid_grasp_server");
	provides_requested_object_grasp = gm.advertiseService("object_grasp_provider", &COBGraspManager::GetRequestedObjectGrasp, this);
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "get_joint_states");
	ros::NodeHandle node_handle_;
	//~ CobPickPlaceActionServer *base_arm_IKSolver = new BaseArmIKSolver();
	
	base_arm_IKSolver->initialize();
	base_arm_IKSolver->run();
	
	return 0;
}

	
	
	
