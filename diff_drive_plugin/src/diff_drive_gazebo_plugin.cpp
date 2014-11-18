#include "diff_drive_plugin/diff_drive_gazebo_plugin.h"
#include <boost/lexical_cast.hpp>


const int PI = 3.1415926535897931;

namespace gazebo
{   
    DiffDrivePlugin::DiffDrivePlugin()
    {
      std::string name = "diff_drive_gazebo_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }

    DiffDrivePlugin::~DiffDrivePlugin()
    {
      node_->shutdown();
      delete node_;
    }

    void DiffDrivePlugin::parseSDF(sdf::ElementPtr sdf){
    	control_topic_ = sdf->Get<std::string>("control");
    	if(control_topic_ == "")
    		control_topic_ = "/control_vector";
    	odom_topic_ = sdf->Get<std::string>("odom");
    	if(odom_topic_ == "")
    	   	odom_topic_ = "odom";
    	joint_states_topic_ = sdf->Get<std::string>("joint_states"); 
    	if(joint_states_topic_ == "")
    		joint_states_topic_= "joint_states";
    	//parse joints
    	r_num_joints_ = sdf->Get<int>("num_joints");
    	if(r_num_joints_ == 0)
    		r_num_joints_ = 4;
    	r_joint_states_.name.resize(r_num_joints_);
    	r_joint_states_.position.resize(r_num_joints_);
    	r_joint_states_.velocity.resize(r_num_joints_);
    	r_joint_states_.effort.resize(r_num_joints_);
    	for (int i = 0; i < r_num_joints_; ++i)
    	{
    		r_joint_states_.position[i] = 0;
    		r_joint_states_.velocity[i] = 0;
    		r_joint_states_.effort[i] = 0;
    	}
    	//left side
    	for(int i = 0; i < r_num_joints_/2; ++i){
    		std::string str_i = boost::lexical_cast<std::string>(i);
    		r_joint_states_.name[i] = sdf->Get<std::string>("left_wheel_" + str_i);
    	}
    	//right side
    	for(int i = r_num_joints_/2; i < r_num_joints_; ++i){
    		std::string str_i = boost::lexical_cast<std::string>(i);
    		r_joint_states_.name[i] = sdf->Get<std::string>("right_wheel_" + str_i);
    	}
    	r_w_sep_ = sdf->Get<double>("wheel_separation");
    	if(r_w_sep_ == *(new double()))
    		r_w_sep_ = 0.5;
    	r_w_rad_ = sdf->Get<double>("wheel_radius");
    	if(r_w_rad_ == *(new double()))
    		r_w_rad_ = 0.1;
    	r_w_torq_ = sdf->Get<double>("torque");
    	if(r_w_torq_ == *(new double()))
    		r_w_torq_ = 10.0;
    	r_max_lin_vel_ = sdf->Get<double>("max_velocity");
    	if(r_max_lin_vel_ == *(new double()))
    		r_max_lin_vel_ = 4.0;
    }

    void DiffDrivePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
	    model_ = parent;
    	if (!model_)
    	{
    		ROS_FATAL("GazeboRosControl need Model!");
    		return;
    	}
    	world_ = parent->GetWorld();
    	if (!world_)
    	{
    		ROS_FATAL("GazeboRosControl can't get world!");
    		return;
    	}
    	parseSDF(sdf);
    	r_joints_ = new physics::JointPtr[r_num_joints_];
    	for (int i = 0; i < r_num_joints_; ++i)
    	{
    		r_joints_[i] = model_->GetJoint(r_joint_states_.name[i]);
    		if (r_joints_[i] == NULL)
    			gzthrow("The controller couldn't get joint " << r_joint_states_.name[i]);
    	}
    	node_ = new ros::NodeHandle("~");
    	control_sub_ = node_->subscribe(control_topic_, 10,	&DiffDrivePlugin::callbackControl, this);
    	joint_state_pub_ = node_->advertise<sensor_msgs::JointState>(joint_states_topic_, 1);
    	base_link_ = model_->GetLink("base_link");
    	if (base_link_ == NULL)
    		gzthrow("The controller couldn't get base_link " << base_link_);
    	ROS_INFO("gazebo_ros_control plugin initialized");
    	update_connection_ = event::Events::ConnectWorldUpdateBegin(
    		boost::bind(&DiffDrivePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void DiffDrivePlugin::OnUpdate()
    {
      ros::spinOnce();
      publishJointStates();
    }

    void DiffDrivePlugin::callbackControl(const control_msgs::ControlVector &msg)
    {
        std::vector<double> control = msg.control;
        for (int i = 0; i < r_num_joints_; i++)
       	    r_joints_[i]->SetMaxForce(0, r_w_torq_);
        for (int i = 0; i < r_num_joints_; i++)
        	r_joints_[i]->SetVelocity(0, control[i]);
    }

    void DiffDrivePlugin::publishJointStates()
    {
      common::Time current_t = world_->GetSimTime();
      r_joint_states_.header.stamp.sec = current_t.sec;
      r_joint_states_.header.stamp.nsec = current_t.nsec;
     	for (int i = 0; i < r_num_joints_; ++i){
        r_joint_states_.position[i] = r_joints_[i]->GetAngle(0).Radian();
        r_joint_states_.velocity[i] = r_joints_[i]->GetVelocity(0);
      }
      joint_state_pub_.publish(r_joint_states_);
    }
}