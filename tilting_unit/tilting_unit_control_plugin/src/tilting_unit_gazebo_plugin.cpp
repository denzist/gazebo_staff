#include "tilting_unit_control_plugin/tilting_unit_gazebo_plugin.h"
#include <boost/lexical_cast.hpp>

const int PI = 3.1415926535897931;

namespace gazebo
{   
    TiltUnitPlugin::TiltUnitPlugin()
    {
      std::string name = "diff_drive_gazebo_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }

    TiltUnitPlugin::~TiltUnitPlugin()
    {
      node_->shutdown();
      delete node_;
    }

    void TiltUnitPlugin::parseSDF(sdf::ElementPtr sdf)
    {
    	joint_states_topic_ = sdf->Get<std::string>("joint_state_topic_name"); 
    	if(joint_states_topic_ == "")
    		joint_states_topic_= "joint_states";
      //tilting unit joint
    	tilting_unit_joint_state_.name.resize(1);
    	tilting_unit_joint_state_.position.resize(1);
    	tilting_unit_joint_state_.velocity.resize(1);
    	tilting_unit_joint_state_.effort.resize(1);
    	tilting_unit_joint_state_.position[0] = 0.0;
    	tilting_unit_joint_state_.velocity[0] = 0.0;
    	tilting_unit_joint_state_.effort[0] = 0.0;
    	tilting_unit_joint_state_.name[0] = sdf->Get<std::string>("tilting_unit_joint");
      if(tilting_unit_joint_state_.name[0] == "")
        tilting_unit_joint_state_.name[0] = "rot_elem_joint";
      //physical params
    	theta_max_ = sdf->Get<double>("theta_max");
    	if(theta_max_ == *(new double()))
    		theta_max_ = PI/3;
    	theta_min_ = sdf->Get<double>("theta_min");
    	if(theta_min_ == *(new double()))
    		theta_min_ = -PI/6;
    	period_ = sdf->Get<double>("period");
    	if(period_ == *(new double()))
    		period_ = 1.0;
    	rot_speed_ = 2*(theta_max_ - theta_min_)/period_;
      return;
    }

    void TiltUnitPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
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
    	tilting_unit_joint_ = model_->GetJoint(tilting_unit_joint_state_.name[0]);
    	if (tilting_unit_joint_ == NULL)
    		gzthrow("The controller couldn't get joint " << tilting_unit_joint_state_.name[0]);

      tilting_unit_joint_->SetAngle(0, gazebo::math::Angle::Zero);
    	node_ = new ros::NodeHandle("~");
    	joint_state_pub_ = node_->advertise<sensor_msgs::JointState>(joint_states_topic_, 1);
    	update_connection_ = event::Events::ConnectWorldUpdateBegin(
    		boost::bind(&TiltUnitPlugin::OnUpdate, this));
      tilting_unit_joint_->SetAngle(0, gazebo::math::Angle::Zero);
      ROS_INFO_STREAM("tilting_unit_gazebo_plugin loaded!");
    }

    // Called by the world update start event
    void TiltUnitPlugin::OnUpdate()
    {
      updateJointState();
      publishJointStates();
    }

    void TiltUnitPlugin::updateJointState()
    {
      if(theta_max_ == theta_min_)
      {
        tilting_unit_joint_->SetAngle(0, gazebo::math::Angle::Zero);
        return;
      }
      double joint_angle = tilting_unit_joint_->GetAngle(0).Radian();
      //ROS_INFO_STREAM("joint angle " << joint_angle / PI * 180);
      if((joint_angle >= theta_max_ && rot_speed_ > 0.0) ||
        (joint_angle <= theta_min_ && rot_speed_ < 0.0))
        rot_speed_ = -1*rot_speed_;
      tilting_unit_joint_->SetMaxForce(0, 5.0);
      tilting_unit_joint_->SetVelocity(0, rot_speed_);
      //tilting_unit_joint_->SetAngle(0, gazebo::math::Angle::Zero);
      return;
    }

    void TiltUnitPlugin::publishJointStates()
    {
      common::Time current_t = world_->GetSimTime();
      tilting_unit_joint_state_.header.stamp.sec = current_t.sec;
      tilting_unit_joint_state_.header.stamp.nsec = current_t.nsec;
      tilting_unit_joint_state_.position[0] = tilting_unit_joint_->GetAngle(0).Radian();
      tilting_unit_joint_state_.velocity[0] = tilting_unit_joint_->GetVelocity(0);
      joint_state_pub_.publish(tilting_unit_joint_state_);
    }
}