#include "ground_truth_publisher_plugin/ground_truth_publisher_plugin.h"


const int PI = 3.1415926535897931;

namespace gazebo
{   
    GroundTruthPublisher::GroundTruthPublisher()
    {
      std::string name = "ground_truth_publisher_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }

    GroundTruthPublisher::~GroundTruthPublisher()
    {
      node_->shutdown();
      delete node_;
    }

    void GroundTruthPublisher::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
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
      ground_truth_pose_ = sdf->Get<std::string>("ground_truth_topic");
      if(ground_truth_pose_ == "")
          ground_truth_pose_ = "ground_truth_pose";
      pub_flag_ = sdf->Get<bool>("pubish_flag");
      tf_flag_ = sdf->Get<bool>("tf_flag");
      rate_ = sdf->Get<double>("rate");
      if(rate_ == double())
          rate_ = 50;
    	node_ = new ros::NodeHandle("~");
      if(pub_flag_)
        ground_truth_pose_pub_ = node_->advertise<nav_msgs::Odometry>(ground_truth_pose_, 1);
    	base_link_ = model_->GetLink("base_link");
    	if (base_link_ == NULL)
    		gzthrow("The controller couldn't get base_link " << base_link_);
    	ROS_INFO("gazebo_ros_control plugin initialized");
      time_ = ros::Time::now();
    	update_connection_ = event::Events::ConnectWorldUpdateBegin(
    		boost::bind(&GroundTruthPublisher::OnUpdate, this));
    }

    // Called by the world update start event
    void GroundTruthPublisher::OnUpdate()
    {
      ros::spinOnce();
      ros::Time t = ros::Time::now();
      if(1/rate_ <= (t - time_).toSec()){
        time_ = t;
        publishGroundTruth();
      }
    }

    void GroundTruthPublisher::publishGroundTruth()
    {
      if(!pub_flag_)
      {
        return;
      }
        
      nav_msgs::Odometry odom;
      ros::Time current_time = ros::Time::now();
      odom.header.stamp = current_time;
      odom.header.frame_id = "ground_truth_pose";
      odom.child_frame_id = "base_link";

      math::Pose base_link_pose = base_link_->GetWorldPose();
      odom.pose.pose.position.x = base_link_pose.pos.x;
      odom.pose.pose.position.y = base_link_pose.pos.y;
      odom.pose.pose.position.z = base_link_pose.pos.z;
      odom.pose.pose.orientation.x = base_link_pose.rot.x;
      odom.pose.pose.orientation.y = base_link_pose.rot.y;
      odom.pose.pose.orientation.z = base_link_pose.rot.z;
      odom.pose.pose.orientation.w = base_link_pose.rot.w;
      for(int i = 0; i < 6; i++)
        odom.pose.covariance[i*6+i] = 0.1;

      math::Vector3 linear_vel = base_link_->GetWorldLinearVel(); 
      math::Vector3 angular_vel = base_link_->GetWorldAngularVel(); 
      odom.twist.twist.linear.x = sqrt(linear_vel.x * linear_vel.x + linear_vel.y*linear_vel.y); 
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = linear_vel.z;
      odom.twist.twist.angular.x = angular_vel.x;
      odom.twist.twist.angular.y = angular_vel.y;
      odom.twist.twist.angular.z = angular_vel.z;
      ground_truth_pose_pub_.publish(odom);

      if(!tf_flag_)
      {
        return;
      }
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = current_time;
      tf_msg.header.frame_id = "ground_truth_pose";
      tf_msg.child_frame_id = "base_link";
      tf_msg.transform.translation.x = odom.pose.pose.position.x;
      tf_msg.transform.translation.y = odom.pose.pose.position.y;
      tf_msg.transform.translation.z = odom.pose.pose.position.z;
      tf_msg.transform.rotation.x = odom.pose.pose.orientation.x;
      tf_msg.transform.rotation.y = odom.pose.pose.orientation.y;
      tf_msg.transform.rotation.z = odom.pose.pose.orientation.z;
      tf_msg.transform.rotation.w = odom.pose.pose.orientation.w;
      tf_br_.sendTransform(tf_msg);
    }
}