#ifndef GROUND_TRUTH_PUBLISHER_PLUGIN_H_
#define GROUND_TRUTH_PUBLISHER_PLUGIN_H_

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{   
  class GroundTruthPublisher : public ModelPlugin
  {
  public:
    GroundTruthPublisher();
    ~GroundTruthPublisher();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void OnUpdate();
    void publishGroundTruth();
  private:
    event::ConnectionPtr update_connection_; // Pointer to the update event connection
    ros::NodeHandle* node_;  // ROS Nodehandle
    ros::Publisher ground_truth_pose_pub_;
    tf::TransformBroadcaster tf_br_;
    double rate_;
    //r_ - robot 
    std::string ground_truth_pose_;
    physics::WorldPtr world_; // pointers to the model and world  
    physics::ModelPtr model_;
    physics::LinkPtr base_link_;
    ros::Time time_;
    bool pub_flag_;
    bool tf_flag_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GroundTruthPublisher)
}

#endif