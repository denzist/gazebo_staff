#ifndef TILTING_UNIT_GAZEBO_PLUGIN_H_
#define TILTING_UNIT_GAZEBO_PLUGIN_H_
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{   
  class TiltUnitPlugin : public ModelPlugin
  {
  public:
    TiltUnitPlugin();
    ~TiltUnitPlugin();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    void OnUpdate();
    void updateJointState();
    void publishJointStates();
    void parseSDF(sdf::ElementPtr sdf);
  private:
    event::ConnectionPtr update_connection_; // Pointer to the update event connection
    ros::NodeHandle* node_;  // ROS Nodehandle
    ros::Publisher joint_state_pub_; // ROS Subscribers and Publishers

    std::string joint_states_topic_;

    double theta_max_; //[rad]
    double theta_min_; //[rad]
    double period_; //[sec]
    double rot_speed_; //[rad/sec]
    
    physics::WorldPtr world_; // pointers to the model and world  
    physics::ModelPtr model_;
    physics::JointPtr tilting_unit_joint_;
    sensor_msgs::JointState tilting_unit_joint_state_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TiltUnitPlugin)
}

#endif