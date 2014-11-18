#!/usr/bin/env python
import rospy
import roslib
import tf
import numpy as np

# Messages
from geometry_msgs.msg import Twist
from control_msgs.msg import ControlVector

roslib.load_manifest('buggy_controller')

class Controller(object):
  def __init__(self, node_name='robot_controller', cmd_vel_topic='/gazebo/cmd_vel', control_topic='/gazebo/control_vector'):
    self.cmd_vel_topic = cmd_vel_topic
    self.control_topic = control_topic
    self.node_name = node_name
    self.enhancing_matrix = np.identity(2)
    self.kinematic_matrix = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, -1.0, 1.0, 1.0]])
    self.enhancing_matrix = np.array([[10.0, 0.0], [0.0, 3.16]])


  def control(self):
    rospy.init_node(self.node_name)
    if rospy.has_param('~cmd_vel'):
        self.cmd_vel_topic = rospy.get_param('~cmd_vel')
    if rospy.has_param('~control'):
        self.control_topic = rospy.get_param('~control')

    self.publisher = rospy.Publisher(self.control_topic, ControlVector, queue_size=10)
    while not rospy.is_shutdown():
      vel = self.vel_to_list(rospy.wait_for_message(self.cmd_vel_topic, Twist))
      vel = [vel[0], vel[5]]
      enh_vel = np.dot(np.array(vel), self.enhancing_matrix).tolist()
      control = np.dot(enh_vel, self.kinematic_matrix).tolist()
      msg = ControlVector()
      msg.header.stamp = rospy.get_rostime()
      msg.length = len(control)
      msg.control = control
      self.publisher.publish(msg)

  def vel_to_list(self, vel):
    return [vel.linear.x, vel.linear.y, vel.linear.z,
    vel.angular.x, vel.angular.y, vel.angular.z]

if __name__ == '__main__':
	controller = Controller();
	try:
		controller.control()
	except rospy.ROSInterruptException:
		pass
