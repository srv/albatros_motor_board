#!/usr/bin/env python

import roslib; roslib.load_manifest('albatros_motor_board')
import rospy
from auv_sensor_msgs.msg import Pressure, Depth

class PressureToDepthNode():
  """
  Node for calculating depth based on pressure,
  subscribes to pressure, publishes depth

  Meant to be run in the namespace of the pressure publishing
  node to provide a depth topic in the same namespace
  """
  def __init__(self):
    self.pub = rospy.Publisher('depth', Depth)
    self.k = 0.068046
    self.offset = rospy.get_param('~offset', 0.0)
    rospy.Subscriber('pressure', Pressure, self.callback)
    rospy.spin()

  def callback(self, pressure):
    depth = pressure.pressure / 100 * self.k - 10 + self.offset
    self.pub.publish(pressure.header, depth)

if __name__ == "__main__":
  rospy.init_node('pressure_to_depth')
  try:
    node = PressureToDepthNode()
  except rospy.ROSInterruptException:
    pass
