/** @file
 *
 * @brief ROS driver for Albatros motor board (node version).
 *
 * This is a ROS driver for a motor board provided by Albatros.
 * It uses a library created for that device, that exposes its modules
 * (pressure and water-in sensors, and two motor pairs with its built-in PID controllers)
 * and handles communications through the serial port.
 *
 * @par Advertises
 *
 * - @b speeds_rpm topic (srv_msgs/MotorLevels)
 *   current motor speeds in rpm
 *
 * - @b status topic (albatros_motorboard/MotorStatus)
 *   motor error counts
 *
 * - @b pressure topic (srv_msgs/Pressure)
 *   pressure sensor sample
 *
 * @par Subscribes
 *
 * - @b speeds_pc (srv_msgs/MotorLevels)
 *   desired speeds in percentage of device nominal speed
 *
 * @par Parameters
 *
 * - @b ~port  device file name for the serial port (default "/dev/ttyS0")
 * - @b ~rate  state (pressure, speeds and status) publishing rate (default 10 hz)
 * - @b ~sensors/pressure/offset offset for the pressure sensor (default 0)
 * - @b ~motors/(front|down)/(left|right)/accel motor acceleration (% per dsec, default 5 %)
 * - @b ~motors/(front|down)/(left|right)/PID/active motor PID controller state (boolean default false)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID proportional constant (double default 0.0)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID integral constant (double default 0.0)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID derivative constant (double default 0.0)
 */

#include "motorboard_node_base.h"

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "motorboard_node");

  ros::NodeHandle node;
  ros::NodeHandle priv("~");
  albatros_motorboard::MotorBoardNodeBase mb_node(node,priv);

  // advertise output topics
  mb_node.advertiseSensorTopics();
  mb_node.advertiseMotorTopics();

  // node parameters
  mb_node.initDynParamsSrv();

  // subscribe input topics
  mb_node.subscribeMotorTopics();

  ros::spin();

  // clean resources on exit
  mb_node.cleanUp();

  return 0;
}
