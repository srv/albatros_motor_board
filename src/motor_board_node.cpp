/**
 * @file
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
 * - @b status topic (albatros_motor_board/MotorStatus)
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
 * - \b "~rate_speeds"   : \b [double] Publishing rate for motor speeds topic (0.0 disables publishing) min: 0.0, default: 10.0, max: 100.0
 * - \b "~rate_status"   : \b [double] Publishing rate for motor_status topic (0.0 disables publishing) min: 0.0, default: 10.0, max: 100.0
 * - \b "~rate_pressure" : \b [double] Publishing rate for pressure topic (0.0 disables publishing) min: 0.0, default: 10.0, max: 100.0
 * - \b "~serial_port" : \b [str] Serial port device file name (including full path) min: , default: /dev/ttyS0, max: 
 * - \b "~pressure_offset" : \b [int] Pressure sensor offset. min: -32768, default: 0, max: 32767
 * - \b "~waterin_offset"  : \b [int] Water in sensor offset. min: -32768, default: 0, max: 32767
 * - \b "~(forward|downward)_(left|right)_PID_on" : \b [bool]   front/down left/right PID state. min: False, default: False, max: True
 * - \b "~(forward|downward)_(left|right)_PID_Kp" : \b [double] front/down left/right PID proportional constant. min: -1.0, default: 0.0, max: 1.0
 * - \b "~(forward|downward)_(left|right)_PID_Ki" : \b [double] front/down left/right PID integral     constant. min: -1.0, default: 0.0, max: 1.0
 * - \b "~(forward|downward)_(left|right)_PID_Kd" : \b [double] front/down left/right PID derivative   constant. min: -1.0, default: 0.0, max: 1.0
 * - \b "~(forward|downward)_(left|right)_accel"  : \b [int]    front/down left/right acceleration (%/ds). min: 0, default: 5, max: 100
 * - \b "~(forward|downward)_(left|right)_invert" : \b [bool]   front/down left/right speed inversion. min: False, default: False, max: True
 */

#include "motor_board_node_base.h"

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "motor_board_node");

  ros::NodeHandle node("motor_board");
  ros::NodeHandle priv("~");
  albatros_motor_board::MotorBoardNodeBase mb_node(node,priv);

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
