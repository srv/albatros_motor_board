/**
 * @file
 * @brief ROS driver for Albatros motor board (nodelet version).
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
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace albatros_motor_board
{

/**
 * @brief Nodelet version of twist controller.
 */
class MotorBoardNodelet: public nodelet::Nodelet
{
public:
  MotorBoardNodelet();
  virtual ~MotorBoardNodelet();
private:
  virtual void onInit();
  boost::shared_ptr<albatros_motor_board::MotorBoardNodeBase> mb_node_;
};

} // namespace

/**
 * @brief Default constructor (doing nothing).
 * @return
 */
albatros_motor_board::MotorBoardNodelet::MotorBoardNodelet()
{}

/**
 * @brief Nodelet initialization.
 * @note Must return immediately.
 */
void albatros_motor_board::MotorBoardNodelet::onInit()
{
  ros::NodeHandle node(getNodeHandle(),"motor_board");
  ros::NodeHandle priv(getPrivateNodeHandle());
  mb_node_.reset(new MotorBoardNodeBase(node,priv));

  // advertise output topics
  mb_node_->advertiseSensorTopics();
  mb_node_->advertiseMotorTopics();

  // node parameters
  mb_node_->initDynParamsSrv();

  // subscribe input topics
  mb_node_->subscribeMotorTopics();

  // spin is done by nodelet machinery
}

/**
 * @brief Destructor (calling the cleanUp() function).
 * @return
 */
albatros_motor_board::MotorBoardNodelet::~MotorBoardNodelet()
{
  mb_node_->cleanUp();
}


// Register this plugin with pluginlib.
// Names must match *nodelet.xml in package directory.
// Parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(albatros_motor_board, motor_board_nodelet, albatros_motor_board::MotorBoardNodelet, nodelet::Nodelet);
