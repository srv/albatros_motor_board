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
 * - @b ~port  device file name for the serial port (default "/dev/ttyS0")
 * - @b ~rate  state (pressure, speeds and status) publishing rate (default 10 hz)
 * - @b ~sensors/pressure/offset offset for the pressure sensor (default 0)
 * - @b ~motors/(front|down)/(left|right)/accel motor acceleration (% per dsec, default 5 %)
 * - @b ~motors/(front|down)/(left|right)/PID/active motor PID controller state (boolean default false)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID proportional constant (double default 0.0)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID integral constant (double default 0.0)
 * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID derivative constant (double default 0.0)
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
