/**
 * @file
 * @brief ROS driver for Albatros motor board (nodelet version).
 * @see motor_board_node_base
 */


#include "motor_board_node_base.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace albatros_motor_board
{

/**
 * @brief Nodelet version of Albatros motor board driver.
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
