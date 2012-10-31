/**
 * @file
 * @brief ROS driver for Albatros motor board (node version).
 * @see albatros_motor_board::MotorBoardNodeBase
 */

#include "motor_board_node_base.h"

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "motor_board_node");

  ros::NodeHandle node;
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
