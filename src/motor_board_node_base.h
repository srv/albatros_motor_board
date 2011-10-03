/**
 * @file
 * @brief ROS driver for Albatros motor board (presentation).
 *
 * This is a ROS driver for a motor board provided by Albatros.
 * It uses a library created for that device, that exposes its modules
 * (pressure and water-in sensors, and two motor pairs with its built-in PID controllers)
 * and handles communications through the serial port.
 *
 * @par Advertises
 *
 * - @b speeds_rpm topic (srv_msgs/MotorLevels)
 *   current motor speeds in rpm.
 *
 * - @b status topic (albatros_motor_board/MotorStatus)
 *   motor error counts.
 *
 * - @b pressure topic (srv_msgs/Pressure)
 *   pressure sensor sample.
 *
 * @par Subscribes
 *
 * - @b speeds_pc (srv_msgs/MotorLevels)
 *   desired speeds in percentage of device nominal speed.
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

#ifndef MOTOR_BOARD_NODE_BASE_H
#define MOTOR_BOARD_NODE_BASE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <srv_msgs/Pressure.h>
#include <srv_msgs/MotorLevels.h>
#include "albatros_motor_board/MotorStatus.h"
#include "albatros_motor_board/MotorBoardDynParamsConfig.h"
#include "albatros_motor_board/motor_board_ctrl.h"

namespace albatros_motor_board
{

class MotorBoardNodeBase
{
public:

  MotorBoardNodeBase(const ros::NodeHandle& node, const ros::NodeHandle& priv);

  void advertiseMotorTopics();

  void advertiseSensorTopics();

  void initDynParamsSrv();

  void subscribeMotorTopics();

  void cleanUp();

private:

  enum OutTopic {MOTOR_SPEEDS, MOTOR_STATUS, SENSOR_PRESSURE};
  static const int NUM_OUT_TOPICS = 3;

  ros::NodeHandle node_;
  ros::NodeHandle priv_;
  ros::Subscriber subs_speeds_;
  ros::Publisher publisher_[NUM_OUT_TOPICS];

  bool do_publish_[NUM_OUT_TOPICS];

  dynamic_reconfigure::Server<MotorBoardDynParamsConfig> dyn_params_srv_;
  MotorBoardDynParamsConfig current_params_;

  ros::Timer publish_timer_[NUM_OUT_TOPICS];

  MotorBoardCtrl mbctrl_;
  bool mbctrl_ready_;

  void getMotorAccelsParams(MotorBoardCtrl::MotorAccels* accels);

  void getMotorCtrlParams(const MotorBoardCtrl::Motor& m,
                          bool *PID_on, MotorBoardCtrl::PIDConstants* PID_Kpid);

  void getSensorOffsetParam(const MotorBoardCtrl::Sensor& s,
                            int* offset);

  void getPublishRateParam(const OutTopic& t, double* rate);

  template <typename T>
  static bool updateParam(T* old_val, const T& new_val);

  bool updateCommNameParam(const MotorBoardDynParamsConfig& params);

  bool updateMotorAccelsParams(const MotorBoardDynParamsConfig& params);

  bool updateMotorCtrlParams(const MotorBoardDynParamsConfig& params,
                             const MotorBoardCtrl::Motor& m);

  bool updateSensorOffsetParam(const MotorBoardDynParamsConfig& params,
                               const MotorBoardCtrl::Sensor& s);

  bool updatePublishRateParam(const MotorBoardDynParamsConfig& params,
                              const OutTopic& t);

  bool updateInvertSpeedParams(const MotorBoardDynParamsConfig& params);

  void updateCommName();

  void updateMotorAccels();

  void updateMotorCtrl(MotorBoardCtrl::Motor m);

  void updateSensorOffset(MotorBoardCtrl::Sensor s);

  void updatePublishRate(const OutTopic& t);

  void checkVersion();

  void checkSensorConfig(MotorBoardCtrl::Sensor s);

  void initialize(const MotorBoardDynParamsConfig& params);

  void dynReconfigureParams(MotorBoardDynParamsConfig& params,
                            uint32_t level);

  void fillMotorSpeeds(const srv_msgs::MotorLevels& m,
                       MotorBoardCtrl::MotorSpeeds* s);

  void fillMotorSpeedsMsg(const MotorBoardCtrl::MotorSpeeds& s,
                          srv_msgs::MotorLevels* m );

  void publishPressure();

  void publishSpeeds();

  void publishStatus();

  void subscriptionCallback(const ros::SingleSubscriberPublisher& ssp,
                            const OutTopic& t);

  void updateSpeedsCallback(const srv_msgs::MotorLevels& msg);

};

} // namespace

#endif // MOTOR_BOARD_NODE_BASE_H
