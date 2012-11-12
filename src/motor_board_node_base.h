/**
 * @file
 * @brief ROS driver for Albatros motor board (presentation).
 */


#ifndef MOTOR_BOARD_NODE_BASE_H
#define MOTOR_BOARD_NODE_BASE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <auv_sensor_msgs/Pressure.h>
#include <auv_sensor_msgs/Humidity.h>
#include <auv_control_msgs/MotorLevels.h>
#include "albatros_motor_board/MotorStatus.h"
#include "albatros_motor_board/MotorBoardDynParamsConfig.h"
#include "albatros_motor_board/motor_board_ctrl.h"

namespace albatros_motor_board
{

/**
 * @brief ROS driver class for Albatros motor board.
 *
 * ROS driver class for a motor board provided by Albatros.
 * It uses a library created for that device, that exposes its modules
 * (pressure and water-in sensors, and two motor pairs with its built-in PID
 * controllers), and handles communications through the serial port.
 *
 * @par Advertises
 *
 * - @b speeds_rpm (auv_control_msgs/MotorLevels)
 *   Current motor speeds in rpm.
 *
 * - @b status (albatros_motor_board/MotorStatus)
 *   Motor error counts.
 *
 * - @b pressure (auv_sensor_msgs/Pressure)
 *   Pressure sensor sample.
 *
 * - @b humidity (auv_sensor_msgs/Humidity)
 *   Humidity sensor sample.
 *
 * @par Subscribes
 *
 * - @b speeds_pc (auv_control_msgs/MotorLevels)
 *   Desired speeds in percentage of device nominal speed.
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

  enum OutTopic {MOTOR_SPEEDS, MOTOR_STATUS, SENSOR_PRESSURE, SENSOR_WATERIN};
  static const int NUM_OUT_TOPICS = 4;
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

  void fillMotorSpeeds(const auv_control_msgs::MotorLevelsConstPtr& m,
                       MotorBoardCtrl::MotorSpeeds* s) const;

  void fillMotorSpeedsMsg(const MotorBoardCtrl::MotorSpeeds& s,
                          const auv_control_msgs::MotorLevelsPtr& m ) const;

  void publishMotorSpeeds();

  void publishMotorStatus();

  void publishSensorPressure();

  void publishSensorWaterIn();

  void subscriptionCallback(const ros::SingleSubscriberPublisher& ssp,
                            const OutTopic& t);

  void updateSpeedsCallback(const auv_control_msgs::MotorLevelsConstPtr& msg);

};

} // namespace

#endif // MOTOR_BOARD_NODE_BASE_H
