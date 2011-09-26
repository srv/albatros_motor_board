/**
 * @file
 * @brief ROS driver base class for Albatros motor board (implementation).
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


#include "motor_board_node_base.h"

albatros_motor_board::MotorBoardNodeBase::MotorBoardNodeBase(const ros::NodeHandle& node,
                                                            const ros::NodeHandle& priv)
: node_(node)
, priv_(priv)
, num_speeds_subscribers_(0)
, num_status_subscribers_(0)
, num_pressure_subscribers_(0)
{}

void albatros_motor_board::MotorBoardNodeBase::advertiseMotorTopics()
{
  ros::SubscriberStatusCallback speeds_subs_cb =
      boost::bind(&MotorBoardNodeBase::speedsSubscribedCallback,this,_1);
  ros::SubscriberStatusCallback speeds_unsubs_cb =
      boost::bind(&MotorBoardNodeBase::speedsUnsubscribedCallback,this,_1);
  publ_speeds_ = node_.advertise<srv_msgs::MotorLevels>("speeds_rpm",1,
                                                        speeds_subs_cb,
                                                        speeds_unsubs_cb);
  ros::SubscriberStatusCallback status_subs_cb =
      boost::bind(&MotorBoardNodeBase::statusSubscribedCallback,this,_1);
  ros::SubscriberStatusCallback status_unsubs_cb =
      boost::bind(&MotorBoardNodeBase::statusUnsubscribedCallback,this,_1);
  publ_status_ = node_.advertise<albatros_motor_board::MotorStatus>("status",1,
                                                                   status_subs_cb,
                                                                   status_unsubs_cb);
}

void albatros_motor_board::MotorBoardNodeBase::advertiseSensorTopics()
{
  ros::SubscriberStatusCallback pressure_subs_cb =
      boost::bind(&MotorBoardNodeBase::pressureSubscribedCallback,this,_1);
  ros::SubscriberStatusCallback pressure_unsubs_cb =
      boost::bind(&MotorBoardNodeBase::pressureUnsubscribedCallback,this,_1);
  publ_pressure_ = node_.advertise<srv_msgs::Pressure>("pressure",1,
                                                       pressure_subs_cb,
                                                       pressure_unsubs_cb);
}

void albatros_motor_board::MotorBoardNodeBase::initDynParamsSrv()
{
  dyn_params_srv_.setCallback( boost::bind(&MotorBoardNodeBase::dynReconfigureParams,this,_1,_2) );
}

void albatros_motor_board::MotorBoardNodeBase::subscribeMotorTopics()
{
  subs_speeds_ = node_.subscribe("speeds_pc",1, &MotorBoardNodeBase::updateSpeedsCallback, this);
}

void albatros_motor_board::MotorBoardNodeBase::cleanUp()
{
    mbctrl_.closeComm();
}


void albatros_motor_board::MotorBoardNodeBase::getMotorAccelsParams(MotorBoardCtrl::MotorAccels* accels)
{
  (*accels)[mbctrl_.FORWARD_LEFT]   = current_params_.forward_left_accel;
  (*accels)[mbctrl_.FORWARD_RIGHT]  = current_params_.forward_right_accel;
  (*accels)[mbctrl_.DOWNWARD_LEFT]  = current_params_.downward_left_accel;
  (*accels)[mbctrl_.DOWNWARD_RIGHT] = current_params_.downward_right_accel;
}

void albatros_motor_board::MotorBoardNodeBase::getMotorCtrlParams(const MotorBoardCtrl::Motor& m,
                                                                 bool *PID_on,
                                                                 MotorBoardCtrl::PIDConstants* PID_Kpid)
{
  switch(m)
  {
    case MotorBoardCtrl::FORWARD_LEFT :
      *PID_on = current_params_.forward_left_PID_on;
      (*PID_Kpid)[mbctrl_.P] = current_params_.forward_left_PID_Kp;
      (*PID_Kpid)[mbctrl_.I] = current_params_.forward_left_PID_Ki;
      (*PID_Kpid)[mbctrl_.D] = current_params_.forward_left_PID_Kd;
      break;
    case MotorBoardCtrl::FORWARD_RIGHT :
      *PID_on = current_params_.forward_right_PID_on;
      (*PID_Kpid)[mbctrl_.P] = current_params_.forward_right_PID_Kp;
      (*PID_Kpid)[mbctrl_.I] = current_params_.forward_right_PID_Ki;
      (*PID_Kpid)[mbctrl_.D] = current_params_.forward_right_PID_Kd;
      break;
    case MotorBoardCtrl::DOWNWARD_LEFT :
      *PID_on = current_params_.downward_left_PID_on;
      (*PID_Kpid)[mbctrl_.P] = current_params_.downward_left_PID_Kp;
      (*PID_Kpid)[mbctrl_.I] = current_params_.downward_left_PID_Ki;
      (*PID_Kpid)[mbctrl_.D] = current_params_.downward_left_PID_Kd;
      break;
    case MotorBoardCtrl::DOWNWARD_RIGHT :
      *PID_on = current_params_.downward_right_PID_on;
      (*PID_Kpid)[mbctrl_.P] = current_params_.downward_right_PID_Kp;
      (*PID_Kpid)[mbctrl_.I] = current_params_.downward_right_PID_Ki;
      (*PID_Kpid)[mbctrl_.D] = current_params_.downward_right_PID_Kd;
      break;
  }
}

void albatros_motor_board::MotorBoardNodeBase::getSensorOffsetParam(const MotorBoardCtrl::Sensor& s,
                                                                   int* offset)
{
  switch(s)
  {
    case MotorBoardCtrl::PRESSURE :
      *offset = current_params_.pressure_offset;
      break;
    case MotorBoardCtrl::WATERIN :
      *offset = current_params_.waterin_offset;
      break;
  }
}

template <typename T>
bool albatros_motor_board::MotorBoardNodeBase::updateParam(T* old_val,
                                                           const T& new_val)
{
  if (*old_val != new_val)
  {
    *old_val = new_val;
    return true;
  }
  else
    return false;
}

bool albatros_motor_board::MotorBoardNodeBase::updateCommNameParam(const MotorBoardDynParamsConfig& params)
{
  bool res = updateParam(&(current_params_.serial_port), params.serial_port);
  return res;
}

bool albatros_motor_board::MotorBoardNodeBase::updateMotorAccelsParams(const MotorBoardDynParamsConfig& params)
{
  bool res = updateParam(&(current_params_.forward_left_accel  ), params.forward_left_accel);
  res += updateParam(&(current_params_.forward_right_accel ), params.forward_right_accel);
  res += updateParam(&(current_params_.downward_left_accel ), params.downward_left_accel);
  res += updateParam(&(current_params_.downward_right_accel), params.downward_right_accel);
  return res;
}

bool albatros_motor_board::MotorBoardNodeBase::updateMotorCtrlParams(const MotorBoardDynParamsConfig& params,
                                                                     const MotorBoardCtrl::Motor& m)
{
  bool res = false;
  switch(m)
  {
    case MotorBoardCtrl::FORWARD_LEFT :
      res += updateParam(&(current_params_.forward_left_PID_on),params.forward_left_PID_on);
      res += updateParam(&(current_params_.forward_left_PID_Kp),params.forward_left_PID_Kp);
      res += updateParam(&(current_params_.forward_left_PID_Ki),params.forward_left_PID_Ki);
      res += updateParam(&(current_params_.forward_left_PID_Kd),params.forward_left_PID_Kd);
      break;
    case MotorBoardCtrl::FORWARD_RIGHT :
      res += updateParam(&(current_params_.forward_right_PID_on),params.forward_right_PID_on);
      res += updateParam(&(current_params_.forward_right_PID_Kp),params.forward_right_PID_Kp);
      res += updateParam(&(current_params_.forward_right_PID_Ki),params.forward_right_PID_Ki);
      res += updateParam(&(current_params_.forward_right_PID_Kd),params.forward_right_PID_Kd);
      break;
    case MotorBoardCtrl::DOWNWARD_LEFT :
      res += updateParam(&(current_params_.downward_left_PID_on),params.downward_left_PID_on);
      res += updateParam(&(current_params_.downward_left_PID_Kp),params.downward_left_PID_Kp);
      res += updateParam(&(current_params_.downward_left_PID_Ki),params.downward_left_PID_Ki);
      res += updateParam(&(current_params_.downward_left_PID_Kd),params.downward_left_PID_Kd);
      break;
    case MotorBoardCtrl::DOWNWARD_RIGHT :
      res += updateParam(&(current_params_.downward_right_PID_on),params.downward_right_PID_on);
      res += updateParam(&(current_params_.downward_right_PID_Kp),params.downward_right_PID_Kp);
      res += updateParam(&(current_params_.downward_right_PID_Ki),params.downward_right_PID_Ki);
      res += updateParam(&(current_params_.downward_right_PID_Kd),params.downward_right_PID_Kd);
      break;
  }
  return res;
}

bool albatros_motor_board::MotorBoardNodeBase::updateSensorOffsetParam(const MotorBoardDynParamsConfig& params,
                                                                       const MotorBoardCtrl::Sensor& s)
{
  bool res = false;
  switch(s)
  {
    case MotorBoardCtrl::PRESSURE :
      res += updateParam(&(current_params_.pressure_offset),params.pressure_offset);
      break;
    case MotorBoardCtrl::WATERIN :
      res += updateParam(&(current_params_.waterin_offset),params.waterin_offset);
      break;
  }
  return res;
}

bool albatros_motor_board::MotorBoardNodeBase::updateTimerRateParam(const MotorBoardDynParamsConfig& params)
{
  bool res = updateParam(&(current_params_.rate),params.rate);
  return res;
}

bool albatros_motor_board::MotorBoardNodeBase::updateInvertSpeedParams(const MotorBoardDynParamsConfig& params)
{
  bool res = updateParam(&(current_params_.forward_left_invert), params.forward_left_invert);
  res += updateParam(&(current_params_.forward_right_invert), params.forward_right_invert);
  res += updateParam(&(current_params_.downward_left_invert), params.downward_left_invert);
  res += updateParam(&(current_params_.downward_right_invert), params.downward_right_invert);
  return res;
}

void albatros_motor_board::MotorBoardNodeBase::updateCommName()
{
  ROS_INFO_STREAM("Setting serial communication port to device "
                  << current_params_.serial_port);
  mbctrl_.openComm(current_params_.serial_port);
}

void albatros_motor_board::MotorBoardNodeBase::updateMotorAccels()
{
  MotorBoardCtrl::MotorAccels accels, accels_res;
  getMotorAccelsParams(&accels);
  ROS_INFO_STREAM("Updating motor accelerations");
  mbctrl_.setAccels( accels, &accels_res);
  ROS_INFO_STREAM("Motor accelerations response : " );
  ROS_INFO_STREAM("  front left  : " << accels_res[mbctrl_.FORWARD_LEFT]);
  ROS_INFO_STREAM("  front right : " << accels_res[mbctrl_.FORWARD_RIGHT]);
  ROS_INFO_STREAM("  down  left  : " << accels_res[mbctrl_.DOWNWARD_LEFT]);
  ROS_INFO_STREAM("  down  right : " << accels_res[mbctrl_.DOWNWARD_RIGHT]);
}

void albatros_motor_board::MotorBoardNodeBase::updateMotorCtrl(MotorBoardCtrl::Motor m)
{
  bool PID_on, PID_on_res;
  MotorBoardCtrl::PIDConstants PID_Kpid, PID_Kpid_res;
  getMotorCtrlParams(m, &PID_on, &PID_Kpid);
  ROS_INFO_STREAM("Updating motor " << m << " built-in PID configuration");
  mbctrl_.setMotorctrl(m, PID_on, PID_Kpid, &PID_on_res, &PID_Kpid_res);
  ROS_INFO_STREAM("Motor " << m << " built-in PID configuration response :");
  ROS_INFO_STREAM("  active : " << PID_on_res);
  ROS_INFO_STREAM("  Kp : " << PID_Kpid_res[MotorBoardCtrl::P]);
  ROS_INFO_STREAM("  Ki : " << PID_Kpid_res[MotorBoardCtrl::I]);
  ROS_INFO_STREAM("  Kd : " << PID_Kpid_res[MotorBoardCtrl::D]);
}

void albatros_motor_board::MotorBoardNodeBase::updateSensorOffset(MotorBoardCtrl::Sensor s)
{
  int offset;
  getSensorOffsetParam(s, &offset);
  ROS_INFO_STREAM("Updating sensor " << s << " offset");
  mbctrl_.setSensorOffset(s, offset, &offset);
  ROS_INFO_STREAM("Sensor " << s << " offset response : " << offset);
}

void albatros_motor_board::MotorBoardNodeBase::updateTimerRate()
{
  if (timed_caller_)
  {
    if (current_params_.rate > 0.0)
    {
      ROS_INFO_STREAM("Reconfiguring rate to : " << current_params_.rate);
      timed_caller_.setPeriod(ros::Duration(1.0/current_params_.rate));
      timed_caller_.start();
    }
    else
    {
      ROS_INFO_STREAM("Stopping rate timer");
      timed_caller_.stop();
    }
  }
  else
  {
    if (current_params_.rate > 0.0)
    {
      ROS_INFO_STREAM("Initializing timer rate to : " << current_params_.rate);
      timed_caller_ = node_.createTimer(ros::Duration(1.0 / current_params_.rate),
                                      boost::bind(&MotorBoardNodeBase::timedPublishCallback,this));
    }
  }
}

void albatros_motor_board::MotorBoardNodeBase::checkVersion()
{
  int version_num;
  ROS_INFO_STREAM("Retrieving firmware version number");
  mbctrl_.getVersion(&version_num);
  ROS_INFO_STREAM("Firmware version response : " << version_num);
}

void albatros_motor_board::MotorBoardNodeBase::checkSensorConfig(MotorBoardCtrl::Sensor s)
{
  int type, resolution, bits, min, max;
  ROS_INFO_STREAM("Retrieving sensor " << s << " configuration");
  mbctrl_.getSensorConfig(s,&type, &resolution, &bits, &min, &max);
  ROS_INFO_STREAM("Sensor " << s << " configuration response : ");
  ROS_INFO_STREAM("  type       : " << type );
  ROS_INFO_STREAM("  resolution : " << resolution);
  ROS_INFO_STREAM("  ADC bits   : " << bits);
  ROS_INFO_STREAM("  range      : [ " << min << " , " << max << " ]");
}

void albatros_motor_board::MotorBoardNodeBase::initialize(const MotorBoardDynParamsConfig& params)
{
  ROS_INFO_STREAM("Opening serial communication on device : " << current_params_.serial_port);
  mbctrl_.openComm(current_params_.serial_port);
  checkVersion();
  checkSensorConfig(mbctrl_.PRESSURE);
  updateMotorAccelsParams(params);
  updateMotorAccels();
  updateMotorCtrlParams(params, mbctrl_.FORWARD_LEFT);
  updateMotorCtrl(mbctrl_.FORWARD_LEFT);
  updateMotorCtrlParams(params, mbctrl_.FORWARD_RIGHT);
  updateMotorCtrl(mbctrl_.FORWARD_RIGHT);
  updateMotorCtrlParams(params, mbctrl_.DOWNWARD_LEFT);
  updateMotorCtrl(mbctrl_.DOWNWARD_LEFT);
  updateMotorCtrlParams(params, mbctrl_.DOWNWARD_RIGHT);
  updateMotorCtrl(mbctrl_.DOWNWARD_RIGHT);
  updateSensorOffsetParam(params, mbctrl_.PRESSURE);
  updateSensorOffset(mbctrl_.PRESSURE);
  updateSensorOffsetParam(params, mbctrl_.WATERIN);
  updateSensorOffset(mbctrl_.WATERIN);
  updateTimerRateParam(params);
  updateTimerRate();
}

void albatros_motor_board::MotorBoardNodeBase::dynReconfigureParams(MotorBoardDynParamsConfig& params, uint32_t level)
{
  try
  {
    /**
     * - @b ~rate  state (pressure, speeds and status) publishing rate (default 10 hz)
     * - @b ~sensors/pressure/offset offset for the pressure sensor (default 0)
     * - @b ~motors/(front|down)/(left|right)/accel motor acceleration (% per dsec, default 5 %)
     * - @b ~motors/(front|down)/(left|right)/PID/active motor PID controller state (boolean)
     * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID proportional constant (double)
     * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID integral constant (double)
     * - @b ~motors/(front|down)/(left|right)/PID/Kp motor PID derivative constant (double)
    */

    if ( updateCommNameParam(params) )
    {
      mbctrl_ready_ = false;
      initialize(params);
      mbctrl_ready_ = true;
    }
    else if (mbctrl_ready_)
    {
      if ( updateMotorAccelsParams(params) ) updateMotorAccels();
      if ( updateMotorCtrlParams(params, mbctrl_.FORWARD_LEFT)   ) updateMotorCtrl(mbctrl_.FORWARD_LEFT);
      if ( updateMotorCtrlParams(params, mbctrl_.FORWARD_RIGHT)  ) updateMotorCtrl(mbctrl_.FORWARD_RIGHT);
      if ( updateMotorCtrlParams(params, mbctrl_.DOWNWARD_LEFT)  ) updateMotorCtrl(mbctrl_.DOWNWARD_LEFT);
      if ( updateMotorCtrlParams(params, mbctrl_.DOWNWARD_RIGHT) ) updateMotorCtrl(mbctrl_.DOWNWARD_RIGHT);
      if ( updateSensorOffsetParam(params, mbctrl_.PRESSURE) ) updateSensorOffset(mbctrl_.PRESSURE);
      if ( updateSensorOffsetParam(params, mbctrl_.WATERIN) ) updateSensorOffset(mbctrl_.WATERIN);
      if ( updateTimerRateParam(params) ) updateTimerRate();
    }
    if ( updateInvertSpeedParams(params) ) {};
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Error reconfiguring motor board node : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::publishPressure()
{
  try
  {
    ros::Time stamp = ros::Time::now();
    int value;
    mbctrl_.getSensorValue(mbctrl_.PRESSURE, &value);
    srv_msgs::Pressure msg;
    msg.header.stamp = stamp;
    msg.pressure = double(value);
    publ_pressure_.publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting pressure : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::fillMotorSpeedsMsg(const MotorBoardCtrl::MotorSpeeds& s,
                                                                  srv_msgs::MotorLevels* m )
{
  m->levels[mbctrl_.FORWARD_LEFT]   = (current_params_.forward_left_invert)
                                               ? -s[mbctrl_.FORWARD_LEFT]
                                               : +s[mbctrl_.FORWARD_LEFT];
  m->levels[mbctrl_.FORWARD_RIGHT]  = (current_params_.forward_right_invert)
                                               ? -s[mbctrl_.FORWARD_RIGHT]
                                               : +s[mbctrl_.FORWARD_RIGHT];
  m->levels[mbctrl_.DOWNWARD_LEFT]  = (current_params_.downward_left_invert)
                                               ? -s[mbctrl_.DOWNWARD_LEFT]
                                               : +s[mbctrl_.DOWNWARD_LEFT];
  m->levels[mbctrl_.DOWNWARD_RIGHT] = (current_params_.downward_right_invert)
                                               ? -s[mbctrl_.DOWNWARD_RIGHT]
                                               : +s[mbctrl_.DOWNWARD_RIGHT];
}

void albatros_motor_board::MotorBoardNodeBase::publishSpeeds()
{
  try
  {
    ros::Time stamp = ros::Time::now();
    MotorBoardCtrl::MotorSpeeds speeds_rpm;
    mbctrl_.getSpeeds(&speeds_rpm);
    srv_msgs::MotorLevels msg;
    msg.header.stamp = stamp;
    fillMotorSpeedsMsg(speeds_rpm, &msg);
    publ_speeds_.publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting motor speeds : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::publishStatus()
{
  try
  {
    ros::Time stamp = ros::Time::now();
    MotorBoardCtrl::MotorStatus status;
    mbctrl_.getStatus(&status);
    MotorStatus msg;
    msg.header.stamp = stamp;
    for (int i=0; i<mbctrl_.NUM_MOTORS; i++)
      msg.status[i] = status[i];
    publ_status_.publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting motor status : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::timedPublishCallback()
{
  if (publish_pressure_)
     publishPressure();
  if (publish_speeds_)
     publishSpeeds();
  if (publish_status_)
     publishStatus();
}

void albatros_motor_board::MotorBoardNodeBase::fillMotorSpeeds(const srv_msgs::MotorLevels& m,
                                                               MotorBoardCtrl::MotorSpeeds* s)
{
  (*s)[mbctrl_.FORWARD_LEFT]   = (current_params_.forward_left_invert)
                                   ? -m.levels[mbctrl_.FORWARD_LEFT]
                                   : +m.levels[mbctrl_.FORWARD_LEFT];
  (*s)[mbctrl_.FORWARD_RIGHT]  = (current_params_.forward_right_invert)
                                   ? -m.levels[mbctrl_.FORWARD_RIGHT]
                                   : +m.levels[mbctrl_.FORWARD_RIGHT];
  (*s)[mbctrl_.DOWNWARD_LEFT]  = (current_params_.downward_left_invert)
                                   ? -m.levels[mbctrl_.DOWNWARD_LEFT]
                                   : +m.levels[mbctrl_.DOWNWARD_LEFT];
  (*s)[mbctrl_.DOWNWARD_RIGHT] = (current_params_.downward_right_invert)
                                   ? -m.levels[mbctrl_.DOWNWARD_RIGHT]
                                   : +m.levels[mbctrl_.DOWNWARD_RIGHT];
}

void albatros_motor_board::MotorBoardNodeBase::updateSpeedsCallback(const srv_msgs::MotorLevels& msg)
{
  const int num_motors = msg.levels.size();
  if ( num_motors != mbctrl_.NUM_MOTORS )
  {
    ROS_ERROR_STREAM("Wrong number of motors (" << num_motors
                     << ") in update speed request, ignoring request.");
    return;
  }
  MotorBoardCtrl::MotorSpeeds speeds_pc;
  fillMotorSpeeds(msg, &speeds_pc);
  try
  {
    mbctrl_.setSpeeds(speeds_pc);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error setting motor speeds : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::speedsSubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (num_speeds_subscribers_++ == 0)
    publish_speeds_ = true;
}

void albatros_motor_board::MotorBoardNodeBase::speedsUnsubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (--num_speeds_subscribers_ == 0)
    publish_speeds_ = false;
}

void albatros_motor_board::MotorBoardNodeBase::statusSubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (num_status_subscribers_++ == 0)
    publish_status_ = true;
}

void albatros_motor_board::MotorBoardNodeBase::statusUnsubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (--num_status_subscribers_ == 0)
    publish_status_ = false;
}

void albatros_motor_board::MotorBoardNodeBase::pressureSubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (num_pressure_subscribers_++ == 0)
    publish_pressure_ = true;
}

void albatros_motor_board::MotorBoardNodeBase::pressureUnsubscribedCallback(const ros::SingleSubscriberPublisher& ssp)
{
  if (--num_pressure_subscribers_ == 0)
    publish_pressure_ = false;
}
