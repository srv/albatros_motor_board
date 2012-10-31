/**
 * @file
 * @brief ROS driver base class for Albatros motor board (implementation).
 * @see MotorBoardNodeBase
 */

#include "motor_board_node_base.h"

albatros_motor_board::MotorBoardNodeBase::MotorBoardNodeBase(const ros::NodeHandle& node,
                                                             const ros::NodeHandle& priv)
: node_(node)
, priv_(priv)
{
  do_publish_[MOTOR_SPEEDS] = false;
  do_publish_[MOTOR_STATUS] = false;
  do_publish_[SENSOR_PRESSURE] = false;
  do_publish_[SENSOR_WATERIN] = false;
}

void albatros_motor_board::MotorBoardNodeBase::advertiseMotorTopics()
{
  ros::SubscriberStatusCallback speeds_subs_cb =
      boost::bind(&MotorBoardNodeBase::subscriptionCallback, this, _1, MOTOR_SPEEDS);
  publisher_[MOTOR_SPEEDS] =
      node_.advertise<srv_msgs::MotorLevels>("motor_speeds", 5, speeds_subs_cb, speeds_subs_cb);
  ros::SubscriberStatusCallback status_subs_cb =
      boost::bind(&MotorBoardNodeBase::subscriptionCallback, this, _1, MOTOR_STATUS);
  publisher_[MOTOR_STATUS] =
      node_.advertise<albatros_motor_board::MotorStatus>("motor_status", 5, status_subs_cb, status_subs_cb);
}

void albatros_motor_board::MotorBoardNodeBase::advertiseSensorTopics()
{
  ros::SubscriberStatusCallback pressure_subs_cb =
      boost::bind(&MotorBoardNodeBase::subscriptionCallback, this, _1, SENSOR_PRESSURE);
  publisher_[SENSOR_PRESSURE] = node_.advertise<srv_msgs::Pressure>("pressure", 5,
                                                                    pressure_subs_cb,
                                                                    pressure_subs_cb);
  ros::SubscriberStatusCallback waterin_subs_cb =
      boost::bind(&MotorBoardNodeBase::subscriptionCallback, this, _1, SENSOR_WATERIN);
  publisher_[SENSOR_WATERIN] = node_.advertise<srv_msgs::WaterIn>("humidity", 5,
                                                                  waterin_subs_cb,
                                                                  waterin_subs_cb);
}

void albatros_motor_board::MotorBoardNodeBase::initDynParamsSrv()
{
  dyn_params_srv_.setCallback( boost::bind(&MotorBoardNodeBase::dynReconfigureParams,this,_1,_2) );
}

void albatros_motor_board::MotorBoardNodeBase::subscribeMotorTopics()
{
  subs_speeds_ = node_.subscribe("motor_levels",1, &MotorBoardNodeBase::updateSpeedsCallback, this);
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
  mbctrl_.saturation_value = current_params_.saturation_value; // fbf 19-07-2012
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

void albatros_motor_board::MotorBoardNodeBase::getPublishRateParam(const OutTopic& t,
                                                                   double* rate)
{
  switch(t)
  {
    case MOTOR_SPEEDS :
      *rate = current_params_.rate_speeds;
      break;
    case MOTOR_STATUS :
      *rate = current_params_.rate_status;
      break;
    case SENSOR_PRESSURE :
      *rate = current_params_.rate_pressure;
	break;
    case SENSOR_WATERIN: //fbf 9-03-2011
      *rate=current_params_.rate_humidity;
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
  res += updateParam(&(current_params_.saturation_value), params.saturation_value); // fbf 19-07-2012 update the saturation value
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

bool albatros_motor_board::MotorBoardNodeBase::updatePublishRateParam(const MotorBoardDynParamsConfig& params,
                                                                      const OutTopic& t)
{
  bool res = false;
  switch (t)
  {
    case MOTOR_SPEEDS :
      res = updateParam(&(current_params_.rate_speeds),params.rate_speeds);
      break;
    case MOTOR_STATUS :
      res = updateParam(&(current_params_.rate_status),params.rate_status);
      break;
    case SENSOR_PRESSURE :
      res = updateParam(&(current_params_.rate_pressure),params.rate_pressure);
      break;
    case SENSOR_WATERIN :
      res = updateParam(&(current_params_.rate_humidity),params.rate_humidity); //fbf 09-03-2012
      break;
  }
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

void albatros_motor_board::MotorBoardNodeBase::updatePublishRate(const OutTopic& t) // timer creation
{
  double rate = 0.0;
  getPublishRateParam(t, &rate);
  const std::string topic = publisher_[t].getTopic();
  if (publish_timer_[t])
  {
    if (rate > 0.0)
    {
      ROS_INFO_STREAM("Updating rate for topic " << topic << " to : " << rate);
      publish_timer_[t].setPeriod(ros::Duration(1.0/rate));
      publish_timer_[t].start();
    }
    else
    {
      ROS_INFO_STREAM("Stopping timer for topic " << topic );
      publish_timer_[t].stop();
    }
  }
  else
  {
    if (rate > 0.0)
    {
      ROS_INFO_STREAM("Initializing rate for topic " << topic << " to : " << rate);
      ros::TimerCallback cb;
      switch (t)
      {
        case MOTOR_SPEEDS :
          cb = boost::bind(&MotorBoardNodeBase::publishMotorSpeeds, this);
          break;
        case MOTOR_STATUS :
          cb = boost::bind(&MotorBoardNodeBase::publishMotorStatus, this);
          break;
        case SENSOR_PRESSURE :
          cb = boost::bind(&MotorBoardNodeBase::publishSensorPressure, this);
          break;
	    case SENSOR_WATERIN :
          cb = boost::bind(&MotorBoardNodeBase::publishSensorWaterIn, this);
          break;
      }
      publish_timer_[t] = node_.createTimer(ros::Duration(1.0 / rate), cb); // publish the corresponding topic at the
      // times frequency indicated
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
  updatePublishRateParam(params, MOTOR_SPEEDS);
  updatePublishRate(MOTOR_SPEEDS);
  updatePublishRateParam(params, MOTOR_STATUS);
  updatePublishRate(MOTOR_STATUS);
  updatePublishRateParam(params, SENSOR_PRESSURE);
  updatePublishRate(SENSOR_PRESSURE);
  updatePublishRateParam(params, SENSOR_WATERIN);
  updatePublishRate(SENSOR_WATERIN);
}

void albatros_motor_board::MotorBoardNodeBase::dynReconfigureParams(MotorBoardDynParamsConfig& params, uint32_t level)
{
  try
  {
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
      if ( updateSensorOffsetParam(params, mbctrl_.WATERIN) )  updateSensorOffset(mbctrl_.WATERIN);
      if ( updatePublishRateParam(params, MOTOR_SPEEDS)    ) updatePublishRate(MOTOR_SPEEDS);
      if ( updatePublishRateParam(params, MOTOR_STATUS)    ) updatePublishRate(MOTOR_STATUS);
      if ( updatePublishRateParam(params, SENSOR_PRESSURE) ) updatePublishRate(SENSOR_PRESSURE);
      if ( updatePublishRateParam(params, SENSOR_WATERIN) ) updatePublishRate(SENSOR_WATERIN);
    }
    if ( updateInvertSpeedParams(params) ) {};
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Error reconfiguring motor board node : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::fillMotorSpeeds(const srv_msgs::MotorLevelsConstPtr& m,
                                                               MotorBoardCtrl::MotorSpeeds* s) const
{
  (*s)[mbctrl_.FORWARD_LEFT]   = (current_params_.forward_left_invert)
                                   ? -m->levels[mbctrl_.FORWARD_LEFT]
                                   : +m->levels[mbctrl_.FORWARD_LEFT];
  (*s)[mbctrl_.FORWARD_RIGHT]  = (current_params_.forward_right_invert)
                                   ? -m->levels[mbctrl_.FORWARD_RIGHT]
                                   : +m->levels[mbctrl_.FORWARD_RIGHT];
  (*s)[mbctrl_.DOWNWARD_LEFT]  = (current_params_.downward_left_invert)
                                   ? -m->levels[mbctrl_.DOWNWARD_LEFT]
                                   : +m->levels[mbctrl_.DOWNWARD_LEFT];
  (*s)[mbctrl_.DOWNWARD_RIGHT] = (current_params_.downward_right_invert)
                                   ? -m->levels[mbctrl_.DOWNWARD_RIGHT]
                                   : +m->levels[mbctrl_.DOWNWARD_RIGHT];

  bool scaling_required = false;
  int max_speed = 0;
  int min_speed = 0;
  for (int i = 0; i < MotorBoardCtrl::NUM_MOTORS; ++i)
  {
    if ((*s)[i] > MotorBoardCtrl::MAX_MOTOR_SPEED || (*s)[i] < MotorBoardCtrl::MIN_MOTOR_SPEED)
      scaling_required = true;
    if ((*s)[i] < min_speed)
      min_speed = (*s)[i];
    if ((*s)[i] > max_speed)
      max_speed = (*s)[i];
  }
  if (scaling_required)
  {
    double scale_max = max_speed != 0 ? 1.0 * MotorBoardCtrl::MAX_MOTOR_SPEED / max_speed : 1.0;
    double scale_min = min_speed != 0 ? 1.0 * MotorBoardCtrl::MIN_MOTOR_SPEED / min_speed : 1.0;
    double scale_factor = std::min(scale_max, scale_min);
    std::ostringstream before;
    std::ostringstream after;
    for (int i = 0; i < MotorBoardCtrl::NUM_MOTORS; ++i)
    {
      if (i != 0)
      {
        before << ",";
        after << ",";
      }
      before << (*s)[i];
      (*s)[i] *= scale_factor;
      after << (*s)[i];
    }
    ROS_WARN_STREAM("Motors are at speed limits, rescaled speeds: (" << 
        before.str() << ") -> (" << after.str() << ")");
  }
}

void albatros_motor_board::MotorBoardNodeBase::fillMotorSpeedsMsg(const MotorBoardCtrl::MotorSpeeds& s,
                                                                  const srv_msgs::MotorLevelsPtr& m ) const
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

void albatros_motor_board::MotorBoardNodeBase::publishMotorSpeeds()
{
  if (do_publish_[MOTOR_SPEEDS])
  try
  {
    MotorBoardCtrl::MotorSpeeds speeds_rpm; // new msg type MotorSpeeds
    mbctrl_.getSpeeds(&speeds_rpm); // get current Motor Speeds from the motorboard and store them in the speeds_rpm variable
    ros::Time stamp = ros::Time::now();
    srv_msgs::MotorLevelsPtr msg(new srv_msgs::MotorLevels()); // declare a new message type MotorLevels
    msg->header.stamp = stamp;
    msg->levels.resize(mbctrl_.NUM_MOTORS);
    fillMotorSpeedsMsg(speeds_rpm, msg); // insert the motor speeds read from the board into the MotorLevels msg
    publisher_[MOTOR_SPEEDS].publish(msg); //publish the motor levels
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting motor speeds : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::publishMotorStatus()
{
  if (do_publish_[MOTOR_STATUS])
  try
  {
    MotorBoardCtrl::MotorStatus status;
    mbctrl_.getStatus(&status);
    ros::Time stamp = ros::Time::now();
    albatros_motor_board::MotorStatusPtr msg(new albatros_motor_board::MotorStatus());
    msg->header.stamp = stamp;
    msg->status.resize(mbctrl_.NUM_MOTORS);
    for (int i=0; i<mbctrl_.NUM_MOTORS; i++)
      msg->status[i] = status[i];
    publisher_[MOTOR_STATUS].publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting motor status : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::publishSensorPressure()
{
  if (do_publish_[SENSOR_PRESSURE])
  try
  {
    ros::Time stamp = ros::Time::now();
    int value;
    mbctrl_.getSensorValue(mbctrl_.PRESSURE, &value); 
    srv_msgs::Pressure msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "/pressure_sensor";
    msg.pressure = double(value);
    publisher_[SENSOR_PRESSURE].publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting pressure : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::publishSensorWaterIn()
{
  if (do_publish_[SENSOR_WATERIN])
  try
  {
    ros::Time stamp = ros::Time::now();
    int value;
    mbctrl_.getSensorValue(mbctrl_.WATERIN, &value); 
    srv_msgs::WaterIn msg;
    msg.header.stamp = stamp;
    msg.humidity = value;
    publisher_[SENSOR_WATERIN].publish(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Error getting humidity : " << e.what());
  }
}

void albatros_motor_board::MotorBoardNodeBase::subscriptionCallback(const ros::SingleSubscriberPublisher& ssp,
                                                                    const OutTopic& t)
{
  if (publisher_[t].getNumSubscribers() == 0) // nobody is subscribed, so do not publish
    do_publish_[t] = false;
  else if (!(do_publish_[t])) // somebody subscribed and not publishing yet
    do_publish_[t] = true;
}

void albatros_motor_board::MotorBoardNodeBase::updateSpeedsCallback(const srv_msgs::MotorLevelsConstPtr& msg)
{
  const int num_motors = msg->levels.size();
  int sat_value = mbctrl_.saturation_value;
  if ( num_motors != mbctrl_.NUM_MOTORS )
  {
    ROS_ERROR_STREAM("Wrong number of motors (" << num_motors
                     << ") in update speed request, ignoring request.");
    return;
  }
  //printf("Capturing the motor levels message and sending them to the motor board");
  MotorBoardCtrl::MotorSpeeds speeds_pc;
  fillMotorSpeeds(msg, &speeds_pc);

/*
  // After some tests in Albatros, it came out that the system is no able to supply enought current when
  // all four motors enter in short-circuit. To be in short circuit, motors have to receive motor commands
  // between -X and X, passing through 0. In the transition from 0 to 0+-delta is when the motors requires more
  // current to overcome the motor joint resistance. It would be convenient to saturate the response around the 0.
  // the value of "sat_value" is experimental and responds to the
  for (int i=0; i<num_motors; i++) // fbf 19-07-2012. if motor speed is in between -sat_Value/2  and sat_value/2, it becomes 0
  {
	  if ( speeds_pc[i]<(sat_value/2) && speeds_pc[i]>(-sat_value/2) ) // fbf 19-07-2012. if motor speed is in between -sat_Value/2  and -sat_value, it becomes -sat_value
    	  {speeds_pc[i]=0; ROS_ERROR_STREAM("Saturating the command motor speed to 0");
    	  //printf("Saturating the command motor speed to 0");
    	  }

	  if ( (abs(speeds_pc[i])>(sat_value/2)) && (abs(speeds_pc[i])<sat_value) ) // fbf 19-07-2012. if motor speed is in between sat_Value/2  and sat_value, it becomes sat_value
  		  { ROS_ERROR_STREAM("Saturating the command motor speed to the saturation value");
  		  //printf("Saturating the command motor speed to the saturation value");
  		  if (speeds_pc[i]<0) speeds_pc[i]=-sat_value;
  		  else speeds_pc[i]=sat_value;
  		  }
  }
*/

  try
  {
    // check for zero crossings first
    MotorBoardCtrl::MotorSpeeds current_speeds;
    mbctrl_.getSpeeds(&current_speeds);
    ROS_INFO("current speeds %i %i %i %i", 
      current_speeds[0], current_speeds[1], current_speeds[2], current_speeds[3]);
    ROS_INFO("requested speeds %i %i %i %i", 
      speeds_pc[0], speeds_pc[1], speeds_pc[2], speeds_pc[3]);


    bool* change_direction = new bool[num_motors];
    bool has_direction_change = false;

    MotorBoardCtrl::MotorSpeeds temp_speeds;
    for (int i = 0; i < num_motors; ++i)
    {
      if (speeds_pc[i] * current_speeds[i] < 0) // zero crossing?
      {
        change_direction[i] = true;
        temp_speeds[i] = 0.0; // if change in the turn direction of motor i force a zero vel. in between
        has_direction_change = true;
      } else {
        change_direction[i] = false;
        temp_speeds[i] = speeds_pc[i]; // do not force a 0 for that motor if no change on the turn direction
      }
    }

    if (has_direction_change) // if one of the motor changes direction of turn, send corresponding speeds wiht the zeros 
    {
      ROS_DEBUG("sending speeds %i %i %i %i", 
        temp_speeds[0], temp_speeds[1], temp_speeds[2], temp_speeds[3]);
      mbctrl_.setSpeeds(temp_speeds);
      usleep(350000);
    }
    
// send desired speeds 
    for (int i = 0; i < num_motors; ++i)
    {
      if (change_direction[i])
      {
        temp_speeds[i] = speeds_pc[i];

      }
      ROS_DEBUG("no change direction sending speeds %i %i %i %i", 
          temp_speeds[0], temp_speeds[1], temp_speeds[2], temp_speeds[3]);
      mbctrl_.setSpeeds(temp_speeds);
      usleep(50000);
    }

    delete [] change_direction;

   // ROS_INFO("sending speeds %i %i %i %i", 
   //  speeds_pc[0], speeds_pc[1], speeds_pc[2], speeds_pc[3]);
   // mbctrl_.setSpeeds(speeds_pc);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error setting motor speeds : " << e.what());
  }
}

