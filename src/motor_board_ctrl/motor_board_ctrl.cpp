/**
 * @file motorboardctrl.cpp
 * @brief Motor board controller class implementation
 * @author Joan Pau Beltran
 * @date 2011-01-03
 *
 * The class implemented in this file provide an interface to communicate with the
 * motor board and control its modules. Currently, these modules are the motors
 * and its built in PID controllers, and two sensors (pressure and water in).
 *
 * Communication with the board is done through a serial port with a set of
 * request/response commands. The SerialComm class provides the serial deals
 * with the serial communication issues.
 * Command's format and syntax is addressed by the commandmsg module.
 */


#include "motor_board_ctrl.h"
#include <cstring>

/**
 * @brief Default constructor
 */
albatros_motor_board::MotorBoardCtrl::MotorBoardCtrl() :
  comm_()
{
}

/**
 * @brief Initialize specified device for serial communications
 * @param dev_name device file name (including full path) representing the
 *              serial port
 */
void albatros_motor_board::MotorBoardCtrl::openComm(std::string dev_name)
{
  int status = 0;
  if (!comm_.openDevice(dev_name, status))
    throw MotorBoardError("Error opening device " + dev_name + " ("
                          + strerror(status) + ")");

  if (!(comm_.setBaudRate(commBaudRate_) && comm_.setDataBits(commDataBits_)
      && comm_.setParity(commParity_) && comm_.setStopBits(commStopBits_)
      && comm_.setReadTimeout(commReadTimeout_)))
    throw MotorBoardError("Error initializing device " + comm_.getDeviceName()
                          + "for serial communication ");

}

/**
 * @brief Close serial communication
 */
void albatros_motor_board::MotorBoardCtrl::closeComm()
{
  int status = 0;
  if (!comm_.closeDevice(status))
    throw MotorBoardError("Error closing device " + comm_.getDeviceName()
                          + " (" + strerror(status) + ")");
}

/**
 * @brief Flush serial communication 
 * 
 * Discard bytes received from the board but not read, 
 * and bytes written but not sent to the board
 */
void albatros_motor_board::MotorBoardCtrl::flushComm()
{
  int status;
  bool success = comm_.flushBuffer(status);
  if (!success)
    throw MotorBoardError("Error flushing device "  + comm_.getDeviceName()
                          + " (" + strerror(status) + ")" );
}

/**
 * @brief Get the device file name of the serial port
 * @return device file name (including full path) set on opening
 */
std::string albatros_motor_board::MotorBoardCtrl::getCommName()
{
  return comm_.getDeviceName();
}

/**
 * @brief Read a command message (response) from the motor board
 * @param cmd command to be read
 *
 * The read operation blocks for a certain amount of time. If there is an error
 * during the reading operation (including no response from the board)
 * an exception is thrown.
 */
void albatros_motor_board::MotorBoardCtrl::readCommand(CmdMsg* cmd)
{
  unsigned long numBytesRead;
  int status = 0;
  bool success = false;
  unsigned long totalBytesRead = 0;
  for ( unsigned int numRetries = 4; (!success) && numRetries>0; numRetries-- )
  {
    success = comm_.readData( CMD_MSG_LENGTH - totalBytesRead, 
                              (*cmd) + totalBytesRead, 
                              numBytesRead, status);
    totalBytesRead += numBytesRead;
  }
  (*cmd)[totalBytesRead] = '\0'; // Add null character after the last character read.
  if (!success)
    throw MotorBoardError(
        "Error reading command from " + comm_.getDeviceName()
        + " (" + ((status == 0) ? "bad response" : strerror(status)) + ") : "
        + std::string(*cmd) );
}


/**
 * @brief Send command message (request) to the motor board
 * @param cmd The command to be sent
 *
 * If there are errors during the writing operation an exception is thrown
 */
void albatros_motor_board::MotorBoardCtrl::sendCommand(const CmdMsg& cmd)
{
  int status;
  unsigned long numBytesWritten;
  bool success = comm_.writeData(CMD_MSG_LENGTH, cmd, numBytesWritten, status);
  if (!success)
    throw MotorBoardError(
        "Error writing command to "  + comm_.getDeviceName()
        + " (" + strerror(status) + ") : " + std::string(cmd,numBytesWritten) );
}

/**
 * @brief Query board and wait for response
 * @param request command message to be sent
 * @param response received command message
 *
 * This function is just a wrapper for the sendCommand() and readCommand()
 * functions. These calls raise an exception when errors
 */
void albatros_motor_board::MotorBoardCtrl::queryCommand(const CmdMsg& request,
                                              CmdMsg* response)
{
  //TODO: Check serial flush before sending the request
  flushComm();
  sendCommand(request);
  readCommand(response);
}


/**
 * @brief Read firmware version from motor board
 * @param num version number returned by to the board
 *
 * Wrapper to the respective parsers and query calls
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getVersion(int* num)
{
  CmdMsg request, response;
  uint8_t num8;
  if(!parseGetVersionRequest(&request))
    throw MotorBoardError( "Error parsing get version request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseGetVersionResponse(response, &num8))
    throw MotorBoardError( "Error parsing get version response : " + std::string(response) );
  *num = num8;
}


/**
 * @brief Read speeds from motor board
 * @param rpm speeds returned to the board (rpm)
 *
 * Wrapper to the respective parsers and query calls
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getSpeeds(MotorSpeeds* rpm)
{
  CmdMsg request, response;
  int16_t rpm16[NUM_MOTORS];
  if(!parseMotorGetDirectionSpeedRequest(&request))
    throw MotorBoardError( "Error parsing get speed request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorGetDirectionSpeedResponse(response, &rpm16[0], &rpm16[1],
                                          &rpm16[2], &rpm16[3]))
    throw MotorBoardError( "Error parsing get speed response : " + std::string(response) );
  for (int i=0; i<NUM_MOTORS; i++)
    (*rpm)[i] = rpm16[i];
}

/**
 * @brief Request new speeds to motor board
 * @param req_pc requested speeds (\% of the nominal speed)
 * @param res_rpm current speeds returned by the motor board (rpm)
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::setSpeeds(const MotorSpeeds& req_pc,
                                           MotorSpeeds* res_rpm)
{
  CmdMsg request, response;
  int16_t rpm16[NUM_MOTORS];
  for (int i=0; i<NUM_MOTORS; i++ )
    if ( req_pc[i]<-100 || req_pc[i]>100 )
      throw MotorBoardError("Bad speed argument (not in -100..100)");
  if(!parseMotorSetDirectionSpeedRequest(&request, req_pc[0], req_pc[1], req_pc[2], req_pc[3]))
    throw MotorBoardError( "Error parsing set speed request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorSetDirectionSpeedResponse(response, &rpm16[0], &rpm16[1], &rpm16[2], &rpm16[3]) )
     throw MotorBoardError( "Error parsing set speed response : " + std::string(response) );
  if (res_rpm)
    for (int i=0; i<NUM_MOTORS; i++)
      (*res_rpm)[i] = rpm16[i];
}

/**
 * @brief Read accelerations from motor board
 * @param pc accelerations returned by the board (\%/ds percentage of nominal speed per decisecond)
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getAccels(MotorAccels* pc)
{
  CmdMsg request, response;
  uint8_t pc8[NUM_MOTORS];
  if(!parseMotorGetAccelRequest(&request))
    throw MotorBoardError( "Error parsing get accel request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorGetAccelResponse(response, &pc8[0], &pc8[1], &pc8[2], &pc8[3]))
    throw MotorBoardError( "Error parsing get accel response : " + std::string(response) );
  for (int i=0; i<NUM_MOTORS; i++)
    (*pc)[i] = pc8[i];
}

/**
 * @brief Request new accelerations to motor board
 * @param req_pc_ds requested accelerations (\%/ds percentage of nominal speed per decisecond)
 * @param res_pc_ds current accelerations returned by the motor board (\%/ds percentage of nominal speed per decisecond)
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::setAccels(const MotorAccels& req_pc_ds,
                                           MotorAccels* res_pc_ds)
{
  CmdMsg request, response;
  uint8_t pc8[NUM_MOTORS];
  for (int i=0; i<NUM_MOTORS; i++ )
    if ( req_pc_ds[i]<0 || req_pc_ds[i]>100 )
      throw MotorBoardError("Bad speed argument (not in 0..100)");
  if(!parseMotorSetAccelRequest(&request, req_pc_ds[0], req_pc_ds[1], req_pc_ds[2], req_pc_ds[3]))
    throw MotorBoardError( "Error parsing set accel request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorSetAccelResponse(response, &pc8[0], &pc8[1], &pc8[2], &pc8[3]) )
     throw MotorBoardError( "Error parsing set accel response : " + std::string(response) );
  if (res_pc_ds)
    for (int i=0; i<NUM_MOTORS; i++)
      (*res_pc_ds)[i] = pc8[i];
}

void albatros_motor_board::MotorBoardCtrl::getStatus(MotorStatus* errors)
{
  CmdMsg request, response;
  uint8_t errors8[4] = {0,0,0,0};
  if(!albatros_motor_board::parseMotorGetStatusRequest(&request))
    throw MotorBoardError("Error parsing get status request : " + std::string(request) );
  queryCommand(request,&response);
  if(!albatros_motor_board::parseMotorGetStatusResponse(response, &errors8[0], &errors8[1], &errors8[2], &errors8[3]))
    throw MotorBoardError("Error parsing get status response : " + std::string(response) );
  for (int i=0; i<NUM_MOTORS; i++)
    (*errors)[i] = errors8[i];
}

/**
 * @brief Read built in PID controller's configuration for a motor
 * @param motor_id selected motor
 * @param active true if the built in controller is enabled, false otherwise
 * @param Kpid control constants Kp, Ki and Kd (in -1..1)
 *
 * Wrapper to the respective parsers and query calls.
 * This function also does the conversion to float from Q15 format returned
 * by the parsers.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getMotorctrl(const Motor motor_id,
                                              bool* active, PIDConstants* Kpid)
{
  CmdMsg request, response;
  int16_t KpidQ15[NUM_CONSTANTS];
  uint8_t motor_num;
  bool active_aux;
  if(!parseMotorctrlGetConstantsRequest(&request, motor_id))
    throw MotorBoardError( "Error parsing get control request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorctrlGetConstantsResponse(response, &motor_num, &active_aux,
                                         &KpidQ15[0], &KpidQ15[1], &KpidQ15[2]))
    throw MotorBoardError( "Error parsing get control response : " + std::string(response) );
  if (motor_num != motor_id)
    throw MotorBoardError( "Error wrong get control response (different motor)" );
  *active = active_aux;
  for (int i=0; i<NUM_CONSTANTS; i++)
    (*Kpid)[i] = KpidQ15[i] / 32768.0; // 2^15 = 32768
}

/**
 * @brief Set built in PID controller's configuration for a motor
 * @param motor_id selected motor
 * @param active enable built in PID controller if true, disable it otherwise
 * @param Kpid Kp, Ki and Kd control constants (in range [-1..1[ )
 * @param res_active response PID controller status
 * @param res_Kpid response PID controller constants
 *
 * Wrapper to the respective parsers and query calls.
 * This function also does the conversion from float to Q15 format as needed by
 * the parsers.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::setMotorctrl(const Motor motor_id,
                                              bool active, const PIDConstants& Kpid,
                                              bool* res_active, PIDConstants* res_Kpid)
{
  CmdMsg request, response;
  uint8_t motor_num;
  bool active_aux;
  int16_t KpidQ15[NUM_CONSTANTS];
  for (int i=0; i<NUM_CONSTANTS; i++)
  {
    if (Kpid[i]<-1.0 || Kpid[i]>=1.0)
      throw MotorBoardError("Bad PID constant ( not in range [-1.0..1.0[ )");
    KpidQ15[i] = Kpid[i]*32768; // 2^15 = 32768
  }
  if(!parseMotorctrlSetConstantsRequest(&request, motor_id, active,
                                        KpidQ15[0], KpidQ15[1], KpidQ15[2]))
    throw MotorBoardError( "Error parsing set control request : " + std::string(request) );
  queryCommand(request,&response);
  if(!parseMotorctrlSetConstantsResponse(response, &motor_num, &active_aux,
                                         &KpidQ15[0], &KpidQ15[1], &KpidQ15[2]))
    throw MotorBoardError( "Error parsing set control response : " + std::string(response) );
  if (motor_num != motor_id)
    throw MotorBoardError( "Error wrong set control response (different motor)" );
  if (res_active)
    *res_active = active_aux;
  if (res_Kpid)
    for (int i=0; i<3; i++)
      (*res_Kpid)[i] = KpidQ15[i] / 32768.0; // 2^15 = 32768
}

/**
 * @brief Read configuration for a sensor
 * @param sensor_id desired sensor
 * @param type kind of sensor
 * @param res output sensor resolution units (e.g. -3 equal mili)
 * @param min minimum sensor value
 * @param max maximum sensor value
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getSensorConfig(const Sensor sensor_id,
                                                 int* type, int* res, int* ADCbits,
                                                 int* min, int* max)
{
  CmdMsg request, response;
  uint8_t sensor_num, type8, ADCbits8;
  int8_t res8;
  int16_t min16, max16;
  if(!parseSensorGetDeviceConfigRequest(&request, sensor_id))
    throw MotorBoardError( "Error parsing get sensor config request : "
                           + std::string(request) );
  queryCommand(request,&response);
  if(!parseSensorGetDeviceConfigResponse(response, &sensor_num, &type8, &res8,
                                         &ADCbits8, &min16, &max16) )
    throw MotorBoardError( "Error parsing get sensor config response : "
                           + std::string(response) );
  if (sensor_num != sensor_id)
    throw MotorBoardError( "Error wrong get sensor config response (different sensor)" );
  *type = type8;
  *res = res8;
  *ADCbits = ADCbits8;
  *min = min16;
  *max = max16;
}

/**
 * @brief Read current offset of a sensor
 * @param sensor_id selected sensor
 * @param offset current offset
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::getSensorOffset(const Sensor sensor_id,
                                                 int* offset)
{
  albatros_motor_board::CmdMsg request, response;
  uint8_t sensor_num;
  int16_t offset16;
  if(!parseSensorGetOffsetRequest(&request, sensor_id))
    throw MotorBoardError( "Error parsing get sensor offset request : "
                           + std::string(request) );
  queryCommand(request,&response);
  if(!parseSensorGetOffsetResponse(response, &sensor_num, &offset16))
    throw MotorBoardError( "Error parsing get sensor offset response : "
                           + std::string(response) );
  if (sensor_num != sensor_id)
    throw MotorBoardError( "Error wrong get sensor config response (different sensor)" );
  *offset = offset16;
}

/**
 * @brief Request new value for sensor's offset
 * @param sensor_id selected sensor
 * @param offset new offset value
 * @param res_offset response offset
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 */
void albatros_motor_board::MotorBoardCtrl::setSensorOffset(const Sensor sensor_id,
                                                 int offset, int* res_offset)
{
  CmdMsg request, response;
  uint8_t sensor_num;
  int16_t offset16;
  if(!parseSensorSetOffsetRequest(&request, sensor_id, offset))
    throw MotorBoardError( "Error parsing set sensor request : "
                           + std::string(request) );
  queryCommand(request,&response);
  if(!parseSensorSetOffsetResponse(response, &sensor_num, &offset16))
    throw MotorBoardError( "Error parsing set sensor response : "
                           + std::string(response));
  if (sensor_num != sensor_id)
    throw MotorBoardError( "Error wrong set sensor config response (different sensor)" );
  if (res_offset)
    *res_offset = offset16;
}

/**
 * @brief Read current value of sensor
 * @param sensor_id selected sensor
 * @param value current sensor's value
 * @param adc_value bit representation of sensor's value from the AD converter
 *
 * Wrapper to the respective parsers and query calls.
 * On errors it throws an exception
 * TODO: Check output meaning (related with offset)
 */
void albatros_motor_board::MotorBoardCtrl::getSensorValue(const Sensor sensor_id,
                                                int* value, int* adc_value)
{
  CmdMsg request, response;
  uint8_t sensor_num;
  int16_t sensor_val16;
  int32_t adc_val32;
  if(!parseSensorGetValueRequest(&request, sensor_id))
    throw MotorBoardError( "Error parsing get sensor value request : "
                           + std::string(request) );
  queryCommand(request, &response);
  if(!parseSensorGetValueResponse(response, &sensor_num, &sensor_val16, &adc_val32))
    throw MotorBoardError( "Error parsing get sensor value response : "
                           + std::string(response) );
  *value = sensor_val16;
  if (adc_value)
    *adc_value = adc_val32;
}

/**
 * @brief Motor identifier output operator
 * @param ostr output stream
 * @param m motor identifier
 * @return reference to the output stream
 */
std::ostream& albatros_motor_board::operator<<(std::ostream& ostr,
                                              const MotorBoardCtrl::Motor& m)
{
  return ostr << int(m);
}

/**
 * @brief Sensor identifier output operator
 * @param ostr output stream
 * @param s sensor identifier
 * @return reference to the output stream
 */
std::ostream& albatros_motor_board::operator<<(std::ostream& ostr,
                                              const MotorBoardCtrl::Sensor& s)
{
  return ostr << int(s);
}

/**
 * @brief PID constant identifier output operator
 * @param ostr output stream
 * @param k PID constant indetifier
 * @return reference to the output stream
 */
std::ostream& albatros_motor_board::operator<<(std::ostream& ostr,
                                              const MotorBoardCtrl::PIDConstant& k)
{
  return ostr << int(k);
}
