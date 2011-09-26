/**
 * @file commandmsg.cpp
 * @brief Motor board command message parsers implementation
 * @author Joan Pau Beltran
 * @date 05/01/2011
 *
 * Functions in this file create and parse the command messages to be send to
 * and received from the motor board via the serial interface.
 *
 * In the functions provided by this file, the conversion between the character
 * representation and the integer value is done via the sscanf() and sprintf()
 * functions in <cstdlib>. sscanf() can only extract hexadecimals as unsigned
 * integers, and the conversion from unsigned integral values to a signed type
 * is not defined when the new type can not represent the unsigned value.
 * However, the conversion is done with static casts and works on gcc,
 * but may be implementation dependent and hence not portable.
 */

#include "commandmsg.h"
#include <cstdio>
#include <cstring>

////////////////////////////////////////////////////////////////////////////////
//!@addtogroup command
//!@{
const albatros_motor_board::CmdId* albatros_motor_board::MODULE_CMD_ID[NUM_MODULE_TYPES] =
                          { LED_CMD,       //!> LED cmd id table
                            MOTOR_CMD,     //!> MOTOR cmd id table
                            MOTORCTRL_CMD, //!> MOTORCTRL cmd id table
                            SENSOR_CMD     //!> SENSOR cmd id table
                          };

//! Command message format string according to the definition in header
const char cmd_msg_fmt[] = "S%03x%016s%04xX\r";

/**
 * @brief Compose a command request from the specified id, payload and checksum
 * @param req command request to compose
 * @param id  command identifier (from MODULE_CMD_ID[module][cmd])
 * @param payload  command payload
 * @param checksum command checksum already computed
 * @return false on errors building the command request, true if success
 */
bool albatros_motor_board::parseRequest(CmdMsg* req,
                                        CmdId id,
                                        const CmdPayload& payload,
                                        CmdChecksum checksum)
{
  const int n = sprintf(*req, cmd_msg_fmt, id, payload, checksum);
  return (n == CMD_MSG_LENGTH);
}

/**
 * @brief Extract command id, payload and checksum from given response
 * @param res command response to parse
 * @param id  pointer to the command identifier to extract
 * @param payload  pointer to the command payload to extract
 * @param checksum pointer to the command checksum to extract
 * @return false on errors parsing the command request, true if success.
 *      Output values may be improperly set if errors.
 */
bool albatros_motor_board::parseResponse(const CmdMsg& res,
                                         CmdId* id,
                                         CmdPayload* payload,
                                         CmdChecksum* checksum)
{
  const int n = sscanf(res, cmd_msg_fmt, id, *payload, checksum);
//std::cout << res[0] << std::endl;
//std::cout << *id << std::endl;
//std::cout << *payload << std::endl;
//std::cout << *checksum << std::endl;
//std::cout << res[CMD_MSG_LENGTH-2] << std::endl;
//std::cout << int(res[CMD_MSG_LENGTH-1]) << std::endl;
//std::cout << "Success " << (n==3) << std::endl;
  return (n == 3);
}
//!@} command group


////////////////////////////////////////////////////////////////////////////////
// LED COMMANDS
//!@addtogroup led
//!@{
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Compose request to get leds' maximum intensities
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseLedGetImaxRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_IMAX];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * Parse response with leds' maximum intensities
 * @param res the received response
 * @param imax0 led 0 maximum intensity (mA)
 * @param imax1 led 1 maximum intensity (mA)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedGetImaxResponse(const CmdMsg& res,
                                                   uint16_t* imax0,
                                                   uint16_t* imax1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_IMAX])
      && (2 == sscanf(payload, "%4x%4x%*8x", &val0, &val1)) )
  {
    *imax0 = val0;
    *imax1 = val1;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get leds' current intensities
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseLedGetIcurrentRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_ICURRENT];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get leds' current intensities
 * @param res response to parse
 * @param icur0 led 0 current intensity (mA)
 * @param icur1 led 1 current intensity (mA)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedGetIcurrentResponse(const CmdMsg& res,
                                                       uint16_t* icur0,
                                                       uint16_t* icur1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_ICURRENT])
      && (2 == sscanf(payload, "%4x%4x%*8x", &val0, &val1)) )
  {
    *icur0 = val0;
    *icur1 = val1;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get leds' levels
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
// TODO: There are 3 levels in albatross' ODS, but only 2 levels in their code
bool albatros_motor_board::parseLedGetLevelRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_LEVEL];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}


/**
 * @brief Parse response to get leds' levels
 * @param res response to parse
 * @param lev0 led 0 level
 * @param lev1 led 1 level
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedGetLevelResponse(const CmdMsg& res,
                                                    uint8_t* lev0,
                                                    uint8_t* lev1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_LEVEL])
      && (2 == sscanf(payload, "%2x%2x%*12x", &val0, &val1)) )
  {
    *lev0 = val0;
    *lev1 = val1;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to set leds' levels
 * @param req pointer to the request to fill
 * @param lev0 led 0 level (% of Imax)
 * @param lev1 led 1 level (% of Imax)
 * @return true on success, false on errors composing the request
 *
 * No check is done to to assert that arguments are in range 0..100.
 */
bool albatros_motor_board::parseLedSetLevelRequest(CmdMsg* req,
                                                   uint8_t lev0,
                                                   uint8_t lev1)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_SET_LEVEL];
  const CmdChecksum checksum = id + lev0 + lev1;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%02x%012x", lev0, lev1, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;

}

/**
 * @brief Parse response to set leds' levels
 * @param res response to parse
 * @param lev0 led 0 current level (% of Imax) //TODO: check meaning and units
 * @param lev1 led 1 current level (% of Imax) //TODO: check meaning and units
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedSetLevelResponse(const CmdMsg& res,
                                                    uint8_t* lev0,
                                                    uint8_t* lev1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_SET_LEVEL])
      && (2 == sscanf(payload, "%2x%2x%*12x", &val0, &val1)) )
  {
    *lev0 = val0;
    *lev1 = val1;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get leds' temperatures
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseLedGetTemperatureRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_TEMPERATURE];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get leds' temperatures
 * @param res response to parse
 * @param temp0 led 0 temperature (ºC)
 * @param temp1 led 1 temperature (ºC)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedGetTemperatureResponse(const CmdMsg& res,
                                                          uint16_t* temp0,
                                                          uint16_t* temp1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_TEMPERATURE])
      && (2 == sscanf(payload, "%4x%4x%*8x", &val0, &val1)) )
  {
    *temp0 = static_cast<int16_t>(val0);
    *temp1 = static_cast<int16_t>(val1);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose a request to get leds' status
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseLedGetStatusRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_STATUS];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get leds' status
 * @param res response to parse
 * @param stat0 led 0 status
 * @param stat1 led 1 status
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseLedGetStatusResponse(const CmdMsg& res,
                                                     uint8_t* stat0,
                                                     uint8_t* stat1)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_STATUS])
      && (2 == sscanf(payload, "%2x%2x%*12x", &val0, &val1)) )
  {
    *stat0 = val0;
    *stat1 = val1;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose a request to get motor board's firmware version
 * @param req pointer to the request to fill
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseGetVersionRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[LED][LED_GET_VERSION];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get motor board's firmware version
 * @param res response to parse
 * @param vers version number
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseGetVersionResponse(const CmdMsg& res,
                                                   uint8_t* vers)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val;
  if (parseResponse(res, &id, &payload, &checksum)
      && (id == MODULE_CMD_ID[LED][LED_GET_VERSION])
      && (1 == sscanf(payload, "%2x%*14x", &val)) )
  {
    *vers = val;
    return true;
  }
  else
    return false;
}

//!@} led group

////////////////////////////////////////////////////////////////////////
// MOTOR Commands
//!@addtogroup motor
//!@{
////////////////////////////////////////////////////////////////////////

/**
 * @brief Compose request to get motor configuration
 * @param req pointer to the request to compose
 * @param motor number of the motor
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetDeviceConfigRequest(CmdMsg* req,
                                                            uint8_t motor)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_DEVICE_CONFIG];
  const unsigned int val = motor << 5;
  const CmdChecksum checksum = id + val;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%014x", val, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get motor configuration
 * @param res response to parse
 * @param motor number of the motor
 * @param max_temp
 * @param rated_speed_rpm
 * @param no_load_speed_rpm
 * @param rated_current_ma
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * The response contain several parameters for the motor provided by the
 * manufacturer. The exact meaning of these parameters needs to be investigated.
 */
bool albatros_motor_board::parseMotorGetDeviceConfigResponse(const CmdMsg& res,
                                                             uint8_t* motor,
                                                             uint16_t* max_temp,
                                                             uint16_t* rated_speed_rpm,
                                                             uint16_t* no_load_speed_rpm,
                                                             uint16_t* rated_current_ma)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum) 
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_DEVICE_CONFIG] )
       && ( 4 == sscanf(payload, "%4x%4x%4x%4x", &val0, &val1, &val2, &val3) ) )
  {
    // the max_temp is in the last 13 bits
    // is the motor number in the first 3 bits? albatross code ignores them
    *motor = val0 >> 13;
    *max_temp = val0 & 0x1FFF;
    *rated_speed_rpm   = val1;
    *no_load_speed_rpm = val2;
    *rated_current_ma  = val3;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to set motors' speeds
 * @param req request to compose
 * @param rpm_pc0 motor 0 speed (+/-% of nominal motor speed)
 * @param rpm_pc1 motor 1 speed (+/-% of nominal motor speed)
 * @param rpm_pc2 motor 2 speed (+/-% of nominal motor speed)
 * @param rpm_pc3 motor 3 speed (+/-% of nominal motor speed)
 * @return true on success, false on errors composing the request
 *
 * No check is done to assert that arguments are in range 0..100.
 * The response to this command provides the speed in rpm instead of %.
 */
bool albatros_motor_board::parseMotorSetDirectionSpeedRequest(CmdMsg* req,
                                                              int8_t rpm_pc0,
                                                              int8_t rpm_pc1,
                                                              int8_t rpm_pc2,
                                                              int8_t rpm_pc3)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_SET_DIRECTION_SPEED];
  // bitwise-AND promotes the left argument to unsigned (because the other one
  // is unsigned) and sets the most significant bits to zero
  const unsigned int val0 = rpm_pc0 & 0xFF;
  const unsigned int val1 = rpm_pc1 & 0xFF;
  const unsigned int val2 = rpm_pc2 & 0xFF;
  const unsigned int val3 = rpm_pc3 & 0xFF;
  const CmdChecksum checksum = id + val0 + val1 + val2 + val3;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%02x%02x%02x%08x", val0, val1, val2, val3, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to set motors' speeds
 * @param res response to parse
 * @param rpm0 motor 0 current speed (rpm)
 * @param rpm1 motor 1 current speed (rpm)
 * @param rpm2 motor 2 current speed (rpm)
 * @param rpm3 motor 3 current speed (rpm)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * A request command sets motors' speeds in % instead of rpm.
 * When a request to set the motors' speeds is queried, motor board's response
 * provides the current speeds. Acceleration must be taken into account
 * (it may be set for each motor with the corresponding command).
 */
bool albatros_motor_board::parseMotorSetDirectionSpeedResponse(const CmdMsg& res,
                                                                 int16_t* rpm0,
                                                                 int16_t* rpm1,
                                                                 int16_t* rpm2,
                                                                 int16_t* rpm3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum) 
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_SET_DIRECTION_SPEED] )
       && ( 4 == sscanf(payload, "%4x%4x%4x%4x", &val0, &val1, &val2, &val3) ) )
  {
    // sscanf() can only extract hex values as unsigned integers
    // the conversion from unsigned to signed is not defined if the unsigned
    // value can not be represented in the signed type
    // the static_cast makes explicit this fact and works with gcc
    // (it works even without the casting) but may be not portable
    *rpm0 = static_cast<int16_t>(val0);
    *rpm1 = static_cast<int16_t>(val1);
    *rpm2 = static_cast<int16_t>(val2);
    *rpm3 = static_cast<int16_t>(val3);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get motors' speeds
 * @param req request to compose
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetDirectionSpeedRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_DIRECTION_SPEED];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get motors' speeds
 * @param res response to parse
 * @param rpm0 motor 0 current speed (rpm)
 * @param rpm1 motor 1 current speed (rpm)
 * @param rpm2 motor 2 current speed (rpm)
 * @param rpm3 motor 3 current speed (rpm)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseMotorGetDirectionSpeedResponse(const CmdMsg& res,
                                                               int16_t* rpm0,
                                                               int16_t* rpm1,
                                                               int16_t* rpm2,
                                                               int16_t* rpm3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum) 
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_DIRECTION_SPEED] )
       && ( 4 == sscanf(payload, "%4x%4x%4x%4x", &val0, &val1, &val2, &val3) ) )
  {
    *rpm0 = static_cast<int16_t>(val0);
    *rpm1 = static_cast<int16_t>(val1);
    *rpm2 = static_cast<int16_t>(val2);
    *rpm3 = static_cast<int16_t>(val3);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get motors' accelerations
 * @param req request to compose
 * @param accel0 motor 0 acceleration (% of nominal acceleration)
 * @param accel1 motor 1 acceleration (% of nominal acceleration)
 * @param accel2 motor 2 acceleration (% of nominal acceleration)
 * @param accel3 motor 3 acceleration (% of nominal acceleration)
 * @return true on success, false on errors composing the request
 *
 *  No check is done to assert that arguments are in range 0..100
 */
bool albatros_motor_board::parseMotorSetAccelRequest(CmdMsg* req,
                                                     uint8_t accel0,
                                                     uint8_t accel1,
                                                     uint8_t accel2,
                                                     uint8_t accel3)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_SET_ACCEL];
  const CmdChecksum checksum = id + accel0 + accel1 + accel2 + accel3;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%02x%02x%02x%08x", accel0, accel1, accel2, accel3, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to set motors' accelerations
 * @param res response to parse
 * @param accel0 motor current 0 acceleration (% of nominal acceleration) //TODO check meaning and units
 * @param accel1 motor current 1 acceleration (% of nominal acceleration) //TODO check meaning and units
 * @param accel2 motor current 2 acceleration (% of nominal acceleration) //TODO check meaning and units
 * @param accel3 motor current 3 acceleration (% of nominal acceleration) //TODO check meaning and units
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseMotorSetAccelResponse(const CmdMsg& res,
                                                      uint8_t* accel0,
                                                      uint8_t* accel1,
                                                      uint8_t* accel2,
                                                      uint8_t* accel3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_SET_ACCEL] )
       && ( 4 == sscanf(payload, "%2x%2x%2x%2x%*8x", &val0, &val1, &val2, &val3) ) )
  {
    *accel0 = val0;
    *accel1 = val1;
    *accel2 = val2;
    *accel3 = val3;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get motors' accelerations
 * @param req request to compose
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetAccelRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_ACCEL];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to set motors' accelerations
 * @param res response to parse
 * @param accel0 motor current 0 acceleration (% of nominal acceleration) //TODO check units
 * @param accel1 motor current 1 acceleration (% of nominal acceleration) //TODO check units
 * @param accel2 motor current 2 acceleration (% of nominal acceleration) //TODO check units
 * @param accel3 motor current 3 acceleration (% of nominal acceleration) //TODO check units
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseMotorGetAccelResponse(const CmdMsg& res,
                                                      uint8_t* accel0,
                                                      uint8_t* accel1,
                                                      uint8_t* accel2,
                                                      uint8_t* accel3)
{
    CmdId id;
    CmdPayload payload;
    CmdChecksum checksum;
    unsigned int val0, val1, val2, val3;
    if ( parseResponse(res, &id, &payload, &checksum)
         && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_ACCEL] )
         && ( 4 == sscanf(payload, "%2x%2x%2x%2x%*8x", &val0, &val1, &val2, &val3) ) )
    {
      *accel0 = val0;
      *accel1 = val1;
      *accel2 = val2;
      *accel3 = val3;
      return true;
    }
    else
      return false;
}

/**
 * @brief Compose request to get motors' temperatures
 * @param req request to compose
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetTemperatureRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_TEMPERATURE];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get motors' temperatures
 * @param res response to parse
 * @param temp0 motor 0 temperature (ºC)
 * @param temp1 motor 1 temperature (ºC)
 * @param temp2 motor 2 temperature (ºC)
 * @param temp3 motor 3 temperature (ºC)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
//TODO: Check the type of arguments (compare to albatross' code, should be uint16_t?)
bool albatros_motor_board::parseMotorGetTemperatureResponse(const CmdMsg& res,
                                                            int16_t* temp0,
                                                            int16_t* temp1,
                                                            int16_t* temp2,
                                                            int16_t* temp3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_TEMPERATURE] )
       && ( 4 == sscanf(payload, "%4x%4x%4x%4x", &val0, &val1, &val2, &val3) ) )
  {
    *temp0 = static_cast<int16_t>(val0);
    *temp1 = static_cast<int16_t>(val1);
    *temp2 = static_cast<int16_t>(val2);
    *temp3 = static_cast<int16_t>(val3);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get current motors' intensities
 * @param req request to compose
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetIcurrentRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_ICURRENT];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get current motors' intensities
 * @param res response to parse
 * @param icurr0 motor 0 current intensity (mA)
 * @param icurr1 motor 1 current intensity (mA)
 * @param icurr2 motor 2 current intensity (mA)
 * @param icurr3 motor 3 current intensity (mA)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
// TODO: Check the type of arguments, they should be int16_t?
bool albatros_motor_board::parseMotorGetIcurrentResponse(const CmdMsg& res,
                                                         uint16_t* icurr0,
                                                         uint16_t* icurr1,
                                                         uint16_t* icurr2,
                                                         uint16_t* icurr3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_ICURRENT] )
       && ( 4 == sscanf(payload, "%4x%4x%4x%4x", &val0, &val1, &val2, &val3) ) )
  {
    *icurr0 = val0;
    *icurr1 = val1;
    *icurr2 = val2;
    *icurr3 = val3;
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get motors' status
 * @param req request to compose
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorGetStatusRequest(CmdMsg* req)
{
  const CmdId id = MODULE_CMD_ID[MOTOR][MOTOR_GET_STATUS];
  const CmdChecksum checksum = id;
  CmdPayload payload;
  const int n = sprintf(payload, "%0*x", CMD_MSG_PAYLOAD_LENGTH, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get motors' status
 * @param res response to parse
 * @param stat0 motor 0 status
 * @param stat1 motor 1 status
 * @param stat2 motor 2 status
 * @param stat3 motor 3 status
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * The status of a motor is supposed to be the accumulated count of motor
 * stops. A motor stops (sets its velocity to 0) when the input current
 * exceeds a security threshold). When the number of stops reaches a fixed (but
 * unknown) value, the motor halts (it does not respond to commands anymore).
 */
bool albatros_motor_board::parseMotorGetStatusResponse(const CmdMsg& res,
                                                       uint8_t* stat0,
                                                       uint8_t* stat1,
                                                       uint8_t* stat2,
                                                       uint8_t* stat3)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTOR][MOTOR_GET_STATUS] )
       && ( 4 == sscanf(payload, "%2x%2x%2x%2x%*8x", &val0, &val1, &val2, &val3) ) )
  {
    *stat0 = val0;
    *stat1 = val1;
    *stat2 = val2;
    *stat3 = val3;
    return true;
  }
  else
    return false;
}

//!@} led group


////////////////////////////////////////////////////////////////////////
// MOTORCTRL Commands
//!@addtogroup motorctrl
//!@{
////////////////////////////////////////////////////////////////////////

/**
 * @brief Compose request to get control constants for a motor
 * @param req request to compose
 * @param motor number of the motor
 * @return true on success, false on errors composing the request
 */
bool albatros_motor_board::parseMotorctrlGetConstantsRequest(CmdMsg* req,
                                                             uint8_t motor)
{
  const CmdId id = MODULE_CMD_ID[MOTORCTRL][MOTORCTRL_GET_CONSTANTS];
  const CmdChecksum checksum = id + motor;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%014x", motor, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get control constants of a motor
 * @param res the response to parse
 * @param motor motor                         //TODO: Check meaning on response
 * @param active if the control is enabled or disabled
 * @param kp_q15 PID proportional constant (Q15 format)
 * @param ki_q15 PID integral constant (Q15 format)
 * @param kd_q15 PID diferential constant (Q15 format)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * The controller may be enabled or disabled with the boolean flag.
 *
 * The constants are in Q15 format.  In general Qm.n format is a fixed point
 * numeric format that uses m+n+1 bits to represent a signed decimal number:
 * 1 bit is for the sign, m bits are set aside for the integral part, and n bits
 * are used for the decimal part. Actually, the number can be thought as an
 * integer count of chunks of size 2^(-n).
 * Conversion to/from float is done by dividing/multiplying by 2^n
 * (with implicit integer rounding in the latter case). So to convert these
 * constants:
 *
 * - Q15 to float:
 * @code
 *   int16_t q; // the value in Q15 format
 *   ...
 *   float f = float(q) / 32768.0; // 2^15 = 32768
 * @endcode
 *
 * - float to Q15:
 * @code
 *   float f; // the value in float
 *   ...
 *   int16_t q = f / 32768.0; // 2^15 = 32768 and implicit rounding to int
 * @endcode
 */
bool albatros_motor_board::parseMotorctrlGetConstantsResponse(const CmdMsg& res,
                                                              uint8_t* motor,
                                                              bool* active,
                                                              int16_t* kp_q15,
                                                              int16_t* ki_q15,
                                                              int16_t* kd_q15)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int mot, act, val0, val1, val2;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTORCTRL][MOTORCTRL_GET_CONSTANTS] )
       && ( 5 == sscanf(payload, "%2x%2x%4x%4x%4x", &mot, &act, &val0, &val1, &val2) ) )
  {
    *motor = mot;
    *active = act & 0x1;
    *kp_q15 = static_cast<int16_t>(val0);
    *ki_q15 = static_cast<int16_t>(val1);
    *kd_q15 = static_cast<int16_t>(val2);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to set control constants for a motor
 * @param req request to compose
 * @param motor motor number
 * @param active if control should be enabled or disabled
 * @param kp_q15 PID proportional constant (Q15 format)
 * @param ki_q15 PID integral constant (Q15 format)
 * @param kd_q15 PID diferential constant (Q15 format)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * No check is done to assert the validity of the arguments.
 *
 * For an explanation of the Q15 format see parseMotorctrlGetConstantsResponse()
 */
bool albatros_motor_board::parseMotorctrlSetConstantsRequest(CmdMsg* req,
                                                             uint8_t motor,
                                                             bool active,
                                                             int16_t kp_q15,
                                                             int16_t ki_q15,
                                                             int16_t kd_q15)
{
  const CmdId id = MODULE_CMD_ID[MOTORCTRL][MOTORCTRL_SET_CONSTANTS];
  unsigned int plsb = kp_q15 & 0xFF;        // proportional least significant byte
  unsigned int pmsb = (kp_q15 >> 8) & 0xFF; // proportional most significant byte
  unsigned int ilsb = ki_q15 & 0xFF;        // integral least significant byte
  unsigned int imsb = (ki_q15 >> 8) & 0xFF; // integral most significant byte
  unsigned int dlsb = kd_q15 & 0xFF;        // derivative least significant byte
  unsigned int dmsb = (kd_q15 >> 8) & 0xFF; // integral most significant byte
  const CmdChecksum checksum = id + motor + active
                                  + plsb + pmsb + ilsb + imsb + dlsb + dmsb;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%02x%02x%02x%02x%02x%02x%02x",
                        motor, active, pmsb, plsb, imsb, ilsb, dmsb, dlsb);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to set control constants of a motor
 * @param res the response to parse
 * @param motor motor                         //TODO: Check meaning on response
 * @param active if the control is enabled or disabled
 * @param kp_q15 PID proportional constant (Q15 format)
 * @param ki_q15 PID integral constant (Q15 format)
 * @param kd_q15 PID diferential constant (Q15 format)
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * For an explanation of the Q15 format see parseMotorctrlGetConstantsResponse()
 */
bool albatros_motor_board::parseMotorctrlSetConstantsResponse(const CmdMsg& res,
                                                              uint8_t* motor,
                                                              bool* active,
                                                              int16_t* kp_q15,
                                                              int16_t* ki_q15,
                                                              int16_t* kd_q15)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int mot, act, val0, val1, val2;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[MOTORCTRL][MOTORCTRL_SET_CONSTANTS] )
       && ( 5 == sscanf(payload, "%2x%2x%4x%4x%4x", &mot, &act, &val0, &val1, &val2) ) )
  {
    *motor = mot;
    *active = act & 0x1;
    *kp_q15 = static_cast<int16_t>(val0);
    *ki_q15 = static_cast<int16_t>(val1);
    *kd_q15 = static_cast<int16_t>(val2);
    return true;
  }
  else
    return false;
}

//!@} motorctrl group


////////////////////////////////////////////////////////////////////////
// SENSOR Commands
//!@addtogroup sensor
//!@{
////////////////////////////////////////////////////////////////////////

/**
 * @brief Compose request to get sensor configuration
 * @param req request to compose
 * @param sensor number of the sensor
 * @return true on success, false on errors
 *
 * No check is done to assert the validity of the sensor number
 */
bool albatros_motor_board::parseSensorGetDeviceConfigRequest(CmdMsg* req,
                                                             uint8_t sensor)
{
  const CmdId id = MODULE_CMD_ID[SENSOR][SENSOR_GET_DEVICE_CONFIG];
  const CmdChecksum checksum = id + sensor;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%014x", sensor, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get sensor configuration
 * @param res response to parse
 * @param sensor number of the sensor  //TODO: Check on response
 * @param type sensor type
 * @param unit_res resolution of the measurement units.
 *              Sensor value is lecture*10^unit_res (e.g. if unit_res is -3 lecture values are in milis)
 * @param ADCbits number of bits used by the sensor analogic-digital conversor //TODO: Just a mere supposition, check it
 * @param min minimum value the sensor is sensitive to
 * @param max maximum value the sensor is sensitive to
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseSensorGetDeviceConfigResponse(const CmdMsg& res,
                                                              uint8_t* sensor,
                                                              uint8_t* type,
                                                              int8_t* unit_res,
                                                              uint8_t* ADCbits,
                                                              int16_t* min,
                                                              int16_t* max)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2, val3, val4, val5;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[SENSOR][SENSOR_GET_DEVICE_CONFIG] )
       && ( 6 == sscanf(payload, "%2x%2x%2x%2x%4x%4x",
                        &val0, &val1, &val2, &val3, &val4, &val5) ) )
  {
    *sensor = val0;
    *type = val1;
    *unit_res = static_cast<int8_t>(val2);
    *ADCbits = static_cast<uint8_t>(val3);
    *min = static_cast<int16_t>(val4);
    *max = static_cast<int16_t>(val5);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get sensor offset
 * @param req request to compose
 * @param sensor number of the sensor
 * @return true on success, false on errors
 *
 * No check is done to assert the validity of the sensor number
 */
bool albatros_motor_board::parseSensorGetOffsetRequest(CmdMsg* req,
                                                       uint8_t sensor)
{
  const CmdId id = MODULE_CMD_ID[SENSOR][SENSOR_GET_OFFSET];
  const CmdChecksum checksum = id + sensor;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%014x", sensor, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get sensor offset
 * @param res response to parse
 * @param sensor number of the sensor
 * @param offset current sensor offset
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseSensorGetOffsetResponse(const CmdMsg& res,
                                                        uint8_t* sensor,
                                                        int16_t* offset)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[SENSOR][SENSOR_GET_OFFSET] )
       && ( 2 == sscanf(payload, "%2x%4x%*10x", &val0, &val1) ) )
  {
    *sensor = val0;
    *offset = static_cast<int16_t>(val1);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to set sensor offset
 * @param req request to compose
 * @param sensor number of the sensor
 * @param offset sensor offset to be set
 * @return true on success, false on errors
 *
 * No check is done to assert the validity of the sensor number
 */
bool albatros_motor_board::parseSensorSetOffsetRequest(CmdMsg* req,
                                                       uint8_t sensor,
                                                       int16_t offset)
{
  const CmdId id = MODULE_CMD_ID[SENSOR][SENSOR_SET_OFFSET];
  const unsigned int off_msb = (offset >> 8) & 0xFF; // offset most significant byte
  const unsigned int off_lsb = offset & 0xFF;        // offset least significant byte
  const CmdChecksum checksum = id + sensor + off_msb + off_lsb;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%02x%02x%010x", sensor, off_msb, off_lsb, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to set sensor offset
 * @param res   response to parse
 * @param sensor number of the sensor
 * @param offset current offset         //TODO: check meaning on response
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 */
bool albatros_motor_board::parseSensorSetOffsetResponse(const CmdMsg& res,
                                                        uint8_t* sensor,
                                                        int16_t* offset)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[SENSOR][SENSOR_SET_OFFSET] )
       && ( 2 == sscanf(payload, "%2x%4x%*10x", &val0, &val1) ) )
  {
    *sensor = val0;
    *offset = static_cast<int16_t>(val1);
    return true;
  }
  else
    return false;
}

/**
 * @brief Compose request to get sensor value
 * @param req request to compose
 * @param sensor number of the sensor
 * @return true on succes, false on errors
 *
 * No check is done to assert the validity of the sensor number
 */
bool albatros_motor_board::parseSensorGetValueRequest(CmdMsg* req,
                                                      uint8_t sensor)
{
  const CmdId id = MODULE_CMD_ID[SENSOR][SENSOR_GET_VALUE];
  const CmdChecksum checksum = id + sensor;
  CmdPayload payload;
  const int n = sprintf(payload, "%02x%014x", sensor, 0);
  if (n == CMD_MSG_PAYLOAD_LENGTH)
    return parseRequest(req, id, payload, checksum);
  else
    return false;
}

/**
 * @brief Parse response to get sensor value
 * @param res response to parse
 * @param sensor number of the sensor
 * @param sensor_val sensor value in magnitude units (depending on the kind of sensor)
 * @param adc_val bit representation of the value from the analogic-digital converter //TODO: Check it
 * @return true on success, false on errors parsing the response
 *      (output values are changed only when the command identifier in response
 *      is correct)
 *
 * The units and resolutions of the sensor samples depend on the kind of sensor
 * and its configuration from parseSensorGetDeviceConfigResponse().
 * The meaning of the sampled value may also depend on sensor's offset from
 * parseSensorGetOffsetResponse().
 */
bool albatros_motor_board::parseSensorGetValueResponse(const CmdMsg& res,
                                                       uint8_t* sensor,
                                                       int16_t* sensor_val,
                                                       int32_t* adc_val)
{
  CmdId id;
  CmdPayload payload;
  CmdChecksum checksum;
  unsigned int val0, val1, val2;
  if ( parseResponse(res, &id, &payload, &checksum)
       && ( id == MODULE_CMD_ID[SENSOR][SENSOR_GET_VALUE] )
       && ( 3 == sscanf(payload, "%2x%4x%8x%*2x", &val0, &val1, &val2) ) )
  {
    *sensor = val0;
    *sensor_val = static_cast<int16_t>(val1);
    *adc_val = static_cast<int32_t>(val1);
    return true;
  }
  else
    return false;
}

//!@} sensor group

