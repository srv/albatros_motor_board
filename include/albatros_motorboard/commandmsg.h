/**
 * @file commandmsg.h
 * @brief Motor board modules and command message parsers presentation
 * @author Joan Pau Beltran
 * @date 05/01/2011
 *
 * The motor board is build as a set of modules, each one with its own commands
 * to control its capabilities. At the moment the modules are leds, motors and ç
 * sensors of several kinds (pressure, temperature, voltage and depth),
 * but not all of them need to be available in a specific board.
 *
 * Communication and control of the motor board modules is done through a set of
 * request-response messages on the serial port.
 *
 * This file presents the different modules and their commands,
 * and provide functions to parse the request/response messages for each module
 * command.
 *
 * A message is a character sequence of the form: "S0120123456789abcdef0123X\r"
 * and has the following parts:
 *
 * - start_cmd : "S" single 'S' char
 * - cmd_id    : "012" 3 chars with the hexadecimal representation of a 11 bit
 *                      command identifier
 * - payload   : "0123456789abcdef" 16 chars with the hexadecimal representation
 *                      of a 8 byte payload with response data or command
 *                      arguments
 * - checksum  : "0123" 4 chars with the hexadecimal representation of the 2 byte
 *                      checksum of the values represented by cmd_id and payload
 *                      (the checksum is ignored in responses since it does not
 *                      seem to make sense: 2 bytes are not enough)
 * - end_cmd   : "X\r" single 'X' char followed by carriage return '\r'
 *                      (the board sends responses line by line, and using
 *                      carriage return character '\r' as end of line mark)
 *
 * Some modules have commands with arguments (e.g. the motor module has a
 * command to set the speed of the motors in % of nominal speed).
 * The parsers do not check the value of these arguments in any way.
 * They are just translated to their hexadecimal representation in characters.
 */

#ifndef COMMANDMSG_H
#define COMMANDMSG_H

#include <string>
#include <stdint.h>

namespace albatros_motorboard
{

/**
 * @defgroup command Command message definition
 * @brief Command format definition
 */
//!@{
// Command message definition
const int CMD_MSG_START_LENGTH = 1;
const int CMD_MSG_ID_LENGTH = 3;
const int CMD_MSG_PAYLOAD_LENGTH = 16;
const int CMD_MSG_CHECKSUM_LENGTH = 4;
const int CMD_MSG_END_LENGTH = 2;

const int CMD_MSG_LENGTH = CMD_MSG_START_LENGTH + CMD_MSG_ID_LENGTH
    + CMD_MSG_PAYLOAD_LENGTH + CMD_MSG_CHECKSUM_LENGTH + CMD_MSG_END_LENGTH;

const char CMD_MSG_START[CMD_MSG_START_LENGTH + 1] = "S";
const char CMD_MSG_END[CMD_MSG_END_LENGTH + 1] = "X\r";

typedef char CmdMsg[CMD_MSG_LENGTH + 1];
typedef unsigned int CmdId;
typedef char CmdPayload[CMD_MSG_PAYLOAD_LENGTH + 1];
typedef unsigned int CmdChecksum;

// Generic command parsers

bool parseRequest(CmdMsg* req, CmdId id, const CmdPayload& payload,
                  CmdChecksum checksum);

bool parseResponse(const CmdMsg& res, CmdId* id, CmdPayload* payload,
                   CmdChecksum* checksum);
//!@}

/**
 * @defgroup led Led commands
 * @brief Led command types and ids
 */
//!@{

//! Led command types
enum LedCommand
{
  LED_GET_IMAX = 0,
  LED_GET_ICURRENT,
  LED_GET_LEVEL,
  LED_SET_LEVEL,
  LED_GET_TEMPERATURE,
  LED_GET_STATUS,
  LED_GET_VERSION
};
//! Number of led commands
const int NUM_LED_CMDS = 7;
//! Led command id table
const CmdId LED_CMD[NUM_LED_CMDS] = {0x100, ///!> GET_IMAX cmd id
                                     0x101, ///!> GET_ICURRENT cmd id
                                     0x102, ///!> GET_LEVEL cmd id
                                     0x103, ///!> SET_LEVEL cmd id
                                     0x104, ///!> GET_TEMPERATURE cmd id
                                     0x105, //!> GET_STATUS cmd id
                                     0x10e //!> GET_VERSION cmd id
            };
//!@}


/**
 * @defgroup motor Motor commands
 * Motor command types and ids.
 */
//!@{

//! Motor command types
enum MotorCommand
{
  MOTOR_GET_DEVICE_CONFIG = 0,
  MOTOR_SET_DIRECTION_SPEED,
  MOTOR_GET_DIRECTION_SPEED,
  MOTOR_SET_ACCEL,
  MOTOR_GET_ACCEL,
  MOTOR_GET_TEMPERATURE,
  MOTOR_GET_ICURRENT,
  MOTOR_GET_STATUS
};
//! Number of motor commands
const int NUM_MOTOR_CMDS = 8;
//! Motor command id table
const CmdId MOTOR_CMD[NUM_MOTOR_CMDS] = {0x182, //!> GET_DEVICE_CONFIG cmd id
                                         0x183, //!> SET_DIRECTION_SPEED cmd id
                                         0x184, //!> GET_DIRECTION_SPEED cmd id
                                         0x185, //!> SET_ACCEL cmd id
                                         0x186, //!> GET_ACCEL cmd id
                                         0x187, //!> GET_TEMPERATURE cmd id
                                         0x188, //!> GET_ICURRENT cmd id
                                         0x189 //!> GET_STATUS cmd id
            };
//!@}


/**
 * @defgroup motorctrl Motor control commands
 * Motor control command types and ids.
 */
//!@{

//! Motor control command types
enum MotorCtrlCommand
{
  MOTORCTRL_GET_CONSTANTS = 0,
  MOTORCTRL_SET_CONSTANTS
};
//! Number of motor commands
const int NUM_MOTORCTRL_CMDS = 2;
//! Motor command id table
const CmdId MOTORCTRL_CMD[NUM_MOTORCTRL_CMDS] = {
                                         0x200, //!> GET_CONSTANTS cmd id
                                         0x201, //!> SET_CONSTANTS cmd id
            };
//!@}


/**
 * @defgroup sensor Sensor commands
 * Sensor command types and ids.
 */
//!@{

//! Sensor command types
enum SensorCommand
{
  SENSOR_GET_DEVICE_CONFIG = 0,
  SENSOR_GET_OFFSET,
  SENSOR_SET_OFFSET,
  SENSOR_GET_VALUE
};
//! Number of motor commands
const int NUM_SENSOR_CMDS = 4;
//! Motor command id table
const CmdId SENSOR_CMD[NUM_SENSOR_CMDS] = {
                                            0x280, //!> GET_DEVICE_CONFIG
                                            0x281, //!> GET_OFFSET
                                            0x282, //!> SET_OFFSET
                                            0x283  //!> GET_VALUE
                                           };
//!@}


/**
 * @defgroup modules Motor board modules
 * @brief Type of modules in the motor board
 */
//!@{

//! Module types
enum ModuleType
{
  LED = 0,
  MOTOR,
  MOTORCTRL,
  SENSOR
};
//! Number of modules in the motor board;
const int NUM_MODULE_TYPES = 4;
//! Number of commands per module
const int NUM_MODULE_CMDS[NUM_MODULE_TYPES] = { NUM_LED_CMDS,
                                                NUM_MOTOR_CMDS,
                                                NUM_MOTORCTRL_CMDS,
                                                NUM_SENSOR_CMDS};
/**
 *  @brief Module-command identifier table
 *
 *  The command identifier for a given module may be get as
 *       MODULE_CMD_ID[module][command]
 */
// The cmd id table is declared extern and defined in the implementation due to
// linker issues (multiple definitions when several files include the header)
extern const CmdId* MODULE_CMD_ID[NUM_MODULE_TYPES];
//!@}


// LED command parsers
// These commands control the lights attached to the motor board, if any.
// Currently there is not any light is attached, so they must be ignored.

bool parseLedGetImaxRequest(CmdMsg* req);
bool parseLedGetImaxResponse(const CmdMsg& res,
                             uint16_t* imax0, uint16_t* imax1);

bool parseLedGetIcurrentRequest(CmdMsg* req);
bool parseLedGetIcurrentResponse(const CmdMsg& res,
                                 uint16_t* icur0, uint16_t* icur1);

// TODO: There are 3 levels in albatross' ODS, but only 2 levels in their code
//       Assuming that there are only 2 leds in the vehicle, need to be check
bool parseLedGetLevelRequest(CmdMsg* req);
bool parseLedGetLevelResponse(const CmdMsg& res, uint8_t* lev0, uint8_t* lev1);

bool parseLedSetLevelRequest(CmdMsg* req, uint8_t lev0, uint8_t lev1);
bool parseLedSetLevelResponse(const CmdMsg& res, uint8_t* lev0, uint8_t* lev1);

bool parseLedGetTemperatureRequest(CmdMsg* req);
bool parseLedGetTemperatureResponse(const CmdMsg& res, uint16_t* temp0, uint16_t* temp1);

bool parseLedGetStatusRequest(CmdMsg* req);
bool parseLedGetStatusResponse(const CmdMsg& res, uint8_t* stat0, uint8_t* stat1);

bool parseGetVersionRequest(CmdMsg* req);
bool parseGetVersionResponse(const CmdMsg& res, uint8_t* vers);


// MOTOR Commands
// These commands control the motors attached to the board

bool parseMotorGetDeviceConfigRequest(CmdMsg* req, uint8_t motor);
bool parseMotorGetDeviceConfigResponse(const CmdMsg& res,
                                       uint8_t* motor,
                                       uint16_t* max_temp,
                                       uint16_t* rated_speed_rpm,
                                       uint16_t* no_load_speed_rpm,
                                       uint16_t* rated_current_ma);

bool parseMotorSetDirectionSpeedRequest(CmdMsg* req,
                                        int8_t rpm_pc0, int8_t rpm_pc1,
                                        int8_t rpm_pc2, int8_t rpm_pc3);
bool parseMotorSetDirectionSpeedResponse(const CmdMsg& res,
                                         int16_t* rpm0, int16_t* rpm1,
                                         int16_t* rpm2, int16_t* rpm3);

bool parseMotorGetDirectionSpeedRequest(CmdMsg* req);
bool parseMotorGetDirectionSpeedResponse(const CmdMsg& res,
                                         int16_t* rpm0, int16_t* rpm1,
                                         int16_t* rpm2, int16_t* rpm3);

bool parseMotorSetAccelRequest(CmdMsg* req,
                               uint8_t accel0, uint8_t accel1,
                               uint8_t accel2, uint8_t accel3);
bool parseMotorSetAccelResponse(const CmdMsg& res,
                                uint8_t* accel0, uint8_t* accel1,
                                uint8_t* accel2, uint8_t* accel3);

bool parseMotorGetAccelRequest(CmdMsg* req);
bool parseMotorGetAccelResponse(const CmdMsg& res,
                                uint8_t* accel0, uint8_t* accel1,
                                uint8_t* accel2, uint8_t* accel3);

// TODO: Check the type of arguments, they should be uint16_t?
bool parseMotorGetTemperatureRequest(CmdMsg* req);
bool parseMotorGetTemperatureResponse(const CmdMsg& res,
                                      int16_t* temp0, int16_t* temp1,
                                      int16_t* temp2, int16_t* temp3);

// TODO: Check the type of arguments, they should be int16_t?
bool parseMotorGetIcurrentRequest(CmdMsg* req);
bool parseMotorGetIcurrentResponse(const CmdMsg& res,
                                   uint16_t* icurr0, uint16_t* icurr1,
                                   uint16_t* icurr2, uint16_t* icurr3);

bool parseMotorGetStatusRequest(CmdMsg* req);
bool parseMotorGetStatusResponse(const CmdMsg& res,
                                 uint8_t* stat0, uint8_t* stat1,
                                 uint8_t* stat2, uint8_t* stat3);


// MOTORCTRL Commands
// These commands provide access to the motors control constants
bool parseMotorctrlGetConstantsRequest(CmdMsg* req, uint8_t motor);
bool parseMotorctrlGetConstantsResponse(const CmdMsg& res, uint8_t* motor,
                                        bool* active, int16_t* kp_q15,
                                        int16_t* ki_q15, int16_t* kd_q15);

bool parseMotorctrlSetConstantsRequest(CmdMsg* req, uint8_t motor,
                                       bool active, int16_t kp_q15,
                                       int16_t ki_q15, int16_t kd_q15);
bool parseMotorctrlSetConstantsResponse(const CmdMsg& res, uint8_t* motor,
                                        bool* active, int16_t* kp_q15,
                                        int16_t* ki_q15, int16_t* kd_q15);

// SENSOR Commands
// These commands provide access to the different sensors

bool parseSensorGetDeviceConfigRequest(CmdMsg* req, uint8_t sensor);
bool parseSensorGetDeviceConfigResponse(const CmdMsg& res,
                                        uint8_t* sensor, uint8_t* type,
                                        int8_t* unit_res, uint8_t* ADCbits,
                                        int16_t* min, int16_t* max);

bool parseSensorGetOffsetRequest(CmdMsg* req, uint8_t sensor);
bool parseSensorGetOffsetResponse(const CmdMsg& res, uint8_t* sensor, int16_t* offset);

bool parseSensorSetOffsetRequest(CmdMsg* req, uint8_t sensor, int16_t offset);
bool parseSensorSetOffsetResponse(const CmdMsg& res,
                                  uint8_t* sensor, int16_t* offset);

bool parseSensorGetValueRequest(CmdMsg* req, uint8_t sensor);
bool parseSensorGetValueResponse(const CmdMsg& res, uint8_t* sensor,
                                 int16_t* sensor_val, int32_t* adc_val);


} // namespace motorboard

#endif // COMMANDMSG_H
