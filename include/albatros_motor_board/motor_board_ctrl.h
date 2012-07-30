/**
 * @file
 * @brief Motor board controller class presentation.
 * @author Joan Pau Beltran
 * @date 2011-01-03
 *
 * The class presented in this file provides an interface to communicate with the
 * motor board and control its modules. Currently, these modules are the motors
 * and their built in PID controllers, and two sensors (pressure and water in).
 *
 * Communication with the board is done through a serial port with a set of
 * request/response commands.
 * The SerialComm class deals with the serial communication issues.
 * Command's format and syntax is addressed by the command_msg module.
 */

#ifndef MOTOR_BOARD_CTRL_H
#define MOTOR_BOARD_CTRL_H

#include <string>
#include <stdexcept>
#include <ostream>
#include "serial_comm.h"
#include "command_msg.h"

namespace albatros_motor_board
{

using serial::SerialComm;

/**
 * @brief Exception to handle motor board errors and exceptions
 */
class MotorBoardError : public std::runtime_error
{
public:
  MotorBoardError(const std::string& msg) :
    std::runtime_error(msg)
  {
  }
};

/**
 * @brief Motor board controller class
 */
class MotorBoardCtrl
{
public:

  MotorBoardCtrl();

  //!@name Serial communication
  //@{
  std::string getCommName();
  void openComm(std::string dev_name);
  void closeComm();
  void flushComm();
  void readCommand(CmdMsg* cmd);
  void sendCommand(const CmdMsg& cmd);
  void queryCommand(const CmdMsg& request, CmdMsg* response);
  //}

  //!@name Firmware version functions
  //@{
  void getVersion(int* num);
  //}


  //!@name Motor identifiers and functions
  //@{
  //! Motor selection
  enum Motor {FORWARD_LEFT=0, FORWARD_RIGHT, DOWNWARD_LEFT, DOWNWARD_RIGHT};
  static const int NUM_MOTORS = 4;

  typedef int MotorSpeeds[NUM_MOTORS];
  typedef int MotorAccels[NUM_MOTORS];
  typedef int MotorStatus[NUM_MOTORS];

  int saturation_value;

  void getSpeeds(MotorSpeeds* rpm);
  void setSpeeds(const MotorSpeeds& req_pc, MotorSpeeds* res_rpm=0);
  void getAccels(MotorAccels* pc);
  void setAccels(const MotorAccels& req_pc_ds, MotorAccels* res_pc_ds=0);
  void getStatus(MotorStatus* errors);

  enum PIDConstant {P=0,I,D};
  static const int NUM_CONSTANTS = 3;

  typedef float PIDConstants[NUM_CONSTANTS];

  void getMotorctrl(const Motor motor_id, bool* active, PIDConstants* Kpid);
  void setMotorctrl(const Motor motor_id, bool active, const PIDConstants& Kpid,
                    bool* res_active = 0, PIDConstants* res_Kpid = 0);
  //}

  //!@name Sensor identifiers and functions
  //@{
  //! Sensor selection
  enum Sensor {PRESSURE, WATERIN};
  static const int NUM_SENSORS = 2;
  void getSensorConfig(const Sensor sensor_id, int* type, int* res,
                       int* ADCbits, int* min, int* max);
  void getSensorOffset(const Sensor sensor_id, int* offset);
  void setSensorOffset(const Sensor sensor_id, int offset, int* res_offset=0);
  void getSensorValue(const Sensor sensor_id, int* value, int* adc_value = 0);
  //}

private:

  //!@name Serial communication
  //@{
  SerialComm comm_;
  static const SerialComm::E_DataBits COMM_DATA_BITS = SerialComm::DB8;
  static const SerialComm::E_StopBits COMM_STOP_BITS = SerialComm::ONE_STOP_BIT;
  static const SerialComm::E_Parity COMM_PARITY = SerialComm::NO_PARITY;
  //static const unsigned long COMM_BAUD_RATE = 38400;
  static const unsigned long COMM_BAUD_RATE = 9600;
  static const unsigned long COMM_READ_TIMEOUT = 500;
  static const unsigned long COMM_READ_RETRIES = 4;
  //@}
};

std::ostream& operator<<(std::ostream& ostr, const MotorBoardCtrl::Motor& m);
std::ostream& operator<<(std::ostream& ostr, const MotorBoardCtrl::Sensor& s);
std::ostream& operator<<(std::ostream& ostr, const MotorBoardCtrl::PIDConstant& k);

} // namespace

#endif // MOTOR_BOARD_CTRL_H
