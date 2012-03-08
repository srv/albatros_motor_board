#include <iostream>
#include <iomanip>
#include <exception>
#include <stdexcept>
#include <cstring>
#include <string>
#include <sstream>
#include <cstdlib>
#include <unistd.h>
// #include <signal.h>

#include "simple_shell.h"
#include "command_msg.h"
#include "motor_board_ctrl.h"

// Global variables

bool g_exit = false;

albatros_motor_board::MotorBoardCtrl g_mbctrl;


////////////////////////////////////////////////////////////////////////////////
// Console commands specification
////////////////////////////////////////////////////////////////////////////////

// Forward declarations

void openHandler(simple_shell::CmdCall cmd_call);
void closeHandler(simple_shell::CmdCall cmd_call);
void sendHandler(simple_shell::CmdCall cmd_call);
void listenHandler(simple_shell::CmdCall cmd_call);
void queryHandler(simple_shell::CmdCall cmd_call);
void helpHandler(simple_shell::CmdCall cmd_call);
void quitHandler(simple_shell::CmdCall cmd_call);
void ledHandler(simple_shell::CmdCall cmd_call);
void getVersionHandler(simple_shell::CmdCall cmd_call);
void getMotorConfigHandler(simple_shell::CmdCall cmd_call);
void getSpeedsHandler(simple_shell::CmdCall cmd_call);
void setSpeedsHandler(simple_shell::CmdCall cmd_call);
void getAccelsHandler(simple_shell::CmdCall cmd_call);
void setAccelsHandler(simple_shell::CmdCall cmd_call);
void getIntensitiesHandler(simple_shell::CmdCall cmd_call);
void getStatusHandler(simple_shell::CmdCall cmd_call);
void getMotorctrlHandler(simple_shell::CmdCall cmd_call);
void setMotorctrlHandler(simple_shell::CmdCall cmd_call);
void getSensorConfigHandler(simple_shell::CmdCall cmd_call);
void getSensorOffset(simple_shell::CmdCall cmd_call);
void setSensorOffset(simple_shell::CmdCall cmd_call);
void getSensorValue(simple_shell::CmdCall cmd_call);
void stopHandler(simple_shell::CmdCall cmd_call);
// void motorPlotHandler(simple_shell::CmdCall cmd_call);

const simple_shell::CmdHandle
                       g_CMD_TABLE[] = {
                                         {"open",
                                          "Open a device for communication",
                                          &openHandler},
                                         {"close",
                                          "Close a device for communication",
                                          &closeHandler},
                                         {"send", "Send one command",
                                          &sendHandler},
                                         {"listen", "Listen to one command",
                                          &listenHandler},
                                         {"query", "Send one command and listen to response",
                                          &queryHandler},
                                         {"help",
                                          "Print this list",
                                          &helpHandler},
                                         {"quit",
                                          "Close communication and quit",
                                          &quitHandler },
                                         {"led",
                                          "[LED] Send selected led command",
                                          &ledHandler},
                                         {"getversion",
                                          "[LED] Query firmware version",
                                          &getVersionHandler},
                                         {"getmotorconfig",
                                          "[MOTOR] Get motor configuration",
                                          &getMotorConfigHandler},
                                         {"getspeeds",
                                          "[MOTOR] Get motors' speeds (rpm)",
                                          &getSpeedsHandler},
                                         {"setspeeds",
                                          "[MOTOR] Set motors' speeds (+/- %)",
                                          &setSpeedsHandler},
                                         {"getaccels",
                                           "[MOTOR] Get motors' acceleration rates (%/ds)",
                                           &getAccelsHandler},
                                         {"getintensities",
                                             "[MOTOR] Get motors' intensities (mA)",
                                             &getIntensitiesHandler},
                                         {"getstatus",
                                             "[MOTOR] Get motors' status (error count)",
                                             &getStatusHandler},
                                         {"setaccels",
                                           "[MOTOR] Set motors' acceleration rates (%/ds)",
                                           &setAccelsHandler},
                                         {"getmotorctrl",
                                           "[MOTORCTRL] Get motor PID controller configuration",
                                           &getMotorctrlHandler},
                                         {"setmotorctrl",
                                           "[MOTORCTRL] Set motor PID controller configuration",
                                           &setMotorctrlHandler},
                                         {"stop",
                                          "Stop all motors (set speeds to 0)",
                                          &stopHandler },
                                         {"getsensorconfig",
                                          "[Sensor] Get sensor configuration",
                                          &getSensorConfigHandler},
                                         {"getsensoroffset",
                                          "[Sensor] Get sensor configuration",
                                           &getSensorOffset},
                                         {"setsensoroffset",
                                          "[Sensor] Get sensor configuration",
                                          &setSensorOffset},
                                         {"getsensorvalue",
                                          "[Sensor] Get sensor lecture",
                                          &getSensorValue}
//                                          {"motorplot",
//                                           "FILL ME",
//                                            &motorPlotHandler}
};

const unsigned short g_CMD_MAX = sizeof(g_CMD_TABLE) / sizeof(simple_shell::CmdHandle);

////////////////////////////////////////////////////////////////////////////////
// Command handlers
////////////////////////////////////////////////////////////////////////////////

using namespace std;

void openHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " FILE" << endl;
    return;
  }
  cout << "Opening device " << cmd_call[1] << "..." << endl;
  g_mbctrl.openComm(cmd_call[1]);
  cout << "Device successfully opened" << endl;
}

void closeHandler(simple_shell::CmdCall cmd_call)
{
  cout << "Closing device " << g_mbctrl.getCommName() << "..." << endl;
  g_mbctrl.closeComm();
  cout << "Device successfully closed" << endl;
}

void sendHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " MSG" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request;
  strncpy(request, cmd_call[1].c_str(), albatros_motor_board::CMD_MSG_LENGTH+1);
  cout << "Sending request : " << request << endl;
  g_mbctrl.sendCommand(request);
  cout << "Request successfully send." << endl;
}

void listenHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg response;
  cout << "Listening to response..." << endl;
  g_mbctrl.readCommand(&response);
  cout << "Received response : " << response << endl;
}

void queryHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " MSG" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request;
  strncpy(request, cmd_call[1].c_str(), albatros_motor_board::CMD_MSG_LENGTH+1);
  albatros_motor_board::CmdMsg response;
  cout << "Querying request : " << request << endl;
  g_mbctrl.queryCommand(request, &response);
  cout << "Received response : " << response << endl;
}

void helpHandler(simple_shell::CmdCall cmd_call)
{
  cout << endl << "COMMANDS LIST:" << endl << endl;
  for (unsigned short i=0; i<g_CMD_MAX; i++)
    cout << "\t" << g_CMD_TABLE[i].name_ << "\t: " << g_CMD_TABLE[i].description_ << endl;
  cout << endl;
}

void quitHandler(simple_shell::CmdCall cmd_call)
{
  cout << "Closing communication port and exit" << endl;
  g_mbctrl.closeComm();
  g_exit = true;
}

void ledHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  uint8_t val0=0;
  uint8_t val1=0;
  cout << "Querying led command..." << endl;
  if(!albatros_motor_board::parseLedGetStatusRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseLedGetStatusResponse(response, &val0, &val1))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "val0 = " << val0  << endl;
  cout << "val1 = " << val1  << endl;
}

void getVersionHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  uint8_t vers = 0;
  cout << "Querying version command..." << endl;
  if(!albatros_motor_board::parseGetVersionRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseGetVersionResponse(response, &vers))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "vers = " << std::showbase << std::hex << vers << endl;
  cout << std::noshowbase << std::dec;
}

void getMotorConfigHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " motor_num" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t motor_num;
  uint16_t max_temp, rated_rpm, no_load_rpm, rated_ma;
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 3)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..3)" <<  endl;
    return;
  }
  motor_num = aux;
  cout << "Requested configuration of motor " << static_cast<unsigned int>(motor_num) << endl;
  cout << "Querying get motor configuration command..." << endl;
  if(!albatros_motor_board::parseMotorGetDeviceConfigRequest(&request, motor_num))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorGetDeviceConfigResponse(response,&motor_num,
                                                    &max_temp, &rated_rpm,
                                                    &no_load_rpm, &rated_ma))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " maximum temperature (ºC) : " << max_temp << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " rated speed (rpm) : " << rated_rpm << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " no load speed (rpm) : " << no_load_rpm << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " rated intensity (mA) : " << rated_ma << endl;
}

void getSpeedsHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  int16_t rpm[4] = {0,0,0,0};
  cout << "Querying get motor speed command..." << endl;
  if(!albatros_motor_board::parseMotorGetDirectionSpeedRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorGetDirectionSpeedResponse(response,
                                                 &rpm[0], &rpm[1], &rpm[2], &rpm[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " speed (rpm) = " << static_cast<int>(rpm[i]) << endl;
}

void setSpeedsHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 5)
  {
    cout << "Usage : " << cmd_call[0] << " pc0 pc1 pc2 pc3" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  int16_t rpm[4];
  int pc[4];
  for (unsigned int i=0; i<4; i++ )
  {
    istringstream ss(cmd_call[i+1]);
    if ( !(ss >> pc[i]) || abs(pc[i])>100)
    {
      cout << "Bad argument " << i+1 << " : " << cmd_call[i+1]
           << " (should be int in -100..100)" <<  endl;
      return;
    }
  }
  cout << "Requested speeds are " << pc[0] << " " << pc[1] << " " << pc[2] << " " << pc[3] << " " << endl;
  cout << "Querying set motor speed command..." << endl;
  if(!albatros_motor_board::parseMotorSetDirectionSpeedRequest(&request, pc[0], pc[1], pc[2], pc[3]))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorSetDirectionSpeedResponse(response,
                                                      &rpm[0], &rpm[1], &rpm[2], &rpm[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " speed = " << static_cast<int>(rpm[i]) << endl;
}


void getAccelsHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  uint8_t acc_pc[4] = {0,0,0,0};
  cout << "Querying get motor acceleration command..." << endl;
  if(!albatros_motor_board::parseMotorGetAccelRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorGetAccelResponse(response, &acc_pc[0], &acc_pc[1], &acc_pc[2], &acc_pc[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " accel (%/ds) = " << static_cast<unsigned int>(acc_pc[i]) << endl;
}

void setAccelsHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 5)
  {
    cout << "Usage : " << cmd_call[0] << " acc0 acc1 acc2 acc3" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t accel[4];
  for (unsigned int aux, i=0; i<4; i++ )
  {
    istringstream ss(cmd_call[i+1]);
    if ( !(ss >> aux) || aux>100)
    {
      cout << "Bad argument " << i+1 << " : " << cmd_call[i+1]
           << " (should be int in 0..100)" <<  endl;
      return;
    }
    else
      accel[i] = aux;
  }
  cout << "Requested accelerations are " << accel[0] << " " << accel[1] << " " << accel[2] << " " << accel[3] << " " << endl;
  cout << "Querying set motor accelerations command..." << endl;
  if(!albatros_motor_board::parseMotorSetAccelRequest(&request, accel[0], accel[1], accel[2], accel[3]))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorSetAccelResponse(response, &accel[0], &accel[1], &accel[2], &accel[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " acceleration (%/ds) = " << static_cast<unsigned int>(accel[i]) << endl;
}

void getIntensitiesHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  uint16_t ma[4] = {0,0,0,0};
  cout << "Querying get motor intensity command..." << endl;
  if(!albatros_motor_board::parseMotorGetIcurrentRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorGetIcurrentResponse(response, &ma[0], &ma[1], &ma[2], &ma[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " current intensity (mA) = " << ma[i] << endl;
}

void getStatusHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  uint8_t status[4] = {0,0,0,0};
  cout << "Querying get motor status command..." << endl;
  if(!albatros_motor_board::parseMotorGetStatusRequest(&request))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorGetStatusResponse(response, &status[0], &status[1], &status[2], &status[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " status (error count) = " << static_cast<unsigned int>(status[i]) << endl;
}

void getMotorctrlHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " motor_num" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t motor_num;
  bool active;
  int16_t Kvec[3];
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 3)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..3)" <<  endl;
    return;
  }
  motor_num = aux;
  cout << "Requested motor " << static_cast<unsigned int>(motor_num) << " control settings" << endl;
  cout << "Querying motor control settings command..." << endl;
  if(!albatros_motor_board::parseMotorctrlGetConstantsRequest(&request, motor_num))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorctrlGetConstantsResponse(response,&motor_num,
                                                     &active, &Kvec[0], &Kvec[1], &Kvec[2]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
 cout << "Motor "<< static_cast<unsigned int>(motor_num) << " control state : " << active << endl;
 cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Kp (Q15) : " << Kvec[0] << "( " << Kvec[0]/32768.0 << " )" << endl;
 cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Ki (Q15) : " << Kvec[1] << "( " << Kvec[1]/32768.0 << " )" << endl;
 cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Kd (Q15) : " << Kvec[2] << "( " << Kvec[2]/32768.0 << " )" << endl;
}

void setMotorctrlHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 6)
  {
    cout << "Usage : " << cmd_call[0] << " motor_num active Kp Ki Kd" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t motor_num;
  bool active;
  int16_t Kvec[3];
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if ( !(ss >> aux) || aux > 3)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..3)" <<  endl;
    return;
  }
  motor_num = aux;
  ss.clear();
  ss.str(cmd_call[2]);
  if (!(ss >> active))
  {
    cout << "Bad argument " << 2 << " : " << cmd_call[2] << endl;
    return;
  }
  for (int k=0,i=3; k<3; k++,i++)
  {
    ss.clear();
    ss.str(cmd_call[i]);
    if (!(ss >> Kvec[k]))
    {
      cout << "Bad argument " << i << " : " << cmd_call[i] << endl;
      return;
    }
  }
  cout << "Requested motor " << static_cast<unsigned int>(motor_num) << " control settings are " << active << " " << Kvec[0] << " " << Kvec[1] << " " << Kvec[2] << endl;
  cout << "Querying motor control settings command..." << endl;
  if(!albatros_motor_board::parseMotorctrlSetConstantsRequest(&request, motor_num, active,
                                                    Kvec[0], Kvec[1], Kvec[2]))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorctrlSetConstantsResponse(response,&motor_num,
                                                     &active, &Kvec[0], &Kvec[1], &Kvec[2]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " control state : " << active << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Kp (Q15) : " << Kvec[0] << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Ki (Q15) : " << Kvec[1] << endl;
  cout << "Motor "<< static_cast<unsigned int>(motor_num) << " Kd (Q15) : " << Kvec[2] << endl;
}

void getSensorConfigHandler(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " sensor_num" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t sensor_num, type, ADCbits;
  int8_t resolution;
  int16_t min, max;
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 1)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..1)" <<  endl;
    return;
  }
  sensor_num = aux;
  cout << "Requested configuration of sensor " << static_cast<unsigned int>(sensor_num) << endl;
  cout << "Querying get sensor configuration command..." << endl;
  if(!albatros_motor_board::parseSensorGetDeviceConfigRequest(&request, sensor_num))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseSensorGetDeviceConfigResponse(response, &sensor_num,
                                                    &type, &resolution,
                                                    &ADCbits, &min, &max))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
 cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " type : " << static_cast<unsigned int>(type) << endl;
 cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " resolution : " << static_cast<int>(resolution) << endl;
 cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " ADCbits : " << static_cast<unsigned int>(ADCbits) << endl;
 cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " min : " << min << endl;
 cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " max : " << max << endl;
}

void getSensorOffset(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " sensor_num" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t sensor_num;
  int16_t offset;
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 1)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..1)" <<  endl;
    return;
  }
  sensor_num = aux;
  cout << "Requested offset of sensor " << static_cast<unsigned int>(sensor_num) << endl;
  cout << "Querying get sensor configuration command..." << endl;
  if(!albatros_motor_board::parseSensorGetOffsetRequest(&request, sensor_num))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseSensorGetOffsetResponse(response, &sensor_num, &offset))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " offset : " << offset << endl;
}

void setSensorOffset(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 3)
  {
    cout << "Usage : " << cmd_call[0] << " sensor_num offset" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t sensor_num;
  int16_t offset;
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 1)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..1)" <<  endl;
    return;
  }
  sensor_num = aux;
  ss.clear();
  ss.str(cmd_call[2]);
  if (!(ss >> offset))
    {
      cout << "Bad argument " << 2 << " : " << cmd_call[2]
           << " (should be int16_t)" <<  endl;
      return;
    }
  cout << "Setting offset of sensor " << static_cast<unsigned int>(sensor_num) << endl;
  cout << "Querying set sensor configuration command..." << endl;
  if(!albatros_motor_board::parseSensorSetOffsetRequest(&request, sensor_num, offset))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseSensorSetOffsetResponse(response, &sensor_num, &offset))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " offset : " << offset << endl;
}

void getSensorValue(simple_shell::CmdCall cmd_call)
{
  if (cmd_call.size() < 2)
  {
    cout << "Usage : " << cmd_call[0] << " sensor_num" << endl;
    return;
  }
  albatros_motor_board::CmdMsg request, response;
  uint8_t sensor_num;
  int16_t sensor_val;
  int32_t adc_val;
  unsigned int aux;
  istringstream ss(cmd_call[1]);
  if (!(ss >> aux) || aux > 1)
  {
    cout << "Bad argument " << 1 << " : " << cmd_call[1]
         << " (should be int in 0..1)" <<  endl;
    return;
  }
  sensor_num = aux;
  cout << "Requested sensor " << static_cast<unsigned int>(sensor_num) << " value" << endl;
  cout << "Querying get sensor value command..." << endl;
  if(!albatros_motor_board::parseSensorGetValueRequest(&request, sensor_num))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request, &response);
  cout << response << endl;
  if(!albatros_motor_board::parseSensorGetValueResponse(response, &sensor_num,
                                              &sensor_val, &adc_val))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " sampled value : " << sensor_val << endl;
  cout << "Sensor "<< static_cast<unsigned int>(sensor_num) << " adc value : " << adc_val << endl;
}

void stopHandler(simple_shell::CmdCall cmd_call)
{
  albatros_motor_board::CmdMsg request, response;
  int16_t rpm[4];
  cout << "Querying set motor speed command..." << endl;
  if(!albatros_motor_board::parseMotorSetDirectionSpeedRequest(&request,0,0,0,0))
    throw(runtime_error( "Error parsing request : " + string(request) ) );
  cout << request << endl;
  g_mbctrl.queryCommand(request,&response);
  cout << response << endl;
  if(!albatros_motor_board::parseMotorSetDirectionSpeedResponse(response, &rpm[0], &rpm[1], &rpm[2], &rpm[3]))
    throw(runtime_error( "Error parsing response : " + string(response) ) );
  for (unsigned int i=0; i<4; i++)
    cout << "Motor "<< i << " speed = " << rpm[i] << endl;
}



//void motorplotSIGINTHandler (int signum)
//{
//  signal(signum,SIG_DFL);
//  throw std::runtime_error("Caught INT signal");
//}
//
//#include <sys/time.h>
//void motorPlotHandler(simple_shell::CmdCall cmd_call)
//{
//  double usec = 0.1*1.e6;
//  signal(SIGINT, motorplotSIGINTHandler);
//  std::ofstream data_file("motordata.gnuplot");
//  timeval start, now;
//  gettimeofday(&start,NULL);
//  for (double t=0.0;;usleep(usec))
//  {
//    gettimeofday(&now,NULL);
//    t = (now.tv_sec-start.tv_sec) + 1.e-6*(now.tv_usec-start.tv_usec);
//    cout << t << endl;
//    data_file << t
//              << " " << 50
//              << " " << rand() % 100
//              << " " << 0
//              << " " << 0
//              << " " << 0 << endl;
//  }
//}

int main(int argc, char* argv[])
{
  simple_shell::SimpleShell mbshell(g_CMD_TABLE, g_CMD_MAX);

  cout << endl << "=== Motor board control console ===" << endl << endl;

  string line;
  simple_shell::CmdCall cmd_call;
//  g_mbctrl.openComm("/dev/ttyS0");
  do
  {
    try {
      mbshell.promptLine(&line);
      mbshell.parseCmdCall(line, &cmd_call);
      if (!mbshell.handleCmdCall(cmd_call))
        cout << "Unknown command : " << cmd_call[0] << endl;
    } catch (exception& e) { cout << e.what() << endl;}
  } while (!g_exit);

  return 0;

}
