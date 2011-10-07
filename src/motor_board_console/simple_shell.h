#ifndef SIMPLE_SHELL_H
#define SIMPLE_SHELL_H

#include <string>
#include <vector>

namespace simple_shell
{

typedef std::vector<std::string> CmdCall;

typedef void (*CmdFunctPtr)(const CmdCall cmd_args);

struct CmdHandle
{
  std::string name_;
  std::string description_;
  CmdFunctPtr funct_ptr_;
};

class SimpleShell
{
public:

  SimpleShell(const CmdHandle* cmd_table, const unsigned short cmd_max);

  void promptLine(std::string * line);
  void parseCmdCall(const std::string & line, CmdCall* cmd_call);
  bool handleCmdCall(const simple_shell::CmdCall & cmd_call);

private:

  const simple_shell::CmdHandle* CMD_HANDLE_TABLE_;
  const unsigned short CMD_HANDLE_MAX_;

  unsigned int cmd_count_;


};

} // namespace simple_shell

#endif // SIMPLE_SHELL_H
