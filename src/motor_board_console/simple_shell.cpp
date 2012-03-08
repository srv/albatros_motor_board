#include "simple_shell.h"
#include <iostream>
#include <sstream>



simple_shell::SimpleShell::SimpleShell(const CmdHandle* cmd_table,const unsigned short cmd_max) :
  CMD_HANDLE_TABLE_(cmd_table), CMD_HANDLE_MAX_(cmd_max), cmd_count_(0)
{
}

void simple_shell::SimpleShell::promptLine(std::string * line)
{
  cmd_count_++;
  line->clear();
  while (line->empty())
  {
    std::cout << "(" << cmd_count_ << ")" << " > ";
    std::getline(std::cin, *line);
  }

}

void simple_shell::SimpleShell::parseCmdCall(const std::string & line, CmdCall* cmd_call)
{
  std::stringstream ss(line);
  std::string word;
  cmd_call->clear();
  while (ss >> word)
    cmd_call->push_back(word);
  if (cmd_call->empty())
  {
    cmd_call->push_back("");
  }
}

bool simple_shell::SimpleShell::handleCmdCall(const simple_shell::CmdCall & cmd_call)
{
  std::string cmd_name = cmd_call[0];
  unsigned short i = 0;
  while (i < CMD_HANDLE_MAX_ && cmd_name != CMD_HANDLE_TABLE_[i].name_)
    i++;
  if (i == CMD_HANDLE_MAX_)
  {
    return false;
  }
  else
  {
    (*CMD_HANDLE_TABLE_[i].funct_ptr_)(cmd_call);
    return true;
  }
}

