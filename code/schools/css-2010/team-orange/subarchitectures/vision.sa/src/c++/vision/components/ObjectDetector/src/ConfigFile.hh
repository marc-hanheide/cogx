/**
 * $Id: ConfigFile.hh,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CONFIG_FILE_HH
#define Z_CONFIG_FILE_HH

#include <string>
#include <fstream>
#include "Namespace.hh"

namespace Z
{

class ConfigFile
{
private:
  fstream file;
  int line_cnt;

private:
  void RemoveEOL(string &str);
  bool IsComment(string &str);
public:
  ConfigFile(const string &name);
  ~ConfigFile();
  bool GetLine(string &str);
  int GetLineCnt() {return line_cnt;}
};

/// Get the next word from a string, starting at pos.
bool GetWord(const string &str, string &word, string::size_type &pos);

}

#endif

