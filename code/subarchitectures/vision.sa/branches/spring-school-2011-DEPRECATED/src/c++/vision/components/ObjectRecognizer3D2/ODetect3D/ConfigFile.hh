/**
 * $Id: ConfigFile.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef P_CONFIG_FILE_HH
#define P_CONFIG_FILE_HH

#include <string>
#include <fstream>
#include "PNamespace.hh"

namespace P
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

