//
// = FILENAME
//   MultiSATester.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2008 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "MultiSATester.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace cast;
using namespace std;
using namespace boost;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork

 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new MultiSATester(_id);
  }
}

MultiSATester::MultiSATester(const string &_id)
:
WorkingMemoryAttachedComponent(_id),
PrivilegedManagedProcess(_id)
{
}

MultiSATester::~MultiSATester()
{
}

void MultiSATester::configure(map<string,string>& _config) 
{
  PrivilegedManagedProcess::configure(_config);
  
  m_testFilename = "";

  // Name of the file to read data from
  if (_config["-nsa"] != "") {
    m_NavSa = _config["-nsa"];
  }
  else {
    m_NavSa = "nav.sa";
  }
  if (_config["-f"] != "") {
    ifstream file;
    file.open(_config["-f"].c_str(), std::ios::in);

    if(!file.is_open()) {
      printf("MultiSATester::configure could not open test file\n");
      exit();
    }
    
    while(!file.eof()) {
      string line;
      getline(file, line);
      // Remove leading white space. Code from
      // http://www.experts-exchange.com/Programming/Programming_Languages/Cplusplus/Q_21282559.html
      char const* delims = " \t\r\n";
      string::size_type notwhite = line.find_first_not_of(delims);
      line.erase(0,notwhite);
      if(line.length() > 0 && line[0] != '#')
        m_testFileString += line;
    }
    m_testFileStream.str(m_testFileString);
    log(m_testFileString);
  } else {
      printf("MultiSATester::no test file specified!\n");
  }
}

void MultiSATester::start() 
{
  PrivilegedManagedProcess::start();

  addChangeFilter(createGlobalTypeFilter<TestingData::TestingCommand>(cdl::OVERWRITE), this);

  log("MultiSATester started");
}

void MultiSATester::taskAdopted(const string &_taskID) {
}

void MultiSATester::taskRejected(const string &_taskID) {
}

void MultiSATester::runComponent() 
{
  log("MultiSATester running");

  //int lineno = 0;

  processLines();
}

void MultiSATester::workingMemoryChanged(const cdl::WorkingMemoryChange &_wmc)
{
  //The command issued has completed. Make sure this is the case, then process the next command.
  shared_ptr<const CASTData<TestingData::TestingCommand> > command =
    getWorkingMemoryEntry<TestingData::TestingCommand>(string(CORBA::string_dup(_wmc.m_address.m_id)), string(CORBA::string_dup(_wmc.m_address.m_subarchitecture)));
  
  TestingData::TestingCommandStatus status = command->getData()->m_status;
  assert(status = TestingData::COMPLETED);

  //Delete the command before adding a new one
  deleteFromWorkingMemory(_wmc.m_address, cdl::BLOCKING);

  processLines();
}

void MultiSATester::processLines()
{
  std::string header; // SA ID or metacommand tag
  std::string body;
  std::string line;
  bool waitForCompletion = false;
  while (!waitForCompletion) {
    if (getline(static_cast<std::istream&>(m_testFileStream), line, static_cast<char>(TestingData::TESTING_COMMAND_DELIMITER))) {

      //      lineno++;

      // Remove leading white space. Code from
      // http://www.experts-exchange.com/Programming/Programming_Languages/Cplusplus/Q_21282559.html
      char const* delims = " \t\r\n";
      string::size_type notwhite = line.find_first_not_of(delims);
      line.erase(0,notwhite);

      // Skip empty lines
      if (line.length() == 0) continue;
      
      std::istringstream str(line);

      getline(str, header, static_cast<char>(TestingData::TESTING_HEADER_DELIMITER));
      getline(str, body, static_cast<char>(TestingData::TESTING_COMMAND_DELIMITER));
    }
    else {
      log("EventFile has come to the end");
      successExit();
    }

    if (header == TestingData::TESTING_METACOMMAND_PREFIX) {
      std::istringstream str(body);
      string commandName;
      str >> commandName;

      if (commandName == "Wait") {
        int milliseconds;
        str >> milliseconds;
        sleepProcess(milliseconds);
      }
    }
    else {
      string msg = "Issuing command \"" + body + "\" to subarchitecture \"" + header + "\""; 
      log(msg);
      issueCommandToSA(header, body);
      waitForCompletion = true;
    }
  } 
}

void
MultiSATester::issueCommandToSA(const string &SAID, const string &body)
{
  TestingData::TestingCommand* command(new TestingData::TestingCommand());
  command->m_status = TestingData::ISSUED;
  command->m_command = body.c_str();

  string commandID(newDataID());
  addToWorkingMemory(commandID,
      SAID,
      command,
      cdl::BLOCKING);
}
