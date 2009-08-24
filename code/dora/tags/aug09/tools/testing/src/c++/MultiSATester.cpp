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
#include <cast/architecture.hpp>
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
  cast::interfaces::CASTComponentPtr newComponent() {
    return new MultiSATester();
  }
}

MultiSATester::MultiSATester()
:
WorkingMemoryReaderComponent(),
  MemberFunctionChangeReceiver<MultiSATester>(this, &MultiSATester::workingMemoryChanged)
{
}

MultiSATester::~MultiSATester()
{
}

void MultiSATester::configure(map<string,string>& _config) 
{
  //PrivilegedManagedProcess::configure(_config);
  
  m_testFilename = "";

  // Name of the file to read data from
  if (_config["-nsa"] != "") {
    m_NavSa = _config["-nsa"];
  }
  else if (_config["--nsa"] != "") {
    m_NavSa = _config["--nsa"];
  }
  else {
    m_NavSa = "nav.sa";
  }
  if (_config["-f"] != "") {
    ifstream file;
    file.open(_config["-f"].c_str(), std::ios::in);

    if(!file.is_open()) {
      printf("MultiSATester::configure could not open test file\n");
      exit(1);
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
  //PrivilegedManagedProcess::start();

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

//void MultiSATester::receiveChangeEvent(const cdl::WorkingMemoryChange &_wmc, const Ice::Current &_ctx)
void MultiSATester::workingMemoryChanged(const cdl::WorkingMemoryChange &_wmc)
{
  //The command issued has completed. Make sure this is the case, then process the next command.
  CASTData<TestingData::TestingCommand> command =
    getMemoryEntryWithData<TestingData::TestingCommand>(_wmc.address);
  
  //TestingData::TestingCommandStatus status = command->getData()->status;
  assert(command.getData()->status == TestingData::COMPLETED);

  //Delete the command before adding a new one
  deleteFromWorkingMemory(_wmc.address);

  processLines();
}

void MultiSATester::processLines()
{
  std::string header; // SA ID or metacommand tag
  std::string body;
  std::string line;
  bool waitForCompletion = false;
  while (!waitForCompletion) {
    if (getline(static_cast<std::istream&>(m_testFileStream), line, static_cast<char>(TestingData::TESTINGCOMMANDDELIMITER[0]))) {

      //      lineno++;

      // Remove leading white space. Code from
      // http://www.experts-exchange.com/Programming/Programming_Languages/Cplusplus/Q_21282559.html
      char const* delims = " \t\r\n";
      string::size_type notwhite = line.find_first_not_of(delims);
      line.erase(0,notwhite);

      // Skip empty lines
      if (line.length() == 0) continue;
      
      std::istringstream str(line);

      getline(str, header, static_cast<char>(TestingData::TESTINGHEADERDELIMITER[0]));
      getline(str, body, static_cast<char>(TestingData::TESTINGCOMMANDDELIMITER[0]));
    }
    else {
      log("EventFile has come to the end");
      successExit();
    }

    if (header == TestingData::TESTINGMETACOMMANDPREFIX) {
      std::istringstream str(body);
      string commandName;
      str >> commandName;

      if (commandName == "Wait") {
        int milliseconds;
        str >> milliseconds;
	char buf[256];
	sprintf(buf, "Sleeping %i ms.", milliseconds);
	log(buf);
        sleepComponent(milliseconds);
      }
      else { // Ignore unknown metacommands, move on to next
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
  TestingData::TestingCommandPtr command(new TestingData::TestingCommand());
  command->status = TestingData::ISSUED;
  command->command = body.c_str();

  string commandID(newDataID());
  addToWorkingMemory(commandID,
      SAID,
      command);
}
