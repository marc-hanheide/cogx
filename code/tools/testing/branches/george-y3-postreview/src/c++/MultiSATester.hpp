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

#ifndef MultiSATester_hpp
#define MultiSATester_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <TestingData.hpp>
#include <string>
#include <fstream>

/**
 *  This component reads a text file line by line. The head of each string 
 *  names the subarchitecture. The tester then writes a simple struct to that 
 *  SA:s WM, containing a completion bool and the rest of the string.
 *  Some SA-specific component will listen for such structs and interpret the 
 *  string (also depending on SA). When the task specified is completed, the 
 *  completion field is overwritten and the tester proceeds to process the 
 *  next line.
 *  The entries can indicate any of a number of things, for example:
 *  - NavControl commands that move the robot
 *  - Fake nav events, creating entries directly on nav WM
 *  - Consistency checks of WM state or Binder state or a 
 *  component's internal state (asserting if failed)
 *  - Fake written input to Comsys
 *  etc. 
 */

using namespace cast;

class MultiSATester : public cast::WorkingMemoryReaderComponent, public cast::MemberFunctionChangeReceiver<MultiSATester>
{
  public:
    MultiSATester();
    virtual ~MultiSATester();

    virtual void runComponent();
    virtual void start();
  protected:
    void taskAdopted(const std::string &_taskID);
    void taskRejected(const std::string &_taskID);
    void configure(std::map<std::string, std::string>& _config);

    virtual void workingMemoryChanged(const cdl::WorkingMemoryChange &_wmc);//, const Ice::Current &_ctx);

    void issueCommandToSA(const std::string &SAID, const std::string &body);
    void processLines();
    /// exits and signals success
    void successExit() const {
      const_cast<MultiSATester&>(*this).sleepComponent(2000); // sleep for a while to allow dotviewer to finish
      log("\nSUCCESS\n");
      ::exit(cast::cdl::testing::CASTTESTPASS);
    }
    /// exits and signals failure
    void failExit() const {
      log("\nFAILURE\n");
      //::exit(cast::cdl::testing::CAST_TEST_FAIL);
      abort();
    }
  private:
    std::string m_testFileString;
    std::string m_testFilename;
    std::string m_NavSa;
    std::istringstream m_testFileStream;
};
#endif
