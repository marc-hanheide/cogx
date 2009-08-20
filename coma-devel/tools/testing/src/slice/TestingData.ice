#ifndef TESTING_DATA_ICE
#define TESTING_DATA_ICE

#include <cast/slice/CDL.ice>

/// module for slice files relating to the multi-SA testing component
module TestingData {
  /// String used to identify the testing subarch in the configuration options
  const string TESTINGSUBARCHCONFIGKEY = "-tsa";
  /// Character used to terminate a command string in the input file
  const string TESTINGCOMMANDDELIMITER = ";";
  /// Character used to terminate command string header
  const string TESTINGHEADERDELIMITER = ":";
  /// String used instead of subarchitecture identifier to indicate a command to the
  /// testing component itself
  const string TESTINGMETACOMMANDPREFIX = "self";

  /// Represents the state of a testing command 
  enum TestingCommandStatus {
    ISSUED, ///< The command is not yet completed
    COMPLETED ///< The command has been completed and can be deleted
  };

  /// Represents a command from the Tester component to a listener component
  /// in a subarchitecture.
  class TestingCommand {
    /// Status 
    TestingCommandStatus status;
    /// The command string. The format and contents of this string is entirely
    /// up to the individual subarchitectures' different testing components
    string command;
  };
};
#endif // TESTING_DATA_ICE
