[["python:package:icemodule"]]
module castcontrol{
   module CastAgent {
      struct CastMessage {
         double time;
         int msgtype;
         string message;
      };
      // TODO: sequence<CastMessage> CastMessageList;
      sequence<string> CastMessageList;
      sequence<string> ProcessList;
      interface Agent {
         idempotent ProcessList getProcessList();
         // TODO: separate structure/class/interface: Process
         idempotent CastMessageList readMessages(string processName, double startTime);
         int startProcess(string name);
         int stopProcess(string name);
      };
   };
};
