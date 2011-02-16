[["python:package:icemodule"]]
module castcontrol{
   module CastAgent {
      // Message *transport format* for messages captured by agents.
      struct CastMessage {
         double time;
         int msgtype;
         string message;
      };
      sequence<CastMessage> CastMessageList;

      struct ProcessInfo {
         string name;
         int status;
         int error;
      };
      sequence<ProcessInfo> ProcessList;
      interface Agent {
         idempotent ProcessList getProcessList();
         CastMessageList readMessages(string processName);
         int startProcess(string name);
         int stopProcess(string name);
         void setLog4jClientProperties(string propText);
      };
   };
};
