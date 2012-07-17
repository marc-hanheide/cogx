#ifndef _PROXY_LOGGERPROXY_H_500569D9_
#define _PROXY_LOGGERPROXY_H_500569D9_

namespace matlab {

class CLoggerProxy
{
   void output(const std::string& text)
   {
      printf ("%s\n", text.c_str());
      fflush(stdout);
   }
public:
   virtual void trace(const std::string& text) { output(text); }
   virtual void debug(const std::string& text) { output(text); }
   virtual void info(const std::string& text)  { output(text); }
   virtual void warn(const std::string& text)  { output(text); }
   virtual void error(const std::string& text) { output(text); }
   static CLoggerProxy& getLogger();
   static void setLogger(CLoggerProxy& log);
};


}

#endif /* _PROXY_LOGGERPROXY_H_500569D9_ */
