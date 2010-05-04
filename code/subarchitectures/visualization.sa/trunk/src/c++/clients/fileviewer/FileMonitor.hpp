/*
 * Author: Marko Mahnič
 * Created: 2010-04-26
 */
#ifndef FILEMONITOR_NALQLVBL
#define FILEMONITOR_NALQLVBL

#include <cast/architecture/ManagedComponent.hpp>

#include <CDisplayClient.hpp>

namespace cogx { namespace display {

class CFileMonitor: public cast::ManagedComponent
{
private:
   cogx::display::CActiveDisplayClient<CFileMonitor> m_display;
   // void handleGuiEvent(const Visualization::TEvent &event);

   struct SWatchInfo {
      int watchId;
      std::string directory;
      std::vector<std::string> filemasks;
      SWatchInfo(const std::string& watchDef);
      bool matches(const std::string& filename);
      void dump(std::ostream& steam);
   };
   std::vector<SWatchInfo*> m_watches;

public:
   CFileMonitor();
   virtual ~CFileMonitor();

protected:
   /**
    * called by the framework to configure our component
    */
   virtual void configure(const std::map<std::string,std::string> & _config);

   /**
    * called by the framework after configuration, before run loop
    */
   virtual void start();

   /**
    * called by the framework upon deletion of the component
    */
   virtual void destroy();

   /**
    * Our run loop. Essentially just wait for key strokes.
    */
   virtual void runComponent();

protected:
   void processFileChange(int watchId, const std::string &fname);
};

}} // namespace
#endif /* end of include guard: FILEMONITOR_NALQLVBL */
