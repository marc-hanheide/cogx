/*
 * Author: Marko Mahnič
 * Created: 2010-04-26
 */
#ifndef FILEMONITOR_NALQLVBL
#define FILEMONITOR_NALQLVBL

#include <cast/architecture/ManagedComponent.hpp>

#include <CDisplayClient.hpp>

namespace cogx { namespace display {

struct SConverter
{
   std::string id;
   std::string command;
   std::string type;
   std::string extensions;
   static std::map<std::string, SConverter> converters;
   static void add(const std::string &_id, const std::string &_command,
         const std::string &type="text", const std::string &exts="");
   static SConverter* find(const std::string &id);
   static SConverter* findByExt(const std::string &ext);
   static std::string names();
};

struct SWatchInfo
{
   std::string watchDef;
   int watchId;
   long changeCount;
   std::string title;
   std::string section;
   std::string directory;
   std::vector<std::string> filemasks;
   SConverter* pConverter;
   SWatchInfo(const std::string& watchDef);
   bool matches(const std::string& filename);
   void dump(std::ostream& steam);
};

class CFileMonitor: public cast::ManagedComponent
{
private:
   cogx::display::CDisplayClient m_display;
   // void handleGuiEvent(const Visualization::TEvent &event);

   friend struct _s_init_converters_;

public:
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

public:
   void processFileChange(int watchId, const std::string &fname);
   using cast::ManagedComponent::sleepComponent;
};

}} // namespace
#endif /* end of include guard: FILEMONITOR_NALQLVBL */
