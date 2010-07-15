#ifndef SIMULATEDVISION_S9AH0TJS
#define SIMULATEDVISION_S9AH0TJS

#include <cast/architecture/ManagedComponent.hpp>
#include <CDisplayClient.hpp>

namespace cogx { namespace vision {

class CVisionSimulator:
   public cast::ManagedComponent
{
private:
   class CDisplayClient: public cogx::display::CDisplayClient
   {
      CVisionSimulator* pSim;
   public:
      CDisplayClient() { pSim = NULL; }
      void setClientData(CVisionSimulator* pSimulator) { pSim = pSimulator; }
      void handleEvent(const Visualization::TEvent &event); /*override*/
      std::string getControlState(const std::string& ctrlId); /*override*/
      void handleForm(const std::string& id, const std::string& partId,
            const std::map<std::string, std::string>& fields); /*override*/
      bool getFormData(const std::string& id, const std::string& partId,
            std::map<std::string, std::string>& fields); /*override*/

      void createForms();
   };
   CDisplayClient m_display;

protected:
   // ManagedComponent overrides
   virtual void configure(const std::map<std::string,std::string> & _config);
   virtual void start();
   virtual void destroy();
   virtual void runComponent();

public:
   CVisionSimulator();
};

}} // namespace
#endif /* end of include guard: SIMULATEDVISION_S9AH0TJS */
// vim:sw=3:ts=8:et
