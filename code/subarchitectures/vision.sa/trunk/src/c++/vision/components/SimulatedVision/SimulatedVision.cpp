#include "SimulatedVision.h"

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CVisionSimulator();
   }
}


namespace cogx { namespace vision {

void CVisionSimulator::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
}

std::string CVisionSimulator::CDisplayClient::getControlState(const std::string& ctrlId)
{
   return "";
}

void CVisionSimulator::CDisplayClient::handleForm(const std::string& id,
      const std::string& partId, const std::map<std::string, std::string>& fields)
{
}

bool CVisionSimulator::CDisplayClient::getFormData(const std::string& id,
      const std::string& partId, std::map<std::string, std::string>& fields)
{
   return false;
}

void CVisionSimulator::CDisplayClient::createForms()
{
   setHtmlForm("Vision.Simulator", "Generator", "TODO: create the fields");
}

CVisionSimulator::CVisionSimulator()
{
}

void CVisionSimulator::configure(const std::map<std::string,std::string> & _config)
{
   m_display.configureDisplayClient(_config);
}

void CVisionSimulator::start()
{
   m_display.connectIceClient(*this);
   m_display.installEventReceiver();
   m_display.createForms();
}

void CVisionSimulator::destroy()
{
}

void CVisionSimulator::runComponent()
{
   while (isRunning()) {
      sleepComponent(500);
   }
}

}} // namespace
// vim:sw=3:ts=8:et
