/*
 * Author: Marko Mahnič
 * Created: July 2010
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

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
      cogx::display::CFormValues m_FormObject;
      cogx::display::CFormValues m_FormScene;

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
