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

class CObjectAttrs
{
public:
   std::string m_name;
   std::map<std::string, double> m_colors;
   double m_ambig_color;
   double m_gain_color;
   std::map<std::string, double> m_shapes;
   double m_ambig_shape;
   double m_gain_shape;
   std::map<std::string, double> m_labels;
   double m_ambig_label;
   double m_gain_label;

public:
   void clear();
   void loadIni(std::istream& ss);
   void saveIni(std::ostream& ss);
   void toForm(cogx::display::CFormValues& form);
   void fromForm(cogx::display::CFormValues& form);
};

class CSceneAttrs
{
public:
   std::string m_name;
   std::vector<std::string> m_objects;

public:
   void loadIni(std::istream& ss);
   void saveIni(std::ostream& ss);
   void toForm(cogx::display::CFormValues& form);
   void fromForm(cogx::display::CFormValues& form);
};


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

private:
   std::vector<std::string> m_defaultColors;
   std::vector<std::string> m_defaultShapes;
   std::vector<std::string> m_defaultLabels;

   // all visual object are in "local" WM, so we only need object_id, and no subarch_id
   std::vector<std::string> m_WmObjectIds;

public:
   std::string m_DataDir; // Data directory; add/remove is handled through other apps (file-browser)
   std::map<std::string, CObjectAttrs> m_Objects;
   std::map<std::string, CSceneAttrs> m_Scenes; // object IDs for the scene

   std::vector<std::string> m_sceneNames;
   std::vector<std::string> m_objectNames;
   std::vector<std::string> m_colorNames;
   std::vector<std::string> m_shapeNames;
   std::vector<std::string> m_labelNames;

protected:
   // ManagedComponent overrides
   virtual void configure(const std::map<std::string,std::string> & _config);
   virtual void start();
   virtual void destroy();
   virtual void runComponent();

   // IO
   void loadObjects();
   void loadScenes();
   void updateValueSets();
   void saveObject(const std::string& objectName);
   void saveScene(const std::string& sceneName);

   // WM
   void applyScene(const std::string& sceneName);
   void applyScene(CSceneAttrs& scene);
   void clearScene();

public:
   CVisionSimulator();
};

}} // namespace
#endif /* end of include guard: SIMULATEDVISION_S9AH0TJS */
// vim:sw=3:ts=8:et
