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

#include "SimulatedVision.h"
#include "Exception.h"
#include "StringFmt.h"

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CVisionSimulator();
   }
}

namespace cxd = cogx::display;

namespace cogx { namespace vision {

#define ID_FORM_OBJECT "001.sim.object"
#define ID_FORM_SCENE  "002.sim.scene"

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
   if (partId == ID_FORM_OBJECT) {
      m_FormObject.get(fields);
      return true;
   }
   else if (partId == ID_FORM_SCENE) {
      m_FormObject.get(fields);
      return true;
   }
   return false;
}

void CVisionSimulator::CDisplayClient::createForms()
{
   // Two forms:
   //    - scene contents: a list of objects that are on the scene.
   //       - multi-list or multiple drop-down boxes (configurable)
   //       - create scene
   //       - clear scene
   //       - load/save scene
   //       - show -> displays properties for object
   //    - object properties
   //       - color distribution (3 cbx+edit, unknown prob RO-calculated)
   //       - shape distribution (-"-)
   //       - label distribution (-"-)
   //       - ambiguity/gain for each (meaning: what do we gain if we ask)
   //       - load/save (submit saves data)
   //       - cbx to select object
   //       - (saliency, time, ...)
   // Data:
   //    - each object has a set of attributes (multiple sets?)
   //    - list of known (possible) colors
   //    - list of possible shapes
   //    - list of possible labels; this can grow; (shared with ObjectRecognizerSrv?)
   //    - list of known objects
   //    - (lists could grow during execution -- learn new stuff)
   
   // scene form
   {
      std::ostringstream ss, objids;

      // TODO: load object names from a list
      for(int i = 0; i < 10; i++) {
         objids << "<option>Object " << i << "</option>";
      }
      // up to 5 objects in a scene
      for(int i = 0; i < 5; i++) {
         ss << "<select name='sceneobj" << i << "' >";
         ss << objids.str();
         ss << "</select>" << "<br/>";
      }

      // Management buttons
      ss << "<input type='submit' name='submit' value='Create Scene' />";
      ss << "<input type='button' @@ONCLICK@@('scene.erase') value='Erase Scene' />";
      ss << "<input type='button' @@ONCLICK@@('scene.load') value='Load Scene' />";
      ss << "<input type='button' @@ONCLICK@@('scene.save') value='Save Scene' />";

      setHtmlForm("Vision.Simulator", ID_FORM_SCENE, ss.str());
   }

   // object form
   {
      struct _local_ {
         static void valueDistrib(std::ostringstream& ss, const std::string& name,
               const std::vector<std::string>& values, cogx::display::CFormValues& form, int count)
         {
            std::ostringstream opts;
            for(int i = 0; i < values.size(); i++) {
               opts << "<option>" << values[i] << "</option>";
            }
            for(int i = 0; i < count; i++) {
               std::string fldname = name + _str_(i);
               ss << "<tr><td>";
               ss << "<select name='" << fldname << "' >";
               ss << opts.str();
               ss << "</select>";
               ss << "</td><td>";
               ss << "<input type='text' name='p" << fldname << "' style='width:5em;' />";
               ss << "</td></tr>";
               form.add(new cxd::CFormValues::choice(fldname, cxd::CFormValues::valuelist(values)));
               form.add(new cxd::CFormValues::field(std::string("p") + fldname));
            }
            ss << "<tr><td>";
            ss << "Unknown: </td><td> 1.0"; // TODO: JS update
            ss << "</td></tr>";
         }
         static void ambiguityGain(std::ostringstream& ss, const std::string& name,
              cogx::display::CFormValues& form, bool bLabel)
         {
            ss << "<tr><td>" << (bLabel ? "Ambiguity:" : "&nbsp") << "</td><td>";
            ss << "<input type='text' name='ambig" << name << "' style='width:5em;' />";
            ss << "</td></tr>";
            form.add(new cxd::CFormValues::field(std::string("ambig") + name));
            ss << "<tr><td>" << (bLabel ? "Gain:" : "&nbsp") << "</td><td>";
            ss << "<input type='text' name='gain" << name << "' style='width:5em;' />";
            ss << "</td></tr>";
            form.add(new cxd::CFormValues::field(std::string("gain") + name));
         }
      };
      std::ostringstream ss;
      std::vector<std::string> clrs, shps, lbls;

      ss << "<table><tr><td>";

      // TODO: load color names from a list
      for(int i = 0; i < 10; i++) {
         //clrs.push_back( ((std::ostringstream&)(std::ostringstream() << "Color" << i)).str() );
         clrs.push_back( std::string("Color") + _str_(i, "d:3:0") );
      }

      // up to 3 colors for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "color", clrs, m_FormObject, 3);
      _local_::ambiguityGain(ss, "color", m_FormObject, true);
      ss << "</table>";

      ss << "</td><td>";

      // TODO: load shape names from a list
      for(int i = 0; i < 3; i++) {
         shps.push_back( std::string("Shape") + _str_(i, "d:3:0") );
      }

      // up to 3 shapes for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "shape", shps, m_FormObject, 3);
      _local_::ambiguityGain(ss, "shape", m_FormObject, true);
      ss << "</table>";

      ss << "</td><td>";

      // TODO: load label names from a list
      for(int i = 0; i < 3; i++) {
         lbls.push_back( std::string("Label") + _str_(i, "d:3:0") );
      }

      // up to 3 labels for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "label", lbls, m_FormObject, 3);
      _local_::ambiguityGain(ss, "label", m_FormObject, true);
      ss << "</table>";

      ss << "</td></tr></table>";
      //std::vector<std::string> dump;
      //m_FormObject.setValue("color0/Color003", "1");
      //m_FormObject.setValue("gaincolor", "0.35");
      //m_FormObject.dump(dump);
      //for (int i = 0; i < dump.size(); i++) {
      //  ss << dump[i] << "<br>";
      //}
      setHtmlForm("Vision.Simulator", ID_FORM_OBJECT, ss.str());
   }
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
