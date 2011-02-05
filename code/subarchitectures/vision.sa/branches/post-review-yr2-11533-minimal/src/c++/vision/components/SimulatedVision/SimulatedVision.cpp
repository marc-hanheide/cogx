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

#include <VisionData.hpp>

#include <iostream>
#include <fstream>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CVisionSimulator();
   }
}

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
struct _path_
{
   static char pathsep;
   static bool exists(const std::string& filename)
   {
      struct stat st;
      int rvs;
      rvs = stat(filename.c_str(), &st);
      return (rvs == 0);
   }

   static std::string join(const std::string& patha, const std::string& pathb)
   {
      size_t ea = patha.find_last_not_of(pathsep);
      size_t sb = pathb.find_first_not_of(pathsep);
      std::string a, b;
      if (ea != std::string::npos) a = patha.substr(0, ea+1);
      if (sb != std::string::npos) b = pathb.substr(sb);
      if (a.size() && b.size()) return a + pathsep + b;
      else if (a.size()) return a;
      else if (b.size()) return b; // TODO: do we keep leading pathsep???
      else return "";
   }

   static bool listdir(const std::string& path, std::vector<std::string>& fileNames)
   {
      DIR *dp = opendir(path.c_str());
      struct dirent *dirp;

      if(dp == NULL) return false; // TODO: throw?
      while ((dirp = readdir(dp)) != NULL) {
         std::string n(dirp->d_name);
         if (n == ".") continue;
         fileNames.push_back(n);
      }
      closedir(dp);
      return true;
   }
};
char _path_::pathsep = '/';

struct _dict_
{
   template<typename K, typename V>
   static std::vector<K> keys(const std::map<K, V>& dict)
   {
      std::vector<K> r;
      typeof(dict.begin()) it;
      for (it = dict.begin(); it != dict.end(); it++) {
         r.push_back(it->first);
      }
      return r;
   }
};

namespace cxd = cogx::display;

namespace cogx { namespace vision {

#define ID_V11N_OBJECT "Vision.Simulator"

#define ID_FORM_OBJECT "001.sim.object"
#define IDC_FORM_OBJECT_NAME "objectname"
#define ID_CMD_OBJECT_RELOAD "!object.reload" // Reload form from object data
#define ID_CMD_OBJECT_SUBMIT_SAVE "!object.submit+save"

#define ID_FORM_SCENE  "002.sim.scene"
#define IDC_FORM_SCENE_NAME "scenename"
#define ID_CMD_SCENE_RELOAD "!scene.reload" // Reload form from scene data
#define ID_CMD_SCENE_APPLY "!scene.apply"
#define ID_CMD_SCENE_ERASE "!scene.erase"
#define ID_CMD_SCENE_SUBMIT_SAVE "!scene.submit+save"

#define ID_CHUNK_INFO  "003.sim.info"
#define ID_CHUNK_HELP  "900.sim.help"

#define MAX_SCENE_OBJECT_COUNT 6

void CVisionSimulator::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
   std::cout << " *** handleEvent " << event.sourceId << std::endl;
   if (pSim && event.sourceId == ID_CMD_OBJECT_RELOAD) {
      typeof(pSim->m_Objects.begin()) it = pSim->m_Objects.find(event.data);
      if (it != pSim->m_Objects.end()) {
         it->second.toForm(m_FormObject);
         std::map<std::string, std::string> fields;
         m_FormObject.get(fields);
         setHtmlFormData(ID_V11N_OBJECT, ID_FORM_OBJECT, fields);
      }
   }
   else if (pSim && event.sourceId == ID_CMD_SCENE_RELOAD) {
      typeof(pSim->m_Scenes.begin()) it = pSim->m_Scenes.find(event.data);
      if (it != pSim->m_Scenes.end()) {
         it->second.toForm(m_FormScene);
         std::map<std::string, std::string> fields;
         m_FormScene.get(fields);
         setHtmlFormData(ID_V11N_OBJECT, ID_FORM_SCENE, fields);
      }
   }
   else if (pSim && event.sourceId == ID_CMD_OBJECT_SUBMIT_SAVE) {
      pSim->saveObject(m_FormObject.get(IDC_FORM_OBJECT_NAME));
   }
   else if (pSim && event.sourceId == ID_CMD_SCENE_SUBMIT_SAVE) {
      pSim->saveScene(m_FormScene.get(IDC_FORM_SCENE_NAME));
   }
   else if (pSim && event.sourceId == ID_CMD_SCENE_ERASE) {
      pSim->clearScene();
   }
}

std::string CVisionSimulator::CDisplayClient::getControlState(const std::string& ctrlId)
{
   return "";
}

void CVisionSimulator::CDisplayClient::handleForm(const std::string& id,
      const std::string& partId, const std::map<std::string, std::string>& fields)
{
   std::cout << " *** handleForm " << partId << std::endl;
   if (partId == ID_FORM_OBJECT) {
      m_FormObject.apply(fields);
      if (pSim) {
         typeof(fields.begin()) pname = fields.find(IDC_FORM_OBJECT_NAME);
         if (pname == fields.end()) {
            std::cout << " *** Error: form has no field " << IDC_FORM_OBJECT_NAME << std::endl;
         }
         else {
            typeof(pSim->m_Objects.begin()) it = pSim->m_Objects.find(pname->second);
            if (it != pSim->m_Objects.end()) {
               it->second.fromForm(m_FormObject);
            }
            else {
               std::cout << " *** Error: object '" << pname->second << "' not found." << std::endl;
            }
         }
      }
   }
   else if (partId == ID_FORM_SCENE) {
      m_FormScene.apply(fields);
      if (pSim) {
         typeof(fields.begin()) pname = fields.find(IDC_FORM_SCENE_NAME);
         if (pname == fields.end()) {
            // Scene with that name does not exist; use a temporary scene object
            CSceneAttrs scn;
            scn.fromForm(m_FormScene);
            pSim->log("applyScene - unnamed");
            pSim->applyScene(scn);
         }
         else {
            typeof(pSim->m_Scenes.begin()) it = pSim->m_Scenes.find(pname->second);
            if (it != pSim->m_Scenes.end()) {
               it->second.fromForm(m_FormScene);
               pSim->log("applyScene - %s", pname->second.c_str());
               pSim->applyScene(pname->second);
            }
            else {
               std::cout << " *** Error: scene '" << pname->second << "' not found." << std::endl;
            }
         }
      }
   }
}

bool CVisionSimulator::CDisplayClient::getFormData(const std::string& id,
      const std::string& partId, std::map<std::string, std::string>& fields)
{
   if (partId == ID_FORM_OBJECT) {
      m_FormObject.get(fields);
      return true;
   }
   else if (partId == ID_FORM_SCENE) {
      m_FormScene.get(fields);
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
              cogx::display::CFormValues& form, bool bLabel, bool bGain=true)
         {
            ss << "<tr><td>" << (bLabel ? "Ambiguity:" : "&nbsp") << "</td><td>";
            ss << "<input type='text' name='ambig" << name << "' style='width:5em;' />";
            ss << "</td></tr>";
            form.add(new cxd::CFormValues::field(std::string("ambig") + name));
            ss << "<tr><td>" << (bLabel && bGain ? "Gain:" : "&nbsp") << "</td><td>";
            ss << "<input type='" << (bGain ? "text" : "hidden") << "' name='gain" <<
               name << "' style='width:5em;' />";
            ss << "</td></tr>";
            form.add(new cxd::CFormValues::field(std::string("gain") + name));
         }
      };
      std::ostringstream ss;
      std::vector<std::string> clrs, shps, lbls;

      ss << "<div>";
      ss << "Object: <select name='" << IDC_FORM_OBJECT_NAME << "' "
         << " onchange=\"CogxJsSendValue('@@FORMID@@','" << ID_CMD_OBJECT_RELOAD << "','"
         << IDC_FORM_OBJECT_NAME << "');\" >";
      for(int i = 0; i < pSim->m_objectNames.size(); i++) {
         ss << "<option>" << pSim->m_objectNames[i] << "</option>";
      }
      ss << "</select>";
      ss << 
         "<span class='v11ninfo'>"
         "Select an object, edit its properties and Apply the changes."
         "</spam>";
      ss << "</div>";
      m_FormObject.add(new cxd::CFormValues::field(IDC_FORM_OBJECT_NAME));

      ss << "<table><tr><td>color</td><td>shape</td><td>type/identity</td></tr>";
      ss << "<tr><td>";

      // up to 3 colors for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "color", pSim->m_colorNames, m_FormObject, 3);
      _local_::ambiguityGain(ss, "color", m_FormObject, true, false);
      ss << "</table>";

      ss << "</td><td>";

      // up to 3 shapes for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "shape", pSim->m_shapeNames, m_FormObject, 3);
      _local_::ambiguityGain(ss, "shape", m_FormObject, true, false);
      ss << "</table>";

      ss << "</td><td>";

      // up to 3 labels for an object
      ss << "<table>";
      _local_::valueDistrib(ss, "label", pSim->m_labelNames, m_FormObject, 3);
      _local_::ambiguityGain(ss, "label", m_FormObject, true, true);
      ss << "</table>";

      ss << "</td></tr></table>";
      ss << "<div class='v11ninfo'>Gains for colors and shapes are calculated from the distribution</div>";

      ss << "<input type='submit' value='Apply' />";
      ss << "<input type='button' value='Apply&amp;Save' "
         << "onclick=\"CogxJsSubmitAndClick('@@FORMID@@','" <<  ID_CMD_OBJECT_SUBMIT_SAVE << "');\" />";

      setHtmlForm(ID_V11N_OBJECT, ID_FORM_OBJECT, ss.str());
   }

   // scene form
   {
      std::ostringstream ss, objids;

      ss << "<div>";
      ss << "Scene: <select name='" << IDC_FORM_SCENE_NAME << "' "
         << " onchange=\"CogxJsSendValue('@@FORMID@@','" << ID_CMD_SCENE_RELOAD << "','"
         << IDC_FORM_SCENE_NAME << "');\" >";
      for(int i = 0; i < pSim->m_sceneNames.size(); i++) {
         ss << "<option>" << pSim->m_sceneNames[i] << "</option>";
      }
      ss << "</select>";
      ss << "<span class='v11ninfo'>"
         "Load a predefined scene.<br>Select the objects that will be on the scene."
         "</span>";
      ss << "</div>";
      m_FormScene.add(new cxd::CFormValues::field(IDC_FORM_SCENE_NAME));

      // load object names from a list
      for(int i = 0; i < pSim->m_objectNames.size(); i++) {
         objids << "<option>" << pSim->m_objectNames[i] << "</option>";
      }
      objids << "<option>" "</option>";

      // up to MAX_SCENE_OBJECT_COUNT objects in a scene
      for(int i = 0; i < MAX_SCENE_OBJECT_COUNT; i++) {
         std::string fldname = "sceneobj" + _str_(i);
         ss << "<select name='" << fldname << "' >";
         ss << objids.str();
         ss << "</select>";
         if (i % 3 == 2) ss << "<br/>";
         m_FormScene.add(new cxd::CFormValues::choice(fldname, pSim->m_objectNames));
      }

      // Management buttons
      ss << "<input type='submit' name='submit' value='Create Scene' />";
      ss << "<input type='button' value='Create&amp;Save Scene' "
         << "onclick=\"CogxJsSubmitAndClick('@@FORMID@@','" <<  ID_CMD_SCENE_SUBMIT_SAVE << "');\" />";
      ss << "<input type='button' @@ONCLICK@@('" << ID_CMD_SCENE_ERASE << "') value='Erase Scene' />";

      setHtmlForm(ID_V11N_OBJECT, ID_FORM_SCENE, ss.str());
   }

}

void CObjectAttrs::clear()
{
   m_colors.clear();
   m_ambig_color = 0.0;
   m_gain_color = 0.0;
   m_shapes.clear();
   m_ambig_shape = 0.0;
   m_gain_shape = 0.0;
   m_labels.clear();
   m_ambig_label = 0.0;
   m_gain_label = 0.0;
}

void CObjectAttrs::saveIni(std::ostream& ss)
{
#define ENDL '\n'
   typeof(m_colors.begin()) it;
   ss << "[color]" << ENDL;
   for(it = m_colors.begin(); it != m_colors.end(); it++) {
      ss << "@" << it->first << "=" << it->second << ENDL;
   }
   ss << "ambiguity=" << m_ambig_color << ENDL;
   ss << "gain=" << m_gain_color << ENDL;

   ss << ENDL << "[shape]" << ENDL;
   for(it = m_shapes.begin(); it != m_shapes.end(); it++) {
      ss << "@" << it->first << "=" << it->second << ENDL;
   }
   ss << "ambiguity=" << m_ambig_shape << ENDL;
   ss << "gain=" << m_gain_shape << ENDL;

   ss << ENDL << "[label]" << ENDL;
   for(it = m_labels.begin(); it != m_labels.end(); it++) {
      ss << "@" << it->first << "=" << it->second << ENDL;
   }
   ss << "ambiguity=" << m_ambig_label << ENDL;
   ss << "gain=" << m_gain_label << ENDL;
#undef ENDL
}

void CObjectAttrs::loadIni(std::istream& ss)
{
   char buf[256];
   std::string section;

   clear();

   std::map<std::string, double> *pDistMap = NULL;
   double *pAmbig = NULL;
   double *pGain = NULL;
   while (ss.getline(buf, 256).gcount() > 0) {
      std::string line = _s_::strip(buf);
      if (line.size() < 1) continue;
      if (line[0] == '[') {
         section = line;
         if (section == "[color]") {
            pDistMap = &m_colors;
            pAmbig = &m_ambig_color;
            pGain = &m_gain_color;
         }
         else if (section == "[shape]") {
            pDistMap = &m_shapes;
            pAmbig = &m_ambig_shape;
            pGain = &m_gain_shape;
         }
         else if (section == "[label]") {
            pDistMap = &m_labels;
            pAmbig = &m_ambig_label;
            pGain = &m_gain_label;
         }
         else {
            pDistMap = NULL;
            pAmbig = NULL;
            pGain = NULL;
         }
         continue;
      }
      if (!pDistMap || section == "") continue;
      if (line[0] == '@') {
         std::vector<std::string> parts = _s_::split(line.substr(1), "=", 1);
         if (pDistMap && parts.size() == 2) {
            std::string key = _s_::lower(_s_::strip(parts[0]));
            (*pDistMap)[key] = atof(_s_::lskipwhite(parts[1].c_str()));
            std::cout << section << key << "=" << (*pDistMap)[key] << std::endl;
         }
      }
      else {
         std::vector<std::string> parts = _s_::split(line, "=", 1);
         if (pDistMap && parts.size() == 2) {
            std::string key = _s_::strip(parts[0]);
            if (key == "ambiguity") {
               if (pAmbig) *pAmbig = atof(_s_::lskipwhite(parts[1].c_str()));
            }
            else if (key == "gain") {
               if (pGain) *pGain = atof(_s_::lskipwhite(parts[1].c_str()));
            }
            else std::cout << " *** SimVision: Invalid object attribute: " << key << std::endl;
         }
      }
   }
}

void CObjectAttrs::toForm(cogx::display::CFormValues& form)
{
   form.clear();

   form.setValue(IDC_FORM_OBJECT_NAME, m_name);

   typeof(m_colors.begin()) it;
   const std::string ffmt = "f:.3";

   int i = 0;
   for(it = m_colors.begin(); it != m_colors.end(); it++) {
      std::string fldname = std::string("color") + _str_(i);
      form.setValue(fldname + "/" + it->first, "1");
      form.setValue(std::string("p") + fldname, _str_(it->second, ffmt));
      i++;
   }
   form.setValue("ambigcolor", _str_(m_ambig_color, ffmt));
   form.setValue("gaincolor", _str_(m_gain_color, ffmt));

   i = 0;
   for(it = m_shapes.begin(); it != m_shapes.end(); it++) {
      std::string fldname = std::string("shape") + _str_(i);
      form.setValue(fldname + "/" + it->first, "1");
      form.setValue(std::string("p") + fldname, _str_(it->second, ffmt));
      i++;
   }
   form.setValue("ambigshape", _str_(m_ambig_shape, ffmt));
   form.setValue("gainshape", _str_(m_gain_shape, ffmt));

   i = 0;
   for(it = m_labels.begin(); it != m_labels.end(); it++) {
      std::string fldname = std::string("label") + _str_(i);
      form.setValue(fldname + "/" + it->first, "1");
      form.setValue(std::string("p") + fldname, _str_(it->second, ffmt));
      i++;
   }
   form.setValue("ambiglabel", _str_(m_ambig_label, ffmt));
   form.setValue("gainlabel", _str_(m_gain_label, ffmt));
}

void CObjectAttrs::fromForm(cogx::display::CFormValues& form)
{
   clear();

   for(int i=0; i < 3; i++) {
      std::string fldname = std::string("color") + _str_(i);
      // form.get(fldname) returns a '\n' delimited list of values selected in (multi)select.
      // We take the first nonempty element as the name of the color.
      std::vector<std::string> vals = _s_::split(form.get(fldname), "\n", 1, false);
      if (vals.size() > 0) {
         std::string key = _s_::strip(vals[0]);
         if (key.size() > 0) {
            // TODO: check if key is a valid color
            m_colors[key] = form.getFloat(std::string("p") + fldname);
         }
      }
   }
   m_ambig_color = form.getFloat("ambigcolor");
   m_gain_color = form.getFloat("gaincolor");

   for(int i=0; i < 3; i++) {
      std::string fldname = std::string("shape") + _str_(i);
      std::vector<std::string> vals = _s_::split(form.get(fldname), "\n", 1, false);
      if (vals.size() > 0) {
         std::string key = _s_::strip(vals[0]);
         if (key.size() > 0) {
            // TODO: check if key is a valid shape
            m_shapes[key] = form.getFloat(std::string("p") + fldname);
         }
      }
   }
   m_ambig_shape = form.getFloat("ambigshape");
   m_gain_shape = form.getFloat("gainshape");

   for(int i=0; i < 3; i++) {
      std::string fldname = std::string("label") + _str_(i);
      std::vector<std::string> vals = _s_::split(form.get(fldname), "\n", 1, false);
      if (vals.size() > 0) {
         std::string key = _s_::strip(vals[0]);
         if (key.size() > 0) {
            // TODO: check if key is a valid label
            m_labels[key] = form.getFloat(std::string("p") + fldname);
         }
      }
   }
   m_ambig_label = form.getFloat("ambiglabel");
   m_gain_label = form.getFloat("gainlabel");
}

void CSceneAttrs::loadIni(std::istream& ss)
{
   char buf[256];
   std::string section;

   m_objects.clear();

   std::map<std::string, double> *pDistMap = NULL;
   double *pAmbig = NULL;
   double *pGain = NULL;
   while (ss.getline(buf, 256).gcount() > 0) {
      std::string line = _s_::strip(buf);
      if (line.size() < 1) continue;
      if (line[0] == '[') {
         section = line;
         continue;
      }
      if (section == "objects]") {
         m_objects.push_back(line);
      }
   }
}

void CSceneAttrs::saveIni(std::ostream& ss)
{
#define ENDL '\n'
   ss << "[objects]" << ENDL;
   typeof(m_objects.begin()) it;
   for(it = m_objects.begin(); it != m_objects.end(); it++) {
      ss << *it << ENDL;
   }
#undef ENDL
}

void CSceneAttrs::toForm(cogx::display::CFormValues& form)
{
   form.clear();

   form.setValue(IDC_FORM_SCENE_NAME, m_name);

   int i = 0;
   typeof(m_objects.begin()) it;
   for(it = m_objects.begin(); it != m_objects.end(); it++) {
      std::string fldname = "sceneobj" + _str_(i);
      form.setValue(fldname + "/" + *it, "1");
      i++;
      if (i >= MAX_SCENE_OBJECT_COUNT) break;
   }
}

void CSceneAttrs::fromForm(cogx::display::CFormValues& form)
{
   m_objects.clear();
   for(int i = 0; i < MAX_SCENE_OBJECT_COUNT; i++) {
      std::string fldname = "sceneobj" + _str_(i);
      std::string val = _s_::strip(form.get(fldname));
      if (val.size() < 1) continue;
      m_objects.push_back(val);
   }
}

CVisionSimulator::CVisionSimulator()
{
   m_DataDir = "subarchitectures/vision.sa/src/c++/vision/components/SimulatedVision/data";
   m_defaultColors = _s_::split("red,green,blue,yellow,orange,magenta,black,white", ",");
   m_defaultShapes = _s_::split("compact,elongated", ",");
   m_defaultLabels = _s_::split("mug,teabox,cube,sphere", ",");
}

void CVisionSimulator::configure(const std::map<std::string,std::string> & _config)
{
   typeof(_config.begin()) it;

   m_display.configureDisplayClient(_config);

   if((it = _config.find("--datadir")) != _config.end()) {
      m_DataDir = it->second;
   }
}

void CVisionSimulator::loadObjects()
{
   if (! _path_::exists(m_DataDir)) return;
   std::vector<std::string> files;
   _path_::listdir(m_DataDir, files);

   m_Objects.clear();
   typeof(files.begin()) itf;
   for (itf = files.begin(); itf != files.end(); itf++) {
      if (_s_::endswith(*itf, ".tobj")) {
         CObjectAttrs obj;
         obj.m_name = itf->substr(0, itf->size() - 5);
         std::ifstream fini;
         fini.open(_path_::join(m_DataDir, *itf).c_str(), std::ofstream::in);
         obj.loadIni(fini);
         fini.close();
         m_Objects[obj.m_name] = obj;
      }
      else if (_s_::endswith(*itf, ".tscn")) {
         CSceneAttrs scn;
         scn.m_name = itf->substr(0, itf->size() - 5);
         std::ifstream fini;
         fini.open(_path_::join(m_DataDir, *itf).c_str(), std::ofstream::in);
         scn.loadIni(fini);
         fini.close();
         m_Scenes[scn.m_name] = scn;
      }
   }
}

void CVisionSimulator::saveObject(const std::string& objectName)
{
   typeof(m_Objects.begin()) it = m_Objects.find(objectName);
   if (it == m_Objects.end()) {
      log("Save Error: Object '%s' does not exist.", objectName.c_str());
      return;
   }
   CObjectAttrs& obj = it->second;
   std::ofstream fout;
   fout.open(_path_::join(m_DataDir, obj.m_name + ".tobj").c_str(), std::ofstream::out);
   obj.saveIni(fout);
   fout.close();
}

void CVisionSimulator::saveScene(const std::string& sceneName)
{
   typeof(m_Scenes.begin()) it = m_Scenes.find(sceneName);
   if (it == m_Scenes.end()) {
      log("Save Error: Scene '%s' does not exist.", sceneName.c_str());
      return;
   }
   CSceneAttrs& scn = it->second;
   std::ofstream fout;
   fout.open(_path_::join(m_DataDir, scn.m_name + ".tscn").c_str(), std::ofstream::out);
   scn.saveIni(fout);
   fout.close();
}

void CVisionSimulator::updateValueSets()
{
   std::map<std::string, bool> color_vals;
   std::map<std::string, bool> shape_vals;
   std::map<std::string, bool> label_vals;
   std::vector<std::string> vals;

   // some intial sets of labels (TODO: read from an ini file)
   for(int i = 0; i < m_defaultColors.size(); i++) {
      color_vals[m_defaultColors[i]] = true;
   }
   for(int i = 0; i < m_defaultShapes.size(); i++) {
      shape_vals[m_defaultShapes[i]] = true;
   }
   for(int i = 0; i < m_defaultLabels.size(); i++) {
      label_vals[m_defaultLabels[i]] = true;
   }

   // Find unique values for color, shape, label in loaded objects
   typeof(m_Objects.begin()) it;
   for (it = m_Objects.begin(); it != m_Objects.end(); it++) {
      CObjectAttrs& obj = it->second;

      vals = _dict_::keys(obj.m_colors);
      for(int i = 0; i < vals.size(); i++) {
         color_vals[vals[i]] = true;
      }
      vals = _dict_::keys(obj.m_shapes);
      for(int i = 0; i < vals.size(); i++) {
        shape_vals[vals[i]] = true;
      }
      vals = _dict_::keys(obj.m_labels);
      for(int i = 0; i < vals.size(); i++) {
        label_vals[vals[i]] = true;
      }
   }

   m_colorNames = _dict_::keys(color_vals);
   m_shapeNames = _dict_::keys(shape_vals);
   m_labelNames = _dict_::keys(label_vals);
   m_objectNames = _dict_::keys(m_Objects);
   m_sceneNames = _dict_::keys(m_Scenes);
}

void CVisionSimulator::clearScene()
{
   typeof(m_WmObjectIds.begin()) itaddr;
   for(itaddr = m_WmObjectIds.begin(); itaddr != m_WmObjectIds.end(); itaddr++) {
      try {
         // TODO: if an object from the desired scene is already in WM, don't delete, overwrite instd.
         // pobject = getMemoryEntry<VisualObject>(*it);
         deleteFromWorkingMemory(*itaddr);
         log("Object deleted: %s", itaddr->c_str());
      }
      catch (cast::DoesNotExistOnWMException e) {
         log("ERROR: removing stuff to WM");
      }
   }
   m_WmObjectIds.clear();

   m_display.setHtml(ID_V11N_OBJECT, ID_CHUNK_INFO, "<hr>Objects in scene");
}

void CVisionSimulator::applyScene(const std::string& sceneName)
{
   typeof(m_Scenes.begin()) itscn = m_Scenes.find(sceneName);
   if (itscn == m_Scenes.end()) {
      log("ERROR: Invalid scene name '%s'", sceneName.c_str());
      return;
   }

   applyScene(itscn->second);
}

void CVisionSimulator::applyScene(CSceneAttrs& scene)
{
   struct _local_ {
      // from matalab: ap2gain.m @ 20101001.
      // ap0 = 1-sum(ap)
      static double ap2gain(double ap, double ap0) {
         const double thrs[] = { 0.7, 0.5, 0.1 }; // Defaults from CLFStart
         double t1 = 1, tY = thrs[0], tPy = thrs[1], tPn = thrs[2];
         double g1 = 0.25, gY = 1, gPy = 0.5, gPn = 0;
         double g = 0;

         if (ap>=tY)
            g=g1+(g1-gY)/(tY-t1)*(t1-ap);
         else if (ap>=tPy)
            g=gY-(gY-gPy)/(tY-tPy)*(tY-ap);
         else if (ap>=tPn)
            g=gPy-(gPy-gPn)/(tPy-tPn)*(tPy-ap);
         else
            g=0;

         return g+ap0;    
      }
   };

   VisionData::VisualObjectPtr pobject;

   typeof(m_WmObjectIds.begin()) itaddr;
   for(itaddr = m_WmObjectIds.begin(); itaddr != m_WmObjectIds.end(); itaddr++) {
      try {
         // TODO: if an object from the desired scene is already in WM, don't delete, overwrite instd.
         // pobject = getMemoryEntry<VisualObject>(*it);
         deleteFromWorkingMemory(*itaddr);
         log("Object deleted: %s", itaddr->c_str());
      }
      catch (cast::DoesNotExistOnWMException e) {
         log("ERROR: removing stuff to WM");
      }
   }
   m_WmObjectIds.clear();

   std::ostringstream ss;

   std::vector<std::string>::iterator itname;
   for(itname = scene.m_objects.begin(); itname != scene.m_objects.end(); itname++) {
      typeof(m_Objects.begin()) itobj = m_Objects.find(*itname);
      if (itobj == m_Objects.end()) continue; // TODO: this could be an error

      CObjectAttrs& obj = itobj->second;
      try {
         VisionData::VisualObjectPtr pvobj = new VisionData::VisualObject();
         pvobj->time = getCASTTime();
         pvobj->identLabels.push_back(obj.m_name);
         pvobj->identDistrib.push_back(1.0f);
         pvobj->identAmbiguity = 0.0f;

         ss << "<br>" << obj.m_name << ":";

         typeof(obj.m_colors.begin()) it;

         double sum = 0;
         for(it = obj.m_colors.begin(); it != obj.m_colors.end(); it++) {
            pvobj->colorLabels.push_back(it->first);
            pvobj->colorDistrib.push_back(it->second);
            sum += it->second;
            ss << " " << it->first << "=" << it-> second;
         }
         if (sum < 1.0) {
            pvobj->colorLabels.push_back("*unknown*");
            pvobj->colorDistrib.push_back(1.0 - sum);
            ss << " *unknown*=" << (1.0-sum);
         }
         pvobj->colorAmbiguity = obj.m_ambig_color;
         double ap0 = sum < 1.0 ? 1.0 - sum : 0.0;
         double maxGain = 0;
         for(it = obj.m_colors.begin(); it != obj.m_colors.end(); it++) {
            double ap = it->second;
            double gain = _local_::ap2gain(ap, ap0);
            pvobj->colorGains.push_back(gain);
            if (gain > maxGain) maxGain = gain;
         }
         pvobj->colorGain = maxGain; // obj.m_gain_color;

         sum = 0;
         for(it = obj.m_shapes.begin(); it != obj.m_shapes.end(); it++) {
            pvobj->shapeLabels.push_back(it->first);
            pvobj->shapeDistrib.push_back(it->second);
            sum += it->second;
            ss << " " << it->first << "=" << it-> second;
         }
         if (sum < 1.0) {
            pvobj->shapeLabels.push_back("*unknown*");
            pvobj->shapeDistrib.push_back(1.0 - sum);
            ss << " *unknown*=" << (1.0-sum);
         }
         pvobj->shapeAmbiguity = obj.m_ambig_shape;
         ap0 = sum < 1.0 ? 1.0 - sum : 0.0;
         maxGain = 0;
         for(it = obj.m_shapes.begin(); it != obj.m_shapes.end(); it++) {
            double ap = it->second;
            double gain = _local_::ap2gain(ap, ap0);
            pvobj->shapeGains.push_back(gain);
            if (gain > maxGain) maxGain = gain;
         }
         pvobj->shapeGain = maxGain; // obj.m_gain_shape;

         sum = 0;
         for(it = obj.m_labels.begin(); it != obj.m_labels.end(); it++) {
            pvobj->identLabels.push_back(it->first);
            pvobj->identDistrib.push_back(it->second);
            sum += it->second;
            ss << " " << it->first << "=" << it-> second;
         }
         if (sum < 1.0) {
            pvobj->identLabels.push_back("*unknown*");
            pvobj->identDistrib.push_back(1.0 - sum);
            ss << " *unknown*=" << (1.0-sum);
         }
         pvobj->identAmbiguity = obj.m_ambig_label;
         // ATM there are no gains for individual object labels
         //ap0 = sum < 1.0 ? 1.0 - sum : 0.0;
         //maxGain = 0;
         //for(it = obj.m_labels.begin(); it != obj.m_labels.end(); it++) {
         //   double ap = it->second;
         //   double gain = _local_::ap2gain(ap, ap0);
         //   pvobj->identGains.push_back(gain);
         //   if (gain > maxGain) maxGain = gain;
         //}
         pvobj->identGain = obj.m_gain_label;

         std::string objId = newDataID();
         addToWorkingMemory(objId, pvobj);
         m_WmObjectIds.push_back(objId);
         log("Object added: %s", objId.c_str());
      }
      catch(...) {
         log("ERROR: adding stuff to WM");
         ss << "FAILED to add";
      }
   }

   m_display.setHtml(ID_V11N_OBJECT, ID_CHUNK_INFO, "<hr>Objects in scene" + ss.str());
}

void CVisionSimulator::start()
{
   m_display.connectIceClient(*this);
   m_display.setClientData(this);
   m_display.installEventReceiver();

   loadObjects();
   updateValueSets();

   m_display.createForms();

   if (m_Objects.size() > 0) {
      CObjectAttrs& obj = m_Objects.begin()->second;
      std::cout << " ***  toform " << obj.m_name << std::endl;
      obj.toForm(m_display.m_FormObject);
   }

   if (m_Scenes.size() > 0) {
      CSceneAttrs& scn = m_Scenes.begin()->second;
      std::cout << " ***  toform " << scn.m_name << std::endl;
      scn.toForm(m_display.m_FormScene);
   }

   m_display.setHtml(ID_V11N_OBJECT, ID_CHUNK_HELP, "<hr>Data directory: " + m_DataDir);
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
