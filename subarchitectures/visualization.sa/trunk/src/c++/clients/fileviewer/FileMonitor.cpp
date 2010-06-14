/*
 * Author: Marko Mahnič
 * Created: 2010-04-26
 */

#include "FileMonitor.hpp"

#include <sys/types.h>
#include <sys/inotify.h>
#include <sstream>
#include <fstream>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::display::CFileMonitor();
   }
}

namespace cogx { namespace display {

std::map<std::string, CFileMonitor::SConverter> CFileMonitor::SConverter::converters;

void CFileMonitor::SConverter::add(const std::string &id, const std::string &command,
      const std::string &type, const std::string &exts)
{
   converters[id].id = id;
   converters[id].command = command;
   converters[id].type = type;
   converters[id].extensions = std::string(",") + exts + ",";
}

CFileMonitor::SConverter* CFileMonitor::SConverter::find(const std::string &id)
{
   typeof(converters.begin()) it = converters.find(id);
   if (it == converters.end()) return NULL;
   return &it->second;
}

CFileMonitor::SConverter* CFileMonitor::SConverter::findByExt(const std::string &ext)
{
   if (ext == "") return NULL;
   std::string fx = std::string(",") + ext + ",";
   for (typeof(converters.begin()) it = converters.begin(); it != converters.end(); it++) {
      int pd = it->second.extensions.find(fx);
      if (pd >= 0) return &it->second;
   }
   return NULL;
}

std::string CFileMonitor::SConverter::names()
{
   std::ostringstream names;
   for (typeof(converters.begin()) it = converters.begin(); it != converters.end(); it++) {
      if (it != converters.begin()) names << ",";
      names << it->first;
   }
   return names.str();
}

struct _s_init_converters_
{
   _s_init_converters_()
   {
      CFileMonitor::SConverter::add("image", "", "binary", "bmp,gif,jpeg,jpg,png,pbm,pgm,ppm,tiff,xbm,xpm");
      CFileMonitor::SConverter::add("dot", "dot -Tsvg", "text", "dot");
      CFileMonitor::SConverter::add("neato", "neato -Tsvg");
      CFileMonitor::SConverter::add("twopi", "twopi -Tsvg");
      CFileMonitor::SConverter::add("circo", "circo -Tsvg");
      CFileMonitor::SConverter::add("fdp", "fdp -Tsvg");
      CFileMonitor::SConverter::add("svg", "", "text", "svg");
      CFileMonitor::SConverter::add("luagl", "", "text", "luagl,lua");
      CFileMonitor::SConverter::add("html", "", "text", "html,htm");
      CFileMonitor::SConverter::add("htmlhead", "", "text", "css");
   }
} _init_converters_;

CFileMonitor::SWatchInfo::SWatchInfo(const std::string &watchDef)
{
   this->watchDef = watchDef;
   watchId = -1;
   pConverter = NULL;

   boost::regex rxWatch ("\\s*((\\w+)\\=)?((\\w+)\\:)?([^{]+)(\\{([^}]+)\\})?\\s*");
   boost::smatch res;
   boost::match_flag_type flags = boost::match_default;

   if ( ! boost::regex_match(watchDef, res, rxWatch, flags)) {
      printf("**Invalid watch format: %s\n", watchDef.c_str());
      printf("**Use: Name=Converter:path/to/files/{mask;mask;...}\n");
      printf("**  If Name and Converter are ommitted they will be deduced from filename.\n");
      printf("**  Known converters: %s.\n", SConverter::names().c_str());
   }
   //printf("2: %s\n", res[2].str().c_str());
   //printf("4: %s\n", res[4].str().c_str());
   //printf("5: %s\n", res[5].str().c_str());
   //printf("7: %s\n", res[7].str().c_str());
   title = res[2];
   directory = res[5];
   std::string converter = res[4];
   std::string masks = res[7];

   if (converter != "") {
      pConverter = SConverter::find(converter);
      if (pConverter == NULL) {
         printf("**Unknown converter %s\n", converter.c_str());
         printf("**  Known converters: %s.\n", SConverter::names().c_str());
      }
   }

   if (masks.length() < 1) {
      unsigned int pd = directory.find_last_of('/');
      if (pd == directory.npos) {
         masks = directory;
         directory = "";
      }
      else {
         masks = directory.substr(pd+1);
         directory = directory.substr(0, pd);
      }
   }
 
   // directory.rstrip("/")
   int l = directory.size();
   while (l > 0 && directory[l-1] == '/') l--;
   directory = directory.substr(0, l);

   //int pd = watchDef.find_last_of('/');
   //std::string masks;
   //if (pd == watchDef.npos) {
   //   directory = "";
   //   masks = watchDef;
   //}
   //else if (pd > 0) {
   //   directory = watchDef.substr(0, pd);
   //   masks = watchDef.substr(pd+1);
   //}

   if (masks.size() > 0) {
      const char delim[] = ";{}";
      char *cstr, *p;
      cstr = new char [masks.size()+1];
      strcpy (cstr, masks.c_str());

      p = strtok (cstr, delim);
      while (p != NULL) {
         if (strlen(p) > 0) {
            std::ostringstream msk;
            while(*p != '\0') {
               switch (*p) {
                  case '.': msk << "\\."; break;
                  case '*': msk << ".*"; break;
                  case '?': msk << "."; break;
                  default: msk << *p; break;
               }
               p++;
            }
            msk << "$";
            filemasks.push_back(msk.str());
         }
         p = strtok(NULL, delim);
      }

      if (cstr) delete[] cstr;
   }
}

bool CFileMonitor::SWatchInfo::matches(const std::string &fname)
{
   for (typeof(filemasks.begin()) it = filemasks.begin(); it != filemasks.end(); it++) {
      boost::regex rx (*it);
      boost::smatch res;
      boost::match_flag_type flags = boost::match_default;
      if (boost::regex_match(fname, res, rx, flags)) {
         // printf("Found match for %s in %s\n", fname.c_str(), (*it).c_str());
         return true;
      }
   }
   return false;
}

void CFileMonitor::SWatchInfo::dump(std::ostream &stream)
{
   stream << "Watch " << watchId << ": " << directory << "/{";
   for (typeof(filemasks.begin()) it = filemasks.begin(); it != filemasks.end(); it++) {
      if (it != filemasks.begin()) stream << ", ";
      stream << (*it);
   }
   stream << "}";
}

CFileMonitor::CFileMonitor()
{
}

CFileMonitor::~CFileMonitor()
{
   for (typeof(m_watches.begin()) it = m_watches.begin(); it != m_watches.end(); it++) {
      SWatchInfo* pinfo = *it;
      delete pinfo;
   }
}

void CFileMonitor::configure(const std::map<std::string,std::string> & _config)
{
   debug("CFileMonitor::configure");

   std::map<std::string, std::string>::const_iterator it, itVar;
   m_display.configureDisplayClient(_config);

   boost::regex rxVar( "(\\w+)=(.*)" );
   boost::smatch res;
   boost::match_flag_type flags = boost::match_default;
   std::map<std::string, std::string> vars;
   if((it = _config.find("--setvars")) != _config.end()) {
      std::istringstream istr(it->second);
      std::string vardef;
      while(istr >> vardef) {
         if (boost::regex_match(vardef, res, rxVar, flags)) {
            vars[res[1]] = res[2];
         }
      }
   }

   if((it = _config.find("--monitor")) != _config.end()) {
      std::istringstream istr(it->second);
      std::string wdef;
      while(istr >> wdef) {
         for (itVar = vars.begin(); itVar != vars.end(); itVar++) {
            boost::replace_all(wdef, std::string("%(") + itVar->first + ")", itVar->second);
         }
         SWatchInfo* pinfo = new SWatchInfo(wdef);
         if (pinfo->filemasks.size() < 1) {
            log("Error --monitor: No files selected with %s", wdef.c_str());
            log("   directory: %s", pinfo->directory.c_str());
            delete pinfo;
         }
         else {
            m_watches.push_back(pinfo);
            std::ostringstream wtch;
            pinfo->dump(wtch);
            debug("%s", wtch.str().c_str());
         }
      }
   }

   debug("configure done");
}

void CFileMonitor::start()
{
}

void CFileMonitor::destroy()
{
}

void describeEvent(inotify_event *event) 
{
   if ( event->mask & IN_CREATE ) {
      if ( event->mask & IN_ISDIR ) {
         printf( "The directory %s was created.\n", event->name );       
      }
      else {
         printf( "The file %s was created.\n", event->name );
      }
   }
   else if ( event->mask & IN_DELETE ) {
      if ( event->mask & IN_ISDIR ) {
         printf( "The directory %s was deleted.\n", event->name );       
      }
      else {
         printf( "The file %s was deleted.\n", event->name );
      }
   }
   else if ( event->mask & IN_MODIFY ) {
      if ( event->mask & IN_ISDIR ) {
         printf( "The directory %s was modified.\n", event->name );
      }
      else {
         printf( "The file %s was modified.\n", event->name );
      }
   }
   else if ( event->mask & IN_CLOSE_WRITE ) {
      if ( event->mask & IN_ISDIR ) {
         printf( "The directory %s was closed (written).\n", event->name );
      }
      else {
         printf( "The file %s was closed (written).\n", event->name );
      }
   }
}

void CFileMonitor::processFileChange(int watchId, const std::string &fname)
{
   debug("FILE CHANGED: %s", fname.c_str());
   for (typeof(m_watches.begin()) it = m_watches.begin(); it != m_watches.end(); it++) {
      SWatchInfo* pinfo = *it;
      if (pinfo->watchId != watchId) continue;
      if (! pinfo->matches(fname)) continue;

      SConverter *pConv = pinfo->pConverter;
      if (pConv == NULL) {
         int fd = fname.find_last_of(".");
         if (fd >= 0) {
            std::string ext = fname.substr(fd+1);
            pConv = SConverter::findByExt(ext);
         }
      }
      if (pConv == NULL) {
         log("Error: Could not find a suitable converter for %s", fname.c_str());
         return;
      }
      std::string title = pinfo->title;
      if (title == "") {
         int fd = fname.find_last_of("/");
         if (fd < 0) title = fname;
         else title = fname.substr(fd+1);
      }

      if (pConv->command != "") {
         debug("Convert cmd: %s -> %s", pConv->command.c_str(), title.c_str());
         std::ostringstream cmd;
         cmd << pConv->command << " ";
         cmd << pinfo->directory << "/" << fname;
         FILE *fp = popen(cmd.str().c_str(), "r");
         if (fp == NULL) {
            log("Popen failed: %s", cmd.str().c_str());
            break;
         }

         if (pConv->type == "text") {
            std::ostringstream xml;
            const int PACK_LEN = 255;
            char buffer[PACK_LEN + 1];
            int len = PACK_LEN;
            while (len == PACK_LEN) {
               len = fread(buffer, 1, PACK_LEN, fp);
               if (len > 0) {
                  buffer[len] = '\0';
                  xml << std::string(buffer);
               }
            }

            m_display.setObject(title, "FileMonitor", xml.str());
         }
         else {
            // TODO: capture binary data from stream
         }
         pclose(fp);
      }
      else {
         std::string fn = pinfo->directory + "/" + fname;
         std::ifstream infile;
         // No converter, read the file and send it
         if (pConv->id == "luagl") {
            debug("Send LuaGl script: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setLuaGlObject(title, "FileMonitor", str.str());
         }
         else if (pConv->id == "image") {
            debug("Send image: %s", title.c_str());
            // read binary data from file (server needs a setCompressedImage)
            std::vector<unsigned char> data;
            infile.open(fn.c_str(), std::ios::in|std::ios::binary|std::ios::ate);
            long size = infile.tellg();
            infile.seekg(0, std::ios::beg);
            data.resize(size);
            infile.read((char*)&data[0], size);
            infile.close();
            m_display.setImage(title, data);
         }
         else if (pConv->id == "html") {
            debug("Send HTML chunk: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setHtml(title, "FileMonitor", str.str());
         }
         else if (pConv->id == "htmlhead") {
            debug("Send HTML HEAD chunk: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setHtmlHead(title, "FileMonitor", str.str());
         }
         else if (pConv->type == "text") {
            debug("Send text ad HTML chunk: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setHtml(title, "FileMonitor", str.str());
         }
      }
   }
}


// see also: man://inotify(7), man://inotify_init, man://select(2), man://select_tut(2) 
void CFileMonitor::runComponent()
{
   m_display.connectIceClient(*this);

   /* size of the event structure, not counting name */
   const size_t EVENT_SIZE = (sizeof (struct inotify_event));

   /* reasonable guess as to size of 1024 events */
   const size_t BUF_LEN = (1024 * (EVENT_SIZE + 16));
   char buffer[BUF_LEN];

   int fd = inotify_init();

   // Add watches for all configured files/directories
   for (typeof(m_watches.begin()) it = m_watches.begin(); it != m_watches.end(); it++) {
      SWatchInfo* pinfo = *it;
      int wd = inotify_add_watch(fd, pinfo->directory.c_str(), IN_CLOSE_WRITE);
      if (wd >= 0) {
         pinfo->watchId = wd;
      }
      else {
         log("inotify_add_watch returned %d for %s", wd, pinfo->directory.c_str());
      }
   }

   // int wd = inotify_add_watch(fd, "/home/mmarko/Documents/doc/Devel/CogX/code/systems/ul", IN_ALL_EVENTS);
   // int wd = inotify_add_watch(fd, ".", IN_CLOSE_WRITE);

   while (isRunning()) {
      // wait for events, but use timeout (select)
      // timeout needs to be intialized every time ( linux implementation of select )
      struct timeval timeout;
      timeout.tv_sec = 2;
      timeout.tv_usec = 0;
      fd_set rfds;
      FD_ZERO (&rfds);
      FD_SET (fd, &rfds);
      int ret = select (fd+1, &rfds, NULL, NULL, &timeout);

      if (ret < 0) {
         debug("Error: select");
      }
      else if (ret == 0) {
         // debug("inotify: select timed out");
      }
      else if (ret && FD_ISSET (fd, &rfds)) { // rfds changed before timeout
         int length = read( fd, buffer, BUF_LEN );  
         // debug("Change occured, length=%d", length);
         int i = 0;
         if (length < 0) {
            /* error, etc. */
         }
         else {
            while (i < length) {
               struct inotify_event *event;
               event = ( struct inotify_event * ) &buffer[ i ];
               if ( event->len ) {
                  // describeEvent(event);
                  if ( event->mask & IN_CLOSE_WRITE && ! (event->mask & IN_ISDIR)) {
                     processFileChange(event->wd, event->name);
                  }
               }
               i += EVENT_SIZE + event->len;
            }
         }
      }
   }

   if (fd >= 0) close(fd);
}

}} // namespace
