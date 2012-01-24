/*
 * Author: Marko Mahnič
 * Created: 2010-04-26
 */

#include "FileMonitor.hpp"

#include <sys/types.h>
#include <sstream>
#include <fstream>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#ifdef __APPLE__
#else
#define USE_INOTIFY 1
#endif

#if USE_INOTIFY
#include <sys/inotify.h>
#else
#endif

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::display::CFileMonitor();
   }
}

namespace cogx { namespace display {

std::map<std::string, SConverter> SConverter::converters;

void SConverter::add(const std::string &id, const std::string &command,
      const std::string &type, const std::string &exts)
{
   converters[id].id = id;
   converters[id].command = command;
   converters[id].type = type;
   converters[id].extensions = std::string(",") + exts + ",";
}

SConverter* SConverter::find(const std::string &id)
{
   typeof(converters.begin()) it = converters.find(id);
   if (it == converters.end()) return NULL;
   return &it->second;
}

SConverter* SConverter::findByExt(const std::string &ext)
{
   if (ext == "") return NULL;
   std::string fx = std::string(",") + ext + ",";
   for (typeof(converters.begin()) it = converters.begin(); it != converters.end(); it++) {
      int pd = it->second.extensions.find(fx);
      if (pd >= 0) return &it->second;
   }
   return NULL;
}

std::string SConverter::names()
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
      SConverter::add("image", "", "binary", "bmp,gif,jpeg,jpg,png,pbm,pgm,ppm,tiff,xbm,xpm");
      SConverter::add("dot", "dot -Tsvg", "text", "dot");
      SConverter::add("neato", "neato -Tsvg");
      SConverter::add("twopi", "twopi -Tsvg");
      SConverter::add("circo", "circo -Tsvg");
      SConverter::add("fdp", "fdp -Tsvg");
      SConverter::add("svg", "", "text", "svg");
      SConverter::add("luagl", "", "text", "luagl,lua");
      SConverter::add("html", "", "text", "html,htm");
      SConverter::add("htmlhead", "", "text", "css");
   }
} _init_converters_;

SWatchInfo::SWatchInfo(const std::string &watchDef)
{
   this->watchDef = watchDef;
   watchId = -1;
   changeCount = 0;
   pConverter = NULL;

   // title#section=converter:path{masks}
   // title --> object on server
   // section --> object part on server, if supported by object type
   //         '%c' in section will be replaced with a sequential number
   boost::regex rxWatch ("\\s*(([\\w\\.]+)(#[\\w%]+)?\\=)?((\\w+)\\:)?([^{]+)(\\{([^}]+)\\})?\\s*");
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
   section = res[3];
   directory = res[6];
   std::string converter = res[5];
   std::string masks = res[8];

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

bool SWatchInfo::matches(const std::string &fname)
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

void SWatchInfo::dump(std::ostream &stream)
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

#if USE_INOTIFY
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
#endif

// processFileChange should be called after a file that has been modified
// is closed.
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
         continue;
      }
      std::string title = pinfo->title;
      if (title == "") {
         int fd = fname.find_last_of("/");
         if (fd < 0) title = fname;
         else title = fname.substr(fd+1);
      }

      std::string section = pinfo->section;
      if (section == "") section = "FileMonitor";
      else {
         size_t pos = section.find ("%c");
         if (pos != std::string::npos) {
            char buf[32];
            ++ pinfo->changeCount;
            sprintf(buf, "%04ld", pinfo->changeCount);
            section.replace(pos, 2, buf);
         }
      }

      if (pConv->command != "") {
         debug("Convert cmd: %s -> %s", pConv->command.c_str(), title.c_str());
         std::ostringstream cmd;
         cmd << pConv->command << " ";
         cmd << pinfo->directory << "/" << fname;
         FILE *fp = popen(cmd.str().c_str(), "r");
         if (fp == NULL) {
            log("Popen failed: %s", cmd.str().c_str());
            continue;
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

            m_display.setObject(title, section, xml.str());
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
            m_display.setLuaGlObject(title, section, str.str());
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
            m_display.setHtml(title, section, str.str());
         }
         else if (pConv->id == "htmlhead") {
            debug("Send HTML HEAD chunk: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setHtmlHead(title, section, str.str());
         }
         else if (pConv->type == "text") {
            debug("Send text ad HTML chunk: %s", title.c_str());
            infile.open(fn.c_str(), std::ifstream::in);
            std::stringstream str;
            str << infile.rdbuf();
            infile.close();
            m_display.setHtml(title, section, str.str());
         }
      }
   }
}

#if USE_INOTIFY
// see also: man://inotify(7), man://inotify_init, man://select(2), man://select_tut(2) 

/* size of the event structure, not counting name */
const size_t EVENT_SIZE = (sizeof (struct inotify_event));

class CINotifyMonitor
{
   CFileMonitor *m_pMonitor;
   int m_fd;
   char* m_buffer;
   size_t m_bufLen;

public:
   CINotifyMonitor(CFileMonitor& Monitor) {
      m_pMonitor = &Monitor;
      m_fd = -1;
      m_buffer = NULL;
      m_bufLen = 0;
   }

   ~CINotifyMonitor() {
      releaseWatches();
   }

   void releaseWatches() {
      if (m_fd >= 0) close(m_fd);
      m_fd = -1;
      if (m_buffer) delete m_buffer;
      m_buffer = NULL;
      m_bufLen = 0;
   }

   void installWatches() {
      releaseWatches();

      /* reasonable guess as to size of 1024 events */
      m_bufLen = (1024 * (EVENT_SIZE + 16));
      m_buffer = new char[m_bufLen];

      m_fd = inotify_init();

      // Add watches for all configured files/directories
      for (typeof(m_pMonitor->m_watches.begin()) it = m_pMonitor->m_watches.begin();
            it != m_pMonitor->m_watches.end(); it++)
      {
         SWatchInfo* pinfo = *it;
         int wd = inotify_add_watch(m_fd, pinfo->directory.c_str(), IN_CLOSE_WRITE);
         if (wd >= 0) {
            pinfo->watchId = wd;
         }
         else {
            bool found = false;
            for (typeof(m_pMonitor->m_watches.begin()) it2 = m_pMonitor->m_watches.begin();
                  it2 != it; it2++)
            {
               SWatchInfo* pinfo2 = *it2;
               m_pMonitor->debug("%s == %d %s", pinfo->directory.c_str(), pinfo2->watchId,
                     pinfo2->directory.c_str());
               if (pinfo2->directory == pinfo->directory && pinfo2->watchId >= 0) {
                  found = true;
                  pinfo->watchId = pinfo2->watchId;
                  break;
               }
            }
            if (! found)
               m_pMonitor->log("inotify_add_watch returned %d for %s", wd, pinfo->directory.c_str());
         }
      }

      // int wd = inotify_add_watch(fd, "/home/mmarko/Documents/doc/Devel/CogX/code/systems/ul", IN_ALL_EVENTS);
      // int wd = inotify_add_watch(fd, ".", IN_CLOSE_WRITE);
   }

   // Run this function periodically in the main loop.
   // If the monitor doesn't need polling, just put a sleepComponent in here.
   void pollWatches() {
      // wait for events, but use timeout (select)
      // timeout needs to be intialized every time ( linux implementation of select )
      struct timeval timeout;
      timeout.tv_sec = 2;
      timeout.tv_usec = 0;
      fd_set rfds;
      FD_ZERO (&rfds);
      FD_SET (m_fd, &rfds);
      int ret = select (m_fd+1, &rfds, NULL, NULL, &timeout);

      if (ret < 0) {
         m_pMonitor->debug("Error: select");
      }
      else if (ret == 0) {
         // debug("inotify: select timed out");
      }
      else if (ret && FD_ISSET (m_fd, &rfds)) { // rfds changed before timeout
         int length = read( m_fd, m_buffer, m_bufLen );  
         // debug("Change occured, length=%d", length);
         int i = 0;
         if (length < 0) {
            /* error, etc. */
         }
         else {
            while (i < length) {
               struct inotify_event *event;
               event = ( struct inotify_event * ) &m_buffer[ i ];
               if ( event->len ) {
                  // describeEvent(event);
                  if ( event->mask & IN_CLOSE_WRITE && ! (event->mask & IN_ISDIR)) {
                     m_pMonitor->processFileChange(event->wd, event->name);
                  }
               }
               i += EVENT_SIZE + event->len;
            }
         }
      }
   }

};
#else
class CDummyMonitor
{
   CFileMonitor *m_pMonitor;
public:
   CDummyMonitor(CFileMonitor &Monitor) {
      m_pMonitor = &Monitor;
   }
   void installWatches() {}
   void pollWatches() {
      m_pMonitor->println("DUMMY MONITOR");
      m_pMonitor->sleepComponent(5000);
   }
   void releaseWatches() {}
};
#endif


void CFileMonitor::runComponent()
{
   m_display.connectIceClient(*this);

#if USE_INOTIFY
   CINotifyMonitor monitor(*this);
#else
   CDummyMonitor monitor(*this);
#endif

   monitor.installWatches();

   debug("Registered watches (-1 = failed to initilaize)");
   for (typeof(m_watches.begin()) it = m_watches.begin(); it != m_watches.end(); it++) {
      SWatchInfo* pinfo = *it;
      std::ostringstream wtch;
      pinfo->dump(wtch);
      debug("%s", wtch.str().c_str());
   }

   while (isRunning()) {
      monitor.pollWatches();
   }

   monitor.releaseWatches();
}

}} // namespace
// vim:sw=3:ts=8:et
