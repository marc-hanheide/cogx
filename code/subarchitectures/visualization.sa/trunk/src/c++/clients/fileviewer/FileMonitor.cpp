
#include "FileMonitor.hpp"

#include <sys/types.h>
#include <sys/inotify.h>
#include <sstream>
#include <boost/regex.hpp>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::display::CFileMonitor();
   }
}

namespace cogx { namespace display {

CFileMonitor::SWatchInfo::SWatchInfo(const std::string &watchDef)
{
   watchId = 0;
   int pd = watchDef.find_last_of('/');
   std::string masks;
   if (pd == watchDef.npos) {
      directory = "";
      masks = watchDef;
   }
   else if (pd > 0) {
      directory = watchDef.substr(0, pd);
      masks = watchDef.substr(pd+1);
   }

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
      if (boost::regex_match(fname.begin(), fname.end(), res, rx, flags)) {
         printf("Found match for %s in %s\n", fname.c_str(), (*it).c_str());
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

   std::map<std::string, std::string>::const_iterator it;
   m_display.configureDisplayClient(_config);

   if((it = _config.find("--monitor")) != _config.end()) {
      std::istringstream istr(it->second);
      std::string wdef;
      while(istr >> wdef) {
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
   m_display.connectIceClient(*this);
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
   debug("WILL CHECK FILE %s", fname.c_str());
   for (typeof(m_watches.begin()) it = m_watches.begin(); it != m_watches.end(); it++) {
      SWatchInfo* pinfo = *it;
      if (pinfo->watchId != watchId) continue;
      if (! pinfo->matches(fname)) continue;
      // TODO: real work
      std::ostringstream cmd;
      cmd << "dot -Tsvg ";
      cmd << pinfo->directory << "/" << fname;
      debug("cmd: %s", cmd.str().c_str());
      FILE *fp = popen(cmd.str().c_str(), "r");
      if (fp == NULL) {
         log("Popen failed: %s", cmd.str().c_str());
         break;
      }

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

      m_display.setObject("FileMonitor", "DOT", xml.str());

      break;
   }
}


// see also: man://inotify(7), man://inotify_init, man://select(2), man://select_tut(2) 
void CFileMonitor::runComponent()
{
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
         debug("Change occured, length=%d", length);
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
