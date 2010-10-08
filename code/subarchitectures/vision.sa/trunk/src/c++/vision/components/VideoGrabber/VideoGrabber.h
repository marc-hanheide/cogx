/**
 * @author Marko Mahnič
 * @date September 2010
 *
 */

#ifndef VIDEOGRABBER_V7JAPUYG
#define VIDEOGRABBER_V7JAPUYG

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cogx
{

class CVideoGrabber:
    public cast::ManagedComponent,
    public cast::VideoClient
{
private:
   /**
    * Which camera to get images from
    */
   std::vector<int> m_camIds;

   /**
    * component ID of the video server to connect to
    */
   std::string m_videoServerName;
   /**
    * our ICE proxy to the video server
    */
   Video::VideoInterfacePrx m_pVideoServer;

   /**
    * wether we are currently receiving images from the server
    */
   bool m_bReceiving;

   int m_frameGrabCount;       // how many frames to save

#ifdef FEAT_VISUALIZATION
   IplImage* m_pDisplayCanvas; // Image for the DisplayServer
   std::vector<unsigned char> m_DisplayBuffer; // For transfering data to the server;

   void prepareCanvas(int width, int height);
   void releaseCanvas();
   
   class CVvDisplayClient: public cogx::display::CDisplayClient
   {
      CVideoGrabber* pViewer;
   public:
      cogx::display::CFormValues m_frmSettings;
      // Access form variables
      std::string getDirectory();
      void setDirectory(const std::string& name);
      bool getCreateDirectory();
      std::string getDeviceNames();
      void setDeviceNames(const std::string& name);
      std::string getImageFilenamePatt();
      void setImageFilenamePatt(const std::string& pattern);
      long getCounterDigits();
      void setCounterDigits(long nDigits);
      long getCounterValue();
      void setCounterValue(long nValue);

   public:
      CVvDisplayClient() { pViewer = NULL; }
      void setClientData(CVideoGrabber* pVideoGrabber) { pViewer = pVideoGrabber; }
      void handleEvent(const Visualization::TEvent &event); /*override*/
      std::string getControlState(const std::string& ctrlId); /*override*/
      void handleForm(const std::string& id, const std::string& partId,
            const std::map<std::string, std::string>& fields);
      bool getFormData(const std::string& id, const std::string& partId,
            std::map<std::string, std::string>& fields);

      void createForms();
   };
   CVvDisplayClient m_display;
#endif

protected:
   virtual void configure(const std::map<std::string,std::string> & _config);
   virtual void start();
   virtual void destroy();
   virtual void runComponent();

public:
   CVideoGrabber()
   {
      m_frameGrabCount = 0;
#ifdef FEAT_VISUALIZATION
      m_pDisplayCanvas = NULL;
      m_display.setClientData(this);
#endif
   }
   virtual ~CVideoGrabber()
   {
#ifdef FEAT_VISUALIZATION
      releaseCanvas();
#endif
   }
   /**
    * The callback function for images pushed by the image server.
    * To be overwritten by derived classes.
    */
   virtual void receiveImages(const std::vector<Video::Image>& images);

   std::vector<std::string> getDeviceNames();
   void saveImages(const std::vector<Video::Image>& images);
};

} // namespace
#endif
/* vim: set sw=3 ts=8 et: */

