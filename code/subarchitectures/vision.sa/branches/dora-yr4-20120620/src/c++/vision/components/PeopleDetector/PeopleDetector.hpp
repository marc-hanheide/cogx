#include <Ice/Ice.h>

#include <Laser.hpp>

#include <cast/core/CASTComponent.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

#include <highgui.h>
#include <cvaux.h>


namespace cast
{

  /**
   * Written by Costas Cristofi at Birmingham.
   */
    class PeopleDetector : public cast::VideoClient, 
			   public ManagedComponent
    {
        bool deinterlacing;
        int sleepForSync;
        double xscale, xcentre;
        double trackerThreshold;
        int removeAfterFrames;
        double removeAfterDistance;

      /**
       * component ID of the video server to connect to
       */
      std::string videoServerName;
      /**
       * Which camera to get images from
       */
      int camId;
      
      /**
       * our Ice proxy to the video server
       */
      Video::VideoInterfacePrx m_videoServer;

      /**
       * How many frames to use when triggered on demand. Default 5.
       */
      unsigned int m_numDetectionAttempts;

      /**
       * Whether to run continuously or not. Default false.
       */
      bool m_runContinuously;

    public:
        /**
         * Constructor
         */
        PeopleDetector();

        /**
         * Destructor
         */
        ~PeopleDetector();


      /**
       * Run the people detector for a single frame.
       */
      void runDetection();


      /**
       * Do detection on demand.
       */
      void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);


    protected:

        virtual void configure(const std::map<std::string,std::string> & config);
        virtual void start();
        virtual void runComponent();

    protected:



        CvMemStorage * upper_storage;
        CvHaarClassifierCascade * upper_cascade;
        CvMemStorage * face_storage;
        CvHaarClassifierCascade * face_cascade;
        CvMemStorage * fullbody_storage;
        CvHaarClassifierCascade * fullbody_cascade;

        Laser::LaserServerPrx m_laserServer;
    };

}
