/**
 * @file StereoDetector.h
 * @author Andreas Richtsfeld, Michael Zillich
 * @date October 2009, 2010
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 */

#ifndef STEREO_DETECTOR_H
#define STEREO_DETECTOR_H

//#define HAVE_CAST 1	// Enable cast-functionality in ./stereo
// #define KINECT_XML_FILE "subarchitectures/vision.sa/config/KinectConfig.xml"

#include "DoxyMain.h"

#include <vector>
#include <stdexcept>

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <VideoUtils.h>
#include <../../VisionUtils.h>

#include "StereoCore.h"
#include "Pose3.h"
#include "StereoBase.h"
#include "Gestalt.hh"
#include "Reasoner.h"
#include "Array.hh"

#include "ObjRep.h"
#include "v4r/TomGine/tgTomGineThread.h"

#include "StereoCamera.h"

namespace cast
{

/**
 * @class StereoDetector
 * @brief Implementation of the stereo detector as cast component.
 */
class StereoDetector : public ManagedComponent,
                       public VideoClient,
                       public PointCloudClient
{
private:
  TomGine::tgTomGineThread *tgRenderer;           ///< 3D render engine
  std::vector<PointCloud::SurfacePoint> points;   ///< 3D point vector
  IplImage *backProjPointCloud;                   ///< Back projected point cloud
  cast::StereoCamera *stereo_cam;                       ///< stereo camera parameters and functions

  int runtime;                                    ///< Overall processing runtime for one image (pair)
  Z::StereoCore *score;                           ///< Stereo core
  Z::StereoCore *p_score[3];                      ///< Processing stereo cores for three frames
  int nr_p_score;                                 ///< Actual number of processing score
  int showFrame;                                  ///< Show another frame (p_score[showFrame])
//	Z::Reasoner *reasoner;                          ///< Reasoner (and Filter)
  Z::ObjRep *objRep;                              ///< Object representation as graph
  
  float cannyAlpha, cannyOmega;                   ///< Alpha and omega value of the canny edge detector											/// TODO muss hier nicht sein?
  std::vector<int> camIds;                        ///< Which cameras to get images from
  std::vector<Video::CameraParameters> camPars;   ///< Camera parameters for each camera
  std::string videoServerName;                    ///< Component ID of the video server to connect to
  std::string stereoconfig;                          ///< Config name of stereo camera config file
  Video::VideoInterfacePrx videoServer;           ///< ICE proxy to the video server
  Video::Image image_l, image_r;                  ///< Left and right stereo image from video server. Original images.
  IplImage *iplImage_l, *iplImage_r;              ///< Converted left and right stereo images (openCV ipl-images)
  IplImage *iplImage_l_hr, *iplImage_r_hr;        ///< High resolution, converted left and right stereo images (openCV ipl-images)
  IplImage *iplImage_l_pr, *iplImage_r_pr;        ///< Pruned, converted left and right stereo images (openCV ipl-images)
  bool isFormat7;	                                ///< True, when camera is in Format7 mode.

  IplImage *poImg;                                ///< Proto object image
  IplImage *poPatchImage;                         ///< Proto object patch image
  int *poMask;                                    ///< Proto object mask

  struct ROIData                                  ///< Region of interest (ROI)
  {
    CvRect rect;                                  ///< The rectangle with the ROI
    CvRect rect640;                               ///< The rectangle with the ROI for a 640x480 pruning from HR-image.
    bool rect640valid;                            ///< True, when rect640 rectangle is valid.
    int roiScale;                                 ///< Scale between video and stereo image (normally 4(:1))
  };
  std::map<std::string, ROIData> rcvROIs;         ///< Received stable region of interests (ROIs), stored with WM address

  bool activeReasoner;                            ///< Initialize true, when reasoner should work.
  bool activeReasonerPlane;                       ///< Initialize true, when reasoner should calculate dominant plane.
  std::string planeID;                            ///< Plane (from reasoner): address on working memory

  bool receiveImagesStarted;                      ///< True, when videoServer->startReceiveImages() was called.
  int mode7;                                      ///< Mode for PointGrey cameras (1,0 = resizing/pruning)
  bool haveImage;                                 ///< True, when image fetched from video server
  bool haveHRImage;                               ///< True, when HR image fetched from video server
  bool havePrunedImage;                           ///< True, when pruned image from high resolution (HR) image captured.
  bool cmd_detect;                                ///< Detection command
  bool cmd_single;                                ///< Single detection commmand
  bool cmd_single_hr;                             ///< Single detecton of pruned HR image
  bool single;                                    ///< Single shot mode for the stereo detector (debugging)
  int detail;                                     ///< Degree of detail for showing features
  bool showImages;                                ///< Show stereo images in openCV window
  bool showDetected;                              ///< Show detected features in stereo images
  bool showSingleGestalt;                         ///< Show single Gestalts
  int showID;                                     ///< ID of single Gestalt to show
  bool showAllStereo;                             ///< Show all stereo features (sent to virtual scene)				/// TODO sollte anders heißen!
  bool showAllStereoMatched;                      ///< Show all matched stereo features
  bool showStereoMatched;                         ///< Show matched features in stereo images
  bool showSingleStereo;                          ///< Show only one single stereo feature
  bool showSegments;                              ///< Show the edges on the stereo image
  bool showMasked;                                ///< Show masked features in stereo images
  bool showROIs;                                  ///< Show the ROIs at the display
  bool showReasoner;                              ///< Show reasoner results
  bool showReasonerUnprojected;                   ///< Show also the unprojected results
  std::vector<std::string> objectIDs;             ///< IDs of the currently stored visual objects
  Z::Gestalt::Type showType;                      ///< Show this type of Gestalt
  Z::StereoBase::Type showStereoType;             ///< Show this type of stereo matched Gestalt
  
  bool write_stereo_lines;                        ///< Write stereo lines to working memory
  bool write_stereo_ellipses;                     ///< Write stereo ellipses to working memory
  bool write_stereo_ljcts;                        ///< Write stereo ljcts to working memory
  bool write_stereo_closures;                     ///< Write stereo closures to working memory
  bool write_stereo_rectangles;                   ///< Write stereo rectangles to working memory
  bool write_stereo_flaps;                        ///< Write stereo flaps to working memory
  bool write_stereo_corners;                      ///< Write stereo corners to working memory

  void receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc);
  void receiveSOI(const cdl::WorkingMemoryChange & _wmc);
  void updatedSOI(const cdl::WorkingMemoryChange & _wmc);
  void deletedSOI(const cdl::WorkingMemoryChange & _wmc);
  void receiveROI(const cdl::WorkingMemoryChange & _wmc);
  void updatedROI(const cdl::WorkingMemoryChange & _wmc);
  void deletedROI(const cdl::WorkingMemoryChange & _wmc);
  void receiveProtoObject(const cdl::WorkingMemoryChange & _wmc);
  void receiveConvexHull(const cdl::WorkingMemoryChange & _wmc);
  void receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc);
  void processImage();
  void ProcessHRImages();
  void ProcessPrunedHRImages();
  void processPrunedHRImage(int oX, int oY, int sc);

  void DrawIntoTomGine();
  void ShowImages(bool convertNewIpl);
  void WriteVisualObjects();
  void WriteToWM(Z::StereoBase::Type type);
  void WriteToWM(Z::Array<VisionData::VisualObjectPtr> objects);
  void DeleteVisualObjectsFromWM();

  void SingleShotMode();
  void MouseEvent();
// 	void ReadSOIs();

  void GetImages();
  void GetHRImages();
  bool GetPrunedHRImages(int offsetX, int offsetY);

  void ChangeFormat7Mode(int mode, int offsetX, int offsetY);
  bool PlausibleROI(ROIData *roiData);

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  StereoDetector() {}
  virtual ~StereoDetector();
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif



