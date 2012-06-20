/**
 * @file Configuration.cpp
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Configuration.
 */

#include "StereoDetector.h"
// #include "StereoBase.h"
#include <VisionData.hpp>

namespace cast
{
  
/**
 * @brief Destructor of class StereoDetector
 */
StereoDetector::~StereoDetector() {}

/**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void StereoDetector::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves (for getRectImage)
  configureServerCommunication(_config);

  nr_p_score = 0;                       // start with first processing score
  
  runtime = 1600;                       // processing time for left AND right image
  cannyAlpha = 0.75;                    // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;

  activeReasoner = false;               // activate reasoner
  activeReasonerPlane = false;          // activate plane C
  
  receiveImagesStarted = false;
  haveImage = false;
  haveHRImage = false;
  havePrunedImage = false;
  cmd_detect = false;
  cmd_single = false;
  cmd_single_hr = false;

  detail = 0;
  showImages = false;
  showDetected = true;
  showSingleGestalt = false;
  showAllStereo = false;
  showID = 0;
  showMasked = false;
  showStereoMatched = true;
  showAllStereoMatched = false;
  showSingleStereo = false;
  single = false;
  showType = Z::Gestalt::SEGMENT;
  showStereoType = Z::StereoBase::STEREO_CLOSURE;
  showSegments = false;
  showROIs = false;
  showReasoner = false;
  showReasonerUnprojected = false;
  
  write_stereo_lines = false;
  write_stereo_ellipses = false;
  write_stereo_ljcts = false;
  write_stereo_closures = false;
  write_stereo_rectangles = false;
  write_stereo_flaps = false;
  write_stereo_corners = false;


  map<string,string>::const_iterator it;
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  if((it = _config.find("--stereoconfig")) != _config.end())
  {
    stereoconfig = it->second;
    try
    {
      score = new Z::StereoCore(stereoconfig);
      p_score[0] = new Z::StereoCore(stereoconfig);
      p_score[1] = new Z::StereoCore(stereoconfig);
      p_score[2] = new Z::StereoCore(stereoconfig);
    }
    catch (exception &e)
    {
      printf("StereoDetector::configure: Error during initialisation of stereo core.\n");
      cout << e.what() << endl;
    }
  }
  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }
  
  // note: it is ok to not specify these, defaults will be chosen in that case
  if((it = _config.find("--camconfigs")) != _config.end())
  {
    istringstream str(it->second);
    string file;
    while(str >> file)
    {
      Video::CameraParameters pars;
      // calibration files can be either monocular calibration files (e.g.
      // "flea0.cal", "flea1.cal") or SVS stereo calib files (e.g.
      // "stereo.ini:L", "stereo.ini:R"). Note that in the latter case the
      // actual file name is extended with a ":" (to indicate we have a stereo
      // case) and a L or R indicating we refer to the LEFT or RIGHT camera of
      // the stereo rig.
      if(file.find(":") == string::npos)
      {
        // monocular files can be either .cal (INI style) or .xml (from OpenCV file storage)
        if(file.find(".xml") == string::npos)
          loadCameraParameters(pars, file);
        else
          loadCameraParametersXML(pars, file);
      }
      else
      {
        // stereo case
        size_t pos = file.find(":");
        if(pos >= file.size() - 1)
          throw runtime_error(exceptionMessage(__HERE__,
                "please indicate camera L or R after ':' in config '%s'", file.c_str()));
        char side = file[pos + 1];
        string pure_filename(file, 0, pos);
        if(side == 'L')
          loadCameraParametersFromSVSCalib(pars, pure_filename, 0);   // 0 == LEFT
        else if(side == 'R')
          loadCameraParametersFromSVSCalib(pars, pure_filename, 1);   // 1 == RIGHT
        else
          throw runtime_error(exceptionMessage(__HERE__,
                "camera '%c' invalid in config '%s', must be either :L or :R",
                side, file.c_str()));
      }
      camPars.push_back(pars);
    }
  }
  
  // in case no camera config files were given, assume default configs
  if(camPars.size() == 0)
  {
    log("configure: Warning: No 'camconfigs' specified!");
    camPars.resize(camIds.size());
    for(size_t i = 0; i < camPars.size(); i++)
      initCameraParameters(camPars[i]);
  }

  // fill camIds and current time stamp into cam parameters
  // TODO: avoid this double ids at some point
  for(size_t i = 0; i < camPars.size(); i++)
  {
    camPars[i].id = camIds[i];
    camPars[i].time = getCASTTime();
  }
  
  if((it = _config.find("--showImages")) != _config.end())
  {
    showImages = true;
  }
  if((it = _config.find("--singleShot")) != _config.end())
  {
    log("single shot modus on.");
    single = true;
  }
  
  ///< OpenNI Interface
//   openNI = new Z::OpenNIInterface(KINECT_XML_FILE);
//   openNI->StartCapture(0);
//   showKinectImage = true;        // TODO
//   log("kinect connected and started capturing!");
  
  
  /// initialize ipl-Images
  iplImage_l = 0;
  iplImage_r = 0;
  iplImage_l_pr = 0;
  iplImage_r_pr = 0;
  iplImage_l_hr = 0;
  iplImage_r_hr = 0;
//   kinectImage = 0;
//   kinectDepthImage = 0;
  
  
//   reasoner = new Z::Reasoner();

  // initialize stereo camera
  stereo_cam = new cast::StereoCamera();
  printf("Configuration::configure:: TODO Change stereocalib from svscalib to opencv xml-style!!!\n");
  if(!stereo_cam->ReadSVSCalib(stereoconfig)) throw (std::runtime_error("StereoDetector::StereoDetector: Cannot open calibration file for stereo camera."));
  
  // initialize tgRenderer
  cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
  cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
  cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
  cv::Vec3d rotCenter(0,0,0.4);
  
  // Initialize 3D render engine 
  tgRenderer = new TomGine::tgTomGineThread(1280, 1024);
  tgRenderer->SetCamera(intrinsic);
  tgRenderer->SetCamera(R, t);                    /// TODO funktioniert nicht => Wieso?
  tgRenderer->SetRotationCenter(rotCenter);       /// TODO funktioniert nicht => Wieso?
//   tgRenderer->SetCoordinateFrame();
  
  // initialize object representation
  objRep = new Z::ObjRep();
}

/**
 * @brief Called by the framework after configuration, before run loop.
 */
void StereoDetector::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // start stereo communication
  startPCCServerCommunication(*this);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  // add change filter for vision commands
  addChangeFilter(createLocalTypeFilter<VisionData::StereoDetectionCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveDetectionCommand));

  // TODO add change filter for SOI changes (add / update / delete)
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
//     new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveSOI));
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
//    new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::updatedSOI));
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
//    new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::deletedSOI));

  // add change filter for ROI changes
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveROI));
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::updatedROI));
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::DELETE),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::deletedROI));

  // add change filter for ProtoObject changes
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveProtoObject));

  // add change filter for ConvexHull changes
  addChangeFilter(createLocalTypeFilter<VisionData::ConvexHull>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveConvexHull));

  // add change filter for camera mount
  addChangeFilter(createLocalTypeFilter<Video::CameraParametersWrapper>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveCameraParameters));
  addChangeFilter(createLocalTypeFilter<Video::CameraParametersWrapper>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveCameraParameters));
      
  // initialize openCV windows
  if(showImages) 
  {
    cvNamedWindow("Stereo left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Stereo right", CV_WINDOW_AUTOSIZE);
//     cvNamedWindow("rectified", CV_WINDOW_AUTOSIZE);
    //    cvNamedWindow("Pruned left", CV_WINDOW_AUTOSIZE);
    //    cvNamedWindow("Pruned right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Back projected point cloud", CV_WINDOW_AUTOSIZE);


    cvMoveWindow("Stereo left", 10, 10);
    cvMoveWindow("Stereo right", 680, 10);
//     cvMoveWindow("rectified",  1350, 10);
    //    cvMoveWindow("Pruned left",  10, 500);
    //    cvMoveWindow("Pruned right",  680, 500);

    int mouseParam=0;
    cvSetMouseCallback("Stereo left", LeftMouseHandler, &mouseParam);
    cvSetMouseCallback("Stereo right", RightMouseHandler, &mouseParam);
  }

//   if(showKinectImage)
//   {
//     cv::namedWindow("Kinect image");
//     cv::namedWindow("Kinect depth image");
// //     cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
// //     cvMoveWindow("Kinect image", 1350, 10);
// //     cvNamedWindow("Kinect depth image", CV_WINDOW_AUTOSIZE);
// //     cvMoveWindow("Kinect depth image", 1350, 500);
//   }

  isFormat7 = videoServer->inFormat7Mode();
}

/**
 * @brief Called by the framework to start component run loop.
 * @TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void StereoDetector::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {}

  // release all ipl images
  cvReleaseImage(&iplImage_l);
  cvReleaseImage(&iplImage_r);
  cvReleaseImage(&iplImage_l_hr);
  cvReleaseImage(&iplImage_r_hr);
  cvReleaseImage(&iplImage_l_pr);
  cvReleaseImage(&iplImage_r_pr);
  
  if(showImages)
  {
    log("destroy openCV windows.");
    cvDestroyWindow("Stereo left");
    cvDestroyWindow("Stereo right");
//     cvDestroyWindow("rectified");
//     cvDestroyWindow("Pruned left");
//     cvDestroyWindow("Pruned right");
  }
  
  log("windows destroyed");
}


}