/**
 * @file StereoDetectorLearner.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Learn something about properties for grouping.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "StereoDetectorKinectLines.h"

#include "Gestalt3D.h"
#include "StereoBase.h"
#include "Draw.hh"

#include "StereoCamera.h"
#include <VisionData.hpp>

#include "VisionCore.hh"
#include "Gestalt.hh"
#include "Line.hh"
#include "Vector.hh"
#include "Draw.cc"

#include "Patch3D.h"
#include "Closure3D.h"
#include "Rectangle3D.h"

#include "PCLCommonHeaders.h"

using namespace std;
using namespace VisionData;
using namespace Video;

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::StereoDetectorKinectLines();
  }
}

/// maximum allowed depth jump from one pixel to the next (0.05m == 5cm)
static double MAXIMUM_DIST = 0.05;

namespace cast
{
  
  
  /**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void StereoDetectorKinectLines::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves (for point clouds)
  configureServerCommunication(_config);

  // for one vision core: only lines
  runtime = 600;                          // processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;                      // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();            // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CORNERS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CLOSURES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_RECTANGLES);

  kcore = new Z::KinectCore();            // New kinect core
  learner = new Z::Learner();             // Learner for features

  showImages = false;
  single = false;

  map<string,string>::const_iterator it;

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
    showImages = true;

  if((it = _config.find("--singleShot")) != _config.end())
  {
    log("single shot modus on.");
    single = true;
  }

  if((it = _config.find("--stereoconfig")) != _config.end())
  {
    log("Warning: Antiquated: Use stereoconfig_xml with openCV stereo calibration");
    stereoconfig = it->second;
  }

  if((it = _config.find("--stereoconfig_xml")) != _config.end())
  {
    int side = 0;  // left
    istringstream str(it->second);
    string fileLeft, fileRight;
    str >> fileLeft;
    str >> fileRight;
      
    stereo_cam = new cast::StereoCamera();
    stereo_cam->ReadFromXML(fileLeft, 0, false);    // 0 = left
    stereo_cam->ReadFromXML(fileRight, 1, false);   // 1 = right
    stereo_cam->SetupImageRectification();
    
    score = new Z::StereoCore(fileLeft, fileRight);
  }
  else
  {
    // initialize stereo camera
    stereo_cam = new cast::StereoCamera();
    if(!stereo_cam->ReadSVSCalib(stereoconfig)) 
      throw (std::runtime_error(exceptionMessage(__HERE__, "Cannot open calibration file for stereo camera.")));
    score = new Z::StereoCore(stereoconfig);
  }
  
  // initialize tgRenderer
  cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
  cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
  cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
  cv::Vec3d rotCenter(0,0,0.4);
  
  // Initialize 3D render engine 
  tgRenderer = new TGThread::TomGineThread(1280, 1024);
  tgRenderer->SetParameter(intrinsic);
  tgRenderer->SetCamera(R, t, rotCenter);
//   tgRenderer->SetRotationCenter(rotCenter);      /// TODO funktioniert nicht => Wieso?
  tgRenderer->SetCoordinateFrame();
}

/**
 * @brief Called by the framework after configuration, before run loop.
 */
void StereoDetectorKinectLines::start()
{
  // start point cloud communication
  startPCCServerCommunication(*this);

  if(showImages) 
  {
    cvNamedWindow("Stereo left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Stereo right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Kinect", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Kinect depth map", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Kinect depth color map", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Kinect edge image", CV_WINDOW_AUTOSIZE);

    cvMoveWindow("Stereo left", 10, 10);
    cvMoveWindow("Stereo right", 680, 10);
    cvMoveWindow("Kinect",  10, 500);
    cvMoveWindow("Kinect depth map",  680, 500);
    cvMoveWindow("Kinect depth color map",  1350, 500);
    cvMoveWindow("Kinect edge image",  1350, 1000);
  }
}

/**
 * @brief Called by the framework to start component run loop.
 * @TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void StereoDetectorKinectLines::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
    processImage();
  }

  // release all ipl images
  cvReleaseImage(&iplImage_l);
  cvReleaseImage(&iplImage_r);
  cvReleaseImage(&iplImage_k);
  cvReleaseImage(&iplImage_depthMap);
//   cvReleaseImage(&iplImage_r_hr);
//   cvReleaseImage(&iplImage_l_pr);
//   cvReleaseImage(&iplImage_r_pr);
  
  if(showImages)
  {
    log("destroy openCV windows.");
    cvDestroyWindow("Stereo left");
    cvDestroyWindow("Stereo right");
    cvDestroyWindow("Kinect");
    cvDestroyWindow("Kinect depth map");
    cvDestroyWindow("Kinect depth color map");
    cvDestroyWindow("Kinect edge image");
  }
  log("windows destroyed");
}

  
/**
 * @brief Convert points from point cloud server to a depth map from view of left stereo camera.
 * @param sc Stereo camera
 * @param c Point vector with kinect data
 * @param cc Color point vector
 * @param depthImage Depth color image
 * @param depthMap Depth map from the kinect
 */
void StereoDetectorKinectLines::Points2DepthMap(cast::StereoCamera *sc, cv::Mat_<cv::Point3f> c, cv::Mat_<cv::Point3f> cc, cv::Mat_<cv::Point3f> &depthImage, cv::Mat_<cv::Point3f> &depthMap)
{
  int imgWidth = 320;                                                                                               /// TODO get image width/height
  int imgHeight = 240;

  depthImage = cv::Mat_<cv::Point3f>(imgHeight, imgWidth);
  depthMap = cv::Mat_<cv::Point3f>(imgHeight, imgWidth);
  for(unsigned i = 0; i<imgWidth; i++)
    for(unsigned j = 0; j<imgHeight; j++)
    {
      depthImage(j, i).x = 0;
      depthImage(j, i).y = 0;
      depthImage(j, i).z = 0;
      depthMap(j, i).x = 0;
      depthMap(j, i).y = 0;
      depthMap(j, i).z = 0;
    }
    
  for(unsigned i = 0; i < imgWidth*imgHeight; i++) // only one row!
  {
    cv::Point2d imgPoint;

    if(c(0, i).z != 0)
    {
      sc->ProjectPoint(c(0, i).x, c(0, i).y, c(0, i).z, imgPoint.x, imgPoint.y, 0, imgWidth);   // 0 == LEFT

      if(imgPoint.x > 0 && imgPoint.x < imgWidth && imgPoint.y > 0 && imgPoint.y < imgHeight)
      {
        depthImage((int) imgPoint.y, (int) imgPoint.x).x = ((float) cc(0, i).z)/255.;   /// change rgb
        depthImage((int) imgPoint.y, (int) imgPoint.x).y = ((float) cc(0, i).y)/255.;
        depthImage((int) imgPoint.y, (int) imgPoint.x).z = ((float) cc(0, i).x)/255.;
        
        depthMap((int) imgPoint.y, (int) imgPoint.x).x = ((float) c(0, i).z)/4.;
        depthMap((int) imgPoint.y, (int) imgPoint.x).y = ((float) c(0, i).z)/4.;
        depthMap((int) imgPoint.y, (int) imgPoint.x).z = ((float) c(0, i).z)/4.;
      }
    }
  }
}
  
  
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void StereoDetectorKinectLines::GetImageData()
{
  log("get image data.");
  pointCloudWidth = 320;                                /// TODO get from cast-file!
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
 
  points.resize(0);
  getPoints(false, pointCloudWidth, points);                  // call get points only once, if noCont option is on!!!
//   getPoints(true, pointCloudWidth, points_fromLeft);

  // convert points to cloud for render engine
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
//   Points2Cloud(points_fromLeft, cloud, colCloud);        // show untransformed kinect point cloud
//   tgRenderer->SetPointCloud(cloud, colCloud);         // show transformed kinect point cloud
  Points2Cloud(points, kinect_point_cloud, kinect_color_point_cloud);
//   tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
  
  // get rectified images from point cloud server
  getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
  getRectImage(1, kinectImageWidth, image_r);            // 0 = left image / we take it with kinect image width
  getRectImage(2, kinectImageWidth, image_k);            // 0 = left image / we take it with kinect image width
  iplImage_l = convertImageToIpl(image_l);
  iplImage_r = convertImageToIpl(image_r);
  iplImage_k = convertImageToIpl(image_k);
  
  // Reproject point cloud to left stereo camera to get the depth image from the view of the left stereo camera
  cv::Mat_<cv::Point3f> depthImage;
  cv::Mat_<cv::Point3f> depthMap;
  Points2DepthMap(stereo_cam, kinect_point_cloud, kinect_color_point_cloud, depthImage, depthMap);
 
  IplImage iplImage_depth = depthImage;
  IplImage iplImage_depthMap = depthMap;

  if(showImages)
  {
    cvShowImage("Stereo left", iplImage_l);
    cvShowImage("Stereo right", iplImage_r);
    cvShowImage("Kinect depth color map", &iplImage_depth);
    cvShowImage("Kinect depth map", &iplImage_depthMap);
  }
}



/**
 *  @brief Process stereo image pair at stereo core.
 *  Use normal resolution of images (640x480).
 *  TODO Try extraction of lines from high resolution image! => sub pixel accuracy
 */
void StereoDetectorKinectLines::processImage()
{
  log("Process new images with runtime: %ums", runtime);
  kcore->ClearResults();
  score->ClearResults();
  vcore->ClearGestalts();
  tgRenderer->Clear();

  // Get kinect image and run vision core with line detection
  GetImageData();
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  

  Z::SetActiveDrawArea(iplImage_k);
  vcore->DrawGestalts(Z::Gestalt::RECTANGLE, 0);
  
//   calc3DSegments();
//   calc3DLines();
//   LearnCollinearities(vcore);
  
//   printf("StereoDetectorKinectLines::processImage: kcore->ProcessKinectData()\n");
  cv::Mat_<cv::Vec4f> kinect_point_cloud_new;
  Points2Cloud(points, kinect_point_cloud_new);
  kcore->ProcessKinectData(vcore, iplImage_k, kinect_point_cloud_new);
//   kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::SEGMENT);
//   kcore->PrintGestalts3D(Z::Gestalt3D::SEGMENT);
//   kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE);
//   kcore->PrintGestalts3D(Z::Gestalt3D::LINE);
//   kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY);
//   kcore->PrintGestalts3D(Z::Gestalt3D::COLLINEARITY);
//   printf("StereoDetectorKinectLines::processImage: kcore->ProcessKinectData() end\n");
//   kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH);
//   kcore->PrintGestalts3D(Z::Gestalt3D::PATCH);
  kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH);

  printf("StereoDetectorKinectLines::processImage: kcore->ProcessKinectData() end\n");
  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

  learner->Process(kcore);

  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

//   // Compare color histograms of different kinect patches
//   for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::PATCH)-1; i++)
//   {
//     Z::Patch3D *p = (Z::Patch3D*)(kcore->Gestalts3D(Z::Gestalt3D::PATCH, i));
//     double comp_val = p->Compare((Z::Patch3D*)kcore->Gestalts3D(Z::Gestalt3D::PATCH, i+1));
//     printf("comp_val patch[%u][%u]: %4.3f\n", i, i+1, comp_val);
//   }
//   for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::CLOSURE)-1; i++)
//   {
//     Z::Closure3D *c = (Z::Closure3D*)(kcore->Gestalts3D(Z::Gestalt3D::CLOSURE, i));
//     double comp_val = c->Compare((Z::Closure3D*)kcore->Gestalts3D(Z::Gestalt3D::CLOSURE, i+1));
//     printf("comp_val closure[%u][%u]: %4.3f\n", i, i+1, comp_val);
//   }
//   for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::RECTANGLE)-1; i++)
//   {
//     Z::Rectangle3D *p = (Z::Rectangle3D*)(kcore->Gestalts3D(Z::Gestalt3D::RECTANGLE, i));
//     double comp_val = p->Compare((Z::Rectangle3D*)kcore->Gestalts3D(Z::Gestalt3D::RECTANGLE, i+1));
//     printf("comp_val rectangle[%u][%u]: %4.3f\n", i, i+1, comp_val);
//   }
  
  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
//   pcl::PointCloud<pcl::PointXYZRGB> cloud;
//   pclU::Points2PCLColCloud(points, cloud);
//   
//   /// Clustering and Segmentation of the point cloud
//   struct timespec start, current;
//   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
// 
//   bool useVoxelGrid = true;
//   double vg_size = 0.008;     /// 0.01 - 0.005
//   pclF::PreProcessPointCloud(cloud, useVoxelGrid, vg_size);
// 
// //   clustered_planes.resize(0);
// //   clustered_planes2.resize(0);
//   
//   
//   bool sac_optimal_distance = true;
//   double sac_optimal_weight_factor = 1.5;
//   double sac_distance = 0.005;            // 5mm
//   int sac_max_iterations = 100;
//   int sac_min_inliers = 25;
//   double ec_cluster_tolerance = 0.015;    // 15mm
//   int ec_min_cluster_size = 25;
//   int ec_max_cluster_size = 1000000;
//   
//   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;
//   std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
//   pclF::FitPlanesRecursive(cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
//                            sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
// //   pclF::FitPlanesRecursiveWithNormals(cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
// //                            sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
// 
//   /// calculate convex hulls
//   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_hulls;
//   pclF::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_hulls);
// 
//   /// convert pcl point clouds to cv matrices
//   std::vector< cv::Mat_<cv::Vec4f> > cv_planes, cv_hulls;
//   pclU::PCLClouds2CvClouds(pcl_plane_clouds, cv_planes);
//   pclU::PCLClouds2CvClouds(pcl_hulls, cv_hulls, true);
//         
// // pclU::PrintPCLCloud(*pcl_hulls[0]);
//   
//   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
//   printf("Runtime for processing the point cloud: %4.3f\n", timespec_diff(&current, &start));
//   start=current;
// 
//   tgRenderer->Clear();
//   tgRenderer->SetPointClouds(cv_planes);
//   tgRenderer->AddConvexHulls(cv_hulls);
//   
// //   if(!makeConvexHull)
// //     tgRenderer->SetPointClouds(clustered_planes);
// //   else
// //     tgRenderer->AddConvexHulls(clustered_planes);

  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

  
  // Stereo core calculations (TODO needs perfekt calibration => solve problem)
//   score->ProcessStereoImage(runtime, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);
    
  
  if(showImages) cvShowImage("Kinect", iplImage_k);
  
  log("Processing of stereo images ended.");
}





/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   F1 ... Show this help
 *   F2 ... Print number of Gestalts
 *   F3 ... Print information about the visual feature (show single gestalts == on)
 *   F4 ... Print runtime statistics
 *   F5 ... Refresh display
 *   F9 ... Process single shot \n
 *   F10 .. Process pruned image (Format7 & ROI) \n
 *   F11 .. Process HR image (pruned & ROI)
 * 
 *   key: 1 ... Szene: Show all stereo features on virtual szene (on/off) \n
 *   key: 2 ... Stereo: Show all matched features (on/off) \n
 *   key: 3 ... Stereo: Show matched features (on/off) \n
 *   key: 4 ... Stereo: Show single stereo match (on/off) \n
 *   key: 5 ... Mono: Show all edge segments (on/off) \n
 *   key: 6 ... Mono: Show detected Gestalts (on/off) \n
 *   key: 7 ... Mono: Show the masked Gestalts (on/off) \n
 *   key: 8 ... Mono: Show single Gestalts \n
 *   key: 9 ... Mono: Show ROI windows \n
 *   key: + ... Increase degree of detail \n
 *   key: - ... Decrease degree of detail \n
 *   key: . ... Increase ID of single showed Gestalt \n
 *   key: , ... Decrease ID of single showed Gestalt \n
 *   key: x ... Stop single-shot processing mode.\n\n
 *
 *   key: q ... Show SEGMENTS \n
 *   key: w ... Show LINES \n
 *   key: e ... Show COLLINEARITIES \n
 *   key: r ... Show L-JUNCTIONS \n
 *   key: t ... Show CLOSURES \n
 *   key: z ... Show RECTANGLES \n
 *   key: u ... Show FLAPS \n
 *   key: i ... Show FLAPS_ARI \n
 *   key: o ... Show CUBES \n
 *
 *   key: a ... Show ARCS \n
 *   key: s ... Show A-JUNCTIONS \n
 *   key: d ... Show CONVEX ARC-GROUPS \n
 *   key: f ... Show ELLIPSES \n
 *   key: g ... Show E-JUNCTIONS \n
 *   key: h ... Show EXT-ELLIPSES: not yet implemented \n
 *   key: j ... Show CYLINDERS \n
 *   key: k ... Show CONES \n
 *   key: l ... Show SPHERES \n
 *   
 *   key: y ... Show REASONER results \n
 *   
 *   key: < ... Switch to older frames \n
 */
void StereoDetectorKinectLines::SingleShotMode()
{
  sleepComponent(10);
  int key = 0;
  key = cvWaitKey(100);      /// TODO Kurzes wait, damit eingelesen werden kann!

//   if (key != -1) printf("StereoDetectorKinectLines::SingleShotMode: Pressed key: %c, %i\n", (char) key, key);

//   if (key == 65470 || key == 1114046) // F1
//     printf("Keyboard commands for single shot mode:\n"
//             "    F1 ... Show this help \n"
//             "    F2 ... Print number of Gestalts\n"
//             "    F3 ... Print information (show single gestalts (5) = on)\n"
//             "    F4 ... Print runtime statistics\n"
//             "    F5 ... Refresh display\n\n"
//             
//             "    F9 ... Process single shot\n"
//             "    F10 .. Process single shot with region of interest (ROI)\n"
//             "    F11 .. Process single shot with HR and ROI\n"
// 
//             "    1 ... Szene: Show all stereo features on virtual szene\n"
//             "    2 ... Stereo: Show all matched features \n"
//             "    3 ... Stereo: Show matched features \n"
//             "    4 ... Stereo: Show single stereo match \n"
//             "    5 ... Mono: Show all edge segments \n"
//             "    6 ... Mono: Show detected Gestalts \n"
//             "    7 ... Mono: Show the masked Gestalts \n"
//             "    8 ... Mono: Show single Gestalts \n"
//             "    9 ... Mono: Show ROI windows\n"
//             "    + ... Mono: Increase degree of detail \n"
//             "    - ... Mono: Decrease degree of detail \n"
//             "    . ... Mono: Increase ID of single showed Gestalt \n"
//             "    , ... Mono: Decrease ID of single showed Gestalt \n"
//             "    x ... Mono: Stop single-shot processing mode.\n\n"
// 
//             "    q ... Show SEGMENTS\n"
//             "    w ... Show LINES\n"
//             "    e ... Show COLLINEARITIES\n"
//             "    r ... Show L-JUNCTIONS\n"
//             "    t ... Show CLOSURES\n"
//             "    z ... Show RECTANGLES\n"
//             "    u ... Show FLAPS\n"
//             "    i ... Show FLAPS_ARI\n"
//             "    o ... Show CUBES\n\n"
// 
//             "    a ... Show ARCS\n"
//             "    s ... Show A-JUNCTIONS\n"
//             "    d ... Show CONVEX ARC-GROUPS\n"
//             "    f ... Show ELLIPSES\n"
//             "    g ... Show E-JUNCTIONS\n"
//             "    h ... Show EXT-ELLIPSES: not yet implemented\n"
//             "    j ... Show CYLINDERS\n"
//             "    k ... Show CONES\n"
//             "    l ... Show SPHERES\n\n"
// 
//             "    y ... Show REASONER results\n\n"
// 
//             "    < ... Switch to older frames\n");

  if (key == 65471 || key == 1114047) // F2
  {
//     const char* text = score->GetGestaltListInfo();
//     printf("\n%s\n", text);
    const char* text = kcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65472 || key == 1114048) // F3
  {
//     if(showSingleGestalt && showID != -1 && (mouseSide == 0 || mouseSide == 1)) 
//     {
//       const char* text = (score->GetMonoCore(mouseSide))->GetInfo(showType, showID);
//       log("Gestalt infos:\n%s\n", text);
//     }
    const char* text = vcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65473 || key == 1114049) // F4
  {
//     score->PrintVCoreStatistics();
    kcore->PrintVCoreStatistics();
  }

//   if (key == 65474 || key == 1114050) // F5
//   {
//     if(!haveImage) GetImages();
//     if(showImages) ShowImages(true);
//   }
// 
// 
  if (key == 65478 || key == 1114054) // F9
  {
    log("process images in single shot mode.");
    lockComponent();
    processImage();
    unlockComponent();
  }
//   if (key == 65479 || key == 1114055) // F10
//   {
//     log("process pruned stereo images (single shot with ROI).");
//     lockComponent();
//     ProcessPrunedHRImages();
//     unlockComponent();
//   }
//   if (key == 65480 || key == 1114056) // F11
//   {
//     log("process HR stereo images (single shot with ROI).");
//     lockComponent();
//     ProcessHRImages();
//     unlockComponent();
//   }
// 
// //  if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
  switch((char) key)
  {
    

    case '1':
//       tgRenderer->Clear();
//       tgRenderer->AddConvexHulls(clustered_planes);
  //       if(showAllStereo) 
//       {
//         showAllStereo = false;
//         log("Szene: Show ALL stereo features at virtual scene: OFF");
//       }
//       else
//       { 
//         showAllStereo = true;
//         log("Szene: Show ALL stereo features at virtual scene: ON");
//       }
//       WriteVisualObjects();
      break;
      
    case '2':
//       tgRenderer->Clear();
//       tgRenderer->SetPointClouds(clustered_planes2);
//       if(showAllStereoMatched) 
//       {
//         showAllStereoMatched = false;
//         log("Stereo: Draw ALL MATCHED stereo features: OFF");
//       }
//       else
//       { 
//         showAllStereoMatched = true;
//         log("Stereo: Draw ALL MATCHED stereo features: ON");
//       }
//       if(showImages) ShowImages(true);
      break;
//     case '3':
//       if(showStereoMatched) 
//       {
//         showStereoMatched = false;
//         log("Stereo: Draw MATCHED stereo features: OFF");
//       }
//       else
//       { 
//         showStereoMatched = true;
//         log("Stereo: Draw MATCHED stereo features: ON");
//       }
//       if(showImages) ShowImages(true);
//       break;
//     case '4':
//       if(showSingleStereo)
//       {
//         showSingleStereo = false;
//         log("Stereo: Show single stereo match: OFF");
//       }
//       else
//       {
//         showSingleStereo = true;
//         log("Stereo: Show single stereo match: ON");
//       }
//       ShowImages(true);
//       break;
//     case '5':
//       if(showSegments)
//       {
//         showSegments = false;
//         log("Mono: Show edge segments: OFF");
//       }
//       else
//       {
//         showSegments = true;
//         log("Mono: Show edge segments: ON");
//       }
//       ShowImages(true);
//       break;
//     case '6':
//       if(showDetected)
//       {
//         showDetected = false;
//         log("Mono: Draw all detected features: OFF");
//       }
//       else
//       {
//         showDetected = true;
//         log("Mono: Draw all detected features: ON");
//       }
//       ShowImages(true);
//       break;
//     case '7':
//       if(showMasked)
//       {
//         showMasked = false;
//         log("Mono: Draw all MASKED features: OFF");
//       }
//       else
//       {
//         showMasked = true;
//         log("Mono: Draw all MASKED features: ON");
//       }
//       ShowImages(true);
//       break;
//     case '8':
//       if(showSingleGestalt)
//       {
//         showSingleGestalt = false;
//         log("Mono: Show single Gestalts: OFF");
//       }
//       else
//       {
//         showSingleGestalt = true;
//         log("Mono: Show single Gestalts: ON");
//       }
//       ShowImages(true);
//       break;
//     case '9':
//       if(showROIs)
//       {
//         showROIs = false;
//         log("Show ROIs: OFF");
//       }
//       else
//       {
//         showROIs = true;
//         log("Show ROIs: ON");
//       }
//       ShowImages(true);
//       break;
//     case '+':
//       if(detail < 15)
//       {
//         detail++;
//         log("Increased degree of display detail to: %u", detail);
//         ShowImages(true);
//       }
//       break;
//     case '-':
//       if(detail > 0)
//       {
//         detail--;
//         log("Decreased degree of display detail to: %u", detail);
//         ShowImages(true);
//       }
//       break;
//     case '.':
//       showID++;
//       ShowImages(true);
//       break;
//     case ',':
//       if(showID > 0)
//         showID--;
//       ShowImages(true);
//       break;
//     case 'x':
//       log("stop debug mode.");
//       single = false;
//       break;
// 
    case 'q':
      log("Show PATCHES");
//       showType = Z::Gestalt::SEGMENT;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
      tgRenderer->Clear();
//       tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH);
      break;    
    case 'w':
      log("Show SEGMENTS");
//       showType = Z::Gestalt::SEGMENT;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
      tgRenderer->Clear();
      tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::SEGMENT);
      break;
    case 'e':
      log("Show LINES");
//       showType = Z::Gestalt::LINE;
//       showStereoType = Z::StereoBase::STEREO_LINE;
//       ShowImages(true);
//       write_stereo_lines = !write_stereo_lines;
//       WriteVisualObjects();
      tgRenderer->Clear();
      tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE);
      break;
    case 'r':
      log("Show COLLINEARITIES");
//       showType = Z::Gestalt::COLLINEARITY;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
      tgRenderer->Clear();
      tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY);
      break;
//     case 't':
//       log("Show CLOSURES");
//       showType = Z::Gestalt::CLOSURE;
//       showStereoType = Z::StereoBase::STEREO_CLOSURE;
//       ShowImages(true);
//       write_stereo_closures = !write_stereo_closures;
//       WriteVisualObjects();
//       break;
    case 'z':
      log("Show CLOSURES");
//       showType = Z::Gestalt::RECTANGLE;
//       showStereoType = Z::StereoBase::STEREO_RECTANGLE;
//       ShowImages(true);
//       write_stereo_rectangles = !write_stereo_rectangles;
//       WriteVisualObjects();
      tgRenderer->Clear();
      tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::CLOSURE);
      break;
    case 'u':
      log("Show RECTANGLES");
//       showType = Z::Gestalt::FLAP;
//       showStereoType = Z::StereoBase::STEREO_FLAP;
//       ShowImages(true);
//       WriteVisualObjects();
      tgRenderer->Clear();
      tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::RECTANGLE);
      break;
//     case 'i':
//       log("Show FLAPS_ARI");
//       showType = Z::Gestalt::FLAP_ARI;
//       showStereoType = Z::StereoBase::STEREO_FLAP_ARI;
//       ShowImages(true);
//       write_stereo_flaps = !write_stereo_flaps;
//       WriteVisualObjects();
//       break;
//     case 'o':
//       log("Show CUBES");
//       showType = Z::Gestalt::CUBE;
//       showStereoType = Z::StereoBase::STEREO_CUBE;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'p':
//       log("Show CORNERS");
//       showType = Z::Gestalt::CORNER;
//       showStereoType = Z::StereoBase::STEREO_CORNER;
//       ShowImages(true);
//       write_stereo_corners = !write_stereo_corners;
//       WriteVisualObjects();
//       break;
// 
//     case 'a':
//       log("Show ARCS");
//       showType = Z::Gestalt::ARC;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 's':
//       log("Show A-JUNCTIONS");
//       showType = Z::Gestalt::A_JUNCTION;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'd':
//       log("Show CONVEX ARC-GROUPS");
//       showType = Z::Gestalt::CONVEX_ARC_GROUP;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'f':
//       log("Show ELLIPSES");
//       showType = Z::Gestalt::ELLIPSE;
//       showStereoType = Z::StereoBase::STEREO_ELLIPSE;
//       ShowImages(true);
//       write_stereo_ellipses = !write_stereo_ellipses;
//       WriteVisualObjects();
//       break;
//       
//     case 'g':
//       log("Show E-JUNCTIONS");
//       showType = Z::Gestalt::E_JUNCTION;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'h':
//       log("Show EXT-ELLIPSES: not yet implemented");
// //      showType = Z::Gestalt::EXTELLIPSE;
// //      showStereoType = Z::StereoBase::UNDEF;
// //      overlays = 204;
// //      ShowImages(true);
// //      WriteVisualObjects();
//       break;
//     case 'j':
//       log("Show CYLINDERS");
//       showType = Z::Gestalt::CYLINDER;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'k':
//       log("Show CONES");
//       showType = Z::Gestalt::CONE;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
//     case 'l':
//       log("Show CIRCLES");
//       showType = Z::Gestalt::CIRCLE;
//       showStereoType = Z::StereoBase::UNDEF;
//       ShowImages(true);
//       WriteVisualObjects();
//       break;
// 
//     case 'y':
//       if(showReasoner)
//       {
//         showReasoner = false;
//         log("Show REASONER results: OFF");
//       }
//       else
//       {
//         showReasoner = true;
//         log("Show REASONER results:: ON");
//       }
//       ShowImages(true);
//       break;
// 
//     case 'c':
//       if(showReasonerUnprojected)
//       {
//         showReasonerUnprojected = false;
//         log("Show unprojected REASONER results: OFF");
//       }
//       else
//       {
//         showReasonerUnprojected = true;
//         log("Show unprojected REASONER results:: ON");
//       }
//       ShowImages(true);
//       break;
// 
//     case '<':
//       showFrame--;
//       if(showFrame < 0) showFrame = 2;
//       log("Switch to older frames: %u", showFrame);
//       score = p_score[showFrame];
//       ShowImages(true);
//       break;
//   default: break;
  }
}

}






