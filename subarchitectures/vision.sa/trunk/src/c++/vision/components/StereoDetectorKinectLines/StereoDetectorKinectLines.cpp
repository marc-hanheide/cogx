/**
 * @file StereoDetectorKinectLines.cpp
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
// static double MAXIMUM_DIST = 0.05;

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
  runtime = 10;                          // processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;                      // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();            // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CORNERS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CLOSURES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_RECTANGLES);

  kcore = new Z::KinectCore(vcore);       // New kinect core
//  learner = new Z::Learner();             // Learner for features

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
    if(!stereo_cam->ReadSVSCalib(stereoconfig))                                                                         /// TODO Kein SVS calib mehr!!!
      throw (std::runtime_error(exceptionMessage(__HERE__, "Cannot open calibration file for stereo camera.")));
    score = new Z::StereoCore(stereoconfig);
  }
  
  // initialize tgRenderer
  cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
  cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
  cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
  cv::Vec3d rotCenter(0,0,0.4);
  
  // Initialize 3D render engine 
  tgRenderer = new TomGine::tgTomGineThread(1280, 1024);
  tgRenderer->SetCamera(intrinsic);
  tgRenderer->SetCamera(R, t);
//   tgRenderer->SetRotationCenter(rotCenter);
  tgRenderer->SetCoordinateFrame();
  
//   planePopout = new pclA::PlanePopout();
//   relations = new Z::CalculateRelations();
//   svmPredictor = new Z::SVMPredictor(2);      // 2 ... Number of SVM's
//   graphCutter = new Z::GraphCut(kcore, relations);
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
    cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Kinect image",  10, 10);
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
  
  if(showImages)
  {
    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
}

 
  
/// TODO Project a 3D point on the 2D image plane
// void StereoDetectorKinectLines::ProjectPoint(double X, double Y, double Z, double &u, double &v, int imgWidth)
// {
//   double scale = 1;
//   assert(Z != 0.);
//   if(imgWidth !=0) scale = camPars[2].width / imgWidth;
//   u = camPars[2].fx*X/scale + camPars[2].cx*Z/scale;
//   v = camPars[2].fy*Y/scale + camPars[2].cy*Z/scale;
//   u /= Z;
//   v /= Z;
// }
  
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void StereoDetectorKinectLines::GetImageData()
{
  pointCloudWidth = 320;                                /// TODO get from cast-file!
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
 
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);            // call get points only once, if noCont option is on!!! (false means no transformation!!!)
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth);
  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, *pcl_cloud);

  // get rectified images from point cloud server
  getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
  getRectImage(1, kinectImageWidth, image_r);            // 1 = right image / we take it with kinect image width
  getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
  iplImage_l = convertImageToIpl(image_l);
  iplImage_r = convertImageToIpl(image_r);
  iplImage_k = convertImageToIpl(image_k);

  // convert points to cloud for render engine
//   cv::Mat_<cv::Point3f> cloud;
//   cv::Mat_<cv::Point3f> colCloud;
//   Points2Cloud(points_fromLeft, cloud, colCloud);        // show untransformed kinect point cloud
//   tgRenderer->SetPointCloud(cloud, colCloud);            // show transformed kinect point cloud
//   Points2Cloud(points, kinect_point_cloud, kinect_color_point_cloud);
//   tgRenderer->SetPointCloud(kinect_point_cloud, kinect_color_point_cloud);

//   // Reproject point cloud to left stereo camera to get the depth image from the view of the left stereo camera
//   cv::Mat_<cv::Point3f> depthImage;
//   cv::Mat_<cv::Point3f> depthMap;
//   Points2DepthMap(stereo_cam, kinect_point_cloud, kinect_color_point_cloud, depthImage, depthMap);
//   IplImage iplImage_depth = depthImage;
//   IplImage iplImage_depthMap = depthMap;

  if(showImages)
  {
//     cvShowImage("Stereo left", iplImage_l);
//     cvShowImage("Stereo right", iplImage_r);
    cvShowImage("Kinect image", iplImage_k);

    // convert point cloud to iplImage
    cv::Mat_<cv::Vec3b> kinect_pc_image;
    pclA::ConvertPCLCloud2Image(*pcl_cloud, kinect_pc_image);     // TODO Funktioniert nicht!!!
    cv::imshow("Kinect Image from point cloud", kinect_pc_image);
  }
}



/**
 *  @brief Process data from stereo or Kinect.
 */
void StereoDetectorKinectLines::processImage()
{
printf("\nStereoDetectorKinectLines::processImage\n");

struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  log("Process new images with runtime: %ums", runtime);
  kcore->ClearResults();
  score->ClearResults();
  vcore->ClearGestalts();
  tgRenderer->Clear();
  sois.clear();

  /// Get kinect data
  GetImageData();

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for StereoDetectorKinectLines::GetImageData: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run vision core
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for StereoDetectorKinectLines: VisionCore: %4.3f\n", timespec_diff(&current, &last));
last = current;
  
  /// Run stereo core
  // Stereo core calculations (TODO needs perfekt calibration => solve problem)
//   score->ProcessStereoImage(runtime, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);

  /// Run kinect core
printf("Run Kinect Core: start\n");
  kcore->Process(vcore, iplImage_k, kinect_point_cloud);
printf("Run Kinect Core: end\n");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for StereoDetectorKinectLines: KinectCore: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run plane popout TODO Move planePopout completely to learner => is not neccessary to have it here!
//   planePopout->CalculateSOIs(pcl_cloud);
//   planePopout->GetSOIs(sois);
//   if(!planePopout->CalculateROIMask()) 
//     printf("StereoDetectorKinectLines: Error while processing ROI mask in PlanePopout!\n");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for StereoDetectorKinectLines: PlanePopout: %4.3f\n", timespec_diff(&current, &last));
last = current;


  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

  /// Label all 3D Gestalts, according to plane-popout sois
  // TODO TODO Where do implement this Object-Labeling?
//   unsigned nrPatches = kcore->NumGestalts3D(Z::Gestalt3D::PATCH);
//   for(unsigned i=0; i<nrPatches; i++)
//   {
//     Z::Patch3D *p = (Z::Patch3D*) kcore->Gestalts3D(Z::Gestalt3D::PATCH, i);
//     p->SetObjectLabel(planePopout->IsInSOI(p->GetCenter3D()));
//   }
  
  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

  /// Learned relation probabilieties (load from file and print)
//   static bool first = true;
//   if(first){
//     learner->ReadResultsFromFile();
//     first = false;
//   }
//   // learner->Process(planePopout, kcore, tgRenderer);
//   
//   double mean, variance, st_devi;
//   /// Proximity between patches
//   learner->GetPosProximityBetweenPatches(mean, variance, st_devi);
//   printf("Pos proximity: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegProximityBetweenPatches(mean, variance, st_devi);
//   printf("Neg proximity: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   /// Color similarity between patches
//   learner->GetPosColorSimilarityBetweenPatches(mean, variance, st_devi);
//   printf("Pos color: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegColorSimilarityBetweenPatches(mean, variance, st_devi);
//   printf("Neg color: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   /// Coplanarity between patches
//   learner->GetPosCoplanarityNormalsBetweenPatches(mean, variance, st_devi);
//   printf("Pos normals: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegCoplanarityNormalsBetweenPatches(mean, variance, st_devi);
//   printf("Neg normals: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   // Coplanarity distance between patches
// //   learner->GetPosCoplanarityDistanceBetweenPatches(mean, variance, st_devi);
// //   printf("Pos distance: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// //   learner->GetNegCoplanarityDistanceBetweenPatches(mean, variance, st_devi);
// //   printf("Neg distance: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
// //   learner->GetClosenessBetweenPatchesProp(const double &val, double &prop);

  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

printf("StereoDetectorKinectLines: CalculateRelations start!\n");
  std::vector<Z::Relation> relation_vector;
  relations->Initialize(kcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  relations->CalcAllRelations(relation_vector);   // without ground-truth

printf("StereoDetectorKinectLines: CalculateRelations end!\n");

printf("StereoDetectorKinectLines: Prediction start!\n");
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    std::vector<double> probability;
    relation_vector[i].prediction = svmPredictor->GetResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    relations->AddPrediction(i, relation_vector[i].prediction);                           /// TODO We add from here the prediction value and the probability???
    relations->AddProbability(i, relation_vector[i].rel_probability);
  }
printf("StereoDetectorKinectLines: Prediction end!\n");
  relations->PrintResults();
  
  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 

  /// TODO Nächste Baustelle => Graph cutter
// printf("    graphCutter: Initialize: start\n");
  graphCutter->Initialize();
// printf("    graphCutter: Initialize: end\n");
  graphCutter->Cut();
// printf("    graphCutter: Cut end\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for StereoDetectorKinectLines: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
last = current;


  // show results from graph cut
//   graphCutter->Show(tgRenderer);

  /// Draw vision core image
  Z::SetActiveDrawArea(iplImage_k);
  vcore->DrawGestalts(Z::Gestalt::SEGMENT, 0);
  if(showImages) cvShowImage("Kinect", iplImage_k);
  
  tgRenderer->AddPointCloud(kinect_point_cloud);
  
  log("Processing of stereo images ended.");
  
  /// free memory
//   delete planePopout;

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for processing StereoDetectorKinectLines: %4.3f\n", timespec_diff(&current, &start));
}

/**
 * @brief Convert a IplImage to a cvMat<Vec3b> image.
 * @param iplImage IplImage source
 * @param matImage cv::Mat image destination
 */
void ConvertImage(IplImage &iplImage, cv::Mat_<cv::Vec3b> &image)
{
  image = cv::Mat_<cv::Vec3b>(iplImage.height, iplImage.width); 
  
  for (int v = 0; v < iplImage.height; ++v)
  {
    uchar *d = (uchar*) iplImage.imageData + v*iplImage.widthStep;
    for (int u = 0; u < iplImage.width; ++u, d+=3)
    {
      cv::Vec3b &ptCol = image(v,u);
      ptCol[0] = d[0];
      ptCol[1] = d[1];
      ptCol[2] = d[2];
    }
  }
}


/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   F1 ... Show this help
 *   F9 ... Process single shot \n
 * 
 *   key: 1 ...   \n
 *   key: 2 ...   \n
 *   key: 3 ...   \n
 *   key: 4 ...   \n
 *   key: x ... Stop single-shot processing mode.\n\n
 *
 *   key: q ... Show SEGMENTS \n
 *   key: w ... Show LINES \n
 *   key: e ... Show COLLINEARITIES \n
 *   key: r ... Show L-JUNCTIONS \n
 *   key: t ... Show CLOSURES \n
 *   key: z ... Show RECTANGLES \n
 *
 *   key: a ... Show ARCS \n
 *   
 */
void StereoDetectorKinectLines::SingleShotMode()
{
  sleepComponent(10);
  int key = 0;
  key = cvWaitKey(100);      /// TODO Kurzes wait, damit eingelesen werden kann!


  if (key == 65471 || key == 1114047) // F2
  {
    const char* text = kcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65472 || key == 1114048) // F3
  {
    const char* text = vcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65473 || key == 1114049) // F4
  {
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



//  if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
  switch((char) key)
  {
    case '1':
      log("Show POINT CLOUD");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      break;
      
    case '2':
      log("Show SOIs");
      for(unsigned i=0; i< sois.size(); i++)
      {
        std::vector<cv::Vec4f> soi_hulls;
        pclA::ConvertPCLCloud2CvVec(*sois[i], soi_hulls);  // convert pcl cloud to cvVec cloud
//         tgRenderer->AddHullPrism(soi_hulls);
        printf("StereoDetectorKinectLines::SingleShotMode: AddHullPrism is antiquated!\n");
      }
      break;
      
    case '3':
      log("Show GRAPH-CUT");
      tgRenderer->Clear();
//       graphCutter->Show(tgRenderer);
      printf("StereoDetectorKinectLines::SingleShotMode: graphCutter->Show is antiquated: change to kcore-> ...\n");
      break;

    case 'q':
      log("Show PATCHES");
      for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::PATCH); i++)
      {
        cv::Mat_<cv::Vec3b> kinect_image;
        ConvertImage(*iplImage_k, kinect_image);                                              /// TODO Convert image after drawing into it!
        patches = kinect_image;
        kcore->DrawGestalts3DToImage(patches, Z::Gestalt3D::PATCH, camPars[2]);
        cv::imshow("Patches", patches);
      }
      tgRenderer->Clear();
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH);
      break;    
    case 'w':
      log("Show SEGMENTS");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::SEGMENT);
      break;
    case 'e':
      log("Show LINES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE);
      break;
    case 'r':
      log("Show COLLINEARITIES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY);
      break;
    case 'z':
      log("Show CLOSURES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::CLOSURE);
      break;
    case 'u':
      log("Show RECTANGLES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::RECTANGLE);
      break;
      
    case 'x':
      log("End Single-Shot mode!");
      single = false;
      break;
  }
}

}






