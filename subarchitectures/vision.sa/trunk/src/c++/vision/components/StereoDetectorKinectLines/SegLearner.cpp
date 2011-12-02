/**
 * @file SegLearner.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "SegLearner.h"

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
    return new cast::SegLearner();
  }
}

namespace cast
{

/** Debug flag **/
static bool deb = true;
  
/**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void SegLearner::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves (for point clouds)
  configureServerCommunication(_config);

  // for one vision core: only lines
  runtime = 10;                                    // processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;                                // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();                      // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CORNERS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CLOSURES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_RECTANGLES);

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
    log("Warning: Antiquated: Use stereoconfig_xml with openCV stereo calibration");
  }
  
  if(showImages)
  {
    // initialize tgRenderer
    cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
    cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Vec3d rotCenter(0,0,0.4);
    
    // Initialize 3D render engine 
    tgRenderer = new TomGine::tgTomGineThread(1280, 1024);
    tgRenderer->SetCamera(intrinsic);
    tgRenderer->SetCamera(R, t);
    tgRenderer->SetRotationCenter(rotCenter);      /// TODO funktioniert nicht => Wieso?
    tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);

  kcore = new Z::KinectCore(vcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  // TODO Enable/Disable KinectPrinciples?
  
  //   learner = new Z::Learner();                       // Learner for features
  svmFileCreator = new Z::SVMFileCreator();         // Creator training files for SVM

  double _minZ = 0.0;
  double _maxZ = 2.0;
  double dwLeaf = 0.02;
  double dwLeafObj = 0.01;
  int nb = 10;
  double thrSac = 0.01;
  double normalDistWeight = 0.1;
  double minObjH = thrSac; // 0.005,           /// HACK Disabled: Set to thrSac
  double maxObjH = 1.0;
  float eucThr = 0.02;
  unsigned minClSize = 100;
  pclA::PlanePopout::Parameter param(_minZ, _maxZ, dwLeaf, dwLeafObj, nb, thrSac, 
                                     normalDistWeight, minObjH, maxObjH, eucThr, minClSize);
  planePopout = new pclA::PlanePopout(param);


  /// ################### The new classes for calculation ################### ///
  
  /// init model fitter
  double minZ = 0.3;
  double maxZ = 1.5;
  pclA::ModelFitter::Parameter mf_param(false, 0.005, true, minZ, maxZ);
  model_fitter = new pclA::ModelFitter(mf_param);

  /// init annotation
  annotation = new pa::Annotation();
  annotation->init("/media/Daten/Object-Database/annotation/box_world%1d.png", 0, 16);

  /// init patch class
  patches = new pclA::Patches();
  patches->setZLimit(0.01);
  
  /// init svm-file-creator
  svm = new svm::SVMFileCreator();
  
}


/**
 * @brief Called by the framework after configuration, before run loop.
 */
void SegLearner::start()
{
  // start point cloud communication
  startPCCServerCommunication(*this);

  if(showImages) 
  {
    cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Kinect image",  10, 500);
  }
}

/**
 * @brief Called by the framework to start component run loop.
 * @TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void SegLearner::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
    processImage();
  }

  if(showImages)
  {
    cvReleaseImage(&iplImage_l);
    cvReleaseImage(&iplImage_r);
    cvReleaseImage(&iplImage_k);

    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
}

  
 
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void SegLearner::GetImageData()
{
  pointCloudWidth = 320;
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
 
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);            // call get points only once, if noCont option is on!!! (false means no transformation!!!)
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true);
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);

  // get rectified images from point cloud server
//  getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
//  getRectImage(1, kinectImageWidth, image_r);            // 1 = right image / we take it with kinect image width
  getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
//  iplImage_l = convertImageToIpl(image_l);
//  iplImage_r = convertImageToIpl(image_r);
  iplImage_k = convertImageToIpl(image_k);

  if(showImages)
  {
    cvShowImage("Kinect image", iplImage_k);

    // convert point cloud to iplImage
    cv::Mat_<cv::Vec3b> kinect_pc_image;
    pclA::ConvertPCLCloud2Image(pcl_cloud, kinect_pc_image);     // TODO Funktioniert nicht!!!
    cv::imshow("Kinect Image from point cloud", kinect_pc_image);
  }
}



/**
 *  @brief Process data from stereo or Kinect.
 */
void SegLearner::processImageNew()
{
  bool debug = true;
  if(debug) printf("SegLearner::processImageNew: start\n");
  
static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  /// Get kinect data
  GetImageData();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Getting images: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  if(debug) printf("############################### end => This takes longer!\n");

  /// ModelFitter
  int nr_models = 0;
  std::vector<int> pcl_model_types;
//   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  std::vector< std::vector<double> > distances;
  std::vector<double> square_errors;
  model_fitter->addModelType(pcl::SACMODEL_PLANE);
//  model_fitter->addModelType(pcl::SACMODEL_NORMAL_PLANE);
//  model_fitter->addModelType(pcl::SACMODEL_CYLINDER);
//  model_fitter->addModelType(pcl::SACMODEL_SPHERE);
  model_fitter->setNormals(pcl_normals);
  model_fitter->useDominantPlane(true);
  model_fitter->setInputCloud(pcl_cloud);
  model_fitter->compute();
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_cloud_indices);
  model_fitter->getError(distances, square_errors);
  nr_models = model_coefficients.size();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Model fitting: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  if(debug)  printf("    Annotation: start\n");
  std::vector<int> anno;
  std::vector< std::vector<int> > anno_pairs;
  annotation->load(pointCloudWidth, anno);            /// TODO Das ist überflüssig - Könnte intern aufgerufen werden
  annotation->setIndices(pcl_model_cloud_indices);
  annotation->calculate();
  annotation->getResults(anno_pairs);
  if(debug) printf("    Annotation: end\n");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Annotation calculation: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  /// Calculate patch relations
  if(debug) printf("SegLearner: Calculate patches start!\n");
//   std::vector< std::vector<unsigned> > neighbors;
  std::vector<Relation> relation_vector;
  patches->setInputCloud(pcl_cloud, pcl_normals);
//   patches->setNeighbors(neighbors);
  patches->setPatches(pcl_model_types, model_coefficients, pcl_model_cloud_indices);
  patches->computeNeighbors();
  patches->computeTestRelations();
//   patches->getNeighbors(neighbors);
  patches->getRelations(relation_vector);
  if(debug) printf("SegLearner: Calculate patches end!\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
last = current;  


  /// write svm-relations to file!
  svm->init(relation_vector);
  svm->process();

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
}

/**
 *  @brief Process data from stereo or Kinect.
 */
void SegLearner::processImage()
{  
// printf("\nLearnPrincipleDistributions::processImage: start\n");
// static struct timespec start, last, current;
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
// printf("Runtime for StereoDetectorKinectLines::Since last processImage call: %4.3f\n", timespec_diff(&start, &last));
// last = start;

  log("Process new images with runtime: %ums", runtime);
  kcore->ClearResults();
  score->ClearResults();
  vcore->ClearGestalts();
  sois.clear();

// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("Runtime for StereoDetectorKinectLines::Clear cores: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  /// Get kinect data
  GetImageData();

// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("Runtime for StereoDetectorKinectLines::GetImageData: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  /// Run vision core
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  

  /// Run stereo core
  // Stereo core calculations (TODO needs perfekt calibration => solve problem)
//   score->ProcessStereoImage(runtime, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);

  /// Run kinect core
  if(deb) printf("Run Kinect Core: start\n");
  kcore->Process(iplImage_k, kinect_point_cloud, pcl_cloud);
  if(deb) printf("Run Kinect Core: end\n");
  
  /// Run plane popout TODO Move planePopout completely to learner => is not neccessary to have it here!
//   if(deb) printf("Run plane-popout: start\n");
//   planePopout->CalculateSOIs(pcl_cloud);
//   if(deb) printf("Run plane-popout: start 1\n");
//   planePopout->GetSOIs(sois, soi_labels);
//   if(deb) printf("Run plane-popout: start 2\n");
//   if(!planePopout->CalculateROIMask())    // TODO we need the ROI mask? only for displaying the results, right?
//     printf("StereoDetectorKinectLines: Error while processing ROI mask in PlanePopout!\n");
//   if(deb) printf("Run plane-popout: start 3\n");
//   kcore->SetObjectLabels(planePopout);    // Set object labels for kinect-Gestalts
//   if(deb) printf("Run plane-popout: end\n");

  /// use annotation instead of plane-popout.
  std::vector<int> anno;
  annotation->load(pointCloudWidth, anno);
  kcore->SetAnnotation(anno);

  
  /// Calculate relations between features and write it to file for SVM learning!
  svmFileCreator->Process(kcore);

  /// Draw results on render engine
  tgRenderer->Clear();
//   tgRenderer->Update();
  kcore->DrawObjects3D(tgRenderer);
  tgRenderer->Update();

  /// TODO Verwenden wir den Learner noch mal???
  /// Run learner
//   learner->Process(planePopout, kcore, tgRenderer);
// 
//   /// Write results of learner to the console
//   double mean, variance, st_devi;
//   learner->GetPosProximityBetweenPatches(mean, variance, st_devi);
//   printf("Pos proximity: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegProximityBetweenPatches(mean, variance, st_devi);
//   printf("Neg proximity: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   // Color similarity between patches
//   learner->GetPosColorSimilarityBetweenPatches(mean, variance, st_devi);
//   printf("Pos color: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegColorSimilarityBetweenPatches(mean, variance, st_devi);
//   printf("Neg color: mean / variance / st-devi: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   // Coplanarity between patches
//   learner->GetPosCoplanarityNormalsBetweenPatches(mean, variance, st_devi);
//   printf("Pos normals: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegCoplanarityNormalsBetweenPatches(mean, variance, st_devi);
//   printf("Neg normals: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   learner->GetPosCoplanarityDistanceBetweenPatches(mean, variance, st_devi);
//   printf("Pos distance: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
//   learner->GetNegCoplanarityDistanceBetweenPatches(mean, variance, st_devi);
//   printf("Neg distance: %4.3f / %4.3f / %4.3f\n", mean, variance, st_devi);
// 
//   // write results of learner to a file
//   learner->WriteResults2File();
  
  
  /// Draw VisionCore image
//   if(showImages)
//   {
//     Z::SetActiveDrawArea(iplImage_k);
//     vcore->DrawGestalts(Z::Gestalt::LINE, 1);
//     cvShowImage("Kinect", iplImage_k);

//     tgRenderer->Clear();
//   tgRenderer->Update();
//     tgRenderer->AddPointCloud(kinect_point_cloud);
//   tgRenderer->Update();
//   }
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
 *   F9 ... Process single shot \n
 *   
 *   1 ...
 *   2 ...
 *   3 ...
 *   
 *   q ... 
 *   w ...
 *   e ...
 *   r ...
 *   
 *   
 *   
 */
void SegLearner::SingleShotMode()
{
  sleepComponent(10);
  int key = 0;
  key = cvWaitKey(20);
  
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
//     score->PrintVCoreStatistics();
    kcore->PrintVCoreStatistics();
  }

  if (key == 65478 || key == 1114054) // F9
  {
//     log("process images in single shot mode.");
    printf("\nSegLearner::ProcessNew: Learn from next image!\n");
    lockComponent();
    processImageNew();
    unlockComponent();
  }

  if (key == 65479 || key == 1114055) // F10
  {
//     log("process images in single shot mode.");
    printf("\nSegLearner::ProcessNew: Learn from next image!\n");
    lockComponent();
    processImage();
    unlockComponent();
  }

//  if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
  switch((char) key)
  {
    case '1':
      log("Show POINT CLOUD");
      if (showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        tgRenderer->Update();
      }
      break;
      
    case '2':
      log("Show SOIs");
      for(unsigned i=0; i< sois.size(); i++)
      {
        std::vector<cv::Vec4f> soi_hull;
        pclA::ConvertPCLCloud2CvVec(sois[i], soi_hull);  // convert pcl clouds to cvVec clouds
        if(showImages)
        {
          int top = soi_hull.size();
          int bottom = top/2; 
          for(int j=0; j < bottom; j++)
          {
            int i = j+1; 
            if(i >= bottom) i=0;
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
          }
          for(int j=bottom; j < top; j++)
          {
            int i = j+1; 
            if(i >= top) i=bottom;
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
          }
          for(int j=0; j < bottom; j++)
          {
            int i = j + bottom; 
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
          }
          if(true)  // draw id of soi
          {
            char label[5];
            snprintf(label, 5, "%u", soi_labels[i]);
            tgRenderer->AddLabel3D(label, 24, 
                                   (soi_hull[0][0] + soi_hull[int(soi_hull.size()/4)][0])/2,
                                   (soi_hull[0][1] + soi_hull[int(soi_hull.size()/4)][1])/2, 
                                   (soi_hull[0][2] + soi_hull[int(soi_hull.size()/4)][2])/2);
          }
          tgRenderer->Update(); 
        }
      }
      break;
      
    case '3':
      log("Show Objects");
      tgRenderer->Clear();
      tgRenderer->Update();
      kcore->DrawObjects3D(tgRenderer);
      tgRenderer->Update();
      break;
      
    case  '7':
    {
      log("Show PlanePopout");
//       tgRenderer->Clear();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      planePopout->GetPopoutsCloud(pp_cloud);
      cv::Mat_<cv::Vec4f> cv_pp_cloud;
      pclA::ConvertPCLCloud2CvMat(pp_cloud, cv_pp_cloud, true);
      tgRenderer->AddPointCloud(cv_pp_cloud);
      tgRenderer->Update();
      break;
    }
          
    case  '8':
    {
      log("Show PlanePopout");
      tgRenderer->Clear();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      planePopout->GetCloudDownsampled(pp_cloud);
      cv::Mat_<cv::Vec4f> cv_pp_cloud;
      pclA::ConvertPCLCloud2CvMat(pp_cloud, cv_pp_cloud, true);
      tgRenderer->AddPointCloud(cv_pp_cloud);
      tgRenderer->Update();
      break;
    }
      
    case '9':
    {
      log("Show PlanePopout");
      tgRenderer->Clear();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
      planePopout->GetTableProjected(table);
      planePopout->GetTableHull(table_hull);
      cv::Mat_<cv::Vec4f> cv_table;
      pclA::ConvertPCLCloud2CvMat(table, cv_table, false);
      tgRenderer->AddPointCloud(cv_table);
      int hull_size = table_hull->points.size();
      for(unsigned i=0; i<hull_size-1; i++)
      {
        tgRenderer->AddLine3D(table_hull->points[i].x, table_hull->points[i].y, table_hull->points[i].z, 
                               table_hull->points[i+1].x, table_hull->points[i+1].y, table_hull->points[i+1].z, 
                               0, 255, 0, 3);
      }
      tgRenderer->AddLine3D(table_hull->points[0].x, table_hull->points[0].y, table_hull->points[0].z, 
                             table_hull->points[hull_size-1].x, table_hull->points[hull_size-1].y, 
                             table_hull->points[hull_size-1].z, 0, 255, 0, 3);
      tgRenderer->Update();
      break;
    }

    case 'q':
      log("Show PATCHES");
      for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::PATCH); i++)
      {
        cv::Mat_<cv::Vec3b> kinect_image;
        ConvertImage(*iplImage_k, kinect_image);                                              /// TODO Convert image after drawing into it!
        patch_image = kinect_image;
        kcore->DrawGestalts3DToImage(patch_image, Z::Gestalt3D::PATCH, camPars[2]);
        cv::imshow("3D-Gestalt", patch_image);
      }
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH);
        tgRenderer->Update();
      }
      break;    
    case 'w':
      log("Show SEGMENTS");
      for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::SEGMENT); i++)
      {
        cv::Mat_<cv::Vec3b> kinect_image;
        ConvertImage(*iplImage_k, kinect_image);                                              /// TODO Convert image after drawing into it!
        kcore->DrawGestalts3DToImage(kinect_image, Z::Gestalt3D::SEGMENT, camPars[2]);
        cv::imshow("3D-Gestalts", kinect_image);
      }
      {
        Z::SetActiveDrawArea(iplImage_k);                                                     /// TODO Show vs3 segments
        vcore->DrawGestalts(Z::Gestalt::SEGMENT, 1);
        cvShowImage("Kinect", iplImage_k);
      }
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::SEGMENT, false);
        tgRenderer->Update();
      }
      break;
    case 'e':
      log("Show LINES");
      for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::LINE); i++)
      {
        cv::Mat_<cv::Vec3b> kinect_image;
        ConvertImage(*iplImage_k, kinect_image);                                              /// TODO Convert image after drawing into it!
        kcore->DrawGestalts3DToImage(kinect_image, Z::Gestalt3D::LINE, camPars[2]);
        cv::imshow("3D-Gestalt", kinect_image);
      }  
      {
        Z::SetActiveDrawArea(iplImage_k);                                                     /// TODO Show vs3 lines
        vcore->DrawGestalts(Z::Gestalt::LINE, 1);
        cvShowImage("Kinect", iplImage_k);
      }
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE);
        tgRenderer->Update();
      }
      break;
    case 'r':
      log("Show COLLINEARITIES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY);
      break;
//     case 't':
//       break;
    case 'z':
      log("Show CLOSURES");
      tgRenderer->Clear();
      tgRenderer->Update();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::CLOSURE);
      tgRenderer->Update();
      break;
    case 'u':
      log("Show RECTANGLES");
      tgRenderer->Clear();
      tgRenderer->Update();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::RECTANGLE);
      tgRenderer->Update();
      break;
      
    case 'x':
      log("End Single-Shot mode!");
      single = false;
      break;
      
    case '^':
      log("Add labels!");
      static bool drawNodeID = false;
      drawNodeID = !drawNodeID;
      kcore->DrawNodeID(drawNodeID);
      break;
      
      
    /// *** For new processing!!! *** ///
    case '+':
      log("Show PATCHES");
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f center3D[pcl_model_cloud_indices.size()];
        RGBValue col[pcl_model_cloud_indices.size()];
        for(unsigned i=0; i<pcl_model_cloud_indices.size(); i++) {
          col[i].float_value = GetRandomColor();
          for(unsigned j=0; j<pcl_model_cloud_indices[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_cloud_indices[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_cloud_indices[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_cloud_indices[i]->indices[j]].z;
            pt[3] = col[i].float_value;
            center3D[i][0] = center3D[i][0] + pt[0];
            center3D[i][1] = center3D[i][1] + pt[1];
            center3D[i][2] = center3D[i][2] + pt[2];
            col_points.push_back(pt);
          }
        }
    
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);

        for(int i=0; i<pcl_model_cloud_indices.size(); i++) {
          char label[5];
          snprintf(label, 5, "%u", i);
          tgRenderer->AddLabel3D(label, 14, 
                                 center3D[i][0]/pcl_model_cloud_indices[i]->indices.size(), 
                                 center3D[i][1]/pcl_model_cloud_indices[i]->indices.size(), 
                                 center3D[i][2]/pcl_model_cloud_indices[i]->indices.size());
        }
        tgRenderer->Update();
      }
      break;    

      
  }
}

}






