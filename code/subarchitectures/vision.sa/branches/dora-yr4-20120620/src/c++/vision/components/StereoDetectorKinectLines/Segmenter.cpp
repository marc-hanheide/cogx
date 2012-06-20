/**
 * @file Segmenter.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Segment a RGB-D image from the kinect.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "Segmenter.h"

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
    return new cast::Segmenter();
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
void Segmenter::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves (for point clouds)
  configureServerCommunication(_config);

  // for one vision core: only lines
  runtime = 10;                 // 10ms processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;            // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();                      // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CORNERS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CLOSURES);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_RECTANGLES);

//  learner = new Z::Learner();                       // Learner for features
//  svmFileCreator = new Z::SVMFileCreator();         // Creator training files for SVM

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
    tgRenderer->SetRotationCenter(rotCenter);
    tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);
  
  kcore = new Z::KinectCore(vcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  kcore->InitKinectPrinciplesForSegmenter();
  // TODO Enable/Disable KinectPrinciples?

//   planePopout = new pclA::PlanePopout();
  relations = new Z::CalculateRelations();
  relations->Initialize(kcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);

  std::vector<const char*> filenames;
  const char* filename = "./instantiations/11-05-11/1021/PP-Trainingsset2.txt.model";
  filenames.push_back(filename);
  filename = "./instantiations/11-05-11/PL-Trainingsset.txt.model";
  filenames.push_back(filename);
  filename = "./instantiations/11-05-11/LL-Trainingsset.txt.model";
  filenames.push_back(filename);
  svmPredictor = new Z::SVMPredictor(2, filenames);      // 2 ... Number of SVM's

  graphCutter = new Z::GraphCut(kcore, relations);
}


/**
 * @brief Called by the framework after configuration, before run loop.
 */
void Segmenter::start()
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
void Segmenter::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
    processImage();
  }

  if(showImages)
  {
    cvReleaseImage(&iplImage_k);

    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
}

  
 
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void Segmenter::GetImageData()
{
  pointCloudWidth = 320;
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
 
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);      // call get points only once, if noCont option is on!!! (false means no transformation!!!)
//   getPoints(false, pointCloudWidth, points);           /// TODO für KinectStereoSeqServer => einbauen von getCompletePoints!!!!
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true);
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);

  // get rectified images from point cloud server
  getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
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
void Segmenter::processImage()
{  
printf("\nSegmenter::processImage: start\n");

  log("Process new images with runtime: %ums", runtime);
  kcore->ClearResults();
printf("\nSegmenter::processImage: after clear\n");
//   score->ClearResults();
  vcore->ClearGestalts();
  sois.clear();

static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  /// Get kinect data
  GetImageData();

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter::GetImageData: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run vision core
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter::VisionCore: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run stereo core
  // Stereo core calculations (TODO needs perfekt calibration => solve problem)
//   score->ProcessStereoImage(runtime, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);

  /// Run kinect core
  kcore->Process(iplImage_k, kinect_point_cloud, pcl_cloud);

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter::KinectCore processing: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// Run plane popout
//   planePopout->CalculateSOIs(pcl_cloud);
//   planePopout->GetSOIs(sois);
//   if(!planePopout->CalculateROIMask()) 
//     printf("StereoDetectorKinectLines: Error while processing ROI mask in PlanePopout!\n");
//   kcore->SetObjectLabels(planePopout);    // Set object labels for kinect-Gestalts
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("Runtime for SegTester::PlanePopout: %4.3f\n", timespec_diff(&current, &last));
// last = current;  

  /// CalculateRelations
  std::vector<Z::Relation> relation_vector;
  relations->CalcAllRelations(relation_vector);   // without ground-truth

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter: CalculateRelations: %4.3f\n", timespec_diff(&current, &last));
last = current;


  /// SVM-Prediction
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    std::vector<double> probability;
    relation_vector[i].prediction = svmPredictor->GetResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    relations->AddPrediction(i, relation_vector[i].prediction);
    relations->AddProbability(i, relation_vector[i].rel_probability);
  }
//   relations->PrintResults();
  relations->ConstrainRelations();
//   relations->PrintRelations();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter: Prediction: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Graph cutter
  graphCutter->Initialize();
  graphCutter->Cut();
  graphCutter->CopyGroupIDToFeatures();

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
last = current;


clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for Segmenter: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
last = current;

  /// Draw VisionCore image
  if(showImages)
  {
    tgRenderer->Clear();
    tgRenderer->Update();
    kcore->DrawObjects3D(tgRenderer);
    tgRenderer->Update();
    
//     Z::SetActiveDrawArea(iplImage_k);
//     vcore->DrawGestalts(Z::Gestalt::LINE, 1);
//     cvShowImage("Kinect", iplImage_k);

//     tgRenderer->Clear();
//     tgRenderer->Update();
//     tgRenderer->AddPointCloud(kinect_point_cloud);
//     tgRenderer->Update();
  }
}

/**
 * @brief Convert a IplImage to a cvMat<Vec3b> image.
 * @param iplImage IplImage source
 * @param matImage cv::Mat image destination
 */
// void ConvertImage(IplImage &iplImage, cv::Mat_<cv::Vec3b> &image)
// {
//   image = cv::Mat_<cv::Vec3b>(iplImage.height, iplImage.width); 
//   
//   for (int v = 0; v < iplImage.height; ++v)
//   {
//     uchar *d = (uchar*) iplImage.imageData + v*iplImage.widthStep;
//     for (int u = 0; u < iplImage.width; ++u, d+=3)
//     {
//       cv::Vec3b &ptCol = image(v,u);
//       ptCol[0] = d[0];
//       ptCol[1] = d[1];
//       ptCol[2] = d[2];
//     }
//   }
// }


/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   F9 ... Process single shot \n
 *   
 *   1 ... Show point cloud
 *   2 ... Show SOIs
 *   3 ... Show object ground truth
 *   4 ... Show graph-cut result
 *   
 *   q ... 
 *   w ...
 *   e ...
 *   r ...
 *   
 *   
 *   
 */
void Segmenter::SingleShotMode()
{
//   sleepComponent(10);
  int key = 0;
  key = cvWaitKey(100);
  
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
    log("Segmenter::Process: Single shot mode: try to segment image.");
    printf("\nSegmenter::Process: Try to segment image!\n");
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
        pclA::ConvertPCLCloud2CvVec(sois[i], soi_hull);  // convert pcl cloud to cvVec cloud
        if(showImages)
        {
          int top = soi_hull.size();
          int bottom = top/2; 
          for(unsigned j=0; j < bottom; j++)
          {
            int i = j+1; 
            if(i >= bottom) i=0;
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
          }
          for(unsigned j=bottom; j < top; j++)
          {
            int i = j+1; 
            if(i >= top) i=bottom;
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
        //     color.float_value = s[3];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
          }
          for(unsigned j=0; j < bottom; j++)
          {
            int i = j + bottom; 
            cv::Vec4f s = soi_hull[j];
            cv::Vec4f e = soi_hull[i];
            tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 3);
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
      
    case '4':
      log("Show GRAPH-CUT");
      tgRenderer->Clear();
      tgRenderer->Update();
      kcore->DrawGraphCut3D(tgRenderer);
      tgRenderer->Update();
      break;
      
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
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH, false);
        tgRenderer->Update();
      }
      break;    
    case 'w':
      log("Show SEGMENTS");
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::SEGMENT, true);
        tgRenderer->Update();
      }
      break;
    case 'e':
      log("Show LINES");
//       for(unsigned i=0; i<kcore->NumGestalts3D(Z::Gestalt3D::LINE); i++)
//       {
//         cv::Mat_<cv::Vec3b> kinect_image;
//         ConvertImage(*iplImage_k, kinect_image);                                              /// TODO Convert image after drawing into it!
//         line_image = kinect_image;
//         kcore->DrawGestalts3DToImage(line_image, Z::Gestalt3D::LINE, camPars[2]);
//         cv::imshow("3D-Gestalt", line_image);
//       }
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE, true);
        tgRenderer->Update();
      }
      break;
    case 'r':
      log("Show COLLINEARITIES");
      tgRenderer->Clear();
      tgRenderer->Update();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY);
      tgRenderer->Update();
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
  }
}

}






