/**
 * @file SegTester.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "SegTester.h"

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
    return new cast::SegTester();
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
void SegTester::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves (for point clouds)
  configureServerCommunication(_config);

  // for one vision core: only lines
  runtime = 10;                                     // processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;                                // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();                      // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
//   vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_JUNCTIONS);
//   vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CORNERS);
//   vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_CLOSURES);
//   vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_RECTANGLES);

//  learner = new Z::Learner();                       // Learner for features

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
      if(file.find(":") == string::npos)
      {
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
  // TODO Enable/Disable KinectPrinciples?

  planePopout = new pclA::PlanePopout();
  relations = new Z::CalculateRelations();
  relations->Initialize(kcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  
  std::vector<const char*> filenames;
  const char* filename = "./instantiations/11-05-11/1026/PP-Trainingsset.txt.model";
  filenames.push_back(filename);
  filename = "./instantiations/11-05-11/PL-Trainingsset.txt.model";
  filenames.push_back(filename);
  filename = "./instantiations/11-05-11/LL-Trainingsset.txt.model";
  filenames.push_back(filename);
  svmPredictor = new Z::SVMPredictor(2, filenames);      // 2 ... Number of SVM's

  graphCutter = new Z::GraphCut(kcore, relations);

  /// ################### The new classes for calculation ################### ///
  
  /// init model fitter
  double minZ = 0.3;
  double maxZ = 1.5;
  pclA::ModelFitter::Parameter param(false, 0.005, true, minZ, maxZ);
  model_fitter = new pclA::ModelFitter(param);

  /// init annotation
  annotation = new pa::Annotation();
  annotation->init("/media/Daten/Object-Database/annotation/box_world%1d.png", 0, 16);

  /// init patch class
  patches = new pclA::Patches();
  patches->setZLimit(0.01);
  
  /// init svm-predictor
//   std::vector<const char*> filenames;
//   const char* filename = "./instantiations/11-05-11/1026/PP-Trainingsset.txt.model";
//   filenames.push_back(filename);
//   filename = "./instantiations/11-05-11/PL-Trainingsset.txt.model";
//   filenames.push_back(filename);
//   filename = "./instantiations/11-05-11/LL-Trainingsset.txt.model";
//   filenames.push_back(filename);
  svm = new svm::SVMPredictor(3, filenames);
  
  /// init graph cutter
  graphCut = new gc::GraphCut();
  
  if(showImages) 
  {
    cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Kinect image",  10, 500);
  }
}


/**
 * @brief Called by the framework after configuration, before run loop.
 */
void SegTester::start()
{
  // start point cloud communication
  startPCCServerCommunication(*this);
}

/**
 * @brief Called by the framework to start component run loop.
 * @TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void SegTester::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
    processImage();
  }

  if(showImages)
  {
//     cvReleaseImage(&iplImage_l);
//     cvReleaseImage(&iplImage_r);
    cvReleaseImage(&iplImage_k);

    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
}

  
 
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void SegTester::GetImageData()
{
  pointCloudWidth = 320;
  pointCloudHeight = pointCloudWidth *3/4;
  rgbWidth = 640;
  rgbHeight = rgbWidth *3/4;
  double rgb2depthScale = rgbWidth/pointCloudWidth;
  
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);      // Get the image grid point cloud with zeros, when no values are available!
//   getPoints(false, pointCloudWidth, points);           /// TODO für KinectStereoSeqServer => einbauen von getCompletePoints!!!!
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true); // replace 0-points by NAN's
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);

  // get rectified images from point cloud server
//   getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
//   getRectImage(1, kinectImageWidth, image_r);            // 1 = right image / we take it with kinect image width
  getRectImage(2, rgbWidth, image_k);                       // 2 = kinect image / we take it with kinect image width
//   iplImage_l = convertImageToIpl(image_l);
//  iplImage_r = convertImageToIpl(image_r);
  iplImage_k = convertImageToIpl(image_k);

  /// calculate normals
  pclA::NormalsFromSortedPCLCloud(pcl_cloud, pcl_normals, 0.02, 5.0);

  if(showImages)
  {
    cvShowImage("Kinect image", iplImage_k);
    // convert point cloud to iplImage
//     cv::Mat_<cv::Vec3b> kinect_pc_image;
//     pclA::ConvertPCLCloud2Image(pcl_cloud, kinect_pc_image);
//     cv::imshow("Kinect Image from point cloud", kinect_pc_image);
  }
  
  /// HACK HACK HACK HACK HACK HACK HACK HACK HACK HACK 
  /// Check if there are same 3D points in the point clouds
//   for(unsigned l=1; l<pointCloudWidth*pointCloudHeight-1; l++)
//   {
//     double dist = ((pcl_cloud->points[l].x-pcl_cloud->points[l+1].x) + 
//                    (pcl_cloud->points[l].y-pcl_cloud->points[l+1].y) + 
//                    (pcl_cloud->points[l].z-pcl_cloud->points[l+1].z));
//     if(dist == 0.) printf("SegTester: Warning: Same 3D point: idx: %u-%u!!!\n", l, l+1);
//   }
}


/**
 *  @brief Process data from stereo or Kinect.
 */
void SegTester::processImageNew()
{
  bool debug = true;
  if(debug) printf("SegTester::processImageNew: start\n");
  
static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  /// Get kinect data
  GetImageData();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Getting images: %4.3f\n", timespec_diff(&current, &last));
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
printf("Runtime for SegTester: Model fitting: %4.3f\n", timespec_diff(&current, &last));
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
printf("Runtime for SegTester: Annotation calculation: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  /// Calculate patch relations
  if(debug) printf("SegTester: Calculate patches start!\n");
//   std::vector< std::vector<unsigned> > neighbors;
  std::vector<Relation> relation_vector;
  patches->setInputCloud(pcl_cloud, pcl_normals);
//   patches->setNeighbors(neighbors);
  patches->setPatches(pcl_model_types, model_coefficients, pcl_model_cloud_indices);
  patches->computeNeighbors();
  patches->computeTestRelations();
//   patches->getNeighbors(neighbors);
  patches->getRelations(relation_vector);
  if(debug) printf("SegTester: Calculate patches end!\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
last = current;  


  /// SVM-Prediction
  if(debug) printf("SegTester: Prediction start: relation_vector.size: %u\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    relation_vector[i].prediction = svm->getResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    if(debug) 
    {
      if(relation_vector[i].groundTruth == 0)
        printf("relation [%u][%u]: gt: false => %4.3f\n", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
      if(relation_vector[i].groundTruth == 1)
        printf("relation [%u][%u]: gt: true => %4.3f\n", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
    }
  }
  if(debug) printf("SegTester: Prediction end!\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Prediction: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// TODO Graph cutter
  if(debug) printf("    graphCutter: start\n");
  graphCut->init(relation_vector);
  if(debug) printf("    graphCutter: initialized\n");
  graphCut->process();
  if(debug) printf("    graphCutter: processed\n");
  std::vector< std::vector<unsigned> > clusters;
  graphCut->getResults(nr_models, clusters);

  if(debug) 
  {
    for(unsigned i=0; i<clusters.size(); i++) { 
      printf("%u: ", i);
      for(unsigned j=0; j<clusters[i].size(); j++)
        printf("%u ", clusters[i][j]);
      printf("\n");
    }
  }
  if(debug) printf("    graphCutter: end: clusters.size(): %u\n", clusters.size());


clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
last = current;

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
}


/**
 *  @brief Process data from stereo or Kinect.
 */
void SegTester::processImage()
{  
printf("SegTester::processImage: start\n");

  log("Process new images with runtime: %ums", runtime);
  kcore->ClearResults();
  score->ClearResults();
  vcore->ClearGestalts();
  sois.clear();

static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  /// Get kinect data
  GetImageData();

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester::GetImageData: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run vision core
  vcore->NewImage(iplImage_k);
printf("Process VisionCore: start\n");
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
printf("Process VisionCore: end\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester::VisionCore: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Run stereo core
  // Stereo core calculations (TODO needs perfekt calibration => solve problem)
//   score->ProcessStereoImage(runtime, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);

  /// Run kinect core
  kcore->Process(iplImage_k, kinect_point_cloud, pcl_cloud);

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester::KinectCore processing: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// Run plane popout
printf("Run plane-popout: start\n");
  planePopout->CalculateSOIs(pcl_cloud);
  planePopout->GetSOIs(sois, soi_labels);
  if(!planePopout->CalculateROIMask()) 
    printf("StereoDetectorKinectLines: Error while processing ROI mask in PlanePopout!\n");
printf("Run plane-popout: end\n");

printf("SetObjectLabels: start\n");
  kcore->SetObjectLabels(planePopout);    // Set object labels for kinect-Gestalts
printf("SetObjectLabels: end\n");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester::PlanePopout: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// CalculateRelations
printf("SegTester: CalculateRelations start!\n");
  std::vector<Z::Relation> relation_vector;
  relations->Reset();
  relations->CalcTestRelations(relation_vector);   // with ground-truth relations
printf("SegTester: CalculateRelations end!\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: CalculateRelations: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// SVM-Prediction
printf("SegTester: Prediction start: relation_vector.size: %u\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    std::vector<double> probability;
    relation_vector[i].prediction = svmPredictor->GetResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    relations->AddPrediction(i, relation_vector[i].prediction);                           /// TODO We add from here the prediction value and the probability???
    relations->AddProbability(i, relation_vector[i].rel_probability);
  }
printf("SegTester: Prediction end!\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Prediction: %4.3f\n", timespec_diff(&current, &last));
last = current;

//   relations->PrintResults();
//   relations->ConstrainRelations();
  relations->PrintRelations();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Constrain results: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Graph cutter
printf("    graphCutter: start\n");
  graphCutter->Initialize();
printf("    graphCutter: initialized\n");
  graphCutter->Cut();
printf("    graphCutter: cutted\n");
  graphCutter->CopyGroupIDToFeatures();
printf("    graphCutter: end.\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
last = current;

// printf("    annotation: start\n");
//   std::vector<int> anno;
//   annotation->load(pointCloudWidth, anno);
//   kcore->CheckAnnotation(anno);
//   double m, s, b;
//   kcore->GetAnnotationResults(m, s, b);
//   printf("SegTester: overall annotation results: %4.3f - %4.3f - %4.3f\n", m*100, s*100, b*100);
// printf("    annotation: end\n");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
last = current;

  /// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
  
  
  /// Draw VisionCore image
  if(showImages)
  {
//     tgRenderer->Clear();
//     kcore->DrawObjects3D(tgRenderer);
//     tgRenderer->Update();

    tgRenderer->Clear();
    kcore->DrawGraphCut3D(tgRenderer);
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


void DrawNormals(Z::KinectCore *kc, TomGine::tgTomGineThread *tgR)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals;// (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;// (new pcl::PointCloud<pcl::Normal>);
  normals = kc->GetPclNormals();
  pcl_cloud = kc->GetPclCloud();
  
  for(unsigned i=0; i<normals->points.size(); i++)
  {
    cv::Vec3f s, e;
    s[0] = pcl_cloud->points[i].x;
    s[1] = pcl_cloud->points[i].y;
    s[2] = pcl_cloud->points[i].z;
    e[0] = pcl_cloud->points[i].x + normals->points[i].normal_x/500.;
    e[1] = pcl_cloud->points[i].y + normals->points[i].normal_y/500.;
    e[2] = pcl_cloud->points[i].z + normals->points[i].normal_z/500.;
    
    tgR->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 1);
  }
}

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
void SegTester::SingleShotMode()
{
//   sleepComponent(10);
  int key = 0;
  key = cvWaitKey(10);
  
  if (key == 65471 || key == 1114047) // F2
  {
//     const char* text = score->GetGestaltListInfo();
//     printf("\n%s\n", text);
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

  if (key == 65474 || key == 1114050) // F5
  {
    relations->PrintResults();
    relations->PrintRelations();
  }


  if (key == 65478 || key == 1114054) // F9
  {
    log("process images in single shot mode.");
    printf("\nSegTester::SingleShotMode: Process next image!\n");
    lockComponent();
    processImageNew();
    unlockComponent();
  }
  if (key == 65479 || key == 1114055) // F10
  {
    log("process images in single shot mode.");
    printf("\nSegTester::SingleShotMode: Process next image!\n");
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
      kcore->DrawObjects3D(tgRenderer);
      tgRenderer->Update();
      break;
      
    case '4':
      log("Show GRAPH-CUT");
      tgRenderer->Clear();
      kcore->DrawGraphCut3D(tgRenderer);
      tgRenderer->Update();
      break;
      
    case '5':
      log("Show Normals");
      if (showImages)
      {
//         tgRenderer->Clear();
        DrawNormals(kcore, tgRenderer);
        tgRenderer->Update();
      }
      break;

    case '6':
      log("Show POINT CLOUD dilatation");
      if (showImages)
      {
        tgRenderer->Clear();
        tgRenderer->Update();
//         tgRenderer->AddPointCloud(kinect_point_cloud_dil);
//         tgRenderer->Update();
      }
      break;

    case '9':
      log("Show Relation debug pixels");
      if (showImages)
      {
        std::vector<cv::Vec4f> pts;
        relations->GetPixelsToDraw(pts);
for(unsigned i=0; i<pts.size(); i++)
  printf("Number of pixels to draw: %u: %4.3f-%4.3f-%4.3f-%4.3f\n", pts.size(), pts[i][0], pts[i][1], pts[i][2], pts[i][3]);
        tgRenderer->Clear();
//         tgRenderer->AddPointCloud(kinect_point_cloud);
        tgRenderer->AddPointCloud(pts);
        tgRenderer->Update();
      }
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
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH, false);
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
        Z::SetActiveDrawArea(iplImage_k);
        vcore->DrawGestalts(Z::Gestalt::SEGMENT, 1);
        cvShowImage("Kinect", iplImage_k);
      }
      if(showImages)
      {
        tgRenderer->Clear();
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
        line_image = kinect_image;
        kcore->DrawGestalts3DToImage(line_image, Z::Gestalt3D::LINE, camPars[2]);
        cv::imshow("3D-Gestalt", line_image);
      }
      if(showImages)
      {
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(kinect_point_cloud);
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::LINE, false);
        tgRenderer->Update();
      }
      break;
    case 'r':
      log("Show COLLINEARITIES");
      tgRenderer->Clear();
      tgRenderer->Update();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::COLLINEARITY, false);
      tgRenderer->Update();
      break;
//     case 't':
//       break;
    case 'z':
      log("Show CLOSURES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::CLOSURE, false);
      tgRenderer->Update();
      break;
    case 'u':
      log("Show RECTANGLES");
      tgRenderer->Clear();
      tgRenderer->AddPointCloud(kinect_point_cloud);
      kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::RECTANGLE, false);
      tgRenderer->Update();
      break;
      
      
      
    case 'a':
      log("Show PCL_PATCHES");
      if(showImages)
      {
        tgRenderer->Clear();
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH, true);
        tgRenderer->Update();
      }
      break;     
    case 's':
      log("Show PCL_SPHERES");
      if(showImages)
      {
        tgRenderer->Clear();
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PCL_SPHERE, true);
        tgRenderer->Update();
      }
      break;    
    case 'd':
      log("Show PCL_CYLINDERS");
      if(showImages)
      {
        tgRenderer->Clear();
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PCL_CYLINDER, true);
        tgRenderer->Update();
      }
      break;    
      
      case 'y':
      log("Show PCL_GESTALTS");
      if(showImages)
      {
        tgRenderer->Clear();
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PATCH, false, true, GetRandomColor());
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PCL_SPHERE, false, true, GetRandomColor());
        kcore->DrawGestalts3D(tgRenderer, Z::Gestalt3D::PCL_CYLINDER, false, true, GetRandomColor());
        tgRenderer->Update();
      }
      break;    
      
    case 'x':                               /// TODO Take F10 or F12???
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






