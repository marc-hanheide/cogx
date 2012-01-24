/**
 * @file SegTester.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

#include "SegUtilsFunctions.h"

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
  
/** Debug flag **/
// static bool deb;
// #ifdef DEBUG
//   #define deb = true;
// #elseif
//   #define deb = false;
// #endif


// **************************** SegTester **************************** //
  
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
  for(size_t i = 0; i < camPars.size(); i++) {
    camPars[i].id = camIds[i];
    camPars[i].time = getCASTTime();
  }
  
  if((it = _config.find("--showImages")) != _config.end())
    showImages = true;

  if((it = _config.find("--singleShot")) != _config.end()) {
    log("single shot modus on.");
    single = true;
  }

  if((it = _config.find("--stereoconfig_xml")) != _config.end()) {
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
    log("Warning: Antiquated: Use stereoconfig_xml with openCV stereo calibration");

  if(showImages)
  {
    // initialize tgRenderer
//     cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
    cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Vec3d rotCenter(0,0,0.4);
    
    cv::Mat intrinsic;
    intrinsic = cv::Mat::zeros(3,3,CV_64F);
    intrinsic.at<double>(0,0) = intrinsic.at<double>(1,1) = 525;
    intrinsic.at<double>(0,2) = 320;
    intrinsic.at<double>(1,2) = 240;
    intrinsic.at<double>(2,2) = 1.;
    
    // Initialize 3D render engine 
    tgRenderer = new TomGine::tgTomGineThread(640, 480);
    tgRenderer->SetCamera(intrinsic);
    tgRenderer->SetCamera(R, t);
    tgRenderer->SetRotationCenter(rotCenter);
    tgRenderer->SetClearColor(1., 1., 1.);
//     tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);

  
  /// ################### The old classes for calculation ################### ///
  kcore = new Z::KinectCore(vcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  // TODO Enable/Disable KinectPrinciples?

  planePopout = new pclA::PlanePopout();
  relations = new Z::CalculateRelations();
  relations->Initialize(kcore, camPars[2].fx, camPars[2].fy, camPars[2].cx, camPars[2].cy);
  
//   std::vector<const char*> filenames;
//   const char* filename = "./instantiations/11-05-11/1207/PP-Trainingsset.txt.model";
//   filenames.push_back(filename);
// //   filename = "./instantiations/11-05-11/PL-Trainingsset.txt.model";
// //   filenames.push_back(filename);
// //   filename = "./instantiations/11-05-11/LL-Trainingsset.txt.model";
// //   filenames.push_back(filename);
//   svmPredictor = new Z::SVMPredictor(filenames.size(), filenames);      // 2 ... Number of SVM's  TODO Remove size!

  graphCutter = new Z::GraphCut(kcore, relations);

  /// ################### The new classes for calculation ################### ///
  
  /// init model fitter
  bool use_voxel_grid = false;
  double voxel_grid_size = 0.005;           // 0.005 - 0.01
  bool do_z_filtering = true;
  double minZ = 0.3;
  double maxZ = 1.5;
  bool sac_optimal_distance = true;
  double sac_optimal_weight_factor = 3.0;
  double sac_distance = 0.002;
  int sac_max_iterations = 250;
  unsigned sac_min_inliers = 25;
  double ec_cluster_tolerance = 0.01; //0.008,  /// TODO Calculate tolerance like sac_optimal_weight_factor?
  int ec_min_cluster_size = 15;
  int ec_max_cluster_size = 1000000;
  pclA::ModelFitter::Parameter mf_param(use_voxel_grid, voxel_grid_size, 
                                        do_z_filtering, minZ, maxZ, 
                                        sac_optimal_distance,
                                        sac_optimal_weight_factor, sac_distance, 
                                        sac_max_iterations, sac_min_inliers, 
                                        ec_cluster_tolerance, ec_min_cluster_size, 
                                        ec_max_cluster_size);
  model_fitter = new pclA::ModelFitter(mf_param);

  /// init nurbsfitting & model-selection
  nurbsfitting::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;                 // 1
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;          // 1
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16; 
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;                   // 0.003
  sfmParams.kappa2 = 1.0;                     // 0.9
  sfmParams.useDominantPlane = true;
  modeling = new surface::SurfaceModeling(sfmParams);
  modeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  modeling->setExtrinsic(pose);

  /// init annotation
  annotation = new anno::Annotation();
//   annotation->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 17, 30);
//   annotation->init("/media/Daten/Object-Database/annotation/box_world_fi%1d.png", 0, 15);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 0, 9);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 10, 23);
  annotation->init("/media/Daten/Object-Database/annotation/cvww_mixed_fi%1d.png", 0, 8);

  /// init patch class
  patches = new surface::Patches();
  patches->setZLimit(0.01);
  
  /// init svm-predictor
  std::vector<const char*> files;
  const char* file = "./instantiations/11-05-11/1215/PP-Trainingsset.txt.scaled.model";
  files.push_back(file);
  svm = new svm::SVMPredictor(files.size(), files);   /// TODO Remove files.size() => sinnlos
  svm->setScaling(true, "./instantiations/11-05-11/1215/param.txt");
  
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
    processImageNew();
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
  log("Get image data started.");
  
  pointCloudWidth = 320;
  pointCloudHeight = pointCloudWidth *3/4;
  rgbWidth = 640;
  rgbHeight = rgbWidth *3/4;
//  double rgb2depthScale = rgbWidth/pointCloudWidth;
  
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);      // Get the image grid point cloud with zeros, when no values are available!
//   getPoints(false, pointCloudWidth, points);           /// TODO für KinectStereoSeqServer => einbauen von getCompletePoints!!!!
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true); // replace 0-points by NAN's
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);
  pclA::ConvertPCLCloud2Image(pcl_cloud, kinect_point_cloud_image);

  // get rectified images from point cloud server
//   getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
//   getRectImage(1, kinectImageWidth, image_r);            // 1 = right image / we take it with kinect image width
  getRectImage(2, rgbWidth, image_k);                       // 2 = kinect image / we take it with kinect image width
//   iplImage_l = convertImageToIpl(image_l);
//  iplImage_r = convertImageToIpl(image_r);
  iplImage_k = convertImageToIpl(image_k);
    /// bilateral filter
//   bilateral->setInputCloud(pcl_cloud);
//   bilateral->compute();
//   bilateral->getCloud(pcl_cloud);
  
  /// subsample point cloud
//   subsample->setInputCloud(pcl_cloud);
//   subsample->compute();
//   subsample->getCloud(pcl_cloud);
//   pointCloudWidth = pointCloudWidth/2.;
//   pointCloudHeight = pointCloudWidth *3/4;

  /// calculate normals
//   pclA::NormalsFromSortedPCLCloud(pcl_cloud, pcl_normals, 0.02, 5.0);

static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;
  pclA::NormalsEstimationNR::Parameter param(5, 0.02, 1000, 0.001, 10);
  pclA::NormalsEstimationNR n;
  n.setParameter(param);
  n.setInputCloud(pcl_cloud);
  n.compute();
  n.getNormals(pcl_normals);
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Getting images => Calculate normals %4.3f\n", timespec_diff(&current, &last));
  

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
  log("Get image data ended.");
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

    /// Run vision core (for canny edges
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
  GetSegmentIndexes(vcore, texture, 320);
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Vision core: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// ModelFitter
  log("Model-Fitter start!");
  std::vector<int> pcl_model_types;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  std::vector< std::vector<double> > error;
  std::vector<double> square_error;
  model_fitter->addModelType(pcl::SACMODEL_PLANE);
//  model_fitter->addModelType(pcl::SACMODEL_NORMAL_PLANE);
//  model_fitter->addModelType(pcl::SACMODEL_CYLINDER);
//  model_fitter->addModelType(pcl::SACMODEL_SPHERE);
  model_fitter->setNormals(pcl_normals);
  model_fitter->useDominantPlane(false);
  model_fitter->setInputCloud(pcl_cloud);
  model_fitter->compute();
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices);
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices_old);
  model_fitter->getError(error, square_error);
  log("Model-Fitter end!");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Model fitting: %4.3f\n", timespec_diff(&current, &last));
last = current; 
  
  /// NURBS-Fitting and model selection
  log("NURBS-Fitting start!");
  modeling->setInputCloud(pcl_cloud);
  modeling->setInputPlanes(pcl_model_types, model_coefficients, pcl_model_indices, error);
  modeling->compute();
  modeling->getSurfaceModels(surfaces);
//   modeling->getPlanes(modelTypes, coeffs, plane_indices, error);
//   modeling->getNurbs(modelTypes, nurbs, nurbs_indices, error);
  modeling->getResults(pcl_model_types, pcl_model_indices, error);
  log("NURBS-Fitting end!");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: NURBS-fitting & model selection: %4.3f\n", timespec_diff(&current, &last));
last = current; 
  
  log("Annotation: start");
  // annotation for evaluation
  std::vector< std::vector<int> > anno_pairs;
  std::vector<int> anno_background_list;
  annotation->load(pointCloudWidth, anno, true);            /// TODO Das ist überflüssig - Könnte intern aufgerufen werden
  //annotation->setIndices(pcl_model_indices);                /// TODO Sollte die indices nicht von surfaces kommen => da sind Nurbs auch dabei!
  annotation->setSurfaceModels(surfaces);
  annotation->calculate();
  annotation->getResults(nr_anno, anno_pairs, anno_background_list);
  log("Annotation: end");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Annotation calculation: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  /// Calculate patch relations
  log("Calculate patch-relations start!");
  std::vector<Relation> relation_vector;
  patches->setInputImage(iplImage_k);
  patches->setInputCloud(pcl_cloud);
  patches->setNormals(pcl_normals);
  patches->setSurfaceModels(surfaces);
  patches->setAnnotion(anno_pairs, anno_background_list);       // TODO for evaluation => Necessary?
  patches->setTexture(texture); 
  patches->computeOptimalPatchModels(true);            /// TODO TODO TODO TODO TODO TODO Sollte nicht compute, sonder set heißen, da ja nichts berechnet wird!
//  patches->computeNeighbors();                  /// TODO TODO Müssen neighbors wirklich vorher berechnet werden? Nicht automatisch?
  patches->computeTestRelations();
//   patches->getNeighbors(neighbors);
  patches->getRelations(relation_vector);
  patches->getOutputCloud(pcl_cloud, pcl_normals);    // pcl_cloud_vis, pcl_normals_vis
  patches->getSurfaceModels(surfaces);
  log("Calculate patch-relations ended!");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// SVM-Prediction
  log("svm-predictor: start");
  if(debug) printf("SegTester: Prediction start: relation_vector.size: %lu\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    relation_vector[i].prediction = svm->getResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    if(debug) 
    {
      if(relation_vector[i].groundTruth == 0) {
        printf("relation [%u][%u]: gt: false => %4.3f", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
        if(relation_vector[i].prediction > 0.5)
          printf(" => is false\n");
        else
          printf(" => is true\n");
      }
      if(relation_vector[i].groundTruth == 1) {
        printf("relation [%u][%u]: gt: true  => %4.3f", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
        if(relation_vector[i].prediction < 0.5)
          printf(" => is false\n");
        else
          printf(" => is true\n");      
      }
    }
  }
  log("svm-predictor: end");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: Prediction: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// Graph cutter
  int nr_models = pcl_model_types.size();               /// number of given models
  log("graph-cutter: start");
  graphCut->init(relation_vector);
  graphCut->process();
  graphCut->getResults(nr_models, graphCutGroups);
  log("graph-cutter: end");
  if(debug) printf("graphCutter: end: clusters.size(): %lu\n", graphCutGroups.size());


clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegTester: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
last = current;

CheckAnnotation(surfaces, anno, graphCutGroups);

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
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  

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
  planePopout->CalculateSOIs(pcl_cloud);
  planePopout->GetSOIs(sois, soi_labels);
  if(!planePopout->CalculateROIMask()) 
    printf("StereoDetectorKinectLines: Error while processing ROI mask in PlanePopout!\n");

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
  log("svm-prediction: start");
printf("SegTester: Prediction start: relation_vector.size: %lu\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    std::vector<double> probability;
    relation_vector[i].prediction = svmPredictor->GetResult(relation_vector[i].type, relation_vector[i].rel_value, relation_vector[i].rel_probability);
    relations->AddPrediction(i, relation_vector[i].prediction);                           /// TODO We add from here the prediction value and the probability???
    relations->AddProbability(i, relation_vector[i].rel_probability);
  }
  log("svm-prediction: end");

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
  log("graphCutter: start");
  graphCutter->Initialize();
  graphCutter->Cut();
  graphCutter->CopyGroupIDToFeatures();
  log("graphCutter: end");

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
      
    /// *** For new processing!!! *** ///
    case '5':
      log("Show PATCHES");
      tgRenderer->ClearModels();
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f center3D[pcl_model_indices.size()];
        RGBValue col[pcl_model_indices.size()];
        for(unsigned i=0; i<pcl_model_indices.size(); i++) {
          col[i].float_value = GetRandomColor();
          for(unsigned j=0; j<pcl_model_indices[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].z;
            pt[3] = col[i].float_value;
            center3D[i][0] = center3D[i][0] + pt[0];
            center3D[i][1] = center3D[i][1] + pt[1];
            center3D[i][2] = center3D[i][2] + pt[2];
            col_points.push_back(pt);
          }
        }
    
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);

        // Add labels
//         for(size_t i=0; i<pcl_model_indices.size(); i++) {
//           char label[5];
//           snprintf(label, 5, "%lu", i);
//           tgRenderer->AddLabel3D(label, 14, 
//                                  center3D[i][0]/pcl_model_indices[i]->indices.size(), 
//                                  center3D[i][1]/pcl_model_indices[i]->indices.size(), 
//                                  center3D[i][2]/pcl_model_indices[i]->indices.size());
//         }
        tgRenderer->Update();
      }
      break; 
      
    case '6':
      log("Draw Nomals");
      DrawNormals(pcl_cloud, pcl_normals, tgRenderer, 1);
      tgRenderer->Update();
      break;
      
    case '7':
      log("Show results from model fitter.");
      tgRenderer->ClearModels();
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f planeCenter3D[pcl_model_indices_old.size()];
        RGBValue col[pcl_model_indices_old.size()];
        for(unsigned i=0; i<pcl_model_indices_old.size(); i++) {
          col[i].float_value = GetRandomColor();
          for(unsigned j=0; j<pcl_model_indices_old[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_indices_old[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_indices_old[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_indices_old[i]->indices[j]].z;
            pt[3] = col[i].float_value;
            planeCenter3D[i][0] = planeCenter3D[i][0] + pt[0];
            planeCenter3D[i][1] = planeCenter3D[i][1] + pt[1];
            planeCenter3D[i][2] = planeCenter3D[i][2] + pt[2];
            col_points.push_back(pt);
          }
        }    
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);

        // Add labels
//         for(size_t i=0; i<pcl_model_indices.size(); i++) {
//           char label[5];
//           snprintf(label, 5, "%lu", i);
//           tgRenderer->AddLabel3D(label, 14, 
//                                  planeCenter3D[i][0]/pcl_model_indices_old[i]->indices.size(), 
//                                  planeCenter3D[i][1]/pcl_model_indices_old[i]->indices.size(), 
//                                  planeCenter3D[i][2]/pcl_model_indices_old[i]->indices.size());
//         }
        tgRenderer->Update();
      }
      break; 
      
      case '8':
      log("Show annotation parts.");
      tgRenderer->ClearModels();
      if(showImages)
      {
        RGBValue col[nr_anno];
        for(int i=0; i<nr_anno; i++)
          col[i].float_value = GetRandomColor();
        
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f planeCenter3D[pcl_model_indices.size()];
        for(size_t i=0; i<pcl_model_indices.size(); i++) {
          for(size_t j=0; j<pcl_model_indices[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].z;
            int number = anno[pcl_model_indices[i]->indices[j]]-1;
            pt[3] = col[number].float_value;
            planeCenter3D[i][0] = planeCenter3D[i][0] + pt[0];
            planeCenter3D[i][1] = planeCenter3D[i][1] + pt[1];
            planeCenter3D[i][2] = planeCenter3D[i][2] + pt[2];
            col_points.push_back(pt);
          }
        }
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);

        // Add labels
        for(size_t i=0; i<pcl_model_indices.size(); i++) {
          char label[5];
          snprintf(label, 5, "%lu", i);
          tgRenderer->AddLabel3D(label, 14, 
                                 planeCenter3D[i][0]/pcl_model_indices[i]->indices.size(), 
                                 planeCenter3D[i][1]/pcl_model_indices[i]->indices.size(), 
                                 planeCenter3D[i][2]/pcl_model_indices[i]->indices.size());
        }
        tgRenderer->Update();
      }
      break;   

      case '9':
      log("Show results after graph cut.");
      tgRenderer->ClearModels();
      if(showImages)
      {
        RGBValue col[graphCutGroups.size()];
        for(size_t i=0; i<graphCutGroups.size(); i++)
          col[i].float_value = GetRandomColor();
        
        std::vector<cv::Vec4f> col_points;
        for(size_t i=0; i<pcl_model_indices.size(); i++) {
          for(size_t j=0; j<pcl_model_indices[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_indices[i]->indices[j]].z;
            unsigned number = WhichGraphCutGroup(i, graphCutGroups);
            pt[3] = col[number].float_value;
            col_points.push_back(pt);
          }
        }
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);

        // Add labels
//         cv::Vec4f planeCenter3D[graphCutGroups.size()];
//             planeCenter3D[i][0] = planeCenter3D[i][0] + pt[0];
//             planeCenter3D[i][1] = planeCenter3D[i][1] + pt[1];
//             planeCenter3D[i][2] = planeCenter3D[i][2] + pt[2];
//         
//         for(size_t i=0; i<pcl_model_indices.size(); i++) {
//           char label[5];
//           snprintf(label, 5, "%lu", i);
//           tgRenderer->AddLabel3D(label, 14, 
//                                  planeCenter3D[i][0]/pcl_model_indices[i]->indices.size(), 
//                                  planeCenter3D[i][1]/pcl_model_indices[i]->indices.size(), 
//                                  planeCenter3D[i][2]/pcl_model_indices[i]->indices.size());
//         }
        tgRenderer->Update();
      }
      break; 
      
    case  '0':
    {
//       log("Show mesh of surfaces");
//       tgRenderer->ClearModels();
//       tgRenderer->Clear();
//       tgRenderer->SetImage(kinect_point_cloud_image);
//       for(unsigned gcg=0; gcg < graphCutGroups.size(); gcg++)
//       {
//         TomGine::tgMaterial mat;
//         mat.Random();
//         for(unsigned i=0; i<graphCutGroups[gcg].size(); i++)
//         {
//           TomGine::tgRenderModel model = surfaces[graphCutGroups[gcg][i]]->mesh;
//           model.m_material = mat;
//           tgRenderer->AddModel(model);
//         }
//       }
      
      log("Show mesh of surfaces. wait ...");
      surface::CreateMeshModel createMesh(surface::CreateMeshModel::Parameter(.1));
      createMesh.setInputCloud(pcl_cloud);
      createMesh.compute(surfaces);

      tgRenderer->ClearModels();
      for (unsigned i=0; i<surfaces.size(); i++)
        tgRenderer->AddModel(&surfaces[i]->mesh);
      tgRenderer->Update();
      log("Show mesh of surfaces. done.");
      
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

  }
}

}






