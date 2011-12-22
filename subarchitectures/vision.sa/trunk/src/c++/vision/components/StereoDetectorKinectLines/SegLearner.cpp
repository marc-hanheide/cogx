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
#include "Segment.hh"   // TODO
#include "Edgel.hh"     // TODO

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
static bool deb;
#ifdef DEBUG
  #define deb = true;
#elseif
  #define deb = false;
#endif


// **************************** SegLearner **************************** //

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
      if(file.find(":") == string::npos)
      {
        // monocular files can be either .cal (INI style) or .xml (from OpenCV file storage)
        if(file.find(".xml") == string::npos)
          loadCameraParameters(pars, file);
        else
          loadCameraParametersXML(pars, file);
      }
      else { // stereo case
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
  if(camPars.size() == 0) {
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

  if((it = _config.find("--stereoconfig")) != _config.end()) {
    log("Warning: Antiquated: Use stereoconfig_xml with openCV stereo calibration");
    stereoconfig = it->second;
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
    cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);  // 0 == LEFT
    cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Vec3d rotCenter(0,0,0.4);
    
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
  
  /// init bilateral filter
  bilateral = new pclA::BilateralFilter(pclA::BilateralFilter::Parameter());
  
  /// init subsample-tool for point cloud
  subsample = new pclA::SubsamplePointCloud(pclA::SubsamplePointCloud::Parameter(2, .02, true, false, false));
      
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
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16; 
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.useDominantPlane = true;
  modeling = new surface::SurfaceModeling(sfmParams);
  modeling->setIntrinsic(525., 525., 320., 240.);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  modeling->setExtrinsic(pose);
  
  /// init annotation for first-level svm learning
  annotation = new pa::Annotation();
//   annotation->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 0, 16);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 0, 9);
  /// eval svm
//   annotation->init("/media/Daten/Object-Database/annotation/box_world_fi%1d.png", 0, 15);
//   annotation->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 17, 30);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 10, 23);
  annotation->init("/media/Daten/Object-Database/annotation/cvww_mixed_fi%1d.png", 0, 8);

  
  
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
  startPCCServerCommunication(*this);     // start point cloud communication
  if(showImages) {
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
//     processImage();
    processImageNew();
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
  
//   delete tgRenderer;
  log("deleted components: runComponent ended.");
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

  /// calculate normals from point cloud
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
  
  
  if(showImages) {
    cvShowImage("Kinect image", iplImage_k);
    // convert point cloud to iplImage
//     cv::Mat_<cv::Vec3b> kinect_pc_image;
//     pclA::ConvertPCLCloud2Image(pcl_cloud, kinect_pc_image);
//     cv::imshow("Kinect Image from point cloud", kinect_pc_image);
  }
}

/**
 *  @brief Process data from stereo or Kinect.
 */
void SegLearner::processImageNew()
{
  surfaces.clear();
  
static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;

  /// Get kinect data
  GetImageData();
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Getting images: %4.3f\n", timespec_diff(&current, &last));
last = current;  
  printf("############################### end => This takes longer!\n");
  
  /// Run vision core (for canny edges
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
  GetSegmentIndexes(vcore, texture, 320);

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Vision core: %4.3f\n", timespec_diff(&current, &last));
last = current;  
  
  /// ModelFitter
  log("ModelFitter start!");
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
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices);         /// TODO Eigentlich sollte man hier schon surfaces kriegen?
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices_old);
  model_fitter->getError(error, square_error);
  log("ModelFitter end!");
  
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
  modeling->getResults(pcl_model_types, pcl_model_indices, error);      // TODO Nur für Anzeige notwendig!
  log("NURBS-Fitting end!");

// printf("surfaces.size(): %u\n", surfaces.size());
// for(unsigned i=0; i< pcl_model_types.size(); i++) 
// {
//   if(pcl_model_types[i] = pcl::SACMODEL_PLANE)
//     printf("plane\n");
//   else 
//     printf("no plane!\n");
// }

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: NURBS & MODEL-SELECTION: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  log("Annotation: start");
  std::vector< std::vector<int> > anno_pairs;
  std::vector<int> anno_background_list;
  annotation->load(pointCloudWidth, anno, true);            /// TODO TODO TODO TODO TODO Das ist überflüssig - Könnte intern aufgerufen werden
//   annotation->setIndices(pcl_model_indices);
  annotation->setSurfaces(surfaces);
  annotation->calculate();
  annotation->getResults(nr_anno, anno_pairs, anno_background_list);
//   for(unsigned i=0; i<anno_pairs.size(); i++) {
//     printf("Annotation pairs for %u: ", i);
//     for(unsigned j=0; j<anno_pairs[i].size(); j++)
//       printf(" %u", anno_pairs[i][j]);
//     printf("\n");
//   }
  log("Annotation: end");
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Annotation calculation: %4.3f\n", timespec_diff(&current, &last));
last = current; 

  /// Calculate patch relations
  log("Calculate patch-relations start!");
  std::vector<Relation> relation_vector;
  patches->setInputCloud(pcl_cloud, pcl_normals);
  patches->setPatches(surfaces);
  patches->setAnnotion(anno_pairs, anno_background_list);
  patches->setTexture(texture);
  patches->computePatchModels(true);
//  patches->computeNeighbors();
  patches->computeLearnRelations();
//   patches->getNeighbors(neighbors);
  patches->getRelations(relation_vector);
  
  pcl_normals_repro.reset(new pcl::PointCloud<pcl::Normal>);
  pcl_normals_repro->points.resize(pcl_normals->points.size());
  patches->getOutputCloud(pcl_cloud, pcl_normals_repro);
  log("Calculate patch-relations ended!");

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for SegLearner: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
last = current;  

  /// write svm-relations to file!
  log("process svm");
  svm->init(relation_vector);
  svm->setAnalyzeOutput(true);
  svm->process();
  log("process svm ended");

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
  static bool repro = true;

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
    log("process image in single shot mode with new implementations.");
    lockComponent();
    processImageNew();
    unlockComponent();
  }

  if (key == 65479 || key == 1114055) // F10
  {
    log("process images in single shot mode with old stuff.");
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
        for(size_t i=0; i<pcl_model_indices.size(); i++) {
          char label[5];
          snprintf(label, 5, "%lu", i);
          tgRenderer->AddLabel3D(label, 14, 
                                 center3D[i][0]/pcl_model_indices[i]->indices.size(), 
                                 center3D[i][1]/pcl_model_indices[i]->indices.size(), 
                                 center3D[i][2]/pcl_model_indices[i]->indices.size());
        }
        tgRenderer->Update();
      }
      break; 
      
    case '6':
      log("Draw normals");
      if(repro) {
        log("Draw original normals");
        DrawNormals(pcl_cloud, pcl_normals, tgRenderer, 1);
        repro = !repro;
      }
      else {
        log("Draw repro normals");
        DrawNormals(pcl_cloud, pcl_normals_repro, tgRenderer, 2);
        repro = !repro;
      }
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
        for(size_t i=0; i<pcl_model_indices.size(); i++) {
          char label[5];
          snprintf(label, 5, "%lu", i);
          tgRenderer->AddLabel3D(label, 14, 
                                 planeCenter3D[i][0]/pcl_model_indices_old[i]->indices.size(), 
                                 planeCenter3D[i][1]/pcl_model_indices_old[i]->indices.size(), 
                                 planeCenter3D[i][2]/pcl_model_indices_old[i]->indices.size());
        }
        tgRenderer->Update();
      }
      break; 
      
    case '8':
      log("Show annotated parts.");
      tgRenderer->ClearModels();
      if(showImages)
      {
        RGBValue col[nr_anno];
        for(int i=0; i<nr_anno; i++)
          col[i].float_value = GetRandomColor();
        
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f planeCenter3D[pcl_model_indices.size()];
        for(unsigned i=0; i<pcl_model_indices.size(); i++) {
          for(unsigned j=0; j<pcl_model_indices[i]->indices.size(); j++) {
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
      
    case  '0':
    {
      log("Show mesh of surfaces");
      tgRenderer->ClearModels();
      tgRenderer->Clear();
      for(unsigned i=0; i<surfaces.size(); i++)
      {
        tgRenderer->AddModel(surfaces[i]->mesh);
//         if(surfaces[i]->type == MODEL_NURBS)
//           DrawNURBS(tgRenderer, surfaces[i]->nurbs, 1);
      }
      tgRenderer->Update();
      break;
    }
//           
//     case  '8':
//     {
//       log("Show PlanePopout");
//       tgRenderer->Clear();
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr pp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//       planePopout->GetCloudDownsampled(pp_cloud);
//       cv::Mat_<cv::Vec4f> cv_pp_cloud;
//       pclA::ConvertPCLCloud2CvMat(pp_cloud, cv_pp_cloud, true);
//       tgRenderer->AddPointCloud(cv_pp_cloud);
//       tgRenderer->Update();
//       break;
//     }
//       
//     case '9':
//     {
//       log("Show PlanePopout");
//       tgRenderer->Clear();
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr table (new pcl::PointCloud<pcl::PointXYZRGB>);
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
//       planePopout->GetTableProjected(table);
//       planePopout->GetTableHull(table_hull);
//       cv::Mat_<cv::Vec4f> cv_table;
//       pclA::ConvertPCLCloud2CvMat(table, cv_table, false);
//       tgRenderer->AddPointCloud(cv_table);
//       int hull_size = table_hull->points.size();
//       for(unsigned i=0; i<hull_size-1; i++)
//       {
//         tgRenderer->AddLine3D(table_hull->points[i].x, table_hull->points[i].y, table_hull->points[i].z, 
//                                table_hull->points[i+1].x, table_hull->points[i+1].y, table_hull->points[i+1].z, 
//                                0, 255, 0, 3);
//       }
//       tgRenderer->AddLine3D(table_hull->points[0].x, table_hull->points[0].y, table_hull->points[0].z, 
//                              table_hull->points[hull_size-1].x, table_hull->points[hull_size-1].y, 
//                              table_hull->points[hull_size-1].z, 0, 255, 0, 3);
//       tgRenderer->Update();
//       break;
//     }

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
      
  }
}

}






