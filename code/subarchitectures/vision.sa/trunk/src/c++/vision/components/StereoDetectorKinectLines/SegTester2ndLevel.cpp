/**
 * @file SegTester2ndLevel.cpp
 * @author Andreas Richtsfeld
 * @date February 2012
 * @version 0.1
 * @brief Segmentation tester for 2-level svm approach.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SegTester2ndLevel.h"

#include "Gestalt3D.h"
#include "StereoBase.h"
#include "Draw.hh"

#include "StereoCamera.h"
#include "VisionData.hpp"

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
    return new cast::SegTester2ndLevel();
  }
}

namespace cast
{


// **************************** SegTester **************************** //
  
/**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void SegTester2ndLevel::configure(const map<string,string> & _config)
{
  deb = true;   // set debug flag
  
  // first let the base classes configure themselves (for point clouds)
  configureServerCommunication(_config);

  // for one vision core: only lines
  runtime = 10;                                     // processing time for image => we need no incremental processing (only line calculation)
  cannyAlpha = 0.75;                                // Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
  cannyOmega = 0.001;
  
  vcore = new Z::VisionCore();                      // New vision core
  vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_SEGMENTS);
//   vcore->EnableGestaltPrinciple(Z::GestaltPrinciple::FORM_LINES);
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

  if(showImages)
  {
    // initialize tgRenderer
    cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Vec3d rotCenter(0,0,0.6);
    
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
//     tgRenderer->SetClearColor(1., 1., 1.);
    tgRenderer->SetClearColor(0., 0., 0.);
    tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);

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
  svm = new svm::SVMPredictor(files);
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
void SegTester2ndLevel::start()
{
  startPCCServerCommunication(*this);  // start point cloud communication
}

/**
 * @brief Called by the framework to start component run loop.
 * @TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void SegTester2ndLevel::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
    processImageNew();
  }
  if(showImages) {
    cvReleaseImage(&iplImage_k);
    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
}
  
 
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void SegTester2ndLevel::GetImageData()
{
  log("Get image data started.");
  
  pointCloudWidth = 320;
  pointCloudHeight = pointCloudWidth *3/4;
  rgbWidth = 640;
  rgbHeight = rgbWidth *3/4;
  
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);      // Get the image grid point cloud with zeros, when no values are available!
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true); // replace 0-points by NAN's
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);
  pclA::ConvertPCLCloud2Image(pcl_cloud, kinect_point_cloud_image);

  tgRenderer->SetImage(kinect_point_cloud_image);
  
  // get rectified images from point cloud server
  getRectImage(2, rgbWidth, image_k);                       // 2 = kinect image / we take it with kinect image width
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

  static struct timespec start, last, current;
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  if(deb) last = start;
  
  /// calculate normals
  pclA::NormalsEstimationNR::Parameter param(5, 0.02, 1000, 0.001, 10);
  pclA::NormalsEstimationNR n;
  n.setParameter(param);
  n.setInputCloud(pcl_cloud);
  n.compute();
  n.getNormals(pcl_normals);

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Getting images => Calculate normals %4.3f\n", timespec_diff(&current, &last));
  

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
void SegTester2ndLevel::processImageNew()
{
  if(deb) printf("SegTester::processImageNew: start\n");
  
  static struct timespec start, last, current;
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  if(deb) last = start;

  /// Get kinect data
  GetImageData();
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: Getting images (not correct): %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  

  /// Run vision core (for canny edges
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
  GetSegmentIndexes(vcore, texture, 320);
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Vision core: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  

  /// ModelFitter
  if(deb) log("Model-Fitter start!");
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
  if(deb) log("Model-Fitter end!");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Model fitting: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 
  
  /// NURBS-Fitting and model selection
  if(deb) log("NURBS-Fitting start!");
  modeling->setInputCloud(pcl_cloud);
  modeling->setInputPlanes(pcl_model_types, model_coefficients, pcl_model_indices, error);
  modeling->compute();
  modeling->getSurfaceModels(surfaces);
  modeling->getResults(pcl_model_types, pcl_model_indices, error);    // TODO sollte hier nicht mehr notwendig sein!
  if(deb) log("NURBS-Fitting end!");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: NURBS-fitting & model selection: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 
  
  if(deb) log("Annotation: start");
  std::vector< std::vector<int> > anno_pairs;
  std::vector<int> anno_background_list;
  annotation->load(pointCloudWidth, anno, true);            /// TODO Das ist überflüssig - Könnte intern aufgerufen werden
  annotation->setSurfaceModels(surfaces);
  annotation->calculate();
  annotation->getResults(nr_anno, anno_pairs, anno_background_list);
  if(deb) log("Annotation: end");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: Annotation calculation: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 

  /// Calculate patch relations
  if(deb) log("Calculate patch-relations start!");
  std::vector<Relation> relation_vector;
  patches->setInputImage(iplImage_k);
  patches->setInputCloud(pcl_cloud);
  patches->setNormals(pcl_normals);
  patches->setSurfaceModels(surfaces);
  patches->setAnnotion(anno_pairs, anno_background_list);
  patches->setTexture(texture); 
  patches->setOptimalPatchModels(true);
  patches->computeTestRelationsOld();
  patches->getRelations(relation_vector);
  patches->getOutputCloud(pcl_cloud, pcl_normals);    // pcl_cloud_vis, pcl_normals_vis
  patches->getSurfaceModels(surfaces);
  if(deb) log("Calculate patch-relations ended!");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  

  /// SVM-Prediction
  if(deb) log("svm-predictor: start");
  if(deb) printf("SegTester: Prediction start: relation_vector.size: %lu\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++)
  {
    relation_vector[i].prediction = svm->getResult(relation_vector[i].type, 
                                                   relation_vector[i].rel_value, 
                                                   relation_vector[i].rel_probability);
    if(deb) 
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
  if(deb) log("svm-predictor: end");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: Prediction: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;

  /// Graph cutter
  if(deb) log("graph-cutter: start");
  int nr_models = surfaces.size();
  graphCut->init(relation_vector);
  graphCut->process();
  graphCut->getResults(nr_models, graphCutGroups);
  if(deb) log("graph-cutter: end");
  
// printf("GraphCut groups:\n");
// for(unsigned i=0; i<graphCutGroups.size(); i++) {
//   printf("  %u: ", i);
//   for(unsigned j=0; j<graphCutGroups[i].size(); j++)
//     printf("%u ", graphCutGroups[i][j]);
//   printf("  \n");
// }

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: GraphCutter: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;

  /// Check annotation for evaluation
  CheckAnnotation(surfaces, anno, graphCutGroups);    /// TODO move this function to annotation class

// printf("    annotation: start\n");
//   std::vector<int> anno;
//   annotation->load(pointCloudWidth, anno);
//   kcore->CheckAnnotation(anno);
//   double m, s, b;
//   kcore->GetAnnotationResults(m, s, b);
//   printf("SegTester: overall annotation results: %4.3f - %4.3f - %4.3f\n", m*100, s*100, b*100);
// printf("    annotation: end\n");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegTester: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
}


/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   F9 ... Process single shot \n
 *   
 *   1 ... Show point cloud
 *   5 ... Show surface models
 *   6 ... Show normals
 *   7 ... Show patches
 *   8 ... Show annotation
 *   9 ... Show results after svm and graph cut
 *   0 ... Show surface mesh models (colored)
 *   
 *   x ... stop single shot mode
 *   s ... 
 *   
 */
void SegTester2ndLevel::SingleShotMode()
{
//   sleepComponent(10);
  int key = 0;
  key = cvWaitKey(10);
  
  if (key == 65471 || key == 1114047)  { // F2
    log("currently unused.");
  }

  if (key == 65472 || key == 1114048)  { // F3
    const char* text = vcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65473 || key == 1114049)  { // F4
    log("currently unused.");
  }

  if (key == 65474 || key == 1114050)  { // F5
    log("currently unused.");
  }


  if (key == 65478 || key == 1114054)  { // F9
    log("process images in single shot mode.");
    lockComponent();
    processImageNew();
    unlockComponent();
  }
  if (key == 65479 || key == 1114055)  { // F10
    log("currently unused.");
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
      log("unused");
      break;
      
    case '3':
      log("unused");
      break;
      
    case '4':
      log("unused");
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
      log("Show mesh of surfaces. wait ...");
      surface::CreateMeshModel createMesh(surface::CreateMeshModel::Parameter(.1));
      createMesh.setInputCloud(pcl_cloud);
      createMesh.compute(surfaces);
      tgRenderer->Clear();
      tgRenderer->ClearModels();
      
      RGBValue color;
      for(unsigned i=0; i<graphCutGroups.size(); i++) {
        color.float_value = GetRandomColor();
        for(unsigned j=0; j<graphCutGroups[i].size(); j++) {    
          surfaces[graphCutGroups[i][j]]->mesh.m_material.Color(color.r/255., color.g/255., color.b/255.);
          tgRenderer->AddModel(&surfaces[graphCutGroups[i][j]]->mesh);
        }
      }
      tgRenderer->Update();
      log("Show mesh of surfaces. done.");
      break;
    }
  
      
    case 'x':                               /// TODO Take F10 or F12???
      log("End Single-Shot mode!");
      single = false;
      break;
  }
}

}






