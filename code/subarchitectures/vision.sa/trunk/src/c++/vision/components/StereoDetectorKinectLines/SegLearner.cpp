/**
 * @file SegLearner.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SegLearner.h"

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

// **************************** SegLearner **************************** //
/**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void SegLearner::configure(const map<string,string> & _config)
{
  stopComponent = false;
  deb = true;     // debug flag
  labels = true;  // enable labels
  
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

  if(showImages)
  {
    // initialize tgRenderer
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
//     tgRenderer->SetClearColor(1., 1., 1.); /// TODO Set color to white
//     tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);

  /// ################### The new classes for calculation ################### ///
  
  // init bilateral filter
  // bilateral = new pclA::BilateralFilter(pclA::BilateralFilter::Parameter());
  
  // init subsample-tool for point cloud
  //subsample = new pclA::SubsamplePointCloud(pclA::SubsamplePointCloud::Parameter(2, .02, true, false, false));
      
  /// init model fitter (== plane fitter)
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

  /// init new plane fitting (MoSPlanes)
  int pyrLevels = 3;                        // Number of pyramid levels
  float nbDist = 0.02;                      // max. neighor distance for subsampling
  float thrAngleNormalClustering = 0.5;     // maximum angle for clustering (0.5 = 28° (+5,7))
  float inlDist = 0.008;                    // Inlier distance for planes
  float sigma = 0.008;                      // TODO What sigma???
  int minPoints = 9;                        // Minimum points for a plane (16)
  planeFitter = new surface::MoSPlanes3D(
      surface::MoSPlanes3D::Parameter(pyrLevels, nbDist, thrAngleNormalClustering, inlDist, sigma, minPoints,
      pclA::NormalsEstimationNR::Parameter(5, nbDist, 1000, 0.001, 5, 0.001),
      surface::GreedySelection::Parameter(100.,1.,0.005) ) //10 .5 
  );
  
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
  annotation = new anno::Annotation();
//   annotation->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 0, 16);
//   annotation->init("/media/Daten/Object-Database/annotation/box_world_fi%1d.png", 0, 8);
  annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 0, 23);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_mixed_fi%1d.png", 0, 8);
  /// eval svm
//   annotation->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 17, 30);
//   annotation->init("/media/Daten/Object-Database/annotation/box_world_fi%1d.png", 9, 15);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 10, 23);
//   annotation->init("/media/Daten/Object-Database/annotation/cvww_mixed_fi%1d.png", 0, 8);
  
  
//   annotation->init("/media/Daten/Object-Database/annotation/texture_box%1d.png", 0, 3);

  /// init patch class
  patches = new surface::Patches();
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
  while(single && isRunning() && !stopComponent){
    SingleShotMode();
  }
  while(isRunning() && !stopComponent) {
    processImageNew();
  }

  if(showImages)
  {
    cvReleaseImage(&iplImage_k);
    log("destroy openCV windows.");
    cvDestroyWindow("Kinect image");
  }
  log("windows destroyed");
  log("deleted components: runComponent ended.");
}

  
 
/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void SegLearner::GetImageData()
{
  pointCloudWidth = 640;
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
  if(deb) log("Get image data with size: %u-%u", pointCloudWidth, pointCloudHeight);
  
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);            // call get points only once, if noCont option is on!!! (false means no transformation!!!)
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true);
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);

  // get rectified kinect image from point cloud server
  getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
  iplImage_k = convertImageToIpl(image_k);

  /// calculate normals
  static struct timespec start, last, current;
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  if(deb) last = start;
  pclA::NormalsEstimationNR::Parameter param(5, 0.02, 1000, 0.001, 10);
  pclA::NormalsEstimationNR n;
  n.setParameter(param);
  n.setInputCloud(pcl_cloud);
  n.compute();
  n.getNormals(pcl_normals);
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Getting images => Calculate normals %4.3f\n", timespec_diff(&current, &last));
  
  // bilateral filter
//   bilateral->setInputCloud(pcl_cloud);
//   bilateral->compute();
//   bilateral->getCloud(pcl_cloud);
  
  // subsample point cloud
//   subsample->setInputCloud(pcl_cloud);
//   subsample->compute();
//   subsample->getCloud(pcl_cloud);
//   pointCloudWidth = pointCloudWidth/2.;
//   pointCloudHeight = pointCloudWidth *3/4;
 
  if(showImages)
    cvShowImage("Kinect image", iplImage_k);
}

/**
 *  @brief Process data from stereo or Kinect.
 */
void SegLearner::processImageNew()
{
  surfaces.clear();
  
  static struct timespec start, last, current;
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  if(deb) last = start;

  /// Get kinect data
  GetImageData();
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: Getting images (not correct): %4.3f", timespec_diff(&current, &last));
  if(deb) last = current;  
  
  /// Run vision core (for canny edges
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
  GetSegmentIndexes(vcore, texture, pointCloudWidth);

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: Vision core: %4.3f", timespec_diff(&current, &last));
  if(deb) last = current;  
  
  /// ModelFitter
//   if(deb) log("ModelFitter start!");
  std::vector<int> pcl_model_types;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  std::vector< std::vector<double> > error;
//   std::vector<double> square_error;
//   model_fitter->addModelType(pcl::SACMODEL_PLANE);
// //  model_fitter->addModelType(pcl::SACMODEL_NORMAL_PLANE);
// //  model_fitter->addModelType(pcl::SACMODEL_CYLINDER);
// //  model_fitter->addModelType(pcl::SACMODEL_SPHERE);
//   model_fitter->setNormals(pcl_normals);
//   model_fitter->useDominantPlane(true);
//   model_fitter->setInputCloud(pcl_cloud);
//   model_fitter->compute();
//   model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices);         /// TODO Eigentlich sollte man hier nur mehr surfaces kriegen? => Anzeige ändern, dann löschen
//   model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices_old);     /// TODO old nur für die Anzeige später!
//   model_fitter->getError(error, square_error);
//   if(deb) log("ModelFitter end!");
  
//   if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
//   if(deb) printf("Runtime for SegLearner: Model fitting: %4.3f\n", timespec_diff(&current, &last));
//   if(deb) last = current; 
  
  /// MOS-Plane fitting
  if(deb) log("MoS-Plane fitter start!");
//   pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud_filtered);
  pclA::FilterZ(pcl_cloud, 0.3, 1.5);      // z filtering for 1.5 meters
  planeFitter->setInputCloud(pcl_cloud);
  planeFitter->compute();
  planeFitter->getSurfaceModels(surfaces);
  planeFitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices);
  planeFitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices_planes);
  planeFitter->getError(error);

// printf("Surfaces:\n");
// for(unsigned i=0; i<surfaces.size(); i++)
//   printf(" model %u: size: %lu\n", i, surfaces[i]->indices.size());

  printf("planeFitter->postprocess: start with %lu (%lu) models\n", surfaces.size(), pcl_model_types.size());
  postProcessIndices.clear();
  planeFitter->postprocessResults(postProcessIndices);
  printf("planeFitter->postprocess: end: found indices: %u\n", postProcessIndices.size());

// printf("Post-surfaces:\n");
// for(unsigned i=0; i<surfaces.size(); i++)
//   printf(" model %u: size: %lu\n", i, surfaces[i]->indices.size());
  if(deb) log("MoS-Plane fitter end: Found %lu models.", surfaces.size());


  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: MoS plane fitting: %4.3f", timespec_diff(&current, &last));
  if(deb) last = current; 
  
  /// NURBS-Fitting and model selection
  if(deb) log("NURBS-Fitting start!");
  modeling->setInputCloud(pcl_cloud);
  modeling->setInputPlanes(pcl_model_types, model_coefficients, pcl_model_indices, error);
  modeling->compute();
  modeling->getSurfaceModels(surfaces);
  modeling->getResults(pcl_model_types, pcl_model_indices, error);      // TODO Nur für Anzeige notwendig!
  if(deb) log("NURBS-Fitting end: size of surfaces: %u", surfaces.size());

  /// Check if we have line-models in the point cloud
  printf("planeFitter->checkPCLines: start with %lu (%lu) models\n", surfaces.size(), pcl_model_types.size());
  planeFitter->checkPCLines(surfaces, checkPCLines);
  printf("planeFitter->checkPCLines: end\n");


  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: NURBS & MODEL-SELECTION: %4.3f", timespec_diff(&current, &last));
  if(deb) last = current; 

  /// Load annotation from file
  if(deb) log("Annotation for 1st level: start");
  std::vector< std::vector<int> > anno_pairs;
  std::vector<int> anno_background_list;
  annotation->load(pointCloudWidth, anno, true);            /// TODO TODO Das ist überflüssig - Könnte intern aufgerufen werden
  annotation->setSurfaceModels(surfaces);
  annotation->calculate();
  annotation->getResults(nr_anno, anno_pairs, anno_background_list);
  for(unsigned i=0; i<anno_pairs.size(); i++) {
    printf("Annotation pairs for %u: ", i);
    for(unsigned j=0; j<anno_pairs[i].size(); j++)
      printf(" %u", anno_pairs[i][j]);
    printf("\n");
  }
  if(deb) log("Annotation for 1st level: end");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: Annotation calculation: %4.3f", timespec_diff(&current, &last));
  if(deb) last = current; 

  /// Calculate patch relations
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_copy (new pcl::PointCloud<pcl::PointXYZRGB>);         ///< PCL point cloud
  pcl::copyPointCloud(*pcl_cloud, *pcl_cloud_copy);
  
  if(deb) log("Calculate patch-relations start!");
  std::vector<Relation> relation_vector;
  patches->setInputImage(iplImage_k);
  patches->setInputCloud(pcl_cloud_copy);                   /// TODO We use a copy to avoid changes!!!
  patches->setNormals(pcl_normals);
  patches->setSurfaceModels(surfaces);
  patches->setAnnotion(anno_pairs, anno_background_list);
  patches->setTexture(texture);
  patches->setOptimalPatchModels(true);                     /// TODO Do we really have the projected normals? Also for NURBS???
  patches->computeLearnRelations();

  if(deb) log("Calculate patch-relations for 2nd SVM: start!");
  patches->computeLearnRelations2();
  if(deb) log("Calculate patch-relations for 2nd SVM: end!");

  patches->getRelations(relation_vector);

  pcl_normals_repro.reset(new pcl::PointCloud<pcl::Normal>);
  pcl_normals_repro->points.resize(pcl_normals->points.size());
  patches->getOutputCloud(pcl_model_cloud, pcl_normals_repro);
  if(deb) log("Calculate patch-relations ended!");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: Calculate patch relations: %4.3f", timespec_diff(&current, &last));
  if(deb) last = current;  

  /// write svm-relations for first level svm to file!
  if(deb) log("write svm learn file: start.");
  svm->setRelations(relation_vector);
  svm->setAnalyzeOutput(false);
  svm->process();
  if(deb) log("write svm learn file: end.");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) log("Runtime for SegLearner: Overall processing time: %4.3f", timespec_diff(&current, &start));
  
  cv::waitKey(500);   // wait for images on opencv windows (when not single-shot-mode
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
 *
 *   0 ... Show surface mesh models 
 *   
 *   x ... stop single shot mode
 *   s ... 
 *   
 */
void SegLearner::SingleShotMode()
{
  static bool repro = true;

  sleepComponent(10);
  int key = 0;
  key = cvWaitKey(20);
  
  if (key == 65471 || key == 1114047)  { // F2
    log("unused");
  }

  if (key == 65472 || key == 1114048)  { // F3
    const char* text = vcore->GetGestaltListInfo();
    printf("\n%s\n", text);
  }

  if (key == 65473 || key == 1114049)  { // F4
    log("unused");
  }

  if (key == 65478 || key == 1114054)  { // F9
    log("process image in single shot mode with new implementations.");
    lockComponent();
    processImageNew();
    unlockComponent();
  }

  if (key == 65479 || key == 1114055)  { // F10
    log("unused");
  }

  // if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
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
      
      
    // *** For new processing!!! *** //
    case '3':
      log("Show results from model fitter (or MoSPlanes3D.");
      tgRenderer->ClearModels();
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        cv::Vec4f planeCenter3D[pcl_model_indices_planes.size()];
        RGBValue col[pcl_model_indices_planes.size()];
        for(unsigned i=0; i<pcl_model_indices_planes.size(); i++) {
          col[i].float_value = GetRandomColor();
          for(unsigned j=0; j<pcl_model_indices_planes[i]->indices.size(); j++) {
            cv::Vec4f pt;
            pt[0] = pcl_cloud->points[pcl_model_indices_planes[i]->indices[j]].x;
            pt[1] = pcl_cloud->points[pcl_model_indices_planes[i]->indices[j]].y;
            pt[2] = pcl_cloud->points[pcl_model_indices_planes[i]->indices[j]].z;
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
        if(labels) {
          for(size_t i=0; i<pcl_model_indices_planes.size(); i++) {
            char label[5];
            snprintf(label, 5, "%lu", i);
            tgRenderer->AddLabel3D(label, 14, 
                                    planeCenter3D[i][0]/pcl_model_indices_planes[i]->indices.size(), 
                                    planeCenter3D[i][1]/pcl_model_indices_planes[i]->indices.size(), 
                                    planeCenter3D[i][2]/pcl_model_indices_planes[i]->indices.size());
          }
        }
        tgRenderer->Update();
      }
      break; 

    case '4':
      log("Show postprocess points (of MoSPlanes3D).");
      tgRenderer->ClearModels();
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        RGBValue col;
        col.float_value = GetRandomColor();
        for(unsigned i=0; i<postProcessIndices.size(); i++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[postProcessIndices[i]].x;
          pt[1] = pcl_cloud->points[postProcessIndices[i]].y;
          pt[2] = pcl_cloud->points[postProcessIndices[i]].z;
          pt[3] = col.float_value;
          printf("Draw a postProcessIndices point [u]: %3.2f-%3.2f-%3.2f\n", postProcessIndices[i], pt[0], pt[1], pt[2]);
          col_points.push_back(pt);
        }
        printf("col_points.size: %lu\n", col_points.size());
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);
        tgRenderer->Update();
      }
      break; 
      
    case '5':
      log("Show checkPCLines points (of MoSPlanes3D).");
      tgRenderer->ClearModels();
      if(showImages)
      {
        std::vector<cv::Vec4f> col_points;
        RGBValue col;
        col.float_value = GetRandomColor();
        for(unsigned i=0; i<checkPCLines.size(); i++) {
          cv::Vec4f pt;
          pt[0] = pcl_cloud->points[checkPCLines[i]].x;
          pt[1] = pcl_cloud->points[checkPCLines[i]].y;
          pt[2] = pcl_cloud->points[checkPCLines[i]].z;
          pt[3] = col.float_value;
          col_points.push_back(pt);
        }
        tgRenderer->Clear();
        tgRenderer->AddPointCloud(col_points);
        tgRenderer->Update();
      }
      break; 

    case '6':
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
        if(labels) {
          for(size_t i=0; i<pcl_model_indices.size(); i++) {
            char label[5];
            snprintf(label, 5, "%lu", i);
            tgRenderer->AddLabel3D(label, 14, 
                                  center3D[i][0]/pcl_model_indices[i]->indices.size(), 
                                  center3D[i][1]/pcl_model_indices[i]->indices.size(), 
                                  center3D[i][2]/pcl_model_indices[i]->indices.size());
          }
        }
        tgRenderer->Update();
      }
      break;
      
    case '7':
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
        if(labels) {
          for(size_t i=0; i<pcl_model_indices.size(); i++) {
            char label[5];
            snprintf(label, 5, "%lu", i);
            tgRenderer->AddLabel3D(label, 14, 
                                    planeCenter3D[i][0]/pcl_model_indices[i]->indices.size(), 
                                    planeCenter3D[i][1]/pcl_model_indices[i]->indices.size(), 
                                    planeCenter3D[i][2]/pcl_model_indices[i]->indices.size());
          }
        }
        tgRenderer->Update();
      }
      break; 
      
    case  '0':
    {
      log("Show mesh of surfaces. wait ...");
        if(surfaces.size() != 0)
          if(surfaces[0]->mesh.m_vertices.empty())
          {
            surface::CreateMeshModel createMesh(surface::CreateMeshModel::Parameter(.1));
            createMesh.setInputCloud(pcl_cloud);
            createMesh.compute(surfaces);
          }
        tgRenderer->Clear();
        tgRenderer->ClearModels();
        for (unsigned i=0; i<surfaces.size(); i++)
          tgRenderer->AddModel(&surfaces[i]->mesh);
        tgRenderer->Update();
      log("Show mesh of surfaces. done.");
      break;
    }
    

    case 's':
    {
      log("Save results into pcd-file: Time to implement! NOW!");
    }   
    break;
      
    case 'x':
      log("End Single-Shot mode!");
      single = false;
      break;
      
    case '^':
      log("Add labels: antiquated");
      labels = !labels;
      break;     
      
     case 'q':
      log("Stop component.");
      stopComponent = true;
      break;  
  }
}

}






