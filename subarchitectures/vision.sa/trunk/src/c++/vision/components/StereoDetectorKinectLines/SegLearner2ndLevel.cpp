/**
 * @file SegLearner2ndLevel.cpp
 * @author Andreas Richtsfeld
 * @date 2011
 * @version 0.1
 * @brief Get properties to learn how to segment.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SegLearner2ndLevel.h"

using namespace std;
using namespace VisionData;
using namespace Video;

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::SegLearner2ndLevel();
  }
}

namespace cast
{

// **************************** SegLearner2ndLevel **************************** //
/**
 * @brief Called by the framework to configure the component.
 * @param _config Configuration
 */
void SegLearner2ndLevel::configure(const map<string,string> & _config)
{
  deb = true; // debug flag
  
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
//     tgRenderer->SetClearColor(1., 1., 1.); /// Set color to white
//     tgRenderer->SetCoordinateFrame();
  }
  else 
    cvShowImage("Control window", iplImage_k);

  
  /// ################### The new classes for calculation ################### ///
  // init bilateral filter
  //bilateral = new pclA::BilateralFilter(pclA::BilateralFilter::Parameter());
  
  // init subsample-tool for point cloud
  //subsample = new pclA::SubsamplePointCloud(pclA::SubsamplePointCloud::Parameter(2, .02, true, false, false));
      
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
  double ec_cluster_tolerance = 0.01; //0.008,  /// Calculate tolerance like sac_optimal_weight_factor?
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
  float inlDist = 0.012;                     // Inlier distance for planes
  float sigma = 0.008;                      // TODO What sigma???
  int minPoints = 16;                       // Minimum points for a plane (16)
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
  
  /// init annotation for both levels
  annotation_1 = new anno::Annotation();
  annotation_2 = new anno::Annotation();

  /// 1st-level annotation
//   annotation_1->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 0, 16);
//   annotation_1->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 0, 9);
  annotation_1->init("/media/Daten/Object-Database/annotation/box_world_fi%1d.png", 0, 15);
//   annotation_1->init("/media/Daten/Object-Database/annotation/ocl_boxes%1d_fi.png", 17, 30);
//   annotation_1->init("/media/Daten/Object-Database/annotation/cvww_cyl_fi%1d.png", 10, 23);
//   annotation_1->init("/media/Daten/Object-Database/annotation/cvww_mixed_fi%1d.png", 0, 8);
//   annotation_1->init("/media/Daten/Object-Database/annotation/texture_box%1d.png", 0, 3);

  /// 2nd level annotation
  annotation_2->init("/media/Daten/Object-Database/annotation/box_world%1d.png", 0, 15);
  
  // init patch class
  patches = new surface::Patches();
  patches->setZLimit(0.01);
  
  /// init first level svm-predictor
  std::vector<const char*> files;
  const char* file = "./instantiations/11-05-11/1215/PP-Trainingsset.txt.scaled.model";
  files.push_back(file);
  svmPredictor = new svm::SVMPredictor(files);
  svmPredictor->setScaling(true, "./instantiations/11-05-11/1215/param.txt");
  
  /// init first level graph cutter ??? => TODO Brauchen wir den hier schon?
  graphCut = new gc::GraphCut();
  
  /// init 2nd level svm-file-creator
  svm = new svm::SVMFileCreator();
}


/**
 * @brief Called by the framework after configuration, before run loop.
 */
void SegLearner2ndLevel::start()
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
void SegLearner2ndLevel::runComponent()
{ 
  while(single && isRunning()){
    SingleShotMode();
  }
  while(isRunning()) {
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
void SegLearner2ndLevel::GetImageData()
{
  pointCloudWidth = 640;
  pointCloudHeight = pointCloudWidth *3/4;
  kinectImageWidth = 640;
  kinectImageHeight = kinectImageWidth *3/4;
 
  points.resize(0);
  getCompletePoints(false, pointCloudWidth, points);            // call get points only once, if noCont option is on!!! (false means no transformation!!!)
  
  ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth, pointCloudHeight, true);
  pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, pcl_cloud);

  // get rectified images from point cloud server
  getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
  iplImage_k = convertImageToIpl(image_k);

  // calculate normals
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
 
  if(showImages)
    cvShowImage("Kinect image", iplImage_k);
}

/**
 *  @brief Process data from stereo or Kinect.
 */
void SegLearner2ndLevel::processImageNew()
{
  surfaces.clear();
  
  static struct timespec start, last, current;
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  if(deb) last = start;

  /// Get kinect data
  GetImageData();
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Getting images: (this is WRONG) %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  
  
  /// Run vision core (for canny edges
  vcore->NewImage(iplImage_k);
  vcore->ProcessImage(runtime, cannyAlpha, cannyOmega);  
  GetSegmentIndexes(vcore, texture, pointCloudWidth);

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Vision core: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  
  
  /// ModelFitter
  if(deb) log("ModelFitter start!");
  std::vector<int> pcl_model_types;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  std::vector< std::vector<double> > error;
  std::vector<double> square_error;
  model_fitter->addModelType(pcl::SACMODEL_PLANE);
  model_fitter->setNormals(pcl_normals);
  model_fitter->useDominantPlane(false);
  model_fitter->setInputCloud(pcl_cloud);
  model_fitter->compute();
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices);         /// TODO Eigentlich sollte man hier schon surfaces kriegen! Altes Zeugs raus!
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_indices_old);     /// TODO old nur für die Anzeige später!
  model_fitter->getError(error, square_error);
  if(deb) log("ModelFitter end!");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Model fitting: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 
  
  /// NURBS-Fitting and model selection
  if(deb) log("NURBS-Fitting start!");
  modeling->setInputCloud(pcl_cloud);
  modeling->setInputPlanes(pcl_model_types, model_coefficients, pcl_model_indices, error);
  modeling->compute();
  modeling->getSurfaceModels(surfaces);
  modeling->getResults(pcl_model_types, pcl_model_indices, error);      // TODO Nur für Anzeige notwendig!
  if(deb) log("NURBS-Fitting end: size of surfaces: %u", surfaces.size());

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: NURBS & MODEL-SELECTION: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 

  /// Annotation
  if(deb) log("Annotation_2: start");
  std::vector< std::vector<int> > anno_pairs2;
  std::vector<int> anno_background_list2;
  annotation_2->load(pointCloudWidth, anno, true);                      /// TODO Das ist überflüssig - Könnte intern aufgerufen werden
  annotation_2->setSurfaceModels(surfaces);
  annotation_2->calculate();
  annotation_2->getResults(nr_anno, anno_pairs2, anno_background_list2);
  for(unsigned i=0; i<anno_pairs2.size(); i++) {
    printf("Annotation2 pairs for %u: ", i);
    for(unsigned j=0; j<anno_pairs2[i].size(); j++)
      printf(" %u", anno_pairs2[i][j]);
    printf("\n");
  }
  if(deb) log("Annotation_2: end");
  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Annotation_1 calculation: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current; 

  /// Calculate patch relations => Do 1st-level svm + graphCut
  if(deb) log("Calculate 1st-level patch-relations start!");
  std::vector<Relation> relation_vector;
  patches->setInputImage(iplImage_k);
  patches->setInputCloud(pcl_cloud);
  patches->setNormals(pcl_normals);
  patches->setSurfaceModels(surfaces);
//   patches->setAnnotion(anno_pairs, anno_background_list);
  patches->setTexture(texture);
  patches->setOptimalPatchModels(true);
  patches->computeTestRelationsOld();                 // TODO We use the old 1st-level test-relations
  patches->getRelations(relation_vector);
  patches->getOutputCloud(pcl_cloud, pcl_normals);    // pcl_cloud_vis, pcl_normals_vis
  patches->getSurfaceModels(surfaces);
  if(deb) log("Calculate 1st-level patch-relations end!");

  if(deb) log("1st-level svm-predictor: start");
  if(deb) printf("SegTester: Prediction start: relation_vector.size: %lu\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++) {
    relation_vector[i].prediction = svmPredictor->getResult(relation_vector[i].type, 
                                                            relation_vector[i].rel_value, 
                                                            relation_vector[i].rel_probability);
    if(deb) 
    {
      if(relation_vector[i].groundTruth == 0) {
        printf(" 1st-level relation [%u][%u]: gt: false => %4.3f", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
        if(relation_vector[i].prediction > 0.5)
          printf(" => is false\n");
        else
          printf(" => is true\n");
      }
      else if(relation_vector[i].groundTruth == 1) {
        printf(" 1st-level relation [%u][%u]: gt: true  => %4.3f", relation_vector[i].id_0, relation_vector[i].id_1, relation_vector[i].rel_probability[1]);
        if(relation_vector[i].prediction < 0.5)
          printf(" => is false\n");
        else
          printf(" => is true\n");      
      }
    }
  }
  if(deb) log("1st-level svm-predictor: end");

  /// Graph cutter
  if(deb) log("graph-cutter: start");
  int nr_models = surfaces.size();
  graphCut->init(relation_vector);
  graphCut->process();
  graphCut->getResults(nr_models, graphCutGroups);
  if(deb) log("graph-cutter: end");
  
printf("GraphCut groups:\n");
for(unsigned i=0; i<graphCutGroups.size(); i++) {
  printf("  %u: ", i);
  for(unsigned j=0; j<graphCutGroups[i].size(); j++)
    printf("%u ", graphCutGroups[i][j]);
  printf("  \n");
}
  
  
  /// TODO TODO TODO And now the second label TODO TODO TODO 
  printf(" ######################### 2nd level learning starts here #########################\n");
  
  /// And now do the 2nd-level svm training
  relation_vector.clear();
  patches->setAnnotion2(anno_pairs2, anno_background_list2);
  patches->computeLearnRelations2();
  patches->getRelations(relation_vector);
    
  printf("and now print the relations vector with size: %lu\n", relation_vector.size());
  for(unsigned i=0; i<relation_vector.size(); i++) {
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
  pcl_normals_repro.reset(new pcl::PointCloud<pcl::Normal>);
  pcl_normals_repro->points.resize(pcl_normals->points.size());
  patches->getOutputCloud(pcl_cloud, pcl_normals_repro);
  if(deb) log("Calculate patch-relations ended!");


  
  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Calculate patch relations: %4.3f\n", timespec_diff(&current, &last));
  if(deb) last = current;  

  
  
  /// write svm-relations to file!
  if(deb) log("create svm file for second level: start");
  svm->setRelations(relation_vector);
  svm->setAnalyzeOutput(true);
  svm->process();
  if(deb) log("create svm file for second level: done");

  if(deb) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
  if(deb) printf("Runtime for SegLearner: Overall processing time: %4.3f\n", timespec_diff(&current, &start));
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
 *   9 ... Show results of first level svm
 *   0 ... Show surface mesh models 
 *   
 *   x ... stop single shot mode
 *   s ... 
 *   
 */
void SegLearner2ndLevel::SingleShotMode()
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
      log("unused");
      break;
      
    case '3':
      log("unused");
      break;
     
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
      
    case '9':
      log("Show results after first level: Time to implement! NOW!");
      break; 
      
    case  '0':
    {
      log("Show mesh of surfaces. wait ...");
        surface::CreateMeshModel createMesh(surface::CreateMeshModel::Parameter(.1));
        createMesh.setInputCloud(pcl_cloud);
        createMesh.compute(surfaces);

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
  }
}

}






