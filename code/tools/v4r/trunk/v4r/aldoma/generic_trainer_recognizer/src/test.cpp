#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

#include "trainers/roll_trainer.h"
#include "recognizers/roll_recognizer.h"

#include "pcl/visualization/pcl_visualizer.h"
#include "estimators/cvfh_estimator.h"
#include "estimators/vfh_estimator.h"
#include "estimators/D2CR_estimator.h"
//#include "estimators/spin_image_estimator.h"
#include "estimators/dummy_estimator.h"
/*#include "estimators/D2Multi_estimator.h"
#include "estimators/geodesic_estimator.h"*/

#include "generic_trainer_recognizer/utils.h"

using namespace pcl;

std::string MODELS_DIR_;
std::string TRAINING_DIR_;
float model_scale_factor_;
int numberNN_;
float Z_DIST_;

/*void
D2MultiConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{
  D2MultiScaleEstimator<PointXYZ, PointNormal, pcl::Histogram<480> > * D2Multi_estimation = new D2MultiScaleEstimator<
      PointXYZ, PointNormal, pcl::Histogram<480> > (true, true, 0.005, 90);

  RollTrainer<PointXYZ, PointNormal, pcl::Histogram<480> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (D2Multi_estimation);
  trainer.train ();

  RollRecognizer<flann::HistIntersectionDistance, PointXYZ, PointNormal, pcl::Histogram<480> > recognizer;
  recognizer.setApplyVoxelGrid (false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (D2Multi_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualize<flann::HistIntersectionDistance, PointXYZ, PointNormal, pcl::Histogram<480> > (cloud, recognizer, Z_DIST_, MODELS_DIR_, model_scale_factor_);
}*/

void
D2CRConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{

  boost::shared_ptr<RollEstimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Roll_Estimation;
  D2CR_Roll_Estimation.reset(new RollEstimator<PointXYZ, PointNormal,
                          pcl::Histogram<416> > (true, true, 0.005, 90) );

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
  D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal,
                         pcl::Histogram<416> > () );

  D2CR_Roll_Estimation->setEstimator(D2CR_Estimation);

  //cast to estimator and pass it to trainer and recognizer
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > Estimation;
  Estimation.reset(D2CR_Roll_Estimation.get());

  RollTrainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (Estimation);
  trainer.setDescriptorName("D2CR");
  trainer.train ();

  RollRecognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (Estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("D2CR");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualize<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, recognizer, Z_DIST_, MODELS_DIR_, model_scale_factor_, 0.0001);
}

void
D2CRConfigurationWithoutPose (PointCloud<PointXYZ>::Ptr & cloud)
{
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
    D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > ());

  Trainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (D2CR_Estimation);
  trainer.setDescriptorName("D2CR");
  trainer.train ();

  //Recognizer<flann::HistIntersectionDistance, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (D2CR_Estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setComputePose(false);
  recognizer.setDescriptorName("D2CR");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualizeNearestNeighbors<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, recognizer, Z_DIST_, MODELS_DIR_, numberNN_, 0.0001);
}

void
VFHConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{
  /*VFHEstimator<PointXYZ, PointNormal, VFHSignature308> * vfh_estimation = new VFHEstimator<PointXYZ, PointNormal,
      VFHSignature308> (true, true, 0.005);

  RollEstimator<PointXYZ, PointNormal, VFHSignature308> * roll_vfh_estimation = new RollEstimator<PointXYZ, PointNormal,
        VFHSignature308> (true, true, 0.005, 90);
  roll_vfh_estimation->setEstimator(vfh_estimation);*/

  boost::shared_ptr<RollEstimator<PointXYZ, PointNormal, VFHSignature308 > > roll_vfh_estimation;
  roll_vfh_estimation.reset(new RollEstimator<PointXYZ, PointNormal,VFHSignature308> (true, true, 0.005, 90));

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > vfh_estimation;
  vfh_estimation.reset(new VFHEstimator<PointXYZ, PointNormal,
                          VFHSignature308> (true, true, 0.005) );

  roll_vfh_estimation->setEstimator(vfh_estimation);

  //cast to estimator and pass it to trainer and recognizer
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > Estimation;
  Estimation.reset(roll_vfh_estimation.get());

  RollTrainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (Estimation);
  trainer.setDescriptorName("VFH");
  trainer.train ();

  RollRecognizer<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (Estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("VFH");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualize<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> (cloud, recognizer, Z_DIST_,
                                                                                           MODELS_DIR_, model_scale_factor_);
}

void
VFHConfigurationWithoutPose (PointCloud<PointXYZ>::Ptr & cloud)
{
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > vfh_estimation;
  vfh_estimation.reset(new VFHEstimator<PointXYZ, PointNormal,
                          VFHSignature308> (true, true, 0.005) );

  typedef VFHEstimator<PointXYZ, PointNormal,
      VFHSignature308> VFHEstimator;

  boost::shared_ptr<VFHEstimator> vfh_estimator =
     boost::dynamic_pointer_cast<VFHEstimator>(vfh_estimation);

  vfh_estimator->setComputeMeshResolution(true);

  Trainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (vfh_estimation);
  trainer.setDescriptorName("VFH");
  trainer.setNosify(false);
  trainer.setRadiusVirtualcamera(1.5);
  trainer.train ();

  //Recognizer<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  Recognizer<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  recognizer.setComputePose(false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (vfh_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("VFH");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  //recognizeAndVisualizeNearestNeighbors<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> (cloud, recognizer);
  recognizeAndVisualizeNearestNeighbors<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> (cloud, recognizer, Z_DIST_, MODELS_DIR_, numberNN_, 0.005);
}

void
CVFHConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{

  boost::shared_ptr<RollEstimator<PointXYZ, PointNormal, VFHSignature308 > > roll_cvfh_estimation;
  roll_cvfh_estimation.reset(new RollEstimator<PointXYZ, PointNormal,VFHSignature308> (true, true, 0.005, 90));

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > cvfh_estimation;
  cvfh_estimation.reset(new CVFHEstimator<PointXYZ, PointNormal,
                          VFHSignature308> (true, true, 0.005) );

  roll_cvfh_estimation->setEstimator(cvfh_estimation);

  //cast to estimator and pass it to trainer and recognizer
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > Estimation;
  Estimation.reset(roll_cvfh_estimation.get());

  RollTrainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (Estimation);
  trainer.setDescriptorName("CVFH");
  trainer.train ();

  RollRecognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (Estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("CVFH");
  recognizer.setTesselationLevel(1);

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualize<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> (cloud,
                                                                                                         recognizer, Z_DIST_, MODELS_DIR_, model_scale_factor_,0.005);
}

void
CVFHConfigurationWithoutPose (PointCloud<PointXYZ>::Ptr & cloud)
{
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > cvfh_estimation;
  cvfh_estimation.reset(new CVFHEstimator<PointXYZ, PointNormal,
                       VFHSignature308> (true, true, 0.005) );

  typedef CVFHEstimator<PointXYZ, PointNormal,
      VFHSignature308> CVFHEstimatorT;

  boost::shared_ptr<CVFHEstimatorT> cvfh_estimator =
     boost::dynamic_pointer_cast<CVFHEstimatorT>(cvfh_estimation);

  cvfh_estimator->setComputeMeshResolution(true);
  cvfh_estimator->setLeafSizeFactor(3);
  cvfh_estimator->setEpsAngleThreshold(0.13);
  cvfh_estimator->setCurvatureThreshold(0.035);
  cvfh_estimator->setNormalizeBins(true);

  Trainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (cvfh_estimation);
  trainer.setDescriptorName("CVFH-noise");
  trainer.setNosify(true);
  trainer.setNoiseLevel(0.0055);
  trainer.setRadiusVirtualcamera(1.5);
  trainer.train ();

  //Recognizer<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  Recognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> recognizer(false);
  recognizer.setComputePose(false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (cvfh_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("CVFH-noise");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  //recognizeAndVisualizeNearestNeighbors<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> (cloud, recognizer, Z_DIST_, MODELS_DIR_, numberNN_, 0.005);
  recognizeAndVisualizeNearestNeighbors<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> (cloud, recognizer, Z_DIST_, MODELS_DIR_, numberNN_, 0.005);
}

/*void
SpinImageConfigurationWithoutPose (PointCloud<PointXYZ>::Ptr & cloud)
{
  SpinImageEstimator<PointXYZ, PointNormal, pcl::Histogram<153> > * si_estimation = new SpinImageEstimator<PointXYZ, PointNormal,
      pcl::Histogram<153> > ();

  Trainer<PointXYZ, PointNormal, pcl::Histogram<153> > trainer (MODELS_DIR_, TRAINING_DIR_);
  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (si_estimation);
  trainer.setDescriptorName("SpinImage");
  trainer.setNosify(false);
  trainer.setRadiusVirtualcamera(1.5);
  trainer.train ();

  //Recognizer<flann::ChiSquareDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  Recognizer<flann::L2, PointXYZ, PointNormal, pcl::Histogram<153> > recognizer;
  recognizer.setComputePose(false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (si_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("SpinImage");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualizeNearestNeighbors<flann::L2, PointXYZ, PointNormal, pcl::Histogram<153> > (cloud, recognizer, Z_DIST_, MODELS_DIR_, numberNN_, 0.005);
}*/

/*void
GeodesicConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{
  GeodesicEstimator<PointXYZ, PointNormal, VFHSignature308> * geodesic_estimation = new GeodesicEstimator<PointXYZ,
      PointNormal, VFHSignature308> (true, true, 0.005, 90);

  RollTrainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (geodesic_estimation);
  trainer.train ();

  //RollRecognizer<flann::HistIntersectionDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  RollRecognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  recognizer.setApplyVoxelGrid (false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (geodesic_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  //recognizeAndVisualize<flann::HistIntersectionDistance, PointXYZ, PointNormal, VFHSignature308>(cloud, recognizer);
  recognizeAndVisualize<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> (cloud,
                                                                                                         recognizer, Z_DIST_, MODELS_DIR_, model_scale_factor_);
}*/

void
DummyConfiguration (PointCloud<PointXYZ>::Ptr & cloud)
{

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
  D2CR_Estimation.reset(new DummyEstimator<PointXYZ, PointNormal,pcl::Histogram<416> > () );

  Trainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (D2CR_Estimation);
  trainer.setDescriptorName("Dummy");
  trainer.setNosify(false);
  trainer.setRadiusVirtualcamera(1.5);
  trainer.train ();

  /*RollRecognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (D2CR_Roll_Estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setDescriptorName("Dummy");

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  recognizeAndVisualize<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, recognizer, Z_DIST_, MODELS_DIR_, model_scale_factor_, 0.0001);*/
}


int
main (int argc, char ** argv)
{

  MODELS_DIR_ = argv[1];
  TRAINING_DIR_ = argv[2];
  model_scale_factor_ = atof(argv[3]);
  std::string descriptor_str(argv[4]);
  numberNN_ = atoi(argv[5]);
  Z_DIST_ = 1.5;
  //load PointCloud, segment it and recognize it
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  io::loadPCDFile (argv[6], *cloud);

  if (descriptor_str == "cvfh")
  {
    CVFHConfiguration (cloud);
  }
  else if (descriptor_str == "cvfh_nopose")
  {
    CVFHConfigurationWithoutPose (cloud);
  }
  else if (descriptor_str == "vfh")
  {
    VFHConfiguration (cloud);
  }
  else if (descriptor_str == "vfh_nopose")
  {
    VFHConfigurationWithoutPose (cloud);
  }
  else if (descriptor_str == "D2CR")
  {
    D2CRConfiguration (cloud);
  }
  else if (descriptor_str == "D2CR_nopose")
  {
    D2CRConfigurationWithoutPose (cloud);
  } /*else if (descriptor_str == "SI_nopose") {
    SpinImageConfigurationWithoutPose (cloud);
  }
  else if (descriptor_str == "D2MultiScale")
  {
    D2MultiConfiguration (cloud);
  }
  else if (descriptor_str == "geodesic")
  {
    GeodesicConfiguration (cloud);
  }*/
  else {
    DummyConfiguration(cloud);
  }
}



/*class OpenNIGrabFrame
{
public:
  OpenNIGrabFrame () :
    no_frame (true)
  {
  }

  void
  cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    if (no_frame)
    {

      PointCloud<PointXYZRGB>::Ptr grid (new PointCloud<PointXYZRGB> (*cloud));

      visualization::PCLVisualizer vis4 ("scene grid");
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr handler;
      handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (grid));

      vis4.addPointCloud<pcl::PointXYZRGB> (grid, *handler, "name");
      vis4.spin ();

      //CVFHConfiguration (grid, atoi (argv[2]));
      no_frame = false;
    }
  }

  template<template<typename > class Storage>
    void
    cloud_cb_image (const boost::shared_ptr<openni_wrapper::Image>& image,
                    const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float constant)
    {

      if (no_frame)
       {
       static int smoothing_nr_iterations = 1;
       static int smoothing_filter_size = 1;

       pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
       typename pcl::cuda::PointCloudAOS<Storage>::Ptr data;

       // Compute the PointCloud on the device
       {
       pcl::cuda::ScopeTimeCPU time ("disparity smoothing");

       pcl::cuda::DisparityToCloud d2c;
       d2c.compute<Storage> (depth_image, image, constant, data, true, 2, smoothing_nr_iterations,
       smoothing_filter_size);

       output.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::cuda::toPCL (*data, *output);
       }

       visualization::PCLVisualizer vis4 ("scene grid");
       pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr handler;
       handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (output));

       vis4.addPointCloud<pcl::PointXYZRGB> (output, *handler, "name");
       vis4.spin ();

       no_frame = true;

       CVFHConfiguration (output, atoi (argv[2]));
       }

    }

  void
  run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber ();

    bool use_images = true;

    if (use_images)
    {
      boost::function<void
      (const boost::shared_ptr<openni_wrapper::Image>& image,
       const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)> f =
          boost::bind (&OpenNIGrabFrame::cloud_cb_image<pcl::cuda::Device>, this, _1, _2, _3);

      boost::signals2::connection c = interface->registerCallback (f);

    }
    else
    {
      // make callback function from member function
      boost::function<void
      (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&OpenNIGrabFrame::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);
    }

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (no_frame)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
  bool no_frame;
  int argc;
  char ** argv;
};*/
