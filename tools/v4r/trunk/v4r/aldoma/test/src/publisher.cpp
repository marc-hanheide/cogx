/*
 * publisher.cpp
 *
 *  Created on: Aug 27, 2011
 *      Author: aitor
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "pcl/apps/dominant_plane_segmentation.h"
#include "overlap.h"

#include "trainers/roll_trainer.h"
#include "recognizers/roll_recognizer.h"

#include "pcl/visualization/pcl_visualizer.h"
#include "estimators/cvfh_estimator.h"
#include "generic_trainer_recognizer/utils.h"
#include "generic_trainer_recognizer/openni_viewer.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  std::string MODELS_DIR_ = argv[1];  //"/home/aitor/data/CAD-net/learned_scaled_models";
  std::string TRAINING_DIR_ = argv[2]; //"/home/aitor/data/CAD-net/learned_scaled_models_trained_cvfh";
  std::string PATH_FOR_THE_MODELS_ = argv[3];
  float model_scale_factor_ = atof(argv[4]);

  boost::shared_ptr < RollEstimator<PointXYZ, PointNormal, VFHSignature308> > Estimation;
  Estimation.reset (new RollEstimator<PointXYZ, PointNormal, VFHSignature308> (true, true, 0.005, 90));

  boost::shared_ptr < Estimator<PointXYZ, PointNormal, VFHSignature308> > cvfh_estimation;
  cvfh_estimation.reset (new CVFHEstimator<PointXYZ, PointNormal, VFHSignature308> (true, true, 0.005));

  Estimation->setEstimator (cvfh_estimation);

  //cast to estimator and pass it to trainer and recognizer
  boost::shared_ptr < Estimator<PointXYZ, PointNormal, VFHSignature308> > roll_cvfh_estimation;
  roll_cvfh_estimation.reset (Estimation.get ());

  RollTrainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setRemoveDuplicateViews (false);
  trainer.setEstimator (roll_cvfh_estimation);
  trainer.train ();

  //load PointCloud, segment it and recognize it
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  if (argc <= 5)
  {
    std::string device_id ("");
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
    if (driver.getNumberDevices () > 0)
    {
      cout << "Device Id not set, using first device." << endl;
    }

    pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);

    typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;

    Cloud cloud_to_use_;
    unsigned mode;
    if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
    {
      SimpleOpenNIViewer<pcl::PointXYZRGB> v (grabber);
      v.run ();
      boost::mutex mutex_;

      {
        boost::mutex::scoped_lock lock (mutex_);
        cloud_to_use_ = v.cloud__;
      }

      pcl::copyPointCloud (cloud_to_use_, *cloud);
    }
    else
    {
      std::cout << "We cannot get an XYZRGB pointcloud..." << std::endl;
      return 0;
    }
  }
  else
  {
    io::loadPCDFile (argv[5], *cloud);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr grid (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::VoxelGrid < pcl::PointXYZ > grid_;
  grid_.setInputCloud (cloud);
  grid_.setLeafSize (0.005, 0.005, 0.005);
  grid_.filter (*grid);

  pcl::visualization::PCLVisualizer vis4 ("scene");
  vis4.addPointCloud (grid);

  //segmentation
  pcl::apps::DominantPlaneSegmentation < pcl::PointXYZ > dps;
  dps.setInputCloud (cloud);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr> >
                                                                                                                   clusters;
  dps.compute (clusters);
  std::cout << "Number of clusters:" << clusters.size () << std::endl;

  RollRecognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> recognizer;
  recognizer.setNumberOfModels (5);
  recognizer.setEstimator (roll_cvfh_estimation);
  recognizer.setModelsDirectory(MODELS_DIR_);
  recognizer.setICPIterations(100);

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  for (size_t i = 0; i < clusters.size (); i++)
  {

    std::stringstream cluster_name;
    cluster_name << "cluster_" << i;

    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler (clusters[i], 255, 0, 0);
    vis4.addPointCloud (clusters[i], handler, cluster_name.str ());

    std::vector<std::string> model_ids;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> confidences;

    recognizer.recognize (clusters[i], model_ids, poses, confidences);

    for(size_t j=0; j < 1; j++) {
      std::cout << model_ids[j] << std::endl;

      stringstream pathPly;
      pathPly << MODELS_DIR_ << "/" << model_ids[j] << ".ply";

      std::cout << pathPly.str () << std::endl;
      vtkSmartPointer < vtkTransform > trans = vtkSmartPointer<vtkTransform>::New ();
      trans->PostMultiply ();
      trans->Scale (model_scale_factor_, model_scale_factor_, model_scale_factor_);
      vtkSmartPointer < vtkTransform > poseTransform = vtkSmartPointer<vtkTransform>::New ();

      vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
      for(size_t kk=0; kk < 4; kk++) {
        for(size_t k=0; k < 4; k++) {
           mat->SetElement(kk,k,poses[j](kk,k));
        }
      }

      poseTransform->SetMatrix(mat);
      trans->Concatenate (poseTransform);
      trans->Modified ();

      std::stringstream model_name;
      model_name << "ply_model_" << i << "_" << j;
      //vis4.addModelFromPLYFile (pathPly.str (), trans, model_name.str());

      /////////////////////////////////////////////////////////////////////////////////////
      //////////////// Compute overlap between transformed model and cluster //////////////
      /////////////////////////////////////////////////////////////////////////////////////

      OverlapLikelihood overlap(PATH_FOR_THE_MODELS_);
      overlap.setModel(pathPly.str(),poses[j],model_scale_factor_);
      overlap.setPartialPointCloud(clusters[i]);
      overlap.compute();

      vtkSmartPointer<vtkPolyData> likelihoodData = vtkSmartPointer<vtkPolyData>::New();
      overlap.getOverlapLikelihoodPolydata(likelihoodData);

      std::stringstream path_stream;
      path_stream << PATH_FOR_THE_MODELS_ << "/test.iv";

      std::string path_iv = path_stream.str();
      std::cout << path_iv << std::endl;
      overlap.generateIVModel(path_iv);

      /*pcl::visualization::PCLVisualizer vis ("likelihoodData");
      vis.addModelFromPolyData(likelihoodData);
      pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_cloud (clusters[i], 255, 255, 255);
      vis.addPointCloud (clusters[i], handler_cloud);
      vis.spin ();*/

      vis4.addModelFromPolyData(likelihoodData,model_name.str());

    }
  }

  vis4.spin ();


  //PUBLISHING PART!!

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hello world " << count;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  chatter_pub.publish(msg);

  ros::spinOnce();

  return 0;
}
