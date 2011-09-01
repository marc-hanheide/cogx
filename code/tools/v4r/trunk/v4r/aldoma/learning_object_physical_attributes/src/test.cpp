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

#include "stable_planes.hpp"
#include "alignment/stable_planes_alignment.h"
#include "generic_trainer_recognizer/utils.h"

#include "vtkPLYReader.h"
#include "vtkPLYWriter.h"

#include "scale_distribution.h"
#include "openni_viewer.h"

using namespace pcl;
namespace bf = boost::filesystem;

std::string MODELS_DIR_;
std::string TRAINING_DIR_;
float model_scale_factor_;
int numberNN_;
float Z_DIST_;
std::string dir_to_save;

void
D2CRConfigurationWithoutPose (
                              PointCloud<PointXYZ>::Ptr & cloud,
                              std::vector<recognition_result> & results,
                              Eigen::Vector4f & table_coeffs,
                              std::vector<pcl::PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<
                                  PointXYZ>::Ptr> > & clusters)

{
  //D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > * D2CR_Estimation = new D2CR_Estimator<PointXYZ,
  //    PointNormal, pcl::Histogram<416> > ();

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
  D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > ());

  //boost::static_pointer_cast<>
  Trainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (D2CR_Estimation);
  trainer.setDescriptorName("D2CR");
  trainer.train ();

  //Recognizer<flann::HistIntersectionDistance, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer(false);
  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (D2CR_Estimation);
  recognizer.setModelsDirectory (MODELS_DIR_);
  recognizer.setComputePose (false);
  recognizer.setDescriptorName("D2CR");

  std::cout << "going to initialize" << std::endl;
  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  std::cout << "end initialize" << std::endl;

  recognizeAndVisualizeNearestNeighbors<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, recognizer,
                                                                                                 Z_DIST_, MODELS_DIR_,
                                                                                                 numberNN_, 0.002);

  recognizeAndReturnResults<flann::L1, PointXYZ, PointNormal, Histogram<416> > (cloud, recognizer, Z_DIST_,
                                                                                MODELS_DIR_, numberNN_, results,
                                                                                table_coeffs, clusters);

}

void
D2CRConfigurationInitRecognizer (Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > & recognizer)
{
  /*D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > * D2CR_Estimation = new D2CR_Estimator<PointXYZ,
        PointNormal, pcl::Histogram<416> > ();*/

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
    D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > ());

  Trainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);

  trainer.setModelScale (model_scale_factor_);
  trainer.setTesselationLevel (1);
  trainer.setEstimator (D2CR_Estimation);
  trainer.setDescriptorName("D2CR");
  trainer.train ();

  recognizer.setNumberOfModels (numberNN_);
  recognizer.setEstimator (D2CR_Estimation);
  recognizer.setModelsDirectory (MODELS_DIR_);
  recognizer.setComputePose (false);
  recognizer.setDescriptorName("D2CR");

  std::cout << "going to initialize" << std::endl;
  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }

  std::cout << "end initialize" << std::endl;
}

void
D2CRRecognize (PointCloud<PointXYZ>::Ptr & cloud,
               std::vector<recognition_result> & results,
               Eigen::Vector4f & table_coeffs,
               std::vector<pcl::PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointXYZ>::Ptr> > & clusters,
               Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > & recognizer)
{
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
    D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > ());

  recognizer.setEstimator (D2CR_Estimation); //pointer from initilization is no longer valid!

  recognizeAndVisualizeNearestNeighbors<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, recognizer,
                                                                                                   Z_DIST_, MODELS_DIR_,
                                                                                                   numberNN_, 0.0001);

  recognizeAndReturnResults<flann::L1, PointXYZ, PointNormal, Histogram<416> > (cloud, recognizer, Z_DIST_,
                                                                                MODELS_DIR_, numberNN_, results,
                                                                                table_coeffs, clusters);

}

void getAllScalesFromScaledModel(std::string file, std::vector<float> & scales) {
  //read scale file and compare with the current scales[idx]
  //TODO: The same model could be scaled several times and therefore
  //all files starting with file should be checked!!

  std::vector<std::string> strs;
  boost::split (strs, file, boost::is_any_of ("/"));

  std::string mask = strs[strs.size() - 1];
  boost::replace_all (mask, ".ply", "");

  //get directory
  std::string directory;
  for(size_t i=0; i < (strs.size() - 1); i++) {
    directory += strs[i] + "/";
  }

  bf::path dir = directory;
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr) {

    std::cout << bf::complete(itr->path()).filename() << " " << mask << std::endl;

    if (bf::is_directory (*itr)) {
      //ignore
    } else {
      //check if it does not end with .ply
      if(!boost::algorithm::ends_with(itr->path ().filename (), ".ply")) {
        //check if it fulfills the mask
        if(boost::algorithm::starts_with(itr->path ().filename (), mask)) {
          float s = read_scale_from_file (directory + (itr->path()).filename());
          scales.push_back(s);
          std::cout << directory + (itr->path()).filename() << s << std::endl;
        }
      }
    }
  }

}

void
learnObject (PointCloud<PointXYZ>::Ptr & cloud,
             std::map<std::string, std::vector<NormalDistribution> > & category_distributions,
             Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > & recognizer, bool use_prov_recognizer)
{
  Eigen::Vector4f table_coeffs;
  std::vector<recognition_result> results;
  typedef pcl::PointCloud<PointXYZ>::Ptr PointInTPtr;
  std::vector<PointInTPtr, Eigen::aligned_allocator<PointInTPtr> > clusters;

  if(use_prov_recognizer) {
    D2CRRecognize(cloud, results, table_coeffs, clusters, recognizer);
  } else {
    D2CRConfigurationWithoutPose (cloud, results, table_coeffs, clusters);
  }

  std::cout << "results size:" << results.size () << std::endl;

  for (size_t i = 0; i < results.size (); i++)
  {

    //cluster results to obtain the models to be aligned and categories
    //category is given in the id string
    std::vector<std::string> unique_plys;

    for (size_t j = 0; j < results[i].ids_.size (); j++)
    {
      unique_plys.push_back (results[i].ids_[j]);
    }

    std::sort (unique_plys.begin (), unique_plys.end ());

    vector<std::string>::iterator it;
    it = std::unique (unique_plys.begin (), unique_plys.end ());

    unique_plys.resize (it - unique_plys.begin ());

    std::cout << unique_plys.size () << std::endl;

    vector<float> scales;
    vector<float> metrics;
    vector < Eigen::Matrix4f > transforms;

    //align partial view to the clustered models (TODO: Parameter being numer of ply files to be aligned...)
    for (size_t j = 0; j < unique_plys.size (); j++)
    {
      std::cout << unique_plys[j] << std::endl;
      stringstream pathPly;
      pathPly << MODELS_DIR_ << "/" << unique_plys[j] << ".ply";

      //ALIGN one 3D MESH with a partial view
      vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
      readerQuery->SetFileName (pathPly.str ().c_str ());
      vtkSmartPointer < vtkPolyData > polydata1 = readerQuery->GetOutput ();
      polydata1->Update ();

      StablePlanesAlignment spa (10000, 0, 0, 1, 512, 1);
      spa.setVisualize (false);
      spa.setName2 (unique_plys[j]);
      spa.setSaveScale (true);

      Eigen::Matrix4f transform;
      Eigen::Vector4f CoM;
      pcl::compute3DCentroid (*clusters[i], CoM);
      Plane plane_1;
      plane_1.center[0] = -table_coeffs[0] * table_coeffs[3];
      plane_1.center[1] = -table_coeffs[1] * table_coeffs[3];
      plane_1.center[2] = -table_coeffs[2] * table_coeffs[3];
      plane_1.normal[0] = -table_coeffs[0];
      plane_1.normal[1] = -table_coeffs[1];
      plane_1.normal[2] = -table_coeffs[2];

      spa.alignPartialViewWithMesh (*clusters[i], plane_1, polydata1, transform);

      //learn scale
      float scale_applied = spa.getScaleApplied ();
      scales.push_back (scale_applied);
      metrics.push_back (spa.getMetricForBestAlignment ());
      transforms.push_back (transform);

      std::cout << "The learned scale is:" << scale_applied << std::endl;

      //show alignment
      /*pcl::visualization::PCLVisualizer aligned_vis ("aligned");
       vtkSmartPointer < vtkTransform > poseTransform = vtkSmartPointer<vtkTransform>::New ();
       vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
       for (size_t kk = 0; kk < 4; kk++)
       for (size_t k = 0; k < 4; k++)
       mat->SetElement (kk, k, transform (kk, k));

       poseTransform->SetMatrix (mat);
       std::stringstream model_name;
       model_name << "ply_model_" << j;

       stringstream pathPly11;
       pathPly11 << MODELS_DIR_ << "/" << unique_plys[j] << ".ply";

       aligned_vis.addModelFromPLYFile (pathPly11.str (), poseTransform, model_name.str ());
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_match (clusters[i], 0, 255, 0);
       model_name << "cluster_";
       aligned_vis.addPointCloud<pcl::PointXYZ> (clusters[i], handler_match, model_name.str ());
       aligned_vis.spin ();*/

    }

    float min = 1000;
    int idx;
    for (size_t j = 0; j < metrics.size (); j++)
    {
      if (metrics[j] < min)
      {
        idx = j;
        min = metrics[j];
      }
    }

    //show alignment
    pcl::visualization::PCLVisualizer aligned_vis ("aligned");
    vtkSmartPointer < vtkTransform > poseTransform = vtkSmartPointer<vtkTransform>::New ();
    vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
    for (size_t kk = 0; kk < 4; kk++)
      for (size_t k = 0; k < 4; k++)
        mat->SetElement (kk, k, transforms[idx] (kk, k));

    poseTransform->SetMatrix (mat);
    std::stringstream model_name;
    model_name << "ply_model_" << i;

    stringstream pathPly;
    pathPly << MODELS_DIR_ << "/" << unique_plys[idx] << ".ply";

    aligned_vis.addModelFromPLYFile (pathPly.str (), poseTransform, model_name.str ());
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_match (clusters[i], 0, 255, 0);
    model_name << "cluster_";
    aligned_vis.addPointCloud<pcl::PointXYZ> (clusters[i], handler_match, model_name.str ());
    aligned_vis.spin ();

    //check alignment by grasping the object?? (FUTURE WORK)

    /*
     * learn other stuff (possible materials?) - also future work?
     */

    //get category
    std::vector<std::string> strs;
    boost::split (strs, unique_plys[idx], boost::is_any_of ("/"));
    std::string category = strs[strs.size () - 2];

    std::cout << "category:" << category << " instance scale:" << scales[idx] << std::endl;
    //check if the alignment fits the learned scale distributions...
    bool scale_ok = checkScaleOnDistributions (category, scales[idx], category_distributions);

    if (!scale_ok)
    {
      //ask the user if the alignment was good and set scale_ok to true...
      char myChar = {0};

      string input = "";
      while (true)
      {
        cout << "PLEASE, enter 1 char to accept the alignment (a - accept or r - reject: ";
        getline (cin, input);

        if (input.length () == 1)
        {
          myChar = input[0];
          break;
        }

        cout << "Invalid character, please try again" << endl;
      }

      if (myChar == 'a')
      {
        scale_ok = true;
      }
    }

    if (scale_ok)
    {

      //check if the CAD model has already been scaled before and the scale matches...
      //in that case do not save the model again

      bool save_file = true;
      {
        std::stringstream path_model;
        path_model << dir_to_save + "/" << unique_plys[idx] << ".ply";

        bf::path path = path_model.str ();
        if (bf::exists (path)) //there is at least one scaled version of this model
        {

          std::vector<float> existing_scales;
          std::string file = path_model.str ();
          getAllScalesFromScaledModel(file, existing_scales);

          std::cout << "Existing scales size:" << existing_scales.size() << std::endl;

          bool equal=false; //equal to all previous scaled versions...
          float percent = 0.98;
          float similar_scale=0;
          for(size_t j=0; j < existing_scales.size() && !equal; j++) {
            std::cout << existing_scales[j] << scales[idx] << std::endl;
            if( scales[idx] == existing_scales[j] ) {
              //scales are equal
              equal = true;
              similar_scale = existing_scales[j];
            } else {
              float max = (std::max)(scales[idx],existing_scales[j]);
              float min = (std::min)(scales[idx],existing_scales[j]);

              if ( (min / max) > percent ) { //scales are similar enough, no need for another scaled version
                equal = true;
                similar_scale = existing_scales[j];
              }
            }
          }

          if(!equal) {
            save_file = true;
            std::stringstream new_name;
            new_name << unique_plys[idx] << "_" << existing_scales.size();
            unique_plys[idx] = new_name.str();
          } else {
            std::cout << "There is already a similar scale:" << scales[idx] << similar_scale << std::endl;
            save_file = false;
          }

          //read scale file and compare with the current scales[idx]
          //TODO: The same model could be scaled several times and therefore
          //all files starting with file should be checked!!

          //std::string file = path_model.str ();
          /*boost::replace_all (file, ".ply", ".scale.txt");

          float s = read_scale_from_file (file);
          std::cout << "s:" << s << " scale[idx]" << scales[idx] << std::endl;

          //compare the scales... TODO
          std::cout << "We need to compare scales" << std::endl;*/
        }
      }

      if (save_file)
      {
        //save the model, together with the instance scale, category is included in the name
        std::string save_path_ply = dir_to_save + "/" + unique_plys[idx] + ".ply";
        std::string save_path_scale (save_path_ply);
        boost::replace_all (save_path_scale, ".ply", ".scale.txt");

        //scale model...
        vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
        reader->SetFileName (pathPly.str ().c_str ());

        vtkSmartPointer < vtkTransformFilter > filter = vtkSmartPointer<vtkTransformFilter>::New ();
        filter->SetInputConnection (reader->GetOutputPort ());
        filter->SetTransform (poseTransform);
        filter->Update ();

        //create category subdirectories if do not exist
        std::stringstream path_model;
        path_model << dir_to_save << "/" << category;
        bf::path path = path_model.str ();
        if (!bf::exists (path))
        {
          bf::create_directory (path);
        }

        std::cout << save_path_ply << std::endl;

        vtkSmartPointer < vtkPLYWriter > writer = vtkSmartPointer<vtkPLYWriter>::New ();
        writer->SetFileName (save_path_ply.c_str ());
        writer->SetInputConnection (filter->GetOutputPort ());
        writer->Write ();

        //write scale file
        ofstream out (save_path_scale.c_str ());
        if (!out)
          cout << "Cannot open file.\n";
        out << scales[idx] << endl;
        out.close ();
      }
    }

  }
}

int
main (int argc, char ** argv)
{

  model_scale_factor_ = 1;
  MODELS_DIR_ = argv[1];
  TRAINING_DIR_ = argv[2]; //where to save the scaled model
  dir_to_save = argv[3];
  model_scale_factor_ = atof (argv[4]);
  std::string descriptor_str (argv[5]);
  numberNN_ = atoi (argv[6]);
  Z_DIST_ = 1.0;

  //use the kinect or load PointCloud, segment it and recognize it
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  if (argc <= 7)
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

      /*Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer(false); //turn cache off, takes too much memory...R
      D2CRConfigurationInitRecognizer(recognizer);*/

      while (true)
      {

        SimpleOpenNIViewer<pcl::PointXYZRGB> v (grabber);
        v.run ();
        boost::mutex mutex_;

        {
          boost::mutex::scoped_lock lock (mutex_);
          cloud_to_use_ = v.cloud__;
        }

        pcl::copyPointCloud (cloud_to_use_, *cloud);
        //load scale distributions for the categories
        /*std::map<std::string, std::vector<float> > category_scale_samples;
        load_scale_samples (dir_to_save, category_scale_samples);

        std::map<std::string, std::vector<NormalDistribution> > category_distributions;
        createDistributions (category_scale_samples, category_distributions);
        learnObject (cloud, category_distributions,recognizer,true);*/

        std::map<std::string, std::vector<float> > category_scale_samples;
        load_scale_samples (dir_to_save, category_scale_samples);

        std::map<std::string, std::vector<NormalDistribution> > category_distributions;
        createDistributions (category_scale_samples, category_distributions);

        Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
        learnObject (cloud, category_distributions,recognizer,false);
      }
    }
    else
    {
      std::cout << "We cannot get an XYZRGB pointcloud..." << std::endl;
      return 0;
    }
  }
  else
  {
    io::loadPCDFile (argv[7], *cloud);
  }

  pcl::visualization::PCLVisualizer aligned_vis ("last pointcloud");
  aligned_vis.addPointCloud<pcl::PointXYZ> (cloud);
  aligned_vis.spin ();

  //load scale distributions for the categories
  std::map<std::string, std::vector<float> > category_scale_samples;
  load_scale_samples (dir_to_save, category_scale_samples);

  std::map<std::string, std::vector<NormalDistribution> > category_distributions;
  createDistributions (category_scale_samples, category_distributions);

  Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > recognizer;
  learnObject (cloud, category_distributions,recognizer,false);
}
