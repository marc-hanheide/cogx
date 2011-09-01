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
#include "estimators/ESF_estimator.h"

#include "generic_trainer_recognizer/utils.h"
#include <iostream>
#include <fstream>
using namespace pcl;

std::string MODELS_DIR_;
std::string TRAINING_DIR_;
std::string TEST_DIR_;
float model_scale_factor_ = 1.0;
int numberNN_ = 1;
float Z_DIST_ = 2.0;

void
d2cr_configuration_init (Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > &recognizer)
{

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > > D2CR_Estimation;
  D2CR_Estimation.reset(new D2CR_Estimator<PointXYZ, PointNormal, pcl::Histogram<416> > ());
  {
    ScopeTime t ("-------- TRAINER_INIT");
    Trainer<PointXYZ, PointNormal, pcl::Histogram<416> > trainer (MODELS_DIR_, TRAINING_DIR_);
    trainer.setModelScale (model_scale_factor_);
    trainer.setUsevertices(false);
    trainer.setTesselationLevel (1);
    trainer.setEstimator (D2CR_Estimation);
    trainer.setDescriptorName ("d2cr");
    trainer.train ();
  }
  {
    ScopeTime t ("-------- RECOGNIZER_INIT");
    recognizer.setNumberOfModels (numberNN_);
    recognizer.setEstimator (D2CR_Estimation);
    recognizer.setModelsDirectory (MODELS_DIR_);
    recognizer.setComputePose (false);
    recognizer.setDescriptorName ("d2cr");
    recognizer.setTesselationLevel (1);
  }
  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }
}

void
esf_configuration_init (Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<640> > &recognizer)
{
  boost::shared_ptr<Estimator<PointXYZ, PointNormal, pcl::Histogram<640> > > ESF_Estimation;
  ESF_Estimation.reset(new ESF_Estimator < PointXYZ, PointNormal, pcl::Histogram<640> > ());
  {
    ScopeTime t ("-------- TRAINER_INIT");
    Trainer<PointXYZ, PointNormal, pcl::Histogram<640> > trainer (MODELS_DIR_, TRAINING_DIR_);
    trainer.setModelScale (model_scale_factor_);
    trainer.setTesselationLevel (1);
    trainer.setUsevertices(false);
    trainer.setEstimator (ESF_Estimation);
    trainer.setDescriptorName ("esf");
    trainer.train ();
  }
  {
    ScopeTime t ("-------- RECOGNIZER_INIT");
    recognizer.setNumberOfModels (numberNN_);
    recognizer.setEstimator (ESF_Estimation);
    recognizer.setModelsDirectory (MODELS_DIR_);
    recognizer.setComputePose (false);
    recognizer.setDescriptorName ("esf");
    recognizer.setTesselationLevel (1);
  }
  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }
}

void
vfh_configuration_init (Recognizer<flann::L1, PointXYZ, PointNormal, VFHSignature308> &recognizer)
{

  boost::shared_ptr<Estimator<PointXYZ, PointNormal, VFHSignature308 > > vfh_estimation;
  vfh_estimation.reset(new VFHEstimator<PointXYZ, PointNormal,
                          VFHSignature308> (true, true, 0.005) );

  typedef VFHEstimator<PointXYZ, PointNormal,
      VFHSignature308> VFHEstimator;

  boost::shared_ptr<VFHEstimator> vfh_estimator =
     boost::dynamic_pointer_cast<VFHEstimator>(vfh_estimation);

  vfh_estimator->setComputeMeshResolution(true);

  {
    ScopeTime t ("-------- TRAINER_INIT");

    /*VFHEstimator<PointXYZ, PointNormal, VFHSignature308> * vfh_estimation = new VFHEstimator<PointXYZ, PointNormal,
        VFHSignature308> (true, true, 0.005);*/

    Trainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
    trainer.setModelScale (model_scale_factor_);
    trainer.setTesselationLevel (1);
    trainer.setEstimator (vfh_estimation);
    trainer.setDescriptorName("VFH");
    trainer.setNosify(false);
    trainer.setRadiusVirtualcamera(1.5);
    trainer.train ();
  }

  {
    ScopeTime t ("-------- RECOGNIZER_INIT");

    /*VFHEstimator<PointXYZ, PointNormal, VFHSignature308> * vfh_estimation = new VFHEstimator<PointXYZ, PointNormal,
        VFHSignature308> (true, true, 0.005);
    vfh_estimation->setComputeMeshResolution(true);*/

    recognizer.setComputePose(false);
    recognizer.setNumberOfModels (numberNN_);
    recognizer.setEstimator (vfh_estimation);
    recognizer.setModelsDirectory(MODELS_DIR_);
    recognizer.setDescriptorName("VFH");
    recognizer.setTesselationLevel (1);
  }

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }
}

void
cvfh_configuration_init (Recognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> &recognizer)
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

  {
    ScopeTime t ("-------- TRAINER_INIT");

    /*CVFHEstimator<PointXYZ, PointNormal, VFHSignature308> * cvfh_estimation = new CVFHEstimator<PointXYZ, PointNormal,
        VFHSignature308> (true, true, 0.005);*/
    Trainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
    trainer.setModelScale (model_scale_factor_);
    trainer.setTesselationLevel (1);
    trainer.setEstimator (cvfh_estimation);
    trainer.setDescriptorName("CVFH-noise");
    trainer.setNosify(true);
    trainer.setNoiseLevel(0.006);
    trainer.setRadiusVirtualcamera(1.5);
    trainer.train ();
  }

  {
    ScopeTime t ("-------- RECOGNIZER_INIT");

    /*CVFHEstimator<PointXYZ, PointNormal, VFHSignature308> * cvfh_estimation = new CVFHEstimator<PointXYZ, PointNormal,
        VFHSignature308> (true, true, 0.005);
    cvfh_estimation->setComputeMeshResolution(true);
    cvfh_estimation->setLeafSizeFactor(3);
    cvfh_estimation->setEpsAngleThreshold(0.13);
    cvfh_estimation->setCurvatureThreshold(0.035);
    cvfh_estimation->setNormalizeBins(true);*/

    recognizer.setComputePose(false);
    recognizer.setNumberOfModels (numberNN_);
    recognizer.setEstimator (cvfh_estimation);
    recognizer.setModelsDirectory(MODELS_DIR_);
    recognizer.setDescriptorName("CVFH-noise");
    recognizer.setTesselationLevel (1);
  }

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }
}

void
cvfh_configuration_init_L1 (Recognizer<flann::L1, PointXYZ, PointNormal, VFHSignature308> &recognizer)
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

  {
    ScopeTime t ("-------- TRAINER_INIT");
    Trainer<PointXYZ, PointNormal, VFHSignature308> trainer (MODELS_DIR_, TRAINING_DIR_);
    trainer.setModelScale (model_scale_factor_);
    trainer.setTesselationLevel (1);
    trainer.setEstimator (cvfh_estimation);
    trainer.setDescriptorName("CVFH");
    trainer.setNosify(false);
    trainer.setRadiusVirtualcamera(1.5);
    trainer.train ();
  }

  {
    ScopeTime t ("-------- RECOGNIZER_INIT");
    recognizer.setComputePose(false);
    recognizer.setNumberOfModels (numberNN_);
    recognizer.setEstimator (cvfh_estimation);
    recognizer.setModelsDirectory(MODELS_DIR_);
    recognizer.setDescriptorName("CVFH");
    recognizer.setTesselationLevel (1);
  }

  {
    ScopeTime t ("-------- INITIALIZE");
    recognizer.initialize (TRAINING_DIR_);
  }
}

int
main (int argc, char ** argv)
{
  if (argc != 6) // argc should be 5 for correct execution
  {
    // We print argv[0] assuming it is the program name
    cout << "usage: " << argv[0]
        << " <modeldirectory>  <trainingdirectory>  <testdirectory>  <descriptor>  <numberOfNN(>1)>\n";
    //cout<<"usage: "<< argv[0] <<" <modeldirectory>  <trainingdirectory>  <testdirectory>  <descriptor>  <numberOfNN>\n";
  }
  else
  {
    MODELS_DIR_ = argv[1];
    TRAINING_DIR_ = argv[2];
    TEST_DIR_ = argv[3];
    std::string descriptor_str (argv[4]);
    numberNN_ = atoi (argv[5]);

    // load every .pcd file in /TEST_DIR_/pcd
    namespace fs = boost::filesystem;
    // comment to reach 6 character requirement
    fs::path tdir (std::string ().append (TEST_DIR_).append ("/pcd"));
    fs::directory_iterator end_iter;

    typedef std::multimap<std::time_t, fs::path> result_set_t;
    result_set_t result_set;

    if (fs::exists (tdir) && fs::is_directory (tdir))
    {
      cout << "iterating directory: " << TEST_DIR_ << "\n";
      ofstream rfile;
      std::string
                  fname =
                      std::string ().append (TEST_DIR_).append ("/").append (descriptor_str).append ("_NN").append (
                                                                                                                    to_string (
                                                                                                                               numberNN_)).append (
                                                                                                                                                   ".results");
      rfile.open (fname.c_str ());
      rfile << "MODELS_DIR: " << MODELS_DIR_ << "\n";
      rfile << "TRAINING_DIR: " << TRAINING_DIR_ << "\n";
      rfile << "TEST_DIR: " << TEST_DIR_ << "\n";
      rfile << "DESCRIPTOR: " << descriptor_str << "\n";
      rfile << "NUMBER_NN: " << numberNN_ << "\n";

      Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > d2cr_rec(false);
      Recognizer<flann::L1, PointXYZ, PointNormal, pcl::Histogram<640> > esf_rec(false);
      Recognizer<flann::L1, PointXYZ, PointNormal, VFHSignature308> vfh_rec(false);
      Recognizer<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, VFHSignature308> cvfh_rec(false);
      Recognizer<flann::L1, PointXYZ, PointNormal, VFHSignature308> cvfh_rec_L1(false);

      if (descriptor_str == "d2cr")
        d2cr_configuration_init (d2cr_rec);
      if (descriptor_str == "esf")
        esf_configuration_init (esf_rec);
      if (descriptor_str == "vfh")
        vfh_configuration_init (vfh_rec);
      if (descriptor_str == "cvfh")
        cvfh_configuration_init (cvfh_rec);
      if (descriptor_str == "cvfh_L1")
        cvfh_configuration_init_L1 (cvfh_rec_L1);

      for (fs::directory_iterator dir_iter (tdir); dir_iter != end_iter; ++dir_iter)
      {
        if (fs::is_regular_file (dir_iter->status ()) && fs::extension (dir_iter->leaf ()).compare (".pcd") == 0)
        {
          rfile << dir_iter->leaf () << " ";

//          if (dir_iter->leaf().compare("tt_tetra_veg26.pcd") != 0)
//        	  continue;
          //load PointCloud, segment it and recognize it. if cluster available, load cluster instead
          PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
          bool is_cluster = false;
          if (fs::is_directory(std::string ().append (TEST_DIR_).append ("/cluster/")) == false)
        	  fs::create_directories(std::string ().append (TEST_DIR_).append ("/cluster/"));
          std::string cdr_str (std::string ().append (TEST_DIR_).append ("/cluster/").append(dir_iter->leaf()));
          fs::path cdr(cdr_str);
          if (fs::is_regular_file(cdr))
          {
        	  io::loadPCDFile (cdr_str, *cloud);
        	  is_cluster = true;
          }
          else
          {
        	  io::loadPCDFile (std::string ().append (TEST_DIR_).append ("/pcd/").append (dir_iter->leaf ()), *cloud);
        	  is_cluster = false;
          }
          std::vector<recognition_result> results;

          cout << "file: " << dir_iter->leaf () << "\n";
          if (descriptor_str == "cvfh")
          {
            ScopeTime t ("-------- segment&classify CVFH");
            recognizeAndReturnResultsTxt<Metrics::HistIntersectionUnionDistance, PointXYZ, PointNormal, pcl::VFHSignature308 > (cloud, cvfh_rec,
                                                                                                      Z_DIST_, MODELS_DIR_,
                                                                                                      numberNN_, results,
                                                                                                      0.02, cdr_str, is_cluster);
          }

          if (descriptor_str == "cvfh_L1")
          {
            ScopeTime t ("-------- segment&classify CVFH L1");
            recognizeAndReturnResultsTxt<flann::L1, PointXYZ, PointNormal, pcl::VFHSignature308 > (cloud, cvfh_rec_L1,
                                                                                                      Z_DIST_, MODELS_DIR_,
                                                                                                      numberNN_, results,
                                                                                                      0.02, cdr_str, is_cluster);
          }

          if (descriptor_str == "vfh")
          {
            ScopeTime t ("-------- segment&classify VFH");
            recognizeAndReturnResultsTxt<flann::L1, PointXYZ, PointNormal, pcl::VFHSignature308 > (cloud, vfh_rec,
                                                                                                  Z_DIST_, MODELS_DIR_,
                                                                                                  numberNN_, results,
                                                                                                  0.02, cdr_str, is_cluster);
          }

          if (descriptor_str == "d2cr")
          {
            ScopeTime t ("-------- segment&classify");
            recognizeAndReturnResultsTxt<flann::L1, PointXYZ, PointNormal, pcl::Histogram<416> > (cloud, d2cr_rec,
                                                                                                  Z_DIST_, MODELS_DIR_,
                                                                                                  numberNN_, results,
                                                                                                  0.02, cdr_str, is_cluster);
          }
          if (descriptor_str == "esf")
          {
            ScopeTime t ("-------- segment&classify");
            recognizeAndReturnResultsTxt<flann::L1, PointXYZ, PointNormal, pcl::Histogram<640> > (cloud, esf_rec,
                                                                                                  Z_DIST_, MODELS_DIR_,
                                                                                                  numberNN_, results,
                                                                                                  0.02, cdr_str, is_cluster);
          }

          for (int i = 0; i < results.size (); i++)
          {
            for (int j = 0; j < results[i].ids_.size (); j++)
              rfile << results[i].ids_[j] << " ";

            break;
          }
          rfile << "\n";
          rfile.flush ();

        }
      }
      rfile.close ();

    }
  }
}

