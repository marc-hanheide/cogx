/*
 * cvfh_trainer.cpp
 *
 *  Created on: May 9, 2011
 *      Author: aitor
 */

/* This class should take care of training the CVFH descriptor using
 * PLY files. Take care of generating views, compute descriptors, make data persistence
 * ,filtering duplicate views,etc.
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "trainers/trainer.h"

#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

//template<typename PointInT, typename PointOutT, typename FeatureT, class EstimatorT>
template<typename PointInT, typename PointOutT, typename FeatureT>
  Trainer<PointInT, PointOutT, FeatureT>::Trainer (std::string files_dir, std::string train_dir)
  {
    MODEL_FILES_DIR_ = files_dir;
    TRAINING_DIR_ = train_dir;
    resolution_ = 150;
    tesselated_sphere_level_ = 0;

    apply_voxel_grid_ = true;
    apply_radius_removal_ = true;
    leaf_size_ = 0.005;
    noisify_ = true;
    noise_ = 0.001;
    model_scale_ = 1.0;
    remove_duplicate_views_ = false;
    radius_virtual_camera_ = 1;
    fov_virtual_camera_ = 57;
    use_vertices_ = true;
    //estimator_ = 0;
  }

template<typename PointInT, typename PointOutT, typename FeatureT>
  Trainer<PointInT, PointOutT, FeatureT>::~Trainer ()
  {
    /*if (estimator_ != 0)
    {
      //delete estimator_;
      //estimator_ = 0;
    }*/
  }

/////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Training //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

//template<typename PointInT, typename PointOutT, typename FeatureT, class EstimatorT>
template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  Trainer<PointInT, PointOutT, FeatureT>::generate_views_for_model (
                                                                    std::string PLYModel,
                                                                    std::vector<pcl::PointCloud<PointInT>,
                                                                        Eigen::aligned_allocator<pcl::PointCloud<
                                                                            PointInT> > > & views,
                                                                    std::vector<Eigen::Matrix4f,
                                                                        Eigen::aligned_allocator<Eigen::Matrix4f> > & poses,
                                                                    std::vector<float> & enthropies)
  {

    //check if the views have already been generated for this model
    //and load them if they exist, also poses and enthropies
    //TODO: load enthropies, they dont get saved now...

    bool exist = true;
    std::string id = Model<View, PointOutT, FeatureT>::getIdFromFilename (PLYModel);

    {
      std::vector < std::string > strs;
      boost::split (strs, id, boost::is_any_of ("/"));

      std::string str = "";

      for (size_t i = 0; i < strs.size (); i++)
      {
        std::stringstream path_model;
        str += strs[i] + "/";
        path_model << TRAINING_DIR_ << "/" << str;

        bf::path path = path_model.str ();
        if (!bf::exists (path))
        {
          exist = false;
          break;
        }
      }

      std::stringstream path_model;
      path_model << TRAINING_DIR_ << "/" << id;
      bf::path path = path_model.str ();
      if (!bf::exists (path))
        exist = false;

      if(exist) {
        //check that the any views are generated... check that view_i.pcd exist
        //NOTE: Assuming that the rest are there as well!!

        int number_of_views=0;
        bf::path dir = path_model.str ();
        bf::directory_iterator end_itr;
        for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
        {
          if (bf::is_directory (*itr))
          {

          }
          else
          {
            //check that it is a ply file and then add, otherwise ignore..
            std::vector<std::string> strs;
            std::vector<std::string> strs_;

            std::string file = itr->path ().filename ();
            boost::split (strs, file, boost::is_any_of ("."));
            boost::split (strs_, file, boost::is_any_of ("_"));

            std::string extension = strs[strs.size () - 1];

            if(extension == "pcd" && strs_[0] == "view") {
               //there are views...
              number_of_views++;
              break;
            }
          }
        }

        if (number_of_views == 0)
          exist = false;
      }
    }

    if(exist) {
      //if exist load them
      std::stringstream path_model;
      path_model << TRAINING_DIR_ << "/" << id;
      bf::path dir = path_model.str ();

      int number_of_views=0;

      //get all views in the directory...
      std::vector<std::string> view_filenames;

      bf::directory_iterator end_itr;
      for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
      {
        //check if its a directory, then get models in it
        if (bf::is_directory (*itr))
        {

        }
        else
        {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
          std::vector<std::string> strs_;

          std::string file = itr->path ().filename ();
          boost::split (strs, file, boost::is_any_of ("."));
          boost::split (strs_, file, boost::is_any_of ("_"));

          std::string extension = strs[strs.size () - 1];

          if(extension == "pcd" && strs_[0] == "view") {
            view_filenames.push_back(itr->path ().filename ());
            number_of_views++;
          }
        }
      }

      for(size_t i=0; i < view_filenames.size(); i++) {
        std::stringstream view_file;
        view_file << path_model.str () << "/" << view_filenames[i];
        typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
        pcl::io::loadPCDFile(view_file.str(), *cloud);

        views.push_back (*cloud);

        std::string file_replaced1 (view_filenames[i]);
        boost::replace_all (file_replaced1, "view", "pose");
        boost::replace_all (file_replaced1, ".pcd", ".txt");

        std::string file_replaced2 (view_filenames[i]);
        boost::replace_all (file_replaced2, "view", "entropy");
        boost::replace_all (file_replaced2, ".pcd", ".txt");

        //read pose as well
        std::stringstream pose_file;
        pose_file << path_model.str () <<  "/" << file_replaced1;

        Eigen::Matrix4f pose;
        PersistenceUtils::readMatrixFromFile (pose_file.str(), pose);

        poses.push_back(pose);

        //read entropy as well
        std::stringstream entropy_file;
        entropy_file << path_model.str () <<  "/" << file_replaced2;
        float entropy=0;
        PersistenceUtils::readFloatFromFile (entropy_file.str(), entropy);
        enthropies.push_back(entropy);

      }

    } else {
      std::string id = Model<View, PointOutT, FeatureT>::getIdFromFilename (PLYModel);
      std::vector < std::string > strs;
      boost::split (strs, id, boost::is_any_of ("/"));

      std::string str = "";

      for (size_t i = 0; i < strs.size (); i++)
      {
        std::stringstream path_model;
        str += strs[i] + "/";
        path_model << TRAINING_DIR_ << "/" << str;

        bf::path path = path_model.str ();
        if (!bf::exists (path))
        {
          bf::create_directory (path);
        }
      }

      std::stringstream path_model;
      path_model << TRAINING_DIR_ << "/" << id;
      bf::path path = path_model.str ();
      if (!bf::exists (path))
        bf::create_directory (path);





      /////////////////////////
      std::stringstream path_ply;
      path_ply << MODEL_FILES_DIR_ << "/" << PLYModel;
      PLYModel = path_ply.str();

      //scale model_
      vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
      reader->SetFileName (PLYModel.c_str ());

      vtkSmartPointer < vtkTransform > trans = vtkSmartPointer<vtkTransform>::New ();
      trans->Scale (model_scale_, model_scale_, model_scale_);
      trans->Modified ();

      vtkSmartPointer < vtkTransformFilter > filter_scale = vtkSmartPointer<vtkTransformFilter>::New ();
      filter_scale->SetTransform (trans);
      filter_scale->SetInputConnection (reader->GetOutputPort ());

      vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
      mapper->SetInputConnection (filter_scale->GetOutputPort ());
      mapper->Update ();

      //load the model and generate views
      pcl::visualization::PCLVisualizer vis;
      vis.addModelFromPolyData (mapper->GetInput (), "mesh1", 0);
      vis.setRepresentationToSurfaceForAllActors ();

      std::cout << "fov:" << fov_virtual_camera_ << " radius:" << radius_virtual_camera_ << std::endl;

      std::vector < pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;
      vis.renderViewTesselatedSphere (resolution_, resolution_, views_xyz, poses, enthropies, tesselated_sphere_level_, fov_virtual_camera_, radius_virtual_camera_, use_vertices_);

      Model<View, PointInT, FeatureT> model (id);

      for (size_t i = 0; i < views_xyz.size (); i++)
      {

        std::cout << "points:" << views_xyz[i].points.size () << std::endl;

        if (views_xyz[i].points.size() <= 20)
        	continue;

        pcl::PointCloud<PointInT> p;
        p.points.resize (views_xyz[i].points.size ());
        p.width = p.points.size ();
        p.height = 1;

        for (size_t j = 0; j < views_xyz[i].points.size (); j++)
        {
          p.points[j].getVector4fMap () = views_xyz[i].points[j].getVector4fMap ();
        }

        views.push_back (p);

        //save views and poses in the upper directory... TODO: Check that the view is not empty!!
        stringstream path_view;
        path_view << path_model.str() << "/view_" << i << ".pcd";
        pcl::io::savePCDFileBinary (path_view.str (), p);

        stringstream path_pose;
        path_pose << path_model.str() << "/pose_" << i << ".txt";
        PersistenceUtils::writeMatrixToFile (path_pose.str (), poses[i]);


        std::cout << "Entropy:" << enthropies[i] << std::endl;
        //save enthropy as well
        stringstream path_entropy;
        path_entropy << path_model.str() << "/entropy_" << i << ".txt";
        PersistenceUtils::writeFloatToFile (path_entropy.str (), enthropies[i]);

      }
    }
  }

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  Trainer<PointInT, PointOutT, FeatureT>::getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far,
                                                                std::vector<std::string> & relative_paths)
  {
    //list models in MODEL_FILES_DIR_ and return list
    bf::directory_iterator end_itr;
    for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
    {
      //check if its a directory, then get models in it
      if (bf::is_directory (*itr))
      {
        std::string so_far = rel_path_so_far + itr->path ().filename () + "/";
        bf::path curr_path = itr->path ();
        getModelsInDirectory (curr_path, so_far, relative_paths);
      }
      else
      {
        //check that it is a ply file and then add, otherwise ignore..
        std::vector<std::string> strs;
        std::string file = itr->path ().filename ();
        boost::split (strs, file, boost::is_any_of ("."));
        std::string extension = strs[strs.size () - 1];

        if(extension == "ply") {
          std::string path = rel_path_so_far + itr->path ().filename ();
          relative_paths.push_back (path);
        }
      }
    }
  }

template<typename PointInT, typename PointOutT, typename FeatureT>
void
Trainer<PointInT, PointOutT, FeatureT>::incrementalTraining (std::vector<std::string> & files)
{
  std::vector < std::string > yet_to_train;
  yet_to_train.resize (files.size ());

  int to_train = 0;
  for (size_t i = 0; i < files.size (); i++)
  {

    //check if directory exists in training_dir
    std::string id = Model<View, PointInT, FeatureT>::getIdFromFilename (files[i]);
    bf::path trained_dir = (TRAINING_DIR_ + "/" + id + "/" + D_NAME_);

    if(bf::exists(trained_dir)) {

    } else {
      yet_to_train[to_train] = files[i];
      to_train++;
    }
  }

  std::cout << "yet to train:" << to_train << std::endl;
  yet_to_train.resize (to_train);
  if (files.size () != yet_to_train.size ())
  {
    std::cout << "WARNING: Some models were already trained. No training will be performed for those models."
        << std::endl;
    files = yet_to_train;
  }
}

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  Trainer<PointInT, PointOutT, FeatureT>::train ()
  {

    //check if directory exist, otherwise create it
    bf::path trained_dir = TRAINING_DIR_;
    if (!bf::exists (trained_dir))
    {
      bf::create_directory (trained_dir);
    }

    std::stringstream tesselated_trained_dir_level;
    tesselated_trained_dir_level << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
    trained_dir = tesselated_trained_dir_level.str();

    TRAINING_DIR_ = tesselated_trained_dir_level.str();

    if (!bf::exists (trained_dir))
    {
      bf::create_directory (trained_dir);
    }

    bf::path ply_files_dir = MODEL_FILES_DIR_;
    std::vector < std::string > files;
    std::string start = "";
    getModelsInDirectory (ply_files_dir, start, files);

    incrementalTraining ( files);
    if (files.size () == 0)
    {
      std::cout << "INFO: All models have been already trained! Nothing to be done." << std::endl;
      return;
    }

    ///////////////////////////////////////////////////////
    //erase flann files as new descriptors will be generated
    std::stringstream flann_files_dir;
    flann_files_dir << TRAINING_DIR_;

    bf::path training_data_h5_file_name = flann_files_dir.str() + "/" + D_NAME_ + "_training_data.h5";
    bf::path training_data_list_file_name = flann_files_dir.str() + "/" + D_NAME_ + "_training_data.list";
    bf::path index_filename = flann_files_dir.str() + "/" + D_NAME_ + "_kdtree.idx";

    PCL_WARN("Erasing flann files for descriptor %s\n", D_NAME_.c_str());

    if (boost::filesystem::exists (training_data_h5_file_name))
      boost::filesystem::remove (training_data_h5_file_name);

    if (boost::filesystem::exists (training_data_list_file_name))
      boost::filesystem::remove (training_data_list_file_name);

    if (boost::filesystem::exists (index_filename))
      boost::filesystem::remove (index_filename);

    //////////////////////////////////////////

    for (size_t i = 0; i < files.size (); i++)
    {
      std::vector < pcl::PointCloud<PointInT>, Eigen::aligned_allocator<pcl::PointCloud<PointInT> > > views;
      std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
      std::vector<float> enthropies;

      generate_views_for_model (files[i], views, poses, enthropies);

      std::string id = Model<View, PointOutT, FeatureT>::getIdFromFilename (files[i]);
      Model<View, PointOutT, FeatureT> model (id);

      //for each generated view...
      for (size_t v = 0; v < views.size (); v++)
      {

        typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
        pcl::copyPointCloud (views[v], *cloud);

        if (cloud->points.size () > 0)
        {

          if (noisify_)
          {
            double noise_std = noise_; //1mm
            struct timeval start;
            gettimeofday (&start, NULL);
            boost::mt19937 rng;
            rng.seed (start.tv_usec);
            boost::normal_distribution<> nd (0.0, noise_std);
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
            // Noisify each point in the dataset
            for (size_t cp = 0; cp < cloud->points.size (); ++cp)
            {
              cloud->points[cp].z += var_nor ();
            }
          }

          //compute signatures
          typename pcl::PointCloud<PointOutT>::Ptr cloud_out (new pcl::PointCloud<PointOutT> ());
          std::vector < pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
          std::vector < Eigen::Vector3f > centroids;

          estimator_->estimate (cloud, cloud_out, signatures, centroids);
          int pos = model.addView (*cloud_out, signatures, centroids, poses[v], enthropies[v]);

          //pass the estimator and a reference to the model to specificTraining that
          //will create a specific view if needed, specific views inherit from the generic view
          specificTraining (estimator_, model, pos);
        }
      }

      //filter duplicate views and remove corresponding information
      Model<View, PointOutT, FeatureT> model_filtered (model.getId ());

      if (remove_duplicate_views_)
        removeDuplicateViews (estimator_, model, model_filtered);
      else
        model_filtered = model;

      //make data persistent (views, poses, CVFH, roll histogram) in TRAINING_DIR_
      makePersistent (model_filtered);

      std::cout << "Training for model:" << i << " finished" << std::endl;
    }

    std::cout << "Training finished! Number of trained models:" << files.size () << std::endl;

  }

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  Trainer<PointInT, PointOutT, FeatureT>::makePersistent (Model<View, PointOutT, FeatureT> & model)
  {
    //create directory for the given id

    std::string id = model.getId ();
    std::vector < std::string > strs;
    boost::split (strs, id, boost::is_any_of ("/"));

    std::string str = "";

    for (size_t i = 0; i < strs.size (); i++)
    {
      std::stringstream path_model;
      str += strs[i] + "/";
      path_model << TRAINING_DIR_ << "/" << str;

      bf::path path = path_model.str ();
      if (!bf::exists (path))
      {
        bf::create_directory (path);
      }
    }

    std::stringstream path_model;
    path_model << TRAINING_DIR_ << "/" << model.getId ();
    bf::path path = path_model.str ();
    if (!bf::exists (path))
    	bf::create_directory (path);

    path = path_model.str () + "/" + D_NAME_;
    if (!bf::exists (path))
    	bf::create_directory (path);

    path = path_model.str ();

    typedef typename Model<View, PointOutT, FeatureT>::view_iterator view_iterator;
    view_iterator views;
    int i = 0;
    for (views = model.begin (); views != model.end (); ++views, ++i)
    {
      (*views)->makePersistent (i, path.string (), D_NAME_);
      specificPersistence (i, path.string (), *views);
    }

  }

//create a specific estimator for Features that inherit from FeatureFromNormals?
//template class Trainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<153> > ; //spin images
template class Trainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<640> > ; //D2 multi shape distributions...
template class Trainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ; //D2 multi shape distributions...
template class Trainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ; //D2 multi shape distributions...
template class Trainer<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ; //CVFH using normals as output point type
template class Trainer<pcl::PointXYZ, pcl::PointXYZ, pcl::VFHSignature308> ; //Geodesic distances (no need for normals)
template class Trainer<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ; //CVFH using normals as output point type
