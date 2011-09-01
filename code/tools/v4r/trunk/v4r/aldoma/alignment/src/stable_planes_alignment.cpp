#include "alignment/sampling.hpp"
#include "alignment/stable_planes_alignment.h"
#include "alignment/internal_pcl_functions.h"
#include <pcl/registration/icp.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <Eigen/StdVector>
#include <omp.h>
#include "alignment/cross_correlations.hpp"
#include "alignment/logPolarTranslation.h"
#include "alignment/logPolarTranslations.hpp"
#include <stdio.h>
#include <cutil_inline.h>
#include <cufft.h>

StablePlanesAlignment::StablePlanesAlignment (int SAMPLE_POINTS, int FS_TYPE, bool PARALLEL, bool GPU, int max_size_projected, int multi_scale_levels)
{
  SAMPLE_POINTS_ = SAMPLE_POINTS;
  VISUALIZE_ = true;
  VISUALIZE_CROSS_ = false;
  MAX_PLANES_1_ = 5;
  MAX_PLANES_2_ = 5;
  MAX_SIZE_PROJECTED_ = max_size_projected;
  polydata_1_ = NULL;
  polydata_2_ = NULL;
  PARALLEL_ALIGNMENT_ = PARALLEL;
  USE_GPU_ = GPU;
  save_scale_applied_ = false;
  scale_applied_ = 1.0;
  multi_scale_levels_ = multi_scale_levels;
  FitnessScore fs (FS_TYPE);
  fs_ = fs;
}

void
StablePlanesAlignment::alignMeshes (vtkSmartPointer<vtkPolyData> mesh1, vtkSmartPointer<vtkPolyData> mesh2,
                                    Plane & plane_1, std::vector<Plane> & planes_2, Eigen::Matrix4f & transform, int & best_plane_2)
{
  //sample meshes
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

  uniform_sampling (mesh1, SAMPLE_POINTS_, *cloud_1);
  uniform_sampling (mesh2, SAMPLE_POINTS_, *cloud_2);

  //visualize meshes
  /*pcl::visualization::PCLVisualizer vis ("Mesh sampling");
  vis.addPointCloud (cloud_1, "cloud1");
  vis.addModelFromPolyData (mesh1, "mesh1", 0);
  vis.spin ();
  vis.removePointCloud ("cloud1");

  pcl::visualization::PCLVisualizer vis2 ("Mesh sampling");
  vis2.addPointCloud (cloud_2, "cloud2");
  vis2.addModelFromPolyData (mesh2, "mesh2", 0);
  vis2.spin ();
  vis2.removePointCloud ("cloud2");*/

  polydata_1_ = mesh1;
  polydata_2_ = mesh2;

  fs_.setPartialViewAlignment (false);
  //call alignPointClouds

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    alignPointClouds (*cloud_1, *cloud_2, plane_1, planes_2, transform, best_plane_2);

    gettimeofday (&end, NULL);
    std::cout << "--------- total alignment took:" << internal_pcl::timing (start, end) << " ms" << " PARALLEL:"
        << PARALLEL_ALIGNMENT_ << std::endl;
  }

  vtkSmartPointer < vtkMatrix4x4 > matrix_trans_vtk = vtkSmartPointer<vtkMatrix4x4>::New ();
  for (size_t k_1 = 0; k_1 < 4; k_1++)
    for (size_t k_2 = 0; k_2 < 4; k_2++)
      matrix_trans_vtk->SetElement (k_1, k_2, transform (k_1, k_2));

  vtkSmartPointer < vtkTransform > trans_vtk = vtkSmartPointer<vtkTransform>::New ();
  trans_vtk->SetMatrix (matrix_trans_vtk);
  trans_vtk->Modified ();
  //apply transfrom to mesh2 and visualize both

  /*float color_1[3] = {1, 1, 0};
  float color_2[3] = {0, 1, 1};
  pcl::visualization::PCLVisualizer vis_meshes ("Mesh sampling");
  vis_meshes.addModelFromPolyData (mesh1, "mesh1", 0);
  vis_meshes.addModelFromPolyData (mesh2, trans_vtk, "mesh2", 0);
  vis_meshes.setBackgroundColor (1, 1, 1);
  vis_meshes.spin ();*/

}

//PUBLIC METHODS
void
StablePlanesAlignment::alignMeshes (vtkSmartPointer<vtkPolyData> mesh1, vtkSmartPointer<vtkPolyData> mesh2,
                                    Eigen::Matrix4f & transform)
{
  std::cout << "alignMeshes called" << std::endl;

  //sample meshes
  pcl::PointCloud<pcl::PointXYZ> cloud_1, cloud_2;
  uniform_sampling (mesh1, SAMPLE_POINTS_, cloud_1);
  uniform_sampling (mesh2, SAMPLE_POINTS_, cloud_2);

  //getVerticesAsPointCloud (mesh1, cloud_1);
  //getVerticesAsPointCloud (mesh2, cloud_2);

  //visualize meshes
  if (false && VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Mesh sampling");
    vis.addPointCloud (cloud_1.makeShared (), "cloud1");
    vis.addModelFromPolyData (mesh1, "mesh1", 0);
    vis.spin ();
    vis.removePointCloud ("cloud1");

    pcl::visualization::PCLVisualizer vis2 ("Mesh sampling");
    vis2.addPointCloud (cloud_2.makeShared (), "cloud2");
    vis2.addModelFromPolyData (mesh2, "mesh2", 0);
    vis2.spin ();
    vis2.removePointCloud ("cloud2");
  }

  polydata_1_ = mesh1;
  polydata_2_ = mesh2;

  fs_.setPartialViewAlignment (false);
  //call alignPointClouds

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    alignPointClouds (cloud_1, cloud_2, transform);

    gettimeofday (&end, NULL);
    std::cout << "--------- total alignment took:" << internal_pcl::timing (start, end) << " ms" << " PARALLEL:"
        << PARALLEL_ALIGNMENT_ << std::endl;
  }

  if (true && VISUALIZE_)
  {
    vtkSmartPointer < vtkMatrix4x4 > matrix_trans_vtk = vtkSmartPointer<vtkMatrix4x4>::New ();
    for (size_t k_1 = 0; k_1 < 4; k_1++)
      for (size_t k_2 = 0; k_2 < 4; k_2++)
        matrix_trans_vtk->SetElement (k_1, k_2, transform (k_1, k_2));

    vtkSmartPointer < vtkTransform > trans_vtk = vtkSmartPointer<vtkTransform>::New ();
    trans_vtk->SetMatrix (matrix_trans_vtk);
    trans_vtk->Modified ();
    //apply transfrom to mesh2 and visualize both

    float color_1[3] = {1, 1, 0};
    float color_2[3] = {0, 1, 1};
    pcl::visualization::PCLVisualizer vis_meshes ("Mesh sampling");
    //vis_meshes.addModelFromPolyData (mesh1, "mesh1", 0, color_1);
    //vis_meshes.addModelFromPolyData (mesh2, trans_vtk, "mesh2", 0, color_2);
    vis_meshes.setBackgroundColor (1, 1, 1);
    vis_meshes.spin ();
  }
}

void
StablePlanesAlignment::alignPartialViewWithMesh (pcl::PointCloud<pcl::PointXYZ> & cloud_1, Plane & plane_1,
                                                 vtkPolyData * mesh, Eigen::Matrix4f & transform)
{
  pcl::PointCloud < pcl::PointXYZ > cloud_2;
  uniform_sampling (mesh, SAMPLE_POINTS_, cloud_2);

  if (VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Mesh sampling");
    vis.addPointCloud (cloud_2.makeShared (), "cloud_2");
    vis.addModelFromPolyData (mesh, "mesh1", 0);
    vis.addPointCloud (cloud_1.makeShared (), "cloud_1");
    vis.spin ();
  }

  polydata_2_ = mesh;
  fs_.setPartialViewAlignment (true);
  alignPartialViewWithPointCloud (cloud_1, plane_1, cloud_2, transform);

  if (false && VISUALIZE_)
  {
    vtkSmartPointer < vtkMatrix4x4 > matrix_trans_vtk = vtkSmartPointer<vtkMatrix4x4>::New ();
    for (size_t k_1 = 0; k_1 < 4; k_1++)
      for (size_t k_2 = 0; k_2 < 4; k_2++)
        matrix_trans_vtk->SetElement (k_1, k_2, transform (k_1, k_2));

    vtkSmartPointer < vtkTransform > trans_vtk = vtkSmartPointer<vtkTransform>::New ();
    trans_vtk->SetMatrix (matrix_trans_vtk);
    trans_vtk->Modified ();

    float color_1[3] = {1, 1, 0};
    float color_2[3] = {0, 1, 1};
    pcl::visualization::PCLVisualizer vis_meshes ("Aligned mesh with partial view");

    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_whole_pc (cloud_1.makeShared (), 255,
                                                                                         255, 0);
    vis_meshes.addPointCloud (cloud_1.makeShared (), handler_whole_pc, "cloud_1");
    //vis_meshes.addModelFromPolyData (mesh, trans_vtk, "mesh2", 0, color_2);
    vis_meshes.setBackgroundColor (1, 1, 1);
    vis_meshes.spin ();
  }
}

inline std::string
getNameSPA (std::string file)
{
  std::vector<std::string> strs;
  boost::split (strs, file, boost::is_any_of ("/"));
  std::string nameWithExt = strs[strs.size () - 1];
  return nameWithExt.substr (0, nameWithExt.find_last_of ("."));
}

//PRIVATE METHODS

void
StablePlanesAlignment::alignPointClouds (pcl::PointCloud<pcl::PointXYZ> & pcd_1, pcl::PointCloud<pcl::PointXYZ> & pcd_2, Plane & plane_1,
                  std::vector<Plane> & stable_planes_2, Eigen::Matrix4f & transform, int & best_plane_2) {

  Eigen::Matrix4f best_transform_j_to_i;
    best_transform_j_to_i.setIdentity ();
    double min_error = std::numeric_limits<float>::max ();

    struct timeval start_no_splanes, end_no_splanes;
    gettimeofday (&start_no_splanes, NULL);

    if (PARALLEL_ALIGNMENT_)
    {
      //PARALLEL ALIGNMENT
      int size_i = 1;
      int size_j = (std::min) ((int)stable_planes_2.size (), MAX_PLANES_2_);
      int size_total = size_i * size_j;

      std::vector<double> errors (size_total, 0.0);
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_j_to_i;
      transforms_j_to_i.resize (size_total);

      std::vector<std::pair<Plane, Plane> > plane_pairs;

      for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
      {
        plane_pairs.push_back (std::make_pair (plane_1, stable_planes_2[j]));
      }

      size_t it = 0;

  #pragma omp parallel for
      for (it = 0; it < (size_t)size_total; it++)
      {
        Eigen::Matrix4f transform_j_to_i;
        double error;

        {
          struct timeval start, end;
          gettimeofday (&start, NULL);

          error = align (pcd_1, pcd_2, plane_pairs[it].first, plane_pairs[it].second, transform_j_to_i);

          gettimeofday (&end, NULL);
          //std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
        }

        //save into correct position
        errors[it] = error;
        transforms_j_to_i[it] = transform_j_to_i;
      }

  #pragma omp barrier //wait for all threads to finish
      //select best one
      for (size_t it = 0; it < (size_t)size_total; it++)
      {
        if (errors[it] < min_error)
        {
          //save transform and update error
          best_transform_j_to_i = transforms_j_to_i[it];
          min_error = errors[it];
          best_plane_2 = it;
        }
      }
    }
    else
    {
      //iterate over planes
      for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
      {
        //call align, run in parallel...
        {
          Eigen::Matrix4f transform_j_to_i;
          double error;

          {
            struct timeval start, end;
            gettimeofday (&start, NULL);

            error = align (pcd_1, pcd_2, plane_1, stable_planes_2[j], transform_j_to_i);

            gettimeofday (&end, NULL);
            //std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
          }

          if (error < min_error)
          {
            //save transform and update error
            best_transform_j_to_i = transform_j_to_i;
            min_error = error;
            best_plane_2 = j;
          }
        }
      }
    }

    gettimeofday (&end_no_splanes, NULL);
    std::cout << "--------- alignment no stable planes:" << internal_pcl::timing (start_no_splanes, end_no_splanes)
        << " ms" << " PARALLEL:" << PARALLEL_ALIGNMENT_ << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (pcd_2, *transformed_2, best_transform_j_to_i);

    if (false && this->VISUALIZE_)
    {
      pcl::visualization::PCLVisualizer vis ("Final alignment");
      vis.addPointCloud (pcd_1.makeShared (), "cloud_I");
      vis.addPointCloud (transformed_2, "cloud_J");
      vis.addCoordinateSystem (0.1, 0);
      vis.spin ();
    }

    //Voxel grid to make ICP faster!
    float max_dist = internal_pcl::getMinMaxDist (pcd_1, *transformed_2);
    float voxel_size = max_dist / 50;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_1_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::VoxelGrid < pcl::PointXYZ > grid_;
    grid_.setInputCloud (pcd_1.makeShared ());
    grid_.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid_.filter (*pcd_1_voxelgrid);

    pcl::VoxelGrid < pcl::PointXYZ > grid_2;
    grid_2.setInputCloud (transformed_2);
    grid_2.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid_2.filter (*transformed_2_voxelgrid);

    if (false & this->VISUALIZE_)
    {
      pcl::visualization::PCLVisualizer vis ("Voxel grided things");
      vis.addPointCloud (pcd_1_voxelgrid, "cloud_I");
      vis.addPointCloud (transformed_2_voxelgrid, "cloud_J");
      vis.addCoordinateSystem (0.1, 0);
      vis.spin ();
    }

    //Perform ICP on the best MODEL!
    pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > reg;
    reg.setInputCloud (transformed_2_voxelgrid);
    reg.setInputTarget (pcd_1_voxelgrid);
    reg.setMaximumIterations (30);
    //reg.setMaxCorrespondenceDistance (max_dist * 0.05);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_ (new pcl::PointCloud<pcl::PointXYZ> ());

    {
      struct timeval start, end;
      gettimeofday (&start, NULL);

      reg.align (*output_);

      gettimeofday (&end, NULL);
      //std::cout << "--------- icp TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
    }

    if (false & this->VISUALIZE_)
    {
      pcl::visualization::PCLVisualizer vis ("Aligned after ICP");
      pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_1 (output_, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_2 (pcd_1_voxelgrid, 255, 0, 0);
      vis.addPointCloud (output_, handler_1, "cloud_I");
      vis.addPointCloud (pcd_1_voxelgrid, handler_2, "cloud_J");
      vis.addCoordinateSystem (0.1, 0);
      vis.spin ();
    }

    Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();

    //update transform to be returned
    //TODO: Attached ICP transform!
    transform = icp_trans * best_transform_j_to_i;

}

void
StablePlanesAlignment::alignPointClouds (pcl::PointCloud<pcl::PointXYZ> & pcd_1,
                                         pcl::PointCloud<pcl::PointXYZ> & pcd_2, Eigen::Matrix4f & transform)
{
  //std::cout << "alignPointClouds called" << std::endl;

  //compute stable planes for both point clouds
  std::vector<Plane> stable_planes_1;
  std::vector<Plane> stable_planes_2;

  stablePlanes sp;
  sp.setThreshold (0.005);
  sp.setVisualize (false);

  if (polydata_1_ != NULL)
  {

    //check if file exists
    sp.setFilenameForCache (getNameSPA (name_1));
    bool cache_exist = sp.getStablePlanesFromCache (stable_planes_1,getNameSPA (name_1));

    if (!cache_exist)
    {
      //std::cout << "polydata 1 is not null" << std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
      getVerticesAsPointCloud (polydata_1_, *cloud_out);
      sp.setInputCloud (cloud_out);
      sp.compute (stable_planes_1);
    }
    else
    {
      std::cout << "Stable planes for polydata 1 load from CACHE" << std::endl;
    }
  }
  else
  {
    //std::cout << "polydata 1 is NULL" << std::endl;
    sp.setInputCloud (pcd_1.makeShared ());
    sp.compute (stable_planes_1);
  }

  if (polydata_2_ != NULL)
  {
    //check if file exists
    sp.setFilenameForCache (getNameSPA (name_2));

    bool cache_exist = sp.getStablePlanesFromCache (stable_planes_2,getNameSPA (name_2));

    if (!cache_exist)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
      getVerticesAsPointCloud (polydata_2_, *cloud_out);
      sp.setInputCloud (cloud_out);
      sp.compute (stable_planes_2);
    }
    else
    {
      std::cout << "Stable planes for polydata 2 load from CACHE" << std::endl;
    }
  }
  else
  {
    sp.setInputCloud (pcd_2.makeShared ());
    sp.compute (stable_planes_2);
  }

  Eigen::Matrix4f best_transform_j_to_i;
  best_transform_j_to_i.setIdentity ();
  double min_error = std::numeric_limits<float>::max ();

  struct timeval start_no_splanes, end_no_splanes;
  gettimeofday (&start_no_splanes, NULL);

  if (PARALLEL_ALIGNMENT_)
  {
    //PARALLEL ALIGNMENT
    int size_i = (std::min) ((int)stable_planes_1.size (), MAX_PLANES_1_);
    int size_j = (std::min) ((int)stable_planes_2.size (), MAX_PLANES_2_);
    int size_total = size_i * size_j;

    std::vector<double> errors (size_total, 0.0);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_j_to_i;
    transforms_j_to_i.resize (size_total);

    std::vector<std::pair<Plane, Plane> > plane_pairs;

    for (size_t i = 0; (i < stable_planes_1.size ()) && (i < (size_t)MAX_PLANES_1_); i++)
    {
      for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
      {
        plane_pairs.push_back (std::make_pair (stable_planes_1[i], stable_planes_2[j]));
      }
    }

    size_t it = 0;

#pragma omp parallel for
    for (it = 0; it < (size_t)size_total; it++)
    {
      Eigen::Matrix4f transform_j_to_i;
      double error;

      {
        struct timeval start, end;
        gettimeofday (&start, NULL);

        error = align (pcd_1, pcd_2, plane_pairs[it].first, plane_pairs[it].second, transform_j_to_i);

        gettimeofday (&end, NULL);
        //std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
      }

      //save into correct position
      errors[it] = error;
      transforms_j_to_i[it] = transform_j_to_i;
    }

#pragma omp barrier //wait for all threads to finish
    //select best one
    for (size_t it = 0; it < (size_t)size_total; it++)
    {
      if (errors[it] < min_error)
      {
        //save transform and update error
        best_transform_j_to_i = transforms_j_to_i[it];
        min_error = errors[it];
      }
    }
  }
  else
  {
    //iterate over planes
    for (size_t i = 0; (i < stable_planes_1.size ()) && (i < (size_t)MAX_PLANES_1_); i++)
    {
      for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
      {
        //call align, run in parallel...
        {
          Eigen::Matrix4f transform_j_to_i;
          double error;

          {
            struct timeval start, end;
            gettimeofday (&start, NULL);

            error = align (pcd_1, pcd_2, stable_planes_1[i], stable_planes_2[j], transform_j_to_i);

            gettimeofday (&end, NULL);
            //std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
          }

          if (error < min_error)
          {
            //save transform and update error
            best_transform_j_to_i = transform_j_to_i;
            min_error = error;
          }
        }
      }
    }
  }

  gettimeofday (&end_no_splanes, NULL);
  std::cout << "--------- alignment no stable planes:" << internal_pcl::timing (start_no_splanes, end_no_splanes)
      << " ms" << " PARALLEL:" << PARALLEL_ALIGNMENT_ << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (pcd_2, *transformed_2, best_transform_j_to_i);

  if (false && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Final alignment");
    vis.addPointCloud (pcd_1.makeShared (), "cloud_I");
    vis.addPointCloud (transformed_2, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  //Voxel grid to make ICP faster!
  float max_dist = internal_pcl::getMinMaxDist (pcd_1, *transformed_2);
  float voxel_size = max_dist / 50;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_1_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::VoxelGrid < pcl::PointXYZ > grid_;
  grid_.setInputCloud (pcd_1.makeShared ());
  grid_.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid_.filter (*pcd_1_voxelgrid);

  pcl::VoxelGrid < pcl::PointXYZ > grid_2;
  grid_2.setInputCloud (transformed_2);
  grid_2.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid_2.filter (*transformed_2_voxelgrid);

  if (false & this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Voxel grided things");
    vis.addPointCloud (pcd_1_voxelgrid, "cloud_I");
    vis.addPointCloud (transformed_2_voxelgrid, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  //Perform ICP on the best MODEL!
  pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > reg;
  reg.setInputCloud (transformed_2_voxelgrid);
  reg.setInputTarget (pcd_1_voxelgrid);
  reg.setMaximumIterations (30);
  //reg.setMaxCorrespondenceDistance (max_dist * 0.05);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_ (new pcl::PointCloud<pcl::PointXYZ> ());

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    reg.align (*output_);

    gettimeofday (&end, NULL);
    std::cout << "--------- icp TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
  }

  if (false & this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Aligned after ICP");
    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_1 (output_, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > handler_2 (pcd_1_voxelgrid, 255, 0, 0);
    vis.addPointCloud (output_, handler_1, "cloud_I");
    vis.addPointCloud (pcd_1_voxelgrid, handler_2, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();

  //update transform to be returned
  //TODO: Attached ICP transform!
  transform = icp_trans * best_transform_j_to_i;

}

//pcd_1 is the partial view, pcd_2 is the mesh
void
StablePlanesAlignment::alignPartialViewWithPointCloud (pcl::PointCloud<pcl::PointXYZ> & pcd_1, Plane & plane_1,
                                                       pcl::PointCloud<pcl::PointXYZ> & pcd_2,
                                                       Eigen::Matrix4f & transform)
{
  //notify fitness score that a partial view is aligned to a mesh
  fs_.setPartialViewAlignment (true);
  PARTIAL_VIEW_ALIGNMENT_ = true;

  stablePlanes sp;
  sp.setThreshold (0.005);
  sp.setVisualize (false);

  std::vector<Plane> stable_planes_2;

  if (polydata_2_ != NULL)
  {
    //check if file exists
    sp.setFilenameForCache (getNameSPA (name_2));

    bool cache_exist = sp.getStablePlanesFromCache (stable_planes_2, getNameSPA (name_2));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());

    if (!cache_exist)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
      getVerticesAsPointCloud (polydata_2_, *cloud_out);
      sp.setInputCloud (cloud_out);
      sp.compute (stable_planes_2);
    }
    else
    {
      std::cout << "Stable planes for polydata 2 load from CACHE" << std::endl;
    }
  }
  else
  {
    sp.setInputCloud (pcd_2.makeShared ());
    sp.compute (stable_planes_2);
  }

  Eigen::Matrix4f best_transform_j_to_i;
  best_transform_j_to_i.setIdentity ();
  double min_error = std::numeric_limits<float>::max ();
  int best_plane=-1;

  if (PARALLEL_ALIGNMENT_)
  {
    int size_j = (std::min) ((int)stable_planes_2.size (), MAX_PLANES_2_);

    std::vector<double> errors (size_j, 0.0);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_j_to_i;
    transforms_j_to_i.resize (size_j);

    std::vector<std::pair<Plane, Plane> > plane_pairs;

    for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
    {
      plane_pairs.push_back (std::make_pair (plane_1, stable_planes_2[j]));
    }

    std::cout << "Going to enter parallel programming:" << std::endl;

    size_t it = 0;

#pragma omp parallel for
    for (it = 0; it < (size_t)size_j; it++)
    {
      Eigen::Matrix4f transform_j_to_i;
      double error;

      {
        struct timeval start, end;
        gettimeofday (&start, NULL);

        error = align (pcd_1, pcd_2, plane_pairs[it].first, plane_pairs[it].second, transform_j_to_i);

        gettimeofday (&end, NULL);
        std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
      }

      //save into correct position
      errors[it] = error;
      transforms_j_to_i[it] = transform_j_to_i;
    }

#pragma omp barrier //wait for all threads to finish
    //select best one
    for (size_t it = 0; it < (size_t)size_j; it++)
    {
      if (errors[it] < min_error)
      {
        //save transform and update error
        best_transform_j_to_i = transforms_j_to_i[it];
        min_error = errors[it];
      }
    }
  }
  else
  {
    //iterate over stable_planes_2 and call align with Plane2_i and plane_1

    for (size_t j = 0; (j < stable_planes_2.size ()) && (j < (size_t)MAX_PLANES_2_); j++)
    {
      Eigen::Matrix4f transform_j_to_i;
      double error;

      {
        struct timeval start, end;
        gettimeofday (&start, NULL);

        //pcd_2 is the mesh, pcd_1 is the partial view
        error = align (pcd_1, pcd_2, plane_1, stable_planes_2[j], transform_j_to_i);

        gettimeofday (&end, NULL);
        //std::cout << "--------- align TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
      }

      if (error < min_error)
      {
        //save transform and update error
        best_transform_j_to_i = transform_j_to_i;
        min_error = error;
        best_plane = j;
      }

    }
  }
  //pcd_2 is the mesh, pcd_1 is the partial view
  //best_transform_j_to_i converts the partial view to pcd_2 mesh coordinates

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (pcd_2, *transformed_2, best_transform_j_to_i);

  //metric stuff...
  pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > metric;
  metric.setInputCloud (pcd_1.makeShared());
  metric.setInputTarget (transformed_2);
  metric_ = metric.getFitnessScore();

  if(save_scale_applied_) {
    //get heights of the model standing on the plane and compute scale...

    Plane plane = stable_planes_2[best_plane];
    EIGEN_ALIGN16 Eigen::Vector3f best_normal, best_centroid;
    best_normal = Eigen::Vector3f (plane.normal[0], plane.normal[1], plane.normal[2]);
    best_centroid = Eigen::Vector3f (plane.center[0], plane.center[1], plane.center[2]);

    EIGEN_ALIGN16 Eigen::Vector3f plane_normal, plane_center;
    plane_normal = Eigen::Vector3f (plane_1.normal[0], plane_1.normal[1], plane_1.normal[2]);
    plane_center = Eigen::Vector3f (plane_1.center[0], plane_1.center[1], plane_1.center[2]);

    //transformation to be on the best stable plane
    Eigen::Matrix4f transform;
    stablePlanes::transformOnPlane (best_centroid, best_normal, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (pcd_2, *transformed, transform); //transformed on canonical cs

    //put aligned model again on the canonical cs
    Eigen::Matrix4f transform_to_sensed_plane;
    stablePlanes::transformOnPlane (plane_center, plane_normal, transform_to_sensed_plane);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_22 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*transformed_2, *transformed_22, transform_to_sensed_plane); //transformed on canonical cs

    //compare heights and get scale...
    float height_aligned, height_original;
    height_original = internal_pcl::getMaxHeight (*transformed);
    height_aligned = internal_pcl::getMaxHeight (*transformed_22);

    scale_applied_ = height_aligned/height_original;

    if (false && this->VISUALIZE_)
    {
      pcl::visualization::PCLVisualizer vis ("Voxel grided things");
      vis.addPointCloud (transformed, "cloud_I");
      vis.addPointCloud (transformed_22, "cloud_J");
      vis.addCoordinateSystem (0.1, 0);
      vis.spin ();
    }

  }

  if (false && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Final alignment");
    vis.addPointCloud (pcd_1.makeShared (), "cloud_I");
    vis.addPointCloud (transformed_2, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  //Voxel grid to make ICP faster!
  float max_dist = internal_pcl::getMinMaxDist (pcd_1, *transformed_2);
  float voxel_size = 0.005;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_2_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_1_voxelgrid (new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::VoxelGrid < pcl::PointXYZ > grid_;
  grid_.setInputCloud (pcd_1.makeShared ());
  grid_.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid_.filter (*pcd_1_voxelgrid);

  pcl::VoxelGrid < pcl::PointXYZ > grid_2;
  grid_2.setInputCloud (transformed_2);
  grid_2.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid_2.filter (*transformed_2_voxelgrid);

  if (false && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Voxel grided things");
    vis.addPointCloud (pcd_1_voxelgrid, "cloud_I");
    vis.addPointCloud (transformed_2_voxelgrid, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  //Perform ICP on the best MODEL!
  pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > reg;
  reg.setInputCloud (pcd_1_voxelgrid);
  reg.setInputTarget (transformed_2_voxelgrid);
  reg.setMaximumIterations (30);
  reg.setMaxCorrespondenceDistance (0.05);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_ (new pcl::PointCloud<pcl::PointXYZ> ());
  reg.align (*output_);

  if (false && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("Aligned after ICP");
    vis.addPointCloud (output_, "cloud_I");
    //vis.addPointCloud (pcd_1_voxelgrid, "cloud_J");
    vis.addPointCloud (transformed_2_voxelgrid, "cloud_J");
    vis.addCoordinateSystem (0.1, 0);
    vis.spin ();
  }

  Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
  //icp_trans.setIdentity();
  Eigen::Matrix4f icp_trans_inverse = icp_trans.inverse ();
  transform = icp_trans_inverse * best_transform_j_to_i;
}

void
StablePlanesAlignment::projectOntoPlane (pcl::PointCloud<pcl::PointXYZ> & pcd, cv::Mat & projection)
{
  float histDist = 2.;
  int bins = MAX_SIZE_PROJECTED_;
  float start[2] = {-1, -1};
  float binSize = histDist / (float)(bins);

  int posX, posZ;
  for (size_t i = 0; i < pcd.points.size (); i++)
  {
    posX = round ((pcd.points[i].x - start[0]) / binSize);
    posZ = round ((pcd.points[i].z - start[1]) / binSize);
    projection.at<float> (posX, posZ) += 1.0; //sum number points falling in this bin
    //projection.at<float> (posX, posZ) += pcd.points[i].y; //sum number points falling in this bin
    //projection.at<float> (posX, posZ) = 1.0;
  }

  /*cv::Mat to_display = projection.clone();
   double minVal_w, maxVal_w;
   cv::minMaxLoc (to_display, &minVal_w, &maxVal_w);
   to_display /= maxVal_w;

   cv::namedWindow ("weight", CV_WINDOW_AUTOSIZE);
   cv::imshow ("weight", to_display);
   cv::waitKey (0);*/

  //TODO: normalize projection matrix has been removed!!
  /*double minVal, maxVal;

   cv::minMaxLoc (projection, &minVal, &maxVal);
   cv::Mat_<float> normalized = projection / maxVal;
   projection = normalized;*/

}

//pcd_i is the partial view
//pcd_j is the mesh

double
StablePlanesAlignment::align (const pcl::PointCloud<pcl::PointXYZ> & pcd_i,
                              const pcl::PointCloud<pcl::PointXYZ> & pcd_j, const Plane & plane_i,
                              const Plane & plane_j, Eigen::Matrix4f & transform)
{
  //std::cout << "Align method called" << std::endl;
  Eigen::Vector3f normal_i, centroid_i;
  normal_i = Eigen::Vector3f (plane_i.normal[0], plane_i.normal[1], plane_i.normal[2]);
  centroid_i = Eigen::Vector3f (plane_i.center[0], plane_i.center[1], plane_i.center[2]);

  Eigen::Vector3f normal_j, centroid_j;
  normal_j = Eigen::Vector3f (plane_j.normal[0], plane_j.normal[1], plane_j.normal[2]);
  centroid_j = Eigen::Vector3f (plane_j.center[0], plane_j.center[1], plane_j.center[2]);

  //std::cout << "normal_i" << normal_i << std::endl << "normal_j" << normal_j << std::endl;

  //transform point clouds to be on the plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_i (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_j (new pcl::PointCloud<pcl::PointXYZ> ());
  Eigen::Matrix4f transform_i, transform_j;

  stablePlanes::transformOnPlane (centroid_i, normal_i, transform_i);
  pcl::transformPointCloud (pcd_i, *transformed_i, transform_i);

  stablePlanes::transformOnPlane (centroid_j, normal_j, transform_j);
  pcl::transformPointCloud (pcd_j, *transformed_j, transform_j);

  //pcl::io::savePCDFileBinary ("/home/aitor/test.pcd", *transformed_j);

  //align 2D
  //(center CoMs on the plane)
  Eigen::Vector4f com_i, com_j;

  if(PARTIAL_VIEW_ALIGNMENT_) {
    //project on the plane, voxel grid to eliminate points on same xz and then compute 3D centroid...
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ> (*transformed_i));
    for(size_t i=0; i < new_cloud->points.size(); i++) {
      new_cloud->points[i].y = 0;
    }

    pcl::VoxelGrid < pcl::PointXYZ > grid_;
    grid_.setInputCloud (new_cloud);
    float voxel_size = 0.005;
    grid_.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid_.filter (*new_cloud);

    pcl::compute3DCentroid (*new_cloud, com_i);

  } else {
    pcl::compute3DCentroid (*transformed_i, com_i);
  }

  pcl::compute3DCentroid (*transformed_j, com_j);

  com_i[1] = com_j[1] = 0;

  pcl::demeanPointCloud (*transformed_i, com_i, *transformed_i);
  pcl::demeanPointCloud (*transformed_j, com_j, *transformed_j);

  //(scale heights so they match) mainly just needed for partial views where the scale does not match!
  //compute height_i, height_j
  float height_i, height_j;
  height_i = internal_pcl::getMaxHeight (*transformed_i);
  height_j = internal_pcl::getMaxHeight (*transformed_j);

  Eigen::Matrix4f height_scale;
  height_scale.setIdentity ();
  height_scale (0, 0) = height_i / height_j;
  height_scale (1, 1) = height_i / height_j;
  height_scale (2, 2) = height_i / height_j;

  pcl::transformPointCloud (*transformed_j, *transformed_j, height_scale);

  //visualize on plane!
  if (true && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("after scaling height");
    pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ
        > handler_whole_pc (transformed_i, 0.0, 0.0, 125.0);
    vis.addPointCloud (transformed_i, handler_whole_pc, "cloud_I");
    vis.addPointCloud (transformed_j, "cloud_J");
    vis.addCoordinateSystem (0.5, 0);
    vis.spin ();
  }

  //scale the models so they fit in the image...

  float image_limit_scale = internal_pcl::getScaleFactorOnPlane (*transformed_i, *transformed_j);
  Eigen::Matrix4f scale_to_fit_on_plane;
  scale_to_fit_on_plane.setIdentity ();
  scale_to_fit_on_plane (0, 0) = image_limit_scale;
  scale_to_fit_on_plane (1, 1) = image_limit_scale;
  scale_to_fit_on_plane (2, 2) = image_limit_scale;

  pcl::transformPointCloud (*transformed_i, *transformed_i, scale_to_fit_on_plane);
  pcl::transformPointCloud (*transformed_j, *transformed_j, scale_to_fit_on_plane);

  //(project points on the plane xz)
  int nbins = MAX_SIZE_PROJECTED_;
  cv::Mat proj_i = cv::Mat_<float>::zeros (nbins, nbins);
  cv::Mat proj_j = cv::Mat_<float>::zeros (nbins, nbins);

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    projectOntoPlane (*transformed_i, proj_i);
    projectOntoPlane (*transformed_j, proj_j);

    gettimeofday (&end, NULL);
    //std::cout << "--------- projectOntoPlane:" << internal_pcl::timing (start, end) << " ms" << std::endl;
  }

  //(compute log-polar transformation, fft and cross correlation)
  double rotation, scale;
  double d_x = 0, d_z = 0;

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    //align2D (proj_i, proj_j, &rotation, &scale);
    //std::cout << "align2D: " << rotation << " s:" << scale << std::endl;

    //proj_i is the partial view
    //proj_j is the mesh
    align2DMultiScale (proj_i, proj_j, &rotation, &scale, &d_x, &d_z);
    //std::cout << "align2DMultiScale: " << rotation << " s:" << scale << " dx: " << d_x << " dz: " << d_z << std::endl;
    gettimeofday (&end, NULL);
    //std::cout << "--------- align2DMultiScale TOOK:" << internal_pcl::timing (start, end) << " ms" << std::endl;
  }

  Eigen::Matrix4f translate_I_on_plane;
  translate_I_on_plane.setIdentity ();
  translate_I_on_plane (0, 3) = d_z;
  translate_I_on_plane (2, 3) = d_x;

  //transform point clouds applying rotation and scale
  double rotation_rad = rotation * M_PI / 180;
  Eigen::Affine3f transformPC (Eigen::AngleAxisf (rotation_rad, Eigen::Vector3f::UnitY ()));

  Eigen::Matrix4f rotation_j_xz = Eigen::Matrix4f ();
  rotation_j_xz.setIdentity (4, 4);

  //TODO: Do this properly!!!
  for (size_t k_1 = 0; k_1 < 3; k_1++)
    for (size_t k_2 = 0; k_2 < 3; k_2++)
      rotation_j_xz (k_1, k_2) = transformPC (k_1, k_2);

  Eigen::Matrix4f scale_2d_transform;
  scale_2d_transform.setIdentity ();
  scale_2d_transform (0, 0) = scale;
  scale_2d_transform (1, 1) = scale;
  scale_2d_transform (2, 2) = scale;

  Eigen::Matrix4f rotation_and_scale = scale_2d_transform * rotation_j_xz;
  pcl::transformPointCloud (*transformed_j, *transformed_j, rotation_and_scale);
  pcl::transformPointCloud (*transformed_i, *transformed_i, translate_I_on_plane);

  //Create the whole transform
  //From initial J to transformed aligned J
  Eigen::Matrix4f J_to_final;
  J_to_final.setIdentity ();

  Eigen::Matrix4f center_J_on_Plane;
  center_J_on_Plane.setIdentity ();
  center_J_on_Plane (0, 3) = -com_j (0);
  center_J_on_Plane (2, 3) = -com_j (2);

  //scale_to_fit_in_plane (inverse)
  Eigen::Matrix4f scale_to_fit_on_plane_inverse = scale_to_fit_on_plane;

  //J_to_final = transform_j * center_J_on_Plane * height_scale * rotation_j_xz * scale_2d_transform;
  J_to_final = scale_2d_transform * rotation_j_xz * scale_to_fit_on_plane_inverse * height_scale * center_J_on_Plane
      * transform_j;

  //From initial I to transformed aligned I
  Eigen::Matrix4f I_to_final;
  I_to_final.setIdentity ();

  Eigen::Matrix4f center_I_on_Plane;
  center_I_on_Plane.setIdentity ();
  center_I_on_Plane (0, 3) = -com_i (0);
  center_I_on_Plane (2, 3) = -com_i (2);

  I_to_final = translate_I_on_plane * scale_to_fit_on_plane_inverse * center_I_on_Plane * transform_i;

  Eigen::Matrix4f I_to_final_inverse = I_to_final.inverse ();
  Eigen::Matrix4f J_to_final_inverse = J_to_final.inverse ();

  transform = I_to_final_inverse * J_to_final;

  if (true && this->VISUALIZE_)
  {
    pcl::visualization::PCLVisualizer vis ("voxel grided stuff, transformed");
    vis.addPointCloud (transformed_i, "cloud_I");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_match (transformed_j, 0, 255, 0);
    vis.addPointCloud (transformed_j, handler_match, "cloud_J");
    vis.addCoordinateSystem (0.5, 0);
    vis.spin ();
  }

  //Compute error using intersection, return number of inliers inversed!
  //pcd_i is the partial view, we want to maximize the number of inliers from pcd_i...
  double fitness;

  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    fitness = fs_.computeFitnessScore (transformed_i, transformed_j);

    gettimeofday (&end, NULL);
    //std::cout << "--------- compute fitness score:" << internal_pcl::timing (start, end) << " ms" << std::endl;
  }

  //std::cout << "FITNESS SCORE:" << fitness << std::endl;

  return fitness;
}

//proj_i is the partial view
//proj_j is the mesh

void
StablePlanesAlignment::align2DMultiScale (const cv::Mat & proj_i, const cv::Mat & proj_j, double * rotation,
                                          double * scale, double * d_x, double * d_y)
{

  struct timeval start, end;

  cv::Mat proj_i_copy = proj_i.clone ();
  cv::Mat proj_j_copy = proj_j.clone ();

  std::vector<cv::Mat> pyramid_i, pyramid_j;

  pyramid_i.push_back (proj_i_copy);
  pyramid_j.push_back (proj_j_copy);

  int i = multi_scale_levels_;
  while (i <= 0)
  {
    cv::Mat proj_i_downsampled, proj_j_downsampled;
    cv::pyrDown (pyramid_i[i], proj_i_downsampled);
    cv::pyrDown (pyramid_j[i], proj_j_downsampled);

    pyramid_i.push_back (proj_i_downsampled);
    pyramid_j.push_back (proj_j_downsampled);
    i++;
  }

  double rotationI = 0;
  double scaleI = 1;
  double ii = 0;
  double jj = 0;
  double accum_scale = 1;

  //im1 is proj_i, im2 is proj_j
  //im1 is partial view, im2 is the whole mesh
  //we want to translate im1 and rotate,scale the mesh

  for (i = (pyramid_i.size () - 1); i >= 0; i--)
  {
    cv::Mat im1, im2;
    im1 = pyramid_i[i].clone ();
    im2 = pyramid_j[i].clone ();

    //translate im1
    cv::Mat rotMatrix1 = cv::getRotationMatrix2D (cv::Point2f (im2.cols / 2, im2.rows / 2), 0, 1);
    rotMatrix1.at<double> (0, 2) += ii * 2;
    rotMatrix1.at<double> (1, 2) += jj * 2;

    cv::Mat mat1Translated;
    cv::warpAffine (im1, mat1Translated, rotMatrix1, im1.size ());

    cv::Mat rotMatrix2 = cv::getRotationMatrix2D (cv::Point2f (im2.cols / 2, im2.rows / 2), rotationI, scaleI);
    cv::Mat mat2Translated;
    cv::warpAffine (im2, mat2Translated, rotMatrix2, im2.size ());

    /*cvNamedWindow ("mat1Trans", 1);
    cv::imshow ("mat1Trans", mat1Translated);
    cvNamedWindow ("mat2Trans", 1);
    cv::imshow ("mat2Trans", mat2Translated);
    cvWaitKey (0);
    cvDestroyWindow ("mat1Trans");
    cvDestroyWindow ("mat2Trans");*/

    double rotationL, scaleL;
    int iiL, jjL;
    rotationL = iiL = jjL = 0;
    scaleL = 1;

    //mat1Translated is the partial view translated...
    //mat2Translated is the mesh rotated and scaled...

    gettimeofday (&start, NULL);

    if (USE_GPU_)
    {
      logPolarTranslationGPU (mat1Translated, mat2Translated, &rotationL, &scaleL, &iiL, &jjL);
      //logPolarTranslation (mat1Translated, mat2Translated, &rotationL, &scaleL, &iiL, &jjL);
    }
    else
      logPolarTranslation (mat1Translated, mat2Translated, &rotationL, &scaleL, &iiL, &jjL);

    //logPolarTranslationDiff (mat1Translated, mat2Translated, &rotationL, &scaleL, &iiL, &jjL);
    gettimeofday (&end, NULL);
    //std::cout << "--------- logPolarTranslation:" << internal_pcl::timing (start, end) << " ms" << std::endl;

    //translation needs to be applied to partial_view
    //rotation,scale needs to mesh

    rotationI += (rotationL * -1);
    scaleI = scaleL; //maybe just the last scale??
    accum_scale *= scaleI;
    //cout << scaleI << " " << rotationI << endl;
    ii = ii * 2 + iiL;
    jj = jj * 2 + jjL;
  }

  //Display images aligned...
  /*cv::Mat im1, im2;
  im1 = pyramid_i[0].clone (); //get biggest image
  im2 = pyramid_j[0].clone ();

  //translate im1
  cv::Mat rotMatrix1 = cv::getRotationMatrix2D (cv::Point2f (im2.cols / 2, im2.rows / 2), 0, 1);
  rotMatrix1.at<double> (0, 2) += ii;
  rotMatrix1.at<double> (1, 2) += jj;

  cv::Mat mat1Translated;
  cv::warpAffine (im1, mat1Translated, rotMatrix1, im1.size ());

  cv::Mat rotMatrix2 = cv::getRotationMatrix2D (cv::Point2f (im2.cols / 2, im2.rows / 2), rotationI, accum_scale);
  cv::Mat mat2Translated;
  cv::warpAffine (im2, mat2Translated, rotMatrix2, im2.size ());

  cvNamedWindow ("mat1Trans", 1);
  cv::imshow ("mat1Trans", mat1Translated);
  cvNamedWindow ("mat2Trans", 1);
  cv::imshow ("mat2Trans", mat2Translated);
  cvWaitKey (0);
  cvDestroyWindow ("mat1Trans");
  cvDestroyWindow ("mat2Trans");*/

  //Save results
  double binSize = 2.0 / MAX_SIZE_PROJECTED_;
  *rotation = (360 - rotationI);
  *scale = accum_scale;
  *d_x = (ii * binSize);
  *d_y = (jj * binSize);
}

void
StablePlanesAlignment::align2D (const cv::Mat & proj_i, const cv::Mat & proj_j, double * rotation, double * scale)
{
  //return more than one peak? could be useful and then use pointcloud intersection

  //log-polar transform for proj_i and proj_j
  int size, imWidth, imHeight;
  imWidth = proj_i.cols;
  size = imHeight = proj_i.rows;
  double MScale = getScaleLogPolar (size);

  cv::Mat weight_matrix = proj_i.clone ();
  getWeightMatrix (weight_matrix, size / 2, size / 2);

  cv::Mat proj_i_lp, proj_j_lp;
  cv::multiply (proj_i, weight_matrix, proj_i_lp);
  cv::multiply (proj_j, weight_matrix, proj_j_lp);

  //cross correlate proj_i_lp and proj_j_lp using FFT
  int maxI, maxJ;
  {
    struct timeval start, end;
    gettimeofday (&start, NULL);

    crossCorrelationFFT (proj_i_lp, proj_j_lp, &maxI, &maxJ, SCALE_LIMIT_);

    gettimeofday (&end, NULL);
    //std::cout << "--------- cross correlation FFT:" << internal_pcl::timing (start, end) << " ms" << std::endl;
    //std::cout << "Rotation FTT:" << maxI / double (proj_j_lp.rows) * 360 << " Scale:" << exp (maxJ / double (MScale))
    //    << std::endl;
  }

  *rotation = (maxI / double (proj_j_lp.rows) * 360);
  *scale = exp (maxJ / double (MScale));

}
