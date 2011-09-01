/*
 * roll_hist_estimator.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: aitor
 */

#include "estimators/normal_estimator.h"
#include "pcl/common/time.h"
#include <pcl/visualization/pcl_visualizer.h>

/*
 * NOTE: Change compute normals so that if the scale of the model is not  known
 * the mesh resolution is computed (average normals from each point to the nearest neighbour)
 * and used as the leaf_size_
 */

template<typename PointInT, typename PointOutT, typename FeatureT>
  bool
  NormalEstimator<PointInT, PointOutT, FeatureT>::computeNormals (PointInTPtr input, PointOutTPtr cloud_normals)
  {
    PointInTPtr grid (new pcl::PointCloud<PointInT> ());
    typedef typename pcl::KdTree<PointInT>::Ptr KdTreeInPtr;

    if (compute_mesh_resolution_)
    {
      KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointInT> > (false);
      tree->setInputCloud (input);

      //update leaf_size_

      std::vector<int> nn_indices (9);
      std::vector<float> nn_distances (9);
      std::vector<int> src_indices;

      float sum_distances = 0.0;
      std::vector<float> avg_distances (input->points.size ());
      // Iterate through the source data set
      for (size_t i = 0; i < input->points.size (); ++i)
      {
        // Search for the closest point in the target data set (number of neighbors to find = 2)
        // the point it self and the nearest neighbour
        tree->nearestKSearch (input->points[i], 9, nn_indices, nn_distances);

        float avg_dist_neighbours = 0.0;
        for (size_t j = 1; j < nn_indices.size (); j++)
        {
          avg_dist_neighbours += sqrt (nn_distances[j]);
        }

        avg_dist_neighbours /= nn_indices.size ();

        avg_distances.push_back(avg_dist_neighbours);
        sum_distances += avg_dist_neighbours;
      }

      float avg = sum_distances / input->points.size ();
      //std::cout << "average leaf size:" << avg << std::endl;

      std::sort(avg_distances.begin(),avg_distances.end());

      //std::cout << "median leaf size:" << avg_distances[avg_distances.size() / 2] << std::endl;
      //avg = avg_distances[avg_distances.size() / 2];
      leaf_size_ = avg;
    }

    if (apply_voxel_grid_)
    {
      pcl::VoxelGrid < PointInT > grid_;
      grid_.setInputCloud (input);
      grid_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      grid_.filter (*grid);

      /*if (compute_mesh_resolution_)
      {
        pcl::visualization::PCLVisualizer vis ("VIS");
        vis.addPointCloud<PointInT> (grid);
        vis.spin ();
      }*/
    }
    else
    {
      std::cout << "Do not apply voxel gridding" << std::endl;
      copyPointCloud (*input, *grid);
    }

    if (grid->points.size () == 0)
    {
      std::cout << "Cloud has no points after voxel grid, wont be able to compute normals!" << std::endl;
      return false;
    }

    PointInTPtr gridFiltered (new pcl::PointCloud<PointInT> ());
    if (apply_radius_removal_)
    {
      //in synthetic views the render grazes some parts of the objects that are barely seen from that VP
      //thus creating a very sparse set of points that causes the normals to be very noisy!!
      //remove these points
      pcl::RadiusOutlierRemoval < PointInT > sor;
      sor.setInputCloud (grid);
      sor.setRadiusSearch (leaf_size_ * leaf_size_factor_);
      sor.setMinNeighborsInRadius (8);
      sor.filter (*gridFiltered);

      /*if (compute_mesh_resolution_)
      {
        pcl::visualization::PCLVisualizer vis ("VIS_filtered");
        vis.addPointCloud<PointInT> (gridFiltered);
        vis.spin ();
      }*/
    }
    else
    {
      copyPointCloud (*grid, *gridFiltered);
      std::cout << "Do not apply radius removal" << std::endl;
    }

    if (gridFiltered->points.size () == 0)
    {
      std::cout << "Cloud has no points after voxel grid and filtering, wont be able to compute normals or descriptor"
          << std::endl;
      return false;
    }

    typedef typename pcl::NormalEstimation<PointInT, PointOutT> NormalEstimator_;
    NormalEstimator_ n3d;

    KdTreeInPtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointInT> > (false);
    normals_tree->setInputCloud (gridFiltered);
    n3d.setRadiusSearch (leaf_size_ * leaf_size_factor_);

    //std::cout << "radius:" << leaf_size_ * leaf_size_factor_ << std::endl;

    n3d.setSearchMethod (normals_tree);
    n3d.setInputCloud (gridFiltered);
    n3d.compute (*cloud_normals);

    //std::cout << "cloud_normals size:" << cloud_normals->points.size () << std::endl;

    for (size_t i = 0; i < cloud_normals->points.size (); ++i)
    {
      cloud_normals->points[i].getVector4fMap () = gridFiltered->points[i].getVector4fMap ();
    }

    //check nans...
    int j = 0;
    for (size_t i = 0; i < cloud_normals->points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_normals->points[i].normal_x) || !pcl_isfinite (cloud_normals->points[i].normal_y)
          || !pcl_isfinite (cloud_normals->points[i].normal_z))
        continue;

      cloud_normals->points[j] = cloud_normals->points[i];
      j++;
    }

    cloud_normals->points.resize (j);
    cloud_normals->width = j;
    cloud_normals->height = 1;
    return true;
  }

template class NormalEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class NormalEstimator<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ;
template class NormalEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::SHOT> ;
template class NormalEstimator<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::SHOT> ;

