#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/ros/conversions.h>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transforms.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <Eigen/StdVector>

#include <alignment_ros_wrapper/stable_planes_alignment.h>

typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

#include "alignment_ros_wrapper/segmentation.h"

void
clusterCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //call segmentation
  std::vector<pcl::PointCloud<Point>,Eigen::aligned_allocator<pcl::PointCloud<Point> > > clusters;
  Eigen::Vector4f table_coeffs;

  {
    pcl::ScopeTime t ("-------- segmentation");
    segmentation (msg, clusters, table_coeffs);
  }

  ROS_INFO ("Number of clusters in scene %zu", clusters.size ());

  std::stringstream vis_name;
  vis_name << "Scene Alignment";
  pcl::visualization::PCLVisualizer vis (vis_name.str ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::fromROSMsg (*msg, *scene_cloud);

  //filter scene_cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PassThrough < pcl::PointXYZRGB > pass_;
  pass_.setFilterFieldName ("z");
  float max_z_bounds_ = 1.25;
  pass_.setFilterLimits (0.0, max_z_bounds_);
  pass_.setInputCloud (scene_cloud);
  pass_.filter (*scene_filtered);

  pcl::VoxelGrid < pcl::PointXYZRGB > grid_;
  grid_.setInputCloud (scene_filtered);
  grid_.setLeafSize (0.005, 0.005, 0.005);
  grid_.filter (*scene_filtered);

  pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZRGB > handler_whole_pc (scene_filtered, 0, 0, 125);
  vis.addPointCloud (scene_filtered, handler_whole_pc, "scene");

  for (size_t clus = 0; clus < clusters.size (); ++clus)
  {
    //transform cluster to msg...
    ROS_INFO ("Points in this cluster %zu", clusters[clus].size ());

    pcl::PointCloud< pcl::PointXYZ >::Ptr cluster_i(new pcl::PointCloud<Point>(clusters[clus]));

    //align object
    StablePlanesAlignment spa(15000, 0, false);
    //std::string path_to_model = "/home/aitor/data/benchmark/db_all/m507.ply";
    std::string path_to_model = "/home/aitor/data/benchmark/db_all/m489.ply";
    spa.setName2(path_to_model);
    Eigen::Matrix4f transform;

    Plane plane_1;
    plane_1.center[0] = -table_coeffs[0] * table_coeffs[3];
    plane_1.center[1] = -table_coeffs[1] * table_coeffs[3];
    plane_1.center[2] = -table_coeffs[2] * table_coeffs[3];

    plane_1.normal[0] = -table_coeffs[0];
    plane_1.normal[1] = -table_coeffs[1];
    plane_1.normal[2] = -table_coeffs[2];

    vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
    readerQuery->SetFileName (path_to_model.c_str());
    vtkSmartPointer < vtkPolyData > polydata1 = readerQuery->GetOutput ();
    polydata1->Update ();

    spa.alignPartialViewWithMesh(*cluster_i, plane_1, polydata1, transform);

    std::stringstream cluster_name;
    cluster_name << "cluster_" << clus;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler (cluster_i, 255, 0, 0);
    vis.addPointCloud (cluster_i, handler, cluster_name.str ());

    vtkSmartPointer < vtkTransform > transform_vtk = vtkSmartPointer<vtkTransform>::New ();
    vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
    //mat = transform->GetMatrix();
    for (size_t i = 0; i < 4; i++)
      for (size_t j = 0; j < 4; j++)
        mat->SetElement (i, j, transform(i,j));

    transform_vtk->SetMatrix (mat);
    vis.addModelFromPLYFile(path_to_model, transform_vtk);
  }

  vis.setBackgroundColor (0, 0, 0);
  vis.spin ();

  return;
}

int
main (int argc, char **argv)
{
  std::stringstream node_name;
  node_name << "scene_alignment";
  ros::init (argc, argv, node_name.str ());
  ros::NodeHandle n;

  //subscribe to clusters messages
  ros::Subscriber sub = n.subscribe ("input_cloud", 1, clusterCallback);
  std::cout << "READY FOR SOME SCENES:" << std::endl;
  ros::spin ();

  return 0;
}

