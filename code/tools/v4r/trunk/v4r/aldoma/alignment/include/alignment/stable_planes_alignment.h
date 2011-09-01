/*
 * stable_planes_alignment.h
 *
 *  Created on: Apr 16, 2011
 *      Author: aa
 */

#ifndef STABLE_PLANES_ALIGNMENT_H_
#define STABLE_PLANES_ALIGNMENT_H_

#include <stable_planes.hpp>
#include "cv.h"
#include "highgui.h"
#include <pcl/common/time.h>
#include <vtkSmartPointer.h>
#include <pcl/io/pcd_io.h>
#include "alignment/fitness_scores.hpp"

class StablePlanesAlignment
{

private:
  double
  align (const pcl::PointCloud<pcl::PointXYZ> & pcd_i, const pcl::PointCloud<pcl::PointXYZ> & pcd_j,
         const Plane & plane_i, const Plane & plane_j, Eigen::Matrix4f & transform);
  void
  alignPointClouds (pcl::PointCloud<pcl::PointXYZ> & pcd_1, pcl::PointCloud<pcl::PointXYZ> & pcd_2,
                    Eigen::Matrix4f & transform);

  void
  alignPointClouds (pcl::PointCloud<pcl::PointXYZ> & pcd_1, pcl::PointCloud<pcl::PointXYZ> & pcd_2, Plane & plane_1,
                    std::vector<Plane> & planes_2, Eigen::Matrix4f & transform, int & best_plane_2);

  void
  alignPartialViewWithPointCloud (pcl::PointCloud<pcl::PointXYZ> & pcd_1, Plane & plane_1,
                                  pcl::PointCloud<pcl::PointXYZ> & pcd_2, Eigen::Matrix4f & transform);
  void
  projectOntoPlane (pcl::PointCloud<pcl::PointXYZ> & pcd, cv::Mat & projection);
  void
  align2D (const cv::Mat & proj_i, const cv::Mat & proj_j, double * rotation, double * scale);
  void
  logPolarTranslation (const cv::Mat & im1, const cv::Mat & im2, double * rotation, double * scale, int * maxII,
                       int * maxJJ);

  void
  logPolarTranslationDiff (const cv::Mat & im1, const cv::Mat & im2, double * rotation, double * scale, int * maxII,
                           int * maxJJ);

  void
  logPolarTranslationGPU (const cv::Mat & im1, const cv::Mat & im2, double * rotation, double * scale, int * maxII,
                          int * maxJJ);

  void
  align2DMultiScale (const cv::Mat & proj_i, const cv::Mat & proj_j, double * rotation, double * scale, double * d_x,
                     double * d_y);
  //double
  //crossCorrelationFFT (const cv::Mat & proj_i, const cv::Mat & proj_j, int * maxIP, int * maxJP, int scale_limit);

  int SAMPLE_POINTS_;
  bool VISUALIZE_;
  bool VISUALIZE_CROSS_;
  int MAX_PLANES_1_, MAX_PLANES_2_;
  int MAX_SIZE_PROJECTED_;
  vtkSmartPointer<vtkPolyData> polydata_1_;
  vtkSmartPointer<vtkPolyData> polydata_2_;
  std::string name_1, name_2;
  int SCALE_LIMIT_;
  bool PARALLEL_ALIGNMENT_;
  bool USE_GPU_;
  bool PARTIAL_VIEW_ALIGNMENT_;
  bool save_scale_applied_;
  float scale_applied_;
  float metric_;
  int multi_scale_levels_;

  FitnessScore fs_;

  inline int
  getScaleLogPolar (int size)
  {
    switch (size)
    {
      case 64:
        return 10;
      case 128:
        return 15;
      case 512:
        return 30;
      case 256:
      default:
        return 20;

    }
  }

  inline int
  getXYRange (int size)
  {
    switch (size)
    {
      case 64:
        return 7;
      case 128:
        return 10;
      case 512:
        return 4;
      case 256:
      default:
        return 5;
    }
  }

  inline int
  getScaleRange (int size)
  {
    switch (size)
    {
      case 64:
        return 3;
      case 128:
        return 6;
      case 512:
        return 10;
      case 256:
      default:
        return 8;
    }
  }

public:

  StablePlanesAlignment (int SAMPLE_POINTS, int FS_TYPE = 0, bool PARALLEL = false, bool GPU = false, int max_size_projected=256, int multi_scale_levels=0);

  /** \brief Align two meshes by sampling them into point clouds and calling alignPointClouds
   * \param mesh1 The query mesh
   * \param mesh2 The target mesh
   * \param transform The transformation that aligns mesh1 to mesh2
   */
  void
  alignMeshes (vtkSmartPointer<vtkPolyData> mesh1, vtkSmartPointer<vtkPolyData> mesh2, Plane & plane_1,
               std::vector<Plane> & planes_2, Eigen::Matrix4f & transform, int & best_plane_2);

  /** \brief Align two meshes by sampling them into point clouds and calling alignPointClouds
   * \param mesh1 The query mesh
   * \param mesh2 The target mesh
   * \param transform The transformation that aligns mesh1 to mesh2
   */
  void
  alignMeshes (vtkSmartPointer<vtkPolyData> mesh1, vtkSmartPointer<vtkPolyData> mesh2, Eigen::Matrix4f & transform);

  /** \brief Align a partial view with a table plane to a mesh. The mesh is sampled and alignPartialViewWithPointCloud is called
   * \param pcd_1 The target partial view
   * \param plane_1 The table plane or support plane where the partial view stands.
   * \param mesh The query mesh
   * \param transform The transformation that aligns mesh to the partial view
   */
  void
  alignPartialViewWithMesh (pcl::PointCloud<pcl::PointXYZ> & pcd_1, Plane & plane_1, vtkPolyData * mesh,
                            Eigen::Matrix4f & transform);

  void
  setName1 (std::string name)
  {
    name_1 = name;
  }

  void
  setName2 (std::string name)
  {
    name_2 = name;
  }

  void
  setVisualize (bool value)
  {
    VISUALIZE_ = value;
  }

  void
  setSaveScale(bool value) {
    save_scale_applied_ = value;
  }

  float getScaleApplied() {
    return scale_applied_;
  }

  float getMetricForBestAlignment() {
    return metric_;
  }
};

#endif /* STABLE_PLANES_ALIGNMENT_H_ */
