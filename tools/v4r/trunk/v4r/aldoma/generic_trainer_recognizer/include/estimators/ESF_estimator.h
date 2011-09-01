/*
 *
 *
 *  Created on: Aug 28, 2011
 *      Author: walter wohlkinger
 */

#ifndef ESF_ESTIMATOR_H_
#define ESF_ESTIMATOR_H_

#include <estimators/roll_hist_estimator.h>
#include <vector>
using namespace std;
#define GRIDSIZE 64
#define GRIDSIZE_H GRIDSIZE/2

template<typename PointInT, typename PointOutT, typename FeatureT>
  class ESF_Estimator : public Estimator<PointInT, PointOutT, FeatureT>
  {

    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

  public:
    vector<vector<vector<bool> > > lut;

    ESF_Estimator ()
    {
      lut.resize (GRIDSIZE);
      for (int i = 0; i < GRIDSIZE; ++i)
      {
        lut[i].resize (GRIDSIZE);
        for (int j = 0; j < GRIDSIZE; ++j)
          lut[i][j].resize (GRIDSIZE);
      }
    }

    void
         estimate (
                   PointInTPtr & in,
                   PointOutTPtr & out,
                   std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                   std::vector<Eigen::Vector3f> & centroids);





    int
    lci(int x1, int y1, int z1, int x2, int y2, int z2, double &ratio, int &incnt, int &pointcount);
    void     ESF (pcl::PointCloud<pcl::PointXYZ> &pc, vector<float> &hist);
    void     voxelize9 (pcl::PointCloud<pcl::PointXYZ> &cluster);
    void     cleanup9 (pcl::PointCloud<pcl::PointXYZ> &cluster);
    void     scale_points_unit_sphere (pcl::PointCloud<pcl::PointXYZ> &pc, float scalefactor);
    void     scale_points_unit_cube (pcl::PointCloud<pcl::PointXYZ> &pc, float scalefactor);

  };

#endif /* ESF_ESTIMATOR_H_ */
