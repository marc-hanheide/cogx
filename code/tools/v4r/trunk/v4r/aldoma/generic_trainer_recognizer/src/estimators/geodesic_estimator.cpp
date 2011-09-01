#include "estimators/geodesic_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/common/time.h>
#include "pcl/io/io.h"
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class GeodesicDistances
{

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;
  typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;
  typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;

private:
  boost::shared_ptr<std::vector<int> > longest_shortest_path_;
  boost::shared_ptr<std::vector<std::vector<int> > > longest_shortest_paths_to_all_;
  boost::shared_ptr<std::vector<double> > distances_;

  bool create_path_;
  double grid_size_;

public:
  GeodesicDistances ()
  {
    create_path_ = false;
    grid_size_ = 0.005;
  }

  void
  setCreatePaths (bool create)
  {
    create_path_ = create;
  }

  void
  setSearchStep (double step)
  {
    grid_size_ = step;
  }

  void
  getLongestShortestPath (boost::shared_ptr<std::vector<int> > & path)
  {
    path = longest_shortest_path_;
  }

  void
  getLongestShortestPaths (boost::shared_ptr<std::vector<std::vector<int> > > & path)
  {
    path = longest_shortest_paths_to_all_;
  }

  void
  getDistances (boost::shared_ptr<std::vector<double> > & distances)
  {
    distances = distances_;
  }

  FeatureTPtr
  computeDescriptor ()
  {

    FeatureTPtr output (new pcl::PointCloud<FeatureT>);
    output->points.resize (1);
    output->width = 1;
    output->height = 1;

    //take longest_shortest_paths_to_all_
    float hist_incr;
    bool normalize_bins_ = false;

    if (normalize_bins_)
    {
      hist_incr = 100.0 / (float)(distances_->size () - 1);
    }
    else
    {
      hist_incr = 1.0;
    }

    double MAX_DIST = -1;
    for (size_t i = 0; i < distances_->size (); i++)
    {
      if (MAX_DIST < distances_->at (i))
      {
        MAX_DIST = distances_->at (i);
      }
    }

    //USE MAX_DIST as normalization factor...

    int nr_bins = 100;
    int h_index;
    for (size_t i = 0; i < distances_->size (); i++)
    {
      h_index = floor (nr_bins * (distances_->at (i) / MAX_DIST));
      output->points[0].histogram[h_index] += hist_incr;
    }

    return output;
  }

  /*FeatureTPtr
  computeDescriptorGeodesicDivEuclidean (PointOutTPtr input)
  {

    pcl::PointCloud<pcl::VFHSignature308>::Ptr geodesic_hist = computeDescriptor ();

    pcl::PointCloud<pcl::VFHSignature308>::Ptr output (new pcl::PointCloud<pcl::VFHSignature308>);
    output->points.resize (1);
    output->width = 1;
    output->height = 1;

    Eigen::Vector4f centroid;
    compute3DCentroid (*input, centroid);
    // TODO: Finish this...

    //take longest_shortest_paths_to_all_
    float hist_incr;
    bool normalize_bins_ = true;

    if (normalize_bins_)
    {
      hist_incr = 100.0 / (float)(distances_->size () - 1);
    }
    else
    {
      hist_incr = 1.0;
    }

    double MAX_DIST = -1;
    for (size_t i = 0; i < distances_->size (); i++)
    {
      if (MAX_DIST < distances_->at (i))
      {
        MAX_DIST = distances_->at (i);
      }
    }

    //USE MAX_DIST as normalization factor...

    int nr_bins = 100;
    int h_index;
    for (size_t i = 0; i < distances_->size (); i++)
    {
      h_index = floor (nr_bins * (distances_->at (i) / MAX_DIST));
      output->points[0].histogram[h_index] += hist_incr;
    }

    return output;
  }*/

  /*
   * Function that computes the geodesic distance from a given starting point.
   * Returns the distance to the point farthest away in the geodesic sense...
   */

  double
  SBDT (const pcl::PointCloud<PointOutT> & cloud, const KdTreePtr &tree, int startp_idx)
  {
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);
    std::vector<double> distances (cloud.points.size (), 0);
    std::vector<std::vector<int> > shortest_path (cloud.points.size (), std::vector<int> ());

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    std::vector<unsigned int> seed_queue;
    int sq_idx = 0;

    seed_queue.push_back (startp_idx);
    processed[startp_idx] = true;

    double distance = 0.0;
    double dist;

    while (sq_idx < (int)seed_queue.size ())
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], grid_size_, nn_indices, nn_distances))
      {
        //No neighbours found...
        sq_idx++;
        continue;
      }

      for (size_t j = 0; j < nn_indices.size (); ++j)

      {
        nn_distances[j] = sqrt (nn_distances[j]);
        if (processed[nn_indices[j]]) // Has this point been processed before ?

        {
          //We might need to update the distance
          dist = (nn_distances[j] + distances[seed_queue[sq_idx]]);
          if (distances[nn_indices[j]] > dist) //TODO: What if distances are equal??

          {
            distances[nn_indices[j]] = dist;
            seed_queue.push_back (nn_indices[j]);

            if (create_path_)
            {
              //update the path as we found a shorter one
              shortest_path[nn_indices[j]] = shortest_path[seed_queue[sq_idx]];
              shortest_path[nn_indices[j]].push_back (seed_queue[sq_idx]); //add point to path...
              shortest_path[nn_indices[j]].push_back (nn_indices[j]);
            }
          }
          continue;
        }

        if (create_path_)
        {
          shortest_path[nn_indices[j]] = shortest_path[seed_queue[sq_idx]]; //copy path
          shortest_path[nn_indices[j]].push_back (seed_queue[sq_idx]); //add point to path...
          shortest_path[nn_indices[j]].push_back (nn_indices[j]);
        }

        distances[nn_indices[j]] = nn_distances[j] + distances[seed_queue[sq_idx]]; //distance to seed + distance from origin to seed
        processed[nn_indices[j]] = true;
        seed_queue.push_back (nn_indices[j]);
      }

      sq_idx++;
    }

    int max_dist_idx = -1;
    double MAX_DIST = -1;
    for (size_t i = 0; i < distances.size (); i++)
    {
      if (distances[i] > MAX_DIST)
      {
        MAX_DIST = distances[i];
        max_dist_idx = i;
      }
    }

    if (create_path_)
    {
      longest_shortest_path_ = boost::make_shared<std::vector<int> > (shortest_path[max_dist_idx]);
      longest_shortest_paths_to_all_ = boost::make_shared<std::vector<std::vector<int> > > (shortest_path);
      distances_ = boost::make_shared<std::vector<double> > (distances);
    }

    return MAX_DIST;
  }

};

typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  GeodesicEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<
      pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
      Eigen::Vector3f> & centroids)
  {

    std::cout << "GeodesicEstimator::estimate()" << std::endl;
    computeNormals (in, out);

    //..compute
    pcl::PointCloud < FeatureT > signature;

    //compute geodesic center
    GeodesicDistances<PointInT, PointOutT, FeatureT> gd;
    gd.setSearchStep (0.0075);

    std::cout << "points:" << out->points.size() << std::endl;

    KdTreePtr tree (new pcl::KdTreeFLANN<pcl::PointNormal> (false));
    tree->setInputCloud (out);

    int geodesic_center = -1;

    {
      pcl::ScopeTime t ("------- GEODESIC CENTER");
      double min_longest_shortest_path = std::numeric_limits<double>::max ();
      int min_idx = -1;

      for (size_t k = 0; k < out->points.size (); k += 2)
      {
        int ptx_idx = k;
        double longest_shortest_path_dist = gd.SBDT (*out, tree, ptx_idx);

        if (longest_shortest_path_dist > 0)
        {
          if (longest_shortest_path_dist < min_longest_shortest_path)
          {
            min_longest_shortest_path = longest_shortest_path_dist;
            min_idx = k;
          }
        }
      }

      geodesic_center = min_idx;
    }

    gd.setSearchStep (0.0075);
    gd.setCreatePaths (true);

    //Build paths from the geodesic center to the other points...
    int ptx_idx = geodesic_center;
    double longest_shortest_path_dist = gd.SBDT (*out, tree, ptx_idx);

    std::vector<int> longest_shortest_path;
    boost::shared_ptr<std::vector<std::vector<int> > > longest_shortest_paths_to_all;
    gd.getLongestShortestPaths (longest_shortest_paths_to_all);

    //Compute descriptor...
    pcl::PointCloud<pcl::VFHSignature308>::Ptr histogram = gd.computeDescriptor ();

    signatures.push_back (*histogram);

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid (*in, centroid4f);
    Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);
    centroids.push_back (centroid3f);

    //and roll histograms for each signature
    std::vector<cv::Mat> roll_histograms_freq_domain;
    roll_histograms_freq_domain.resize (signatures.size ());

    for (size_t idx = 0; idx < signatures.size (); idx++)
    {
      computeRollHistogram (out, centroids[idx], roll_histograms_freq_domain[idx]);
    }

    //save roll_histograms_freq_domain_
    roll_histograms_freq_domain_ = roll_histograms_freq_domain;
  }

template class GeodesicDistances<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class GeodesicEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
