/**
 * @file PCLCommonHeaders.h
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief All headers for pcl utils and functions
 */

#ifndef PCLADDONS_COMMON_HEADERS_H
#define PCLADDONS_COMMON_HEADERS_H

#include <opencv/cxcore.h>

#include <boost/make_shared.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include "pcl/ModelCoefficients.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/io/pcd_io.h"

#include <pcl/features/normal_3d.h>

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/project_inliers.h"

#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/surface/convex_hull.h"

#include "pcl/sample_consensus/sac_model_plane.h"


#endif

/// TODO paththrough filter!!!
// #include "pcl/filters/passthrough.h"
