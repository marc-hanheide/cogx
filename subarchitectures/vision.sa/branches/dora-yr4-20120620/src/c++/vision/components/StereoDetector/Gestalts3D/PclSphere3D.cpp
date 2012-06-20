/**
 * @file PclSphere3D.cpp
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for pcl-calculated spheres in 3D.
 */

#include "PclSphere3D.h"
#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class PclSphere3D for KinectPclModel
 * @param _points Point cloud of the Sphere
 * @param _indexes Indexes of the point cloud refering to the 2D image
 */
PclSphere3D::PclSphere3D(std::vector<cv::Vec4f> _points,
                         std::vector<int> _indices,
                         std::vector<int> _mask_hull_indices) : Gestalt3D(Gestalt3D::PCL_SPHERE)
{
  points = _points;
  indices = _indices;
  mask_hull_indices = _mask_hull_indices;
  
  // Take center of first and last point !!! quick HACK => Get later model parameter from pcl
  center3D[0] = (points[0][0] + points[points.size()-1][0])/2.;
  center3D[1] = (points[0][1] + points[points.size()-1][1])/2.;
  center3D[2] = (points[0][2] + points[points.size()-1][2])/2.;

}

/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void PclSphere3D::CalculateSignificance(double sigLeft, double sigRight)
{
//   sig = 1- (1-sigLeft) * (1-sigRight);
}

/**
 * @brief Draw this 3D Gestalt to the TomGine render engine.
 * @param tgRenderer Render engine
 * @param randomColor Use random color
 * @param use_color Use the delivered color
 * @param color RGB color value
 */
void PclSphere3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, 
                                bool randomColor, bool use_color, float color)
{
  RGBValue col;
  if(randomColor) col.float_value = GetRandomColor();
  else if(use_color) col.float_value = color;
  
  std::vector<cv::Vec4f> col_points;
  for(unsigned i=0; i<points.size(); i++)
  {
    col_points.push_back(points[i]);
    if(!use_color && !randomColor) col.float_value = points[i][3];
    col_points[i][3] = col.float_value;
  }
  tgRenderer->AddPointCloud(col_points);
    
}

}


