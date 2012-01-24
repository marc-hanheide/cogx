/**
 * @file PclCylinder3D.cpp
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for pcl-calculated cylinders in 3D.
 */

#include "PclCylinder3D.h"
#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class PclCylinder3D
 */
PclCylinder3D::PclCylinder3D() : Gestalt3D(Gestalt3D::PCL_CYLINDER)
{
  printf("PclCylinder3D::PclCylinder3D: Warning: Antiquated header!\n");
  calcCenter3D = false;
}

/**
 * @brief Constructor of class PclCylinder3D for KinectPclCylinders
 * @param _points Vector with point cloud in 3D
 * @param _indexes Indexes of the points refering to the 2D image
 */
PclCylinder3D::PclCylinder3D(std::vector<cv::Vec4f> _points, 
                             std::vector<int> _indices,
                             std::vector<int> _mask_hull_indices)  : Gestalt3D(Gestalt3D::PCL_CYLINDER)
{
  points = _points;
  indices = _indices;
  mask_hull_indices = _mask_hull_indices;
  calcCenter3D = false;
}

/**
 * @brief Calculate center point in 3D
 */
void PclCylinder3D::CalcCenter3D()
{
  if(!calcCenter3D)
  {
    int cnt = 0;
    center3D = cv::Vec3f(0., 0., 0.);
    unsigned inc = (unsigned) (points.size()/50);
    if(inc == 0) inc = 1;
    for(unsigned i=0; i<points.size(); i+=inc)
    {
      center3D[0] += points[i][0];
      center3D[1] += points[i][1];
      center3D[2] += points[i][2];
      cnt++;
    }
    center3D[0] /= (double) cnt;
    center3D[1] /= (double) cnt;
    center3D[2] /= (double) cnt;
    calcCenter3D = true;
  }
}


/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void PclCylinder3D::CalculateSignificance(double sigLeft, double sigRight)
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
void PclCylinder3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, 
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
    
  if(drawNodeID)
  {
    char label[5];
    snprintf(label, 5, "%u", GetNodeID());
    tgRenderer->AddLabel3D(label, 12, center3D[0], center3D[1], center3D[2]);
  }
  if(true)  // print object label!
  {
    char label[5];
    snprintf(label, 5, "%u", GetObjectLabel());
    tgRenderer->AddLabel3D(label, 12, center3D[0], center3D[1], center3D[2]);
  }
}

}


