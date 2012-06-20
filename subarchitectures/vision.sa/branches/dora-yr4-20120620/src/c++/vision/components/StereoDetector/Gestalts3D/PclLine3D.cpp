/**
 * @file PclLine3D.cpp
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#include "PclLine3D.h"
#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class PclLine3D
 */
PclLine3D::PclLine3D() : Gestalt3D(Gestalt3D::PCL_LINE)
{
}

/**
 * @brief Constructor of class PclLine3D for KinectPclLines
 */
PclLine3D::PclLine3D(std::vector<cv::Vec4f> _points)
         : Gestalt3D(Gestalt3D::PCL_LINE)
{
  points = _points;
}

/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void PclLine3D::CalculateSignificance(double sigLeft, double sigRight)
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
void PclLine3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, 
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
    
  // add convex hull of patch
/*  for(unsigned j=0; j < hull_points.size(); j++)
  {
    int k = j+1;
    if(k == hull_points.size()) k=0;
    cv::Vec4f s = hull_points[j];
    cv::Vec4f e = hull_points[k];
    if(!use_color && !randomColor) col.float_value = hull_points[j][3];
    tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 
                          col.r, col.g, col.b, 2);
  } */ 
}

}


