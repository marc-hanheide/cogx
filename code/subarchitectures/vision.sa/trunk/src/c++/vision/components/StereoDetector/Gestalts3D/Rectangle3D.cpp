/**
 * @file Rectangle3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#include "Rectangle3D.h"

namespace Z
{

/**
 * @brief Constructor of Class Rectangle3D
 */
Rectangle3D::Rectangle3D() : Gestalt3D(Gestalt3D::RECTANGLE)
{
}

/**
 * @brief Constructor of Class Closure3D for KinectClosure
 */
Rectangle3D::Rectangle3D(std::vector<cv::Vec4f> _plane, std::vector<cv::Vec4f> _hull) : Gestalt3D(Gestalt3D::RECTANGLE)
{
  plane = _plane;
  hull = _hull;
  
  int nr_bins = 10;
  hist = new ColorHistogram(nr_bins, plane);
}

/**
 * @brief Compare histogram of closure with another closure's histogram.
 * @param p Patch to compare with
 */
double Rectangle3D::Compare(Rectangle3D *r)
{
  return hist->Compare(r->hist);
}

/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void Rectangle3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = 1- (1-sigLeft) * (1-sigRight);
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Rectangle3D::GetLinks(vector<GraphLink> &links)
{
//   printf("Rectangle3D::GetLinks: sig: %4.3f\n", sig);

  for(unsigned id=0; id<surf.vertices.Size(); id++)
  {
    unsigned id2 = id+1;
    if(id2 >= surf.vertices.Size()) id2 = 0;
    
    GraphLink l;
    l.node[0].x = surf.vertices[id].p.x;
    l.node[0].y = surf.vertices[id].p.y;
    l.node[0].z = surf.vertices[id].p.z;
    l.node[1].x = surf.vertices[id2].p.x;
    l.node[1].y = surf.vertices[id2].p.y;
    l.node[1].z = surf.vertices[id2].p.z;
    l.probability = sig;
    links.push_back(l);
  }
  return true;
}

/**
 * @brief Draw this 3D Gestalt to the TomGine render engine.
 * @param tgRenderer Render engine
 * @param randomColor Use random color  TODO unused
 */
void Rectangle3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor)
{
  RGBValue color;
  color.float_value = hull[0][3];
  for(unsigned i=0; i < hull.size()-1; i++)
    tgRenderer->AddLine3D(hull[i][0], hull[i][1], hull[i][2], hull[i+1][0], hull[i+1][1], hull[i+1][2], color.r, color.g, color.b);
  tgRenderer->AddLine3D(hull[0][0], hull[0][1], hull[0][2], hull[hull.size()-1][0], hull[hull.size()-1][1], hull[hull.size()-1][2], color.r, color.g, color.b);
}

}


