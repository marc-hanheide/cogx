/**
 * @file Segment3D.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Base class for 3D segments.
 */

#include "Segment3D.h"

namespace Z
{

/**
 * @brief Constructor of class Segment3D for Kinect.
 */
Segment3D::Segment3D(unsigned _vs3ID, std::vector<cv::Vec4f> &_p) : Gestalt3D(Gestalt3D::SEGMENT)
{
  vs3ID = _vs3ID;
  points = _p;
}


/**
 * @brief Constructor of class Segment3D for stereo calculations.
 */
Segment3D::Segment3D(unsigned vs3IDleft, unsigned vs3IDright) : Gestalt3D(Gestalt3D::SEGMENT)
{
  vs3IDs[0] = vs3IDleft;
  vs3IDs[1] = vs3IDright;
}


/**
 * @brief Calculate significance for the stereo Gestalt.
 * 2D horizontal lines are worser to match in 3D, therefore lower significance.
 * 3D Lines with main direction in z-coordinte.
 * @param angle2Dleft
 * @param angle2Dright 
 * @param angle3Dz Opening angle between z-axis and Line in 3D
 */
void Segment3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
{
//   // Significance is between 0 and 1
//   double sig2Dleft = fabs(angle2Dleft/(M_PI/2.));
//   double sig2Dright = fabs(angle2Dright/(M_PI/2.));
//   double sig3D = 1- angle3Dz/(M_PI/2.);	
//   
//   sig = 1- (1-sig2Dleft) * (1-sig2Dright) * (1-sig3D);
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Segment3D::GetLinks(vector<GraphLink> &links)
{
printf("Segment3D::GetLinks: Not yet implemented!\n");
//   GraphLink l;
//   l.node[0].x = isct3D[0].p.x;
//   l.node[0].y = isct3D[0].p.y;
//   l.node[0].z = isct3D[0].p.z;
//   l.node[1].x = isct3D[1].p.x;
//   l.node[1].y = isct3D[1].p.y;
//   l.node[1].z = isct3D[1].p.z;
//   l.probability = sig;
//   
//   links.push_back(l);
//   return true;
  return false;
}

/**
 * @brief Draw this 3D Gestalt to the TomGine render engine.
 * @param tgRenderer Render engine
 */
void Segment3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor)
{
  RGBValue color;
  if(randomColor)
  {
    color.r = std::rand()%255;
    color.g = std::rand()%255;
    color.b = std::rand()%255;
  }

  for(unsigned idx=0; idx<points.size()-1; idx++)
  {
    if(!randomColor) color.float_value = points[idx][3];
    tgRenderer->AddLine3D(points[idx][0], points[idx][1], points[idx][2], points[idx+1][0], points[idx+1][1], points[idx+1][2], color.r, color.g, color.b, 1);
  }
}

/**
 * @brief Print this 3D Gestalt to the console.
 */
void Segment3D::PrintGestalt3D()
{
  printf("\nSegment3D: %u\n", id);
  printf(" vs3ID: %u\n", vs3ID);
  printf(" nrSegments: %u\n", points.size());
  
  for(unsigned idx=0; idx<points.size(); idx++)
    printf(" point: %4.3f / %4.3f / %4.3f\n", points[idx][0], points[idx][1], points[idx][2]);
}


}


