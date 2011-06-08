/**
 * @file Patch3D.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Base class for 3D plane patches.
 */

#include "Patch3D.h"

namespace Z
{

/**
 * @brief Constructor of class Patch3D for Kinect.
 */
Patch3D::Patch3D(std::vector<cv::Vec4f> _p, std::vector<cv::Vec4f> _h_p) : Gestalt3D(Gestalt3D::PATCH)
{
  points = _p;
  hull_points = _h_p;
  
  int nr_bins = 10;
  hist = new ColorHistogram(nr_bins, points);
}


/**
 * @brief Calculate significance for the stereo Gestalt.
 * 2D horizontal lines are worser to match in 3D, therefore lower significance.
 * 3D Lines with main direction in z-coordinte.
 * @param angle2Dleft
 * @param angle2Dright 
 * @param angle3Dz Opening angle between z-axis and Line in 3D
 */
void Patch3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
{
//   // Significance is between 0 and 1
//   double sig2Dleft = fabs(angle2Dleft/(M_PI/2.));
//   double sig2Dright = fabs(angle2Dright/(M_PI/2.));
//   double sig3D = 1- angle3Dz/(M_PI/2.);	
//   
//   sig = 1- (1-sig2Dleft) * (1-sig2Dright) * (1-sig3D);
}

/**
 * @brief Compare histogram of patch with another patch's histogram.
 * @param p Patch to compare with
 */
double Patch3D::Compare(Patch3D *p)
{
  return hist->Compare(p->hist);
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Patch3D::GetLinks(vector<GraphLink> &links)
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
void Patch3D::DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool randomColor)
{
  tgRenderer->AddPointCloud(points);
  tgRenderer->AddConvexHull(hull_points);
}

/**
 * @brief Print this 3D Gestalt to the console.
 */
void Patch3D::PrintGestalt3D()
{
  printf("\Patch3D: %u\n", id);
  printf(" points.size: %u\n", points.size());
  
  for(unsigned idx=0; idx<points.size(); idx++)
    printf(" point: %4.3f / %4.3f / %4.3f\n", points[idx][0], points[idx][1], points[idx][2]);
}


}


