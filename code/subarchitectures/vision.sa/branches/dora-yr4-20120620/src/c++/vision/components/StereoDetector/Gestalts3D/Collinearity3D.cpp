/**
 * @file Collinearity3D.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Base class for 3D collinearities.
 */

#include "Collinearity3D.h"
#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class Collinearity3D for Kinect.
 */
Collinearity3D::Collinearity3D(unsigned _vs3ID, Z::Line3D *_line[2]) : Gestalt3D(Gestalt3D::COLLINEARITY)
{
  vs3ID = _vs3ID;
  line[0] = _line[0];
  line[1] = _line[1];

// printf("New Collinearity 3D wird erzeugt: line[2]:\n");
// printf("  line[0].start/end: %4.4f/%4.4f/%4.4f und %4.4f/%4.4f/%4.4f\n",
//    _line[0]->point[0][0], _line[0]->point[0][1], _line[0]->point[0][2],
//    _line[0]->point[1][0], _line[0]->point[1][1], _line[0]->point[1][2]);
// printf("  line[1].start/end: %4.4f/%4.4f/%4.4f und %4.4f/%4.4f/%4.4f\n",
//    _line[1]->point[0][0], _line[1]->point[0][1], _line[1]->point[0][2],
//    _line[1]->point[1][0], _line[1]->point[1][1], _line[1]->point[1][2]);
  
  point[0] = _line[0]->point[1];                                            /// TODO TODO TODO Linien sind anscheinend richtig, aber die Punkte ergeben Blödsinn? 
  point[1][0] = (_line[0]->point[0][0] + _line[1]->point[0][0])/2.;
  point[1][1] = (_line[0]->point[0][1] + _line[1]->point[0][1])/2.;
  point[1][2] = (_line[0]->point[0][2] + _line[1]->point[0][2])/2.;
  point[1][3] = _line[0]->point[0][3];
  point[2] = _line[1]->point[1];

// printf("             points: %4.4f/%4.4f/%4.4f und %4.4f/%4.4f/%4.4f und %4.4f/%4.4f/%4.4f\n",
//   point[1][0], point[1][1], point[1][2],  point[0][0], point[0][1], point[0][2], point[2][0], point[2][1], point[2][2]);
}


/**
 * @brief Constructor of class Collinearity3D for stereo calculations.
 */
Collinearity3D::Collinearity3D(unsigned vs3IDleft, unsigned vs3IDright) : Gestalt3D(Gestalt3D::COLLINEARITY)
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
void Collinearity3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
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
bool Collinearity3D::GetLinks(vector<GraphLink> &links)
{
printf("Collinearity3D::GetLinks: Not yet implemented!\n");
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
 * @param tgRenderer TomGine render engine
 * @param use_color Use the delivered color
 * @param color Color as float value
 */
void Collinearity3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color, float color)
{
// printf("Collinearity3D::DrawGestalt3D: Time to implement!\n");
// printf("Collinearity3D::DrawGestalt3D: NodeIDs: %u-%u\n", line[0]->GetNodeID(), line[1]->GetNodeID());
// printf("Collinearity3D::DrawGestalt3D: NodeIDs: %u-%u\n", line[0]->GetNodeID(), line[1]->GetNodeID());
  
  RGBValue col;
  if(randomColor) col.float_value = GetRandomColor();
  else if(use_color) col.float_value = color;
  else col.float_value = point[0][3];
  
  line[0]->DrawGestalt3D(tgRenderer, true, col.float_value);
  line[1]->DrawGestalt3D(tgRenderer, true, col.float_value);
  
  tgRenderer->AddLine3D(point[0][0], point[0][1], point[0][2], point[1][0], point[1][1], point[1][2], col.r, col.g, col.b, 2);
  tgRenderer->AddLine3D(point[2][0], point[2][1], point[2][2], point[1][0], point[1][1], point[1][2], col.r, col.g, col.b, 2);
  tgRenderer->AddPoint3D(point[1][0], point[1][1], point[1][2], 255, 0, 0, 2);
}

/**
 * @brief Print this 3D Gestalt to the console.
 */
void Collinearity3D::PrintGestalt3D()
{
//   printf("\nSegment3D: %u\n", id);
//   printf(" vs3ID: %u\n", vs3ID);
//   printf(" nrSegments: %u\n", points.size());
//   
//   for(unsigned idx=0; idx<points.size(); idx++)
//     printf(" point: %4.3f / %4.3f / %4.3f\n", points[idx][0], points[idx][1], points[idx][2]);
}


}


