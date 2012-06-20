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
 * @param _vs3ID Id from vs3
 * @param _p Edge points
 * @param _es Edge-support (color, depth, mask, curvature)
 */
Segment3D::Segment3D(unsigned _vs3ID, 
                     std::vector<cv::Vec4f> &_points, 
                     std::vector<int> &_indexes,
                     std::vector<cv::Vec4f> &_es) : Gestalt3D(Gestalt3D::SEGMENT)
{
  vs3ID = _vs3ID;
  points = _points;
  indexes = _indexes;
  edge_support = _es;
  point[0][0] = points[0][0];
  point[0][1] = points[0][1];
  point[0][2] = points[0][2];
  point[1][0] = points[points.size()-1][0];
  point[1][1] = points[points.size()-1][1];
  point[1][2] = points[points.size()-1][2];
  center3D[0] = (points[0][0] + points[points.size()-1][0])/2.;
  center3D[1] = (points[0][1] + points[points.size()-1][1])/2.;
  center3D[2] = (points[0][2] + points[points.size()-1][2])/2.;
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
 * @param randomColor Use a random color
 * @param use_color Use the delivered color
 * @param color Color as float value
 */
void Segment3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color, float color)
{
  RGBValue col;
  if(randomColor) col.float_value = GetRandomColor();
  else if(use_color) col.float_value = color;
  
  for(unsigned idx=0; idx<points.size()-1; idx++)
  {
    if(!use_color && ! randomColor)
    {
//       col.float_value = points[idx][3];
      col.r = 255;
      col.g = 255;
      col.b = 255;
      // We use colors from the edge support
      if(edge_support[idx][1] > edge_support[idx][2])
      {
        if(edge_support[idx][1] > edge_support[idx][3]) {       // depth .. 1
          col.g = (1-edge_support[idx][1])*255;
          col.b = (1-edge_support[idx][1])*255;
        }
        else {                                                  // curvature ..3
          col.r = (1-edge_support[idx][3])*255;
          col.g = (1-edge_support[idx][3])*255;
        }
      } else if(edge_support[idx][2] > edge_support[idx][3]) {  // mask .. 2
          col.r = (1-edge_support[idx][2])*255;
          col.b = (1-edge_support[idx][2])*255;
        }
        
    }
    tgRenderer->AddLine3D(points[idx][0], points[idx][1], points[idx][2], 
                          points[idx+1][0], points[idx+1][1], points[idx+1][2], 
                          col.r, col.g, col.b, 5);
  }
  if(drawNodeID)
  {
    char label[5];
    snprintf(label, 5, "%u", GetNodeID());
    tgRenderer->AddLabel3D(label, 14, center3D[0], center3D[1], center3D[2]);
  }
}

/**
 * @brief Draw this 3D Gestalt to an image
 * @param image Image
 * @param camPars Camera parameters
 */
void Segment3D::DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters &camPars)
{
  double u1, v1, u2, v2;
  double scale = 1;

  CvScalar color = CV_RGB(std::rand()%255, std::rand()%255, std::rand()%255);
  for(unsigned i=0; i<points.size()-1; i++)
  {
    
    if(image.cols !=0) scale = camPars.width / image.cols; /// image width
    u1 = camPars.fx*points[i][0]/scale + camPars.cx*points[i][2]/scale;
    v1 = camPars.fy*points[i][1]/scale + camPars.cy*points[i][2]/scale;
    u1 /= points[i][2];
    v1 /= points[i][2];
    
    if(image.cols !=0) scale = camPars.width / image.cols; /// image width
    u2 = camPars.fx*points[i+1][0]/scale + camPars.cx*points[i+1][2]/scale;
    v2 = camPars.fy*points[i+1][1]/scale + camPars.cy*points[i+1][2]/scale;
    u2 /= points[i+1][2];
    v2 /= points[i+1][2];
    
    int x1 = (int) u1;
    int y1 = (int) v1;
    int x2 = (int) u2;
    int y2 = (int) v2;
    cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
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


