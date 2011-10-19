/**
 * @file Line3D.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Base class for stereo calculated line in 3D.
 */

#include "Line3D.h"
#include "Line.hh"
//#include "VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class Line3D for Kinect.
 */
Line3D::Line3D(unsigned _vs3ID, std::vector<cv::Vec4f> &_p) : Gestalt3D(Gestalt3D::LINE)
{
  vs3ID = _vs3ID;
  point[0] = _p[0];
  point[1] = _p[_p.size()-1];
  center3D[0] = (point[0][0]+point[1][0]) / 2;
  center3D[1] = (point[0][1]+point[1][1]) / 2;
  center3D[2] = (point[0][2]+point[1][2]) / 2;
  dir.x = point[1][0] - point[0][0];
  dir.y = point[1][1] - point[0][1];
  dir.z = point[1][2] - point[0][2];

if(dir.x == 0 && dir.y == 0 && dir.z == 0){
  printf("dir: %4.3f-%4.3f-%4.3f\n", dir.x, dir.y, dir.z);
}
  double norm = cv::norm(dir);
  dir.x = dir.x/norm;
  dir.y = dir.y/norm;
  dir.z = dir.z/norm;
if(dir.x != dir.x){
  printf("        norm: %4.3f\n", norm);
  printf("        dir: %4.3f-%4.3f-%4.3f\n", dir.x, dir.y, dir.z);
}

  edge = _p;
}
  
/**
 * @brief Constructor of class Line3D for Stereo.
 */
Line3D::Line3D(unsigned vs3IDleft, unsigned vs3IDright) : Gestalt3D(Gestalt3D::LINE)
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
void Line3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
{
  // Significance is between 0 and 1
  double sig2Dleft = fabs(angle2Dleft/(M_PI/2.));
  double sig2Dright = fabs(angle2Dright/(M_PI/2.));
  double sig3D = 1- angle3Dz/(M_PI/2.);	
  
  sig = 1- (1-sig2Dleft) * (1-sig2Dright) * (1-sig3D);
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Line3D::GetLinks(vector<GraphLink> &links)
{
printf("Line3D::GetLinks: Not yet implemented!\n");

// // printf("Line3D::GetLinks: sig: %4.3f\n", sig);
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
 * @param use_color Use the color value
 */
void Line3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color, float color)
{
  RGBValue col;
  if(randomColor) col.float_value = GetRandomColor();
  else if(use_color) col.float_value = color;
  else col.float_value = point[0][3];

  tgRenderer->AddLine3D(point[0][0], point[0][1], point[0][2], 
                        point[1][0], point[1][1], point[1][2], 
                        col.r, col.g, col.b, 2);
  
  if(drawNodeID)
  {
    char label[5];
    snprintf(label, 5, "%u", GetNodeID());
    tgRenderer->AddLabel3D(label, 12,
                          (point[0][0] + point[1][0])/2., 
                          (point[0][1] + point[1][1])/2.,
                          (point[0][2] + point[1][2])/2.);
  }
}


/**
 * @brief Draw this 3D Gestalt to an image
 * @param image Image
 * @param camPars Camera parameters
 */
void Line3D::DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters &camPars)
{
//   static int times = 1;          /// TODO Write only every 10th line to image! Remove later!
//   times--;
//   
//   if(times == 0)
//   {
//     times = 10;
    
  double u1, v1, u2, v2;
  double scale = 1;
  if(point[0][0] == 0.) point[0][0]=0.000001;   // instead: assert(Z != 0.);
  if(point[1][0] == 0.) point[1][0]=0.000001;   // instead: assert(Z != 0.);

  if(image.cols !=0) scale = camPars.width / image.cols; /// image width
  u1 = camPars.fx*point[0][0]/scale + camPars.cx*point[0][2]/scale;
  v1 = camPars.fy*point[0][1]/scale + camPars.cy*point[0][2]/scale;
  u1 /= point[0][2];
  v1 /= point[0][2];

  if(image.cols !=0) scale = camPars.width / image.cols; /// image width
  u2 = camPars.fx*point[1][0]/scale + camPars.cx*point[1][2]/scale;
  v2 = camPars.fy*point[1][1]/scale + camPars.cy*point[1][2]/scale;
  u2 /= point[1][2];
  v2 /= point[1][2];
  
  int x1 = (int) u1;
  int y1 = (int) v1;
  int x2 = (int) u2;
  int y2 = (int) v2;
    
  cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), CV_RGB(std::rand()%255, std::rand()%255, std::rand()%255), 2);
  
//   char labl[5];
//   double c_u = camPars.fx*center3D.x + camPars.cx*center3D.z;
//   double c_v = camPars.fy*center3D.y + camPars.cy*center3D.z;
//   int c_x = (int) (c_u / center3D.z);
//   int c_y = (int) (c_v / center3D.z);
//   snprintf(labl, 5, "%u", GetNodeID());
//   cv::putText(image, string(labl), cv::Point(c_x, c_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 1);
//   }
}

/**
 * @brief Print this 3D Gestalt to the console.
 */
void Line3D::PrintGestalt3D()
{
  printf("\nLine3D: %u\n", id);
  printf(" vs3ID: %u\n", vs3ID);
//   printf(" nrSegments: %u\n", points.size());
  
//   for(unsigned idx=0; idx<points.size(); idx++)
//     printf(" point: %4.3f / %4.3f / %4.3f\n", points[idx][0], points[idx][1], points[idx][2]);
}

/**
 * @brief Calculate minimum proximity between endpoints
 * @param l Second line
 */
double Line3D::LLProximity(Line3D *l)
{
  double dist0 = sqrt(pow(point[0][0] - l->point[0][0], 2) + 
                     pow(point[0][1] - l->point[0][1], 2) + 
                     pow(point[0][2] - l->point[0][2], 2));
  double dist1 = sqrt(pow(point[0][0] - l->point[1][0], 2) + 
                     pow(point[0][1] - l->point[1][1], 2) + 
                     pow(point[0][2] - l->point[1][2], 2));
  double dist2 = sqrt(pow(point[1][0] - l->point[0][0], 2) + 
                     pow(point[1][1] - l->point[0][1], 2) + 
                     pow(point[1][2] - l->point[0][2], 2));
  double dist3 = sqrt(pow(point[1][0] - l->point[1][0], 2) + 
                     pow(point[1][1] - l->point[1][1], 2) + 
                     pow(point[1][2] - l->point[1][2], 2));
  return min(min(dist0, dist1), min(dist2, dist3));
}

/**
 * @brief Calculate minimum proximity between endpoints
 * @param l Second line
 */
double Line3D::LLParallelity(Line3D *l)
{
  cv::Point3f dir1 = l->GetDirection();
  double dot = dir.x * dir1.x + dir.y * dir1.y + dir.z * dir1.z;
  double alpha = acos(dot);
  if (alpha > M_PI/2) alpha = M_PI - alpha;

  if(alpha != alpha)
  {
    printf("Line3D::LLParallelity: Warning: Angle between lines returns 'nan'\n");
    return -1;
  }
  return alpha;
}

}


