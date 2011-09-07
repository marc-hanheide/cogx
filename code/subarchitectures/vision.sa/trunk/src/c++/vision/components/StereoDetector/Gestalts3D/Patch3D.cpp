/**
 * @file Patch3D.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Base class for 3D plane patches.
 */

#include "Patch3D.h"
#include "MathUtils.h"
#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Constructor of class Patch3D for Kinect.
 * @param _p All points of the patch
 * @param _h_p Convex hull points of the patch
 */
Patch3D::Patch3D(std::vector<cv::Vec4f> _p, std::vector<cv::Vec4f> _h_p) : Gestalt3D(Gestalt3D::PATCH)
{
  points = _p;
  hull_points = _h_p;
  
  center3D.x = 0.;
  center3D.y = 0.;
  center3D.z = 0.;
  for(unsigned i=0; i< points.size(); i++)          /// TODO Schlechte Berechnung, vor allem bei vielen Punkten
  {
    center3D.x += points[i][0];
    center3D.y += points[i][1];
    center3D.z += points[i][2];
  }
  center3D.x = center3D.x / points.size();
  center3D.y = center3D.y / points.size();
  center3D.z = center3D.z / points.size();

  CalculatePlaneNormal();
  
  int nr_bins = 10;
  hist = new ColorHistogram(nr_bins, points);
}

/**
 * @brief Calculate the plane normal from the convex hull points.
 */
void Patch3D::CalculatePlaneNormal()
{
  // take 3 points of the convex hull and build two vectors => then cross product, then normalisation
  cv::Vec3f p0, p1, cross;
  p0[0] = hull_points[0][0] - hull_points[1][0];
  p0[1] = hull_points[0][1] - hull_points[1][1];
  p0[2] = hull_points[0][2] - hull_points[1][2];
  p1[0] = hull_points[1][0] - hull_points[2][0];
  p1[1] = hull_points[1][1] - hull_points[2][1];
  p1[2] = hull_points[1][2] - hull_points[2][2];
  cross = p0.cross(p1);
  double norm = cv::norm(cross);
  normal.x = cross[0]/norm;
  normal.y = cross[1]/norm;
  normal.z = cross[2]/norm;
  
  // Check if normal goes into direction of camera (opening angle < PI/2
  double norm_center = sqrt(center3D.x*center3D.x + center3D.y*center3D.y + center3D.z*center3D.z);
  double dot_p_center = normal.x*center3D.x + normal.y*center3D.y + normal.z*center3D.z;
  double angle = acos(dot_p_center/(norm_center));
  if(angle < 1.57)
    normal = -normal;
}

/**
 * @brief Calculate significance for the 3D Gestalt.
 * 2D horizontal lines are worser to match in 3D, therefore lower significance.
 * 3D Lines with main direction in z-coordinte.
 * @param angle2Dleft
 * @param angle2Dright 
 * @param angle3Dz Opening angle between z-axis and Line in 3D
 */
void Patch3D::CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz)
{
  printf("Patch3D::CalculateSignificance: Time to implement!\n");
//   // Significance is between 0 and 1
//   double sig2Dleft = fabs(angle2Dleft/(M_PI/2.));
//   double sig2Dright = fabs(angle2Dright/(M_PI/2.));
//   double sig3D = 1- angle3Dz/(M_PI/2.);	
//   
//   sig = 1- (1-sig2Dleft) * (1-sig2Dright) * (1-sig3D);
}

/**
 * @brief Are the patches close? How to measure closeness?
 * Defined closeness: Minimum distance between two hull points.
 * @param p Patch to compare with
 * @return Returns the distance in meter.
 */
double Patch3D::CalculateProximity(Patch3D *p)
{
  double distance = 100.;
  for(unsigned i=0; i<hull_points.size(); i++)
  {
    for(unsigned j=0; j<p->hull_points.size(); j++)
    {
      double current_distance = Distance(hull_points[i], p->hull_points[j]);
      if(current_distance < distance)
        distance = current_distance;
    }    
  }
  return (distance);
}

/**
 * @brief Compare histogram of patch with another patch's histogram.
 * @param p Patch to compare with
 * @return Returns normalised color distance (0-1).
 */
double Patch3D::CompareColor(Patch3D *p)
{
  return hist->Compare(p->hist);
}

double Patch3D::DistancePoint2Plane(double a, double b, double c, double d, double x, double y, double z)
{
    return (fabs(a*x + b*y + c*z - d) / sqrt(a*a + b*b + c*c));
}

double Patch3D::CalculatePointDistance(double x, double y, double z)
{
  double np = normal.x*center3D.x + normal.y*center3D.y + normal.z*center3D.z;
  return DistancePoint2Plane(normal.x, normal.y, normal.z, np, x, y, z);
}

/**
 * @brief Compare two patches for coplanarity. Extract difference of normals and
 * the distance of the planes 
 * @param p Patch to compare with
 * @param normal_angle Angle between the two plane normals
 * @param plane_distance Distance between planes, after rotation to mean normal.
 */
void Patch3D::CalculateCoplanarity(Patch3D *p, double &normal_angle, double &plane_distance)
{
  // direction from patch center to patch center
  cv::Point3f pp = center3D - p->center3D;
  double norm_pp = cv::norm(pp);
  cv::Point3f pp_dir;
  pp_dir.x = pp.x/norm_pp;
  pp_dir.y = pp.y/norm_pp;
  pp_dir.z = pp.z/norm_pp;
  
//   double pp_dir = norm(pp);
  
  // angel between normal and pp_dir
  double ang0 = acos(pp_dir.x*normal.x + pp_dir.y*normal.y + pp_dir.z*normal.z);
  double ang1 = acos(pp_dir.x*p->normal.x + pp_dir.y*p->normal.y + pp_dir.z*p->normal.z);
  
// printf("ang0-ang1: %4.3f-%4.3f\n", ang0, ang1);
  if(ang0 > M_PI/2.) ang0 = M_PI - ang0;
  if(ang1 > M_PI/2.) ang1 = M_PI - ang1;
// printf("  g0-ang1: %4.3f-%4.3f\n", ang0, ang1);
  ang0 = fabs(ang0 - M_PI/2.);
  ang1 = fabs(ang1 - M_PI/2.);
  
// printf("     ang1: %4.3f-%4.3f\n", ang0, ang1);
  normal_angle = ang0 + ang1;
// printf("     norm: %4.3f\n", normal_angle);
  
//   double a_normal_pp_dir = p->normal.x*normal.x + p->normal.y*normal.y + p->normal.z*normal.z;
//   normal_angle = acos(dot_p_pp);
  
  // First calculate the deviation of the normals
//   double dot_p_pp = p->normal.x*normal.x + p->normal.y*normal.y + p->normal.z*normal.z;
//   normal_angle = acos(dot_p_pp);
// //   printf("NormalAngle: %4.4f\n", normal_angle);
//   

  // calculate plane distance:
  // calculate mean normal
  cv::Point3f meanNormal;
  meanNormal = normal + p->normal;

  double norm = cv::norm(meanNormal);
  meanNormal.x = meanNormal.x/norm;
  meanNormal.y = meanNormal.y/norm;
  meanNormal.z = meanNormal.z/norm;
  
  double np = meanNormal.x*center3D.x + meanNormal.y*center3D.y + meanNormal.z*center3D.z;
  plane_distance = DistancePoint2Plane(meanNormal.x, meanNormal.y, meanNormal.z, np, p->center3D.x, p->center3D.y, p->center3D.z);
}

/**
 * @brief Calculate parallelity between (line) direction and plane patch.
 * @param dir Direction of visual feature (normalised).
 * @return Returns the angle between dir and the plane normal -M_PI/2.
 */
double Patch3D::CalculateParallelity(cv::Point3f &dir)
{
  // calculate opening angle!!!
  double dot = normal.x * dir.x + normal.y * dir.y + normal.z * dir.z;
  return (acos(dot) - (M_PI/2.));
}

/**
 * @brief Get the nodes vector for this 3D Gestalt.
 * @param nodes Vector with n nodes, consisting of 3 position values.
 * @return Returns true, if nodes from this 3D Gestalt are available.
 */
bool Patch3D::GetLinks(vector<GraphLink> &links)
{
printf("Patch3D::GetLinks: Not yet implemented!\n");
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
 * @param use_color Use specific color, or random color
 * @param float color
 */
void Patch3D::DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool use_color, float color)
{
  if(use_color)
  {
    RGBValue col;
    col.float_value = color;
    std::vector<cv::Vec4f> col_points, col_hull_points;
    for(unsigned i=0; i<points.size(); i++)
    {
      col_points.push_back(points[i]);
      col_points[i][3] = color;
    }
    tgRenderer->AddPointCloud(col_points);
    
    // add convex hull of patch
    for(unsigned j=0; j < hull_points.size(); j++)
    {
      int k = j+1;
      if(k == hull_points.size()) k=0;
      cv::Vec4f s = hull_points[j];
      cv::Vec4f e = hull_points[k];
      tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 
                            col.r, col.g, col.b, 2);
    }  
  }    
  else
  {
    tgRenderer->AddPointCloud(points);
    // add convex hull of patch
    for(unsigned j=0; j < hull_points.size(); j++)
    {
      int k = j+1;
      if(k == hull_points.size()) k=0;
      cv::Vec4f s = hull_points[j];
      cv::Vec4f e = hull_points[k];
      RGBValue col;
      col.float_value = hull_points[j][3];
      tgRenderer->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 
                            col.r, col.g, col.b, 2);
    }  
  }
}


/**
 * @brief Draw this 3D Gestalt to an image
 * @param image Image
 * @param camPars CameraParameters
 */
void Patch3D::DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters camPars)
{
  // get random color
  RGBValue color;
  color.r = std::rand()%255;
  color.g = std::rand()%255;
  color.b = std::rand()%255;

  for(unsigned i=0; i<points.size(); i++)
  {
    double u, v;
    double scale = 1;
    if(points[i][0] == 0.) points[i][0]=0.000001;   // instead: assert(Z != 0.);
    if(image.cols !=0) scale = camPars.width / image.cols; /// image width
    u = camPars.fx*points[i][0]/scale + camPars.cx*points[i][2]/scale;
    v = camPars.fy*points[i][1]/scale + camPars.cy*points[i][2]/scale;
    u /= points[i][2];
    v /= points[i][2];

    int x = (int) u;
    int y = (int) v;
    
    // print points in double size
    if(x >= 0 && x < 640 && y >= 0 && y < 480)
    {
      // add point to image
      cv::Vec3b &ptCol = image(y, x);
      ptCol[0] = color.r; 
      ptCol[1] = color.g; 
      ptCol[2] = color.b;
      cv::Vec3b &ptCol2 = image(y, x+1);
      ptCol2[0] = color.r; 
      ptCol2[1] = color.g; 
      ptCol2[2] = color.b;      
      cv::Vec3b &ptCol3 = image(y+1, x);
      ptCol3[0] = color.r; 
      ptCol3[1] = color.g; 
      ptCol3[2] = color.b;
      cv::Vec3b &ptCol4 = image(y+1, x+1);
      ptCol4[0] = color.r; 
      ptCol4[1] = color.g; 
      ptCol4[2] = color.b;  
    }  
  }
  
  double c_u = camPars.fx*center3D.x + camPars.cx*center3D.z;
  double c_v = camPars.fy*center3D.y + camPars.cy*center3D.z;
  int c_x = (int) (c_u / center3D.z);
  int c_y = (int) (c_v / center3D.z);
    
  char labl[5];
  snprintf(labl, 5, "%u", GetNodeID());
  cv::putText(image, string(labl), cv::Point(c_x, c_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 2);
}

/**
 * @brief Print this 3D Gestalt to the console.
 */
void Patch3D::PrintGestalt3D()
{
  printf("Patch3D: %u\n", id);
  printf(" points.size: %u\n", points.size());
  
  for(unsigned idx=0; idx<points.size(); idx++)
    printf(" point: %4.3f / %4.3f / %4.3f\n", points[idx][0], points[idx][1], points[idx][2]);
}


}


