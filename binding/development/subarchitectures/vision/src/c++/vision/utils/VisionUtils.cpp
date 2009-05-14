/**
 * Utility stuff for Visual subarchitecture.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#include <cstdio>
#include <cstdarg>
#include <cassert>
#include <fstream>
#include <algorithm>
#include <sys/time.h>
#include "VisionUtils.h"


//---------- General stuff ------------------------------------------

/**
 * Except constructor.
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
BALTException::BALTException(const char *file, const char *function, int line,
  const char *format, ...) throw()
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
  _what = msg;
}

namespace Vision {

//---------- Images -------------------------------------------------

void SaveImage(const ImageFrame &img, const string &filename)
{
  IplImage *tmp = cvCreateImage(cvSize(img.m_width, img.m_height),
     IPL_DEPTH_8U, 3);
  // note that here we assume that ImageFrame and IplImage have same colour
  // format
  for(unsigned j = 0; j < img.m_image.length(); j++)
    tmp->imageData[j] = img.m_image[j];
  cvSaveImage(filename.c_str(), tmp);
  cvReleaseImage(&tmp);
}

/**
 * Get sub image defined by bounding box from whole image.
 * The bounding box is clipped to fit the whole image, and may thus be changed
 * on return.
 * Returns true if at least part of the bounding box was inside whole image,
 * false if the bounding box was completely outside.
 */
/*
bool GetSubImage(const ImageFrame &whole, Image &part, BBox2D &box)
{
  int x, y, c, xp, yp;
  int xs = (int)(box.m_center.m_x - box.m_size.m_x/2.); // x start
  int xe = (int)(box.m_center.m_x + box.m_size.m_x/2.); // x end
  int ys = (int)(box.m_center.m_y - box.m_size.m_y/2.); // y start
  int ye = (int)(box.m_center.m_y + box.m_size.m_y/2.); // y end
  xs = max((int)xs, (int)0);
  xe = min((int)xe, (int)(whole.m_width - 1));
  ys = max((int)ys, (int)0);
  ye = min((int)ye, (int)(whole.m_height - 1));
  if(xe > xs && ye > ys)
  {
    part.m_width = xe - xs + 1;
    part.m_height = ye - ys + 1;
    part.m_nChannels = 3;  // BGR24
    part.m_image.length(part.m_width*part.m_height*part.m_nChannels);
    for(y = ys, yp = 0; y <= ye; y++, yp++)
      for(x = xs, xp = 0; x <= xe; x++, xp++)
        // copy B,G,R
        for(c = 0; c < 3; c++)
          part.m_image[3*(yp*part.m_width + xp) + c] =
            whole.m_image[3*(y*whole.m_width + x) + c];
    box.m_center.m_x = (xs + xe)/2;
    box.m_center.m_y = (ys + ye)/2;
    box.m_size.m_x = xe - xs + 1;
    box.m_size.m_y = ye - ys + 1;
    return true;
  }
  else
  {
    part.m_width = 0;
    part.m_height = 0;
    part.m_nChannels = 3;  // BGR24
    // setting a CORBA array to 0 throws an exception, so just leave it
    box.m_center.m_x = 0.;
    box.m_center.m_y = 0.;
    box.m_size.m_x = 0.;
    box.m_size.m_y = 0.;
    return false;
  }
}
*/

/**
 * Downsamle an image frame by a given factor.
 * Note: image width must be a multiple of factor.
 * Note: no anti-aliasing is performed.
 */
void DownsampleImage(ImageFrame *src, ImageFrame *dst, int f)
{
  int x, y, c;

  assert(f > 0);
  dst->m_width = src->m_width/f;
  dst->m_height = src->m_height/f;
  dst->m_time = src->m_time;
  dst->m_camNum = src->m_camNum;
  dst->m_image.length(src->m_image.length()/(f*f));
  for(y = 0; y < src->m_height; y += f)
    for(x = 0; x < src->m_width; x += f)
  for(c = 0; c < 3; c++)
    dst->m_image[3*((y/f)*dst->m_width + (x/f)) + c] =
      src->m_image[3*(y*src->m_width + x) + c];
}


//---------- Vector2D -----------------------------------------------

Vector2D Center(const vector<Vector2D> &points)
{
  Vector2D c = {0., 0.};
  for(unsigned i = 0; i < points.size(); i++)
    c = c + points[i];
  return c/(double)points.size();
}


//---------- Vector3D -----------------------------------------------

istream& operator>>(istream &is, Vector3D &v)
{
  return is >> v.m_x >> v.m_y >> v.m_z;
}

ostream& operator<<(ostream &os, const Vector3D &v)
{
  return os << v.m_x << ' ' << v.m_y << ' ' << v.m_z;
}

Vector3D Center(const vector<Vector3D> &points)
{
  Vector3D c = {0., 0.};
  for(unsigned i = 0; i < points.size(); i++)
    c = c + points[i];
  return c/(double)points.size();
}

Vector3D Normalise(const Vector3D &v)
{
  double l = Length(v);
  Vector3D w;
  assert(l != 0.);
  w.m_x = v.m_x/l;
  w.m_y = v.m_y/l;
  w.m_z = v.m_z/l;
  return w;
}


//---------- Pose3D -------------------------------------------------

istream& operator>>(istream &is, Pose3D &pose)
{
  return is >> pose.m_position >> pose.m_orientation;
}

ostream& operator<<(ostream &os, const Pose3D &pose)
{
  return os << pose.m_position << ' ' << pose.m_orientation;
}

const char* ToCString(const Pose3D &p, bool matrix)
{
  static char str[512];
  if(!matrix)
  {
    snprintf(str, 512, "t %s r %s", ToCString(p.m_position),
        ToCString(p.m_orientation));
  }
  else
  {
    double R[3][3];
    RotationAxisAngleToMatrix(p.m_orientation, R);
    snprintf(str, 512, "t %s\n"
      "  | %6.3f %6.3f %6.3f |\n"
      "R | %6.3f %6.3f %6.3f |\n"
      "  | %6.3f %6.3f %6.3f |\n", ToCString(p.m_position),
      R[0][0], R[0][1], R[0][2],
      R[1][0], R[1][1], R[1][2],
      R[2][0], R[2][1], R[2][2]);
  }
  return str;
}

/**
 * Reads pose from file.
 * Pose is stored as translation and rotation vector:
 *   t  x y z
 *   r  x y z
 * Empty lines and comment lines starting with '#' are ignored.
 */
void ReadPose(const char *filename, Pose3D &pose) throw(BALTException)
{
  const char comment = '#';
  const char *spaces = " \t\n";
  const size_t bufsize = 1024;
  char buf[bufsize];
  bool have_t = false, have_r = false;
  FILE *file = fopen(filename, "r");
  if(file == 0)
    throw BALTException(__HERE__, "failed to open file '%s'", filename);
  while(fgets(buf, bufsize, file) != NULL)
  {
    size_t linelen = strlen(buf);
    // move to first non-space (= comment or start of name)
    size_t start = strspn(buf, spaces);
    if(start < linelen && buf[start] != comment)
    {
      // move to first space (= end of name)
      size_t stop = start + strcspn(&buf[start], spaces);
      size_t len = stop - start;
      char *name = &buf[start];
      char *value = &buf[stop];
      if(len == strlen("t") && strncmp(name, "t", len) == 0)
      {
        float x, y, z;
        if(sscanf(value, "%f %f %f", &x, &y, &z) == 3)
        {
          pose.m_position.m_x = x;
          pose.m_position.m_y = y;
          pose.m_position.m_z = z;
          have_t = true;
        }
      }
      else if(len == strlen("r") && strncmp(name, "r", len) == 0)
      {
        float x, y, z;
        if(sscanf(value, "%f %f %f", &x, &y, &z) == 3)
        {
          pose.m_orientation.m_x = x;
          pose.m_orientation.m_y = y;
          pose.m_orientation.m_z = z;
          have_r = true;
        }
      }
    }
  }
  fclose(file);
  if(!(have_t && have_r))
    throw BALTException(__HERE__, "failed to read complete pose from '%s'",
                        filename);
}

/**
 * w = R o + t
 * o = R^T w - R^T t
 */
Pose3D InvertPose3D(const Pose3D &pose)
{
  Pose3D inv;
  inv.m_orientation = -pose.m_orientation;
  inv.m_position = -Rotate(inv.m_orientation, pose.m_position);
  return inv;
}


//---------- Various transformations --------------------------------

// set M to identity matrix
void Identity3x3(double M[3][3])
{
  int i, j;
  for(i = 0; i < 3; i++)
    for(j = 0; j < 3; j++)
      M[i][j] = 0.;
  M[0][0] = M[1][1] = M[2][2] = 1.;
}

void RotationAxisAngleToMatrix(const Vector3D &r, double R[3][3])
{
  double th = Length(r);
  if(th != 0)
  {
    double x = r.m_x/th, y = r.m_y/th, z = r.m_z/th;
    double co = cos(th), si = sin(th);
    R[0][0] = x*x*(1. - co) + co;
    R[0][1] = x*y*(1. - co) - z*si;
    R[0][2] = x*z*(1. - co) + y*si;
    R[1][0] = x*y*(1. - co) + z*si;
    R[1][1] = y*y*(1. - co) + co;
    R[1][2] = y*z*(1. - co) - x*si;
    R[2][0] = x*z*(1. - co) - y*si;
    R[2][1] = y*z*(1. - co) + x*si;
    R[2][2] = z*z*(1. - co) + co;
  }
  else
  {
    Identity3x3(R);
  }
}

void RotationMatrixToAxisAngle(double R[3][3], Vector3D &r)
{
  double r1[3];
  CvMat r1_mat = cvMat(3, 1, CV_64FC1, r1);
  CvMat R_mat = cvMat(3, 3, CV_64FC1, R);

  cvRodrigues2(&R_mat, &r1_mat);
  r.m_x = (double)r1[0];
  r.m_y = (double)r1[1];
  r.m_z = (double)r1[2];
}

Vector3D Rotate(const Vector3D &r, const Vector3D &p)
{
  double R[3][3];
  Vector3D q;
  RotationAxisAngleToMatrix(r, R);
  q.m_x = R[0][0]*p.m_x + R[0][1]*p.m_y + R[0][2]*p.m_z;
  q.m_y = R[1][0]*p.m_x + R[1][1]*p.m_y + R[1][2]*p.m_z;
  q.m_z = R[2][0]*p.m_x + R[2][1]*p.m_y + R[2][2]*p.m_z;
  return q;
}

Vector3D Translate(const Vector3D &t, const Vector3D p)
{
  return t + p;
}

// Transform a point from local to global frame.
Vector3D TransformPointToGlobal(const Pose3D &pose, const Vector3D &p)
{
  return Translate(pose.m_position, Rotate(pose.m_orientation, p));
}

// Transform a point from local to global frame.
Vector3D TransformPointToLocal(const Pose3D &pose, const Vector3D &p)
{
  return TransformPointToGlobal(InvertPose3D(pose), p);
}

// Transform a direction from local to global frame.
Vector3D TransformDirToGlobal(const Pose3D &pose, const Vector3D &d)
{
  return Rotate(pose.m_orientation, d);
}

// Transform a direction from global to local frame.
Vector3D TransformDirToLocal(const Pose3D &pose, const Vector3D &d)
{
  return TransformDirToGlobal(InvertPose3D(pose), d);
}

/**
 * Transform a pose from local to global frame.
 * pose is the reference frame (T2) and p is the pose to be transformed (T1).
 * q = R1 p + t1
 * r = R2 q + t2
 *   = R2 R1 p + R2 t1 + t2
 *   = R3 p + t3
 */
Pose3D TransformPoseToGlobal(const Pose3D &pose, const Pose3D &p)
{
  Pose3D pt;
  float r1[3], R1[9], r2[3], R2[9], r3[3], R3[9];
  CvMat r1_mat = cvMat(3, 1, CV_32FC1, r1);
  CvMat R1_mat = cvMat(3, 3, CV_32FC1, R1);
  CvMat r2_mat = cvMat(3, 1, CV_32FC1, r2);
  CvMat R2_mat = cvMat(3, 3, CV_32FC1, R2);
  CvMat r3_mat = cvMat(3, 1, CV_32FC1, r3);
  CvMat R3_mat = cvMat(3, 3, CV_32FC1, R3);

  // R3 = R2 R1
  r1[0] = p.m_orientation.m_x;
  r1[1] = p.m_orientation.m_y;
  r1[2] = p.m_orientation.m_z;
  r2[0] = pose.m_orientation.m_x;
  r2[1] = pose.m_orientation.m_y;
  r2[2] = pose.m_orientation.m_z;
  cvRodrigues2(&r1_mat, &R1_mat);
  cvRodrigues2(&r2_mat, &R2_mat);
  cvMatMul(&R2_mat, &R1_mat, &R3_mat);
  cvRodrigues2(&R3_mat, &r3_mat);
  pt.m_orientation.m_x = r3[0];
  pt.m_orientation.m_y = r3[1];
  pt.m_orientation.m_z = r3[2];
  // t3 = R2 t1 + t2
  pt.m_position = TransformPointToGlobal(pose, p.m_position);
  return pt;
}

/**
 * Transform a pose from global to local frame.
 * pose is the reference frame and p is the pose to be transformed.
 */
Pose3D TransformPoseToLocal(const Pose3D &pose, const Pose3D &p)
{
  return TransformPoseToGlobal(InvertPose3D(pose), p);
}

/**
 * Transform camera image point in [pixel] to 3D view ray in camera
 * co-ordinates.
 * Note: Does not do image undistortion!
 * @param cam  camera
 * @param i  image point in [pixel]
 * @param p  origin of the ray, the focal point of the camera
 * @param d  direction of the ray, unit vector
 */
void ImagePointToLocalRay(const Camera &cam, const Vector2D &i,
    Vector3D &p, Vector3D &d)
{
  // first find direction in camera coordinates
  d.m_x = i.m_x - cam.m_cx;
  d.m_y = i.m_y - cam.m_cy;
  // note: i assume that fx is (nearly) equal to fy, so it does not matter
  // whether we use fx or fy
  d.m_z = cam.m_fx;
  // up to now d is in [pixel], normalise to 1 [m]
  d = Normalise(d);
  p.m_x = 0.;
  p.m_y = 0.;
  p.m_z = 0.;
}

/**
 * Transform camera image point in [pixel] to 3D view ray in world co-ordinates.
 * Note: Does not do image undistortion!
 * @param cam  camera
 * @param i  image point in [pixel]
 * @param p  origin of the ray, the focal point of the camera
 * @param d  direction of the ray, unit vector
 */
void ImagePointToGlobalRay(const Camera &cam, const Vector2D &i,
    Vector3D &p, Vector3D &d)
{
  ImagePointToLocalRay(cam, i, p, d);
  // transform dir and eye point to world coordinates
  d = TransformDirToGlobal(cam.m_pose, d);
  p = TransformPointToGlobal(cam.m_pose, p);
}

/**
 * Find point on groundplane (normal to z) for a given image point.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param p  image point in pixel coordinates
 * Returns 3D point on ground plane, with z = ground_z.
 * Note: Does not do image undistortion!
 */
Vector3D ProjectImagePointToGroundplane(double ground_z,
    const Camera &cam, const Vector2D &p)
{
  Vector3D e;  // eye point = origin of line of sight [m]
  Vector3D d;  // direction of line of sight [m]
  Vector3D g;  // point on ground plane [m]
  double l;    // length of line of sight until intersection

  ImagePointToGlobalRay(cam, p, e, d);

  // find intersection with z = ground_z plane
  l = (ground_z - e.m_z)/d.m_z;
  g.m_x = e.m_x + l*d.m_x;
  g.m_y = e.m_y + l*d.m_y;
  g.m_z = ground_z;

  return g;
}

/**
 * First find point on groundplane (normal to z) for a given image point, then
 * projects given image distance (or size) to 3D distance at that point.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param p  image point in pixel coordinates
 * @param s  distance in pixel coordinates
 * Note: Does not do image undistortion!
 */
double ProjectDistanceToGroundplane(double ground_z, const Camera &cam,
    const Vector2D &p, double s)
{
  Vector3D e;  // eye point = origin of line of sight
  Vector3D d;  // direction of line of sight
  double L;    // distance on line of sight to intersection
  double l;    // distance on line of sight to image plane

  ImagePointToGlobalRay(cam, p, e, d);

  // find intersection with z = ground_z plane
  L = (ground_z - e.m_z)/d.m_z;
  // S/s = L/l
  l = sqrt(Sqr(cam.m_fx) + Sqr(d.m_x));

  return L*s/l;
}

/**
 * Takes a 2D image ROI and calculates the according SceneObject parameters
 * for a scene object on the ground plane.
 * @param ground_z  height of ground plane (typically 0)
 * @param cam  camera parameters
 * @param roi  2D image ROI in pixel coordinates
 * @param obj  scene object, pose and bounding box fill be filled
 * Note: Does not do image undistortion!
 */
void ProjectRoiToGroundplane(double ground_z, const Camera &cam,
   ROI &roi, SceneObject &obj)
{

//#warning  ProjectRoiToGroundplane has not been defined.
//    throw BALTException(__HERE__, "Function not yet defined");

  Vector2D p;
  Vector3D g;
  double w, h;

  // bottom center of bounding box
  p.m_x = roi.m_bbox.m_center.m_x;
  p.m_y = roi.m_bbox.m_center.m_y + (roi.m_bbox.m_size.m_y/2.);

  // project points to ground plane
  g = ProjectImagePointToGroundplane(ground_z, cam, p);
  // get 3D sizes
  w = ProjectDistanceToGroundplane(ground_z, cam, p, roi.m_bbox.m_size.m_x);
  h = ProjectDistanceToGroundplane(ground_z, cam, p, roi.m_bbox.m_size.m_y);

  obj.m_bbox.m_centroid.m_x = g.m_x;
  obj.m_bbox.m_centroid.m_y = g.m_y;
  obj.m_bbox.m_centroid.m_z = g.m_z + h/2.;
  obj.m_bbox.m_size.m_x = w;
  obj.m_bbox.m_size.m_y = w;
  obj.m_bbox.m_size.m_z = h;
  obj.m_pose.m_position = obj.m_bbox.m_centroid;
  SetZero(obj.m_pose.m_orientation);
  obj.m_time = roi.m_time;
  // other attributes (color, shape, label, surfaces) are untouched

}



//---------- Miscellaneous stuff ------------------------------------

void SetDefaultCamera(Camera &cam)
{
  cam.m_num = 0;
  SetIdentity(cam.m_pose);
  cam.m_width = 0;
  cam.m_height = 0;
  cam.m_fx = cam.m_fy = 1.;
  cam.m_cx = cam.m_cy = 0.;
  cam.m_k1 = cam.m_k2 = cam.m_k3 = 0.;
  cam.m_t1 = cam.m_t2 = 0.;
}



// String labels for the values in the enumeration ObjectProperties in
// the Vision.idl file...
static const string objectPropertyLabels[] = {
  "UNKNOWN",
  // Color properties...
  "RED", // BLUE
  "GREEN", // BLACK
  "BLUE", // YELLOW
  "YELLOW",
  
  // Size properties...
  "SMALL",
  "LARGE",
  
  // Shape properties...
  "SQUARED",
  "CIRCULAR",
  "TRIANGULAR",
  "RECTANGULAR",
};



/**
 * Utility function that converts from an indexed entry of an
 * enumeraation, to the corresponding string label -- used currently
 * to translate sceneobject's color, size and shape properties to the
 * corresponding string labels...
 */
string enum2String( int enumVal ) {
  if( enumVal < 1 || enumVal >= Vision::MAX_OBJECT_PROPERTIES ){
    // std::cout << "enum2str -- not within bounds... " <<  enumVal << " \n";
    return objectPropertyLabels[0];
  }
  else {
    // std::cout << "enum2str -- valid index... " <<  enumVal << " \n";
    return objectPropertyLabels[enumVal];
  }
}



/**
 * Common function used by several vision components to convert the
 * IDL struct to the OpenCV image format...
 */
IplImage* buffer2image( Vision::ImageFrame* _pImage ) {
  IplImage *tmpimg = cvCreateImage( cvSize(_pImage->m_width, _pImage->m_height), 
				    IPL_DEPTH_8U, 3 );
  unsigned char* dst = (unsigned char*) tmpimg->imageData;
  char* src = (char *)&(_pImage->m_image[0]);
  memcpy(dst, src, _pImage->m_image.length());
  
  return tmpimg;
}



/**
 * Function is sort of the inverse of the function above -- OpenCV
 * structure to the format used in the Vision IDL (struct Image)...
 */
void image2buffer( IplImage* _pImage, Vision::Image& _buffer ) {
  // Set the header or face dire consequences during display...
  _buffer.m_width = _pImage->width;
  _buffer.m_height = _pImage->height;
  _buffer.m_nChannels = _pImage->nChannels;

  // Prepare the storage data strcuture by suitably resizing it...
  unsigned dataLength = ( _pImage->height *_pImage->widthStep);//_pImage->width * _pImage->height 
			  //* _pImage->nChannels );
  _buffer.m_image.length(dataLength);

  // Since OpenCV seems to align images in some way, we cannot just
  // use the memcpy but we have to use the nested for loops -- ah
  // well...
  memcpy( &( _buffer.m_image[0] ), _pImage->imageData, dataLength );
  /*
  // Get the pointer to the begining of resulting array.
  unsigned char* data = &( _buffer.m_image[0] );

  // Calculate the plane size (this will be used for multi-channel
  // images)...
  unsigned planeSize = _pImage->width * _pImage->height;

  // Set up the indices of the color channels depending on the input
  // image type (i.e. grayscale--1 or color--3)...
  unsigned channelIndices[3];
  if( _pImage->nChannels == 1 ) {
    channelIndices[0] = 0;
  }
  else {
    for( int i = 0; i < 3; ++i ) {
      char channelName = _pImage->channelSeq[i];
      if (channelName == 'R') channelIndices[i] = 0;
      else if (channelName == 'G') channelIndices[i] = 1;
      else if (channelName == 'B') channelIndices[i] = 2;
    } // for
  } // if - else

  // Finally, it is time to copy over the actual image data -- phew!
  for( int y = 0; y < _pImage->height; ++y ) {
    // Get pointer to the next row.
    unsigned char* ptr = (unsigned char*)
      (_pImage->imageData + y * _pImage->widthStep);
    
    for( int x = 0; x < _pImage->width; ++x ) {
      for( int c = 0; c < _pImage->nChannels; ++c, ++ptr ) {
        *( data + channelIndices[c] * 
	   planeSize + y * _pImage->width + x ) = *ptr;
      } // for
    } // for
  } // for*/
}



/**
 * Function draws a polygon on input image, connecting the input
 * pointset in a pair-wise manner, in the color specified as input...
 */
void drawPolygon( IplImage* image, const vector<Vector2D>& points, CvScalar col ) {
  for( unsigned i = 0; i < points.size(); ++i ) {
    int j = ( ( i < points.size() - 1 ) ? ( i + 1 ) : 0 );
    cvLine( image, cvPoint( (int)points[i].m_x, (int)points[i].m_y ), 
	    cvPoint( (int)points[j].m_x, (int)points[j].m_y ), col );
  }
}


}

