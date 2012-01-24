/**
 * @author Michael Zillich, Andreas Richtsfeld
 * @date May 2009, 2011
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <iostream>
#include <VideoUtils.h>
#include <Sphere3.h>
#include <Box3.h>
#include <VisionData.hpp>
#include "VisionCommander.h"
#include "VisionObjects.h"

/**
 * Project a (bounding) sphere to an image rectangle.
 */
inline cogx::Math::Rect2 projectSphere(const Video::CameraParameters &cam, const cogx::Math::Sphere3 &sphere)
{
  cogx::Math::Rect2 rect;
  rect.pos = projectPoint(cam, sphere.pos);
  // assuming that the image is a raw, distorted camera image, we now have to
  // distort the position to match the image
  distortPoint(cam, rect.pos, rect.pos);
  rect.width = projectSize(cam, sphere.pos, 2.*sphere.rad);
  rect.height = projectSize(cam, sphere.pos, 2.*sphere.rad);
  return rect;
}

/**
 * Project a SOI in world co-ordinates to an image ROI.
 */
inline VisionData::ROIPtr projectSOI(const Video::CameraParameters &cam, const VisionData::SOI &soi)
{
   VisionData::ROIPtr roi = new VisionData::ROI;
   roi->rect = projectSphere(cam, soi.boundingSphere);
   roi->time = soi.time;
   return roi;
}

inline bool pointInsideSOI(const VisionData::SOI &soi, const cogx::Math::Vector3 &p)
{
  return pointInsideSphere(soi.boundingSphere, p) &&
         pointInsideBox(soi.boundingBox, p);
}

/**
* @brief Compute normal vectors of vertices from face normals
*/
inline void computeNormalsFromFaces(VisionData::GeometryModelPtr model)
{
	int i,j;
	VisionData::Face* f;
	VisionData::VertexSeq v;
	cogx::Math::Vector3 v0, v1, v2, e1, e2, n;
	
	// calculate vertex normals using the face normal
	for(i=0; i<(int)model->faces.size(); i++){
		f = &model->faces[i];
		v = model->vertices;
		
		v0 = cogx::Math::vector3(v[f->vertices[0]].pos.x, v[f->vertices[0]].pos.y, v[f->vertices[0]].pos.z);
		v1 = cogx::Math::vector3(v[f->vertices[1]].pos.x, v[f->vertices[1]].pos.y, v[f->vertices[1]].pos.z);
		v2 = cogx::Math::vector3(v[f->vertices[2]].pos.x, v[f->vertices[2]].pos.y, v[f->vertices[2]].pos.z);
		e1 = v1 - v0;
		e2 = v2 - v0;
		
		n = cogx::Math::cross(e1,e2);
		cogx::Math::normalise(n);
		for(j=0; j<(int)f->vertices.size(); j++){
			model->vertices[f->vertices[j]].normal.x = n.x;
			model->vertices[f->vertices[j]].normal.y = n.y;
			model->vertices[f->vertices[j]].normal.z = n.z;
		}	
	}
}

inline cogx::Math::Vector3 getCenterOfGeometryModel(VisionData::GeometryModelPtr model)
{
	int j;
	VisionData::VertexSeq v;
	cogx::Math::Vector3 vCenter = cogx::Math::vector3(0.0,0.0,0.0);
	
	for(j=0; j<(int)v.size(); j++){
		vCenter.x += v[j].pos.x;
		vCenter.y += v[j].pos.y;
		vCenter.z += v[j].pos.z;
	}
	
	vCenter.x /= v.size();
	vCenter.y /= v.size();
	vCenter.z /= v.size();
	
	return vCenter;
}

/**
 * Print RGB color in text form to a stream: '[r g b]`
 */
inline void writeText(std::ostream &os, const cogx::Math::ColorRGB &col)
{
  os << '[' << col.r << ' ' << col.g << ' ' << col.b << ']';
}

/**
 * Read RGB color in text from a stream.
 * The expected format is: '[r g b]', white spaces are ignored.
 */
inline void readText(std::istream &is, cogx::Math::ColorRGB &col) throw(std::runtime_error)
{
  char c;
  is >> c;
  if(c == '[')
  {
    // note: we have to read values as ints not as unsigned chars, because
    // is >> col.r >> col.g >> col.b >> c;
    // would read [128 128 128] as col.r = 1, col.g = 1, col.b = 8  etc.
    int r, g, b;
    is >> r >> g >> b >> c;
    col.r = r;
    col.g = g;
    col.b = b;
    if(c != ']')
    {
      throw std::runtime_error(cast::exceptionMessage(__HERE__,
            "error reading ColorRGB: ']' expected, have '%c'", c));
    }
  }
  else
    throw std::runtime_error(cast::exceptionMessage(__HERE__,
          "error reading ColorRGB: '[' expected, have '%c'", c));
}


/**
 * Writing to a stream is taken to be a textual output, rather than a
 * serialisation of the actual binary data.
 */
inline std::ostream& operator<<(std::ostream &os, const cogx::Math::ColorRGB &col)
{
  writeText(os, col);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline std::istream& operator>>(std::istream &is, cogx::Math::ColorRGB &col)
{
  readText(is, col);
  return is;
}


/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char b;  // Blue channel
    unsigned char g;  // Green channel
    unsigned char r;  // Red channel
    unsigned char a;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


inline float GetRandomColor()
{
  RGBValue x;
  x.b = std::rand()%255;
  x.g = std::rand()%255;
  x.r = std::rand()%255;
  x.a = 0.; 
  return x.float_value;
}

/**
 * @brief Convert points from point cloud server to opencv matrix.
 * @param points Points from the point cloud as vector
 * @param cloud Point cloud
 * @param colCloud Color values for the point cloud
 */
inline void Points2Cloud(const std::vector<PointCloud::SurfacePoint> &points, 
                         cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud)
{
  printf("VisionUtils::Points2Cloud: Antiquated function: Use the cv::Vec4f matrix!\n");
  
  cloud = cv::Mat_<cv::Point3f>(1, points.size());
  colCloud = cv::Mat_<cv::Point3f>(1, points.size());    

  for(unsigned i = 0; i<points.size(); i++)
  {
    cv::Point3f p, cp;
    p.x = (float) points[i].p.x;
    p.y = (float) points[i].p.y;
    p.z = (float) points[i].p.z;
    cp.x = (uchar) points[i].c.b; // change rgb to bgr
    cp.y = (uchar) points[i].c.g;
    cp.z = (uchar) points[i].c.r;

    cloud.at<cv::Point3f>(0, i) = p;
    colCloud.at<cv::Point3f>(0, i) = cp;
  }
}


/**
 * @brief Converts a point cloud from cogx-points to cv::Mat.
 * If substituteNAN flag is true, replace the zero values by NAN-values.
 * @param points Points from the point cloud as vector
 * @param cv_cloud Point cloud
 * @param width Width of the cloud
 * @param height Height of the cloud
 * @param substituteNAN Substitute FLT_MAX values by NAN values
 */
inline void ConvertKinectPoints2MatCloud(const std::vector<PointCloud::SurfacePoint> &points, 
                                         cv::Mat_<cv::Vec4f> &cv_cloud,
                                         unsigned width, unsigned height, bool substituteNAN)
{
  unsigned position = 0;
  cv_cloud = cv::Mat_<cv::Vec4f>(height, width);    // rows = height / cols = width
  RGBValue color;
  color.a = 0;
  
  for(unsigned row = 0; row < height; row++)
  {
    for(unsigned col = 0; col < width; col++)
    {
      cv::Vec4f &p = cv_cloud.at<cv::Vec4f>(row, col);
      position = row*width + col;

      if(substituteNAN)
      {
        if(points[position].p.x == 0. && points[position].p.y == 0. && points[position].p.z == 0.)
        {
          p[0] = NAN;
          p[1] = NAN;
          p[2] = NAN;
        }
        else
        {
          p[0] = (float) points[position].p.x;
          p[1] = (float) points[position].p.y;
          p[2] = (float) points[position].p.z;
        }
      }
      
      color.r = points[position].c.b;
      color.g = points[position].c.g;
      color.b = points[position].c.r;
      p[3] = color.float_value;
    }
  }
}

#endif

