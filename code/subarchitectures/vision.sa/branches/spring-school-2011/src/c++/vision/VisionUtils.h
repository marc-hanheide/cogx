/**
 * @author Michael Zillich
 * @date May 2009
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

#include <iostream>
#include <VideoUtils.h>
#include <Sphere3.h>
#include <Box3.h>
#include <VisionData.hpp>

/**
 * Project a SOI in world co-ordinates to an image ROI.
 */
inline VisionData::ROIPtr projectSOI(const Video::CameraParameters &cam, const VisionData::SOI &soi)
{
   VisionData::ROIPtr roi = new VisionData::ROI;
   roi->rect.pos = projectPoint(cam, soi.boundingSphere.pos);
   // assuming that the image is a raw, distorted camera image, we now have to
   // to distort the position to match the image
   distortPoint(cam, roi->rect.pos, roi->rect.pos);
   roi->rect.width = projectSize(cam, soi.boundingSphere.pos, 2.*soi.boundingSphere.rad);//std::cout<<"roi->rect.width "<<roi->rect.width<<std::endl;
   roi->rect.height = projectSize(cam, soi.boundingSphere.pos, 2.*soi.boundingSphere.rad);
   roi->time = soi.time;
   return roi;
}

inline bool pointInsideSOI(const VisionData::SOI &soi, const cogx::Math::Vector3 &p)
{
  return pointInsideSphere(soi.boundingSphere, p) &&
         pointInsideBox(soi.boundingBox, p);
}

/**
* @brief Compute normal vectors of vertices by using the face normal
*/
inline void computeNormalsByFaces(VisionData::GeometryModelPtr model)
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
inline void writeText(std::ostream &os, const VisionData::ColorRGB &col)
{
  os << '[' << col.r << ' ' << col.g << ' ' << col.b << ']';
}

/**
 * Read RGB color in text from a stream.
 * The expected format is: '[r g b]', white spaces are ignored.
 */
inline void readText(std::istream &is, VisionData::ColorRGB &col) throw(std::runtime_error)
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
inline std::ostream& operator<<(std::ostream &os, const VisionData::ColorRGB &col)
{
  writeText(os, col);
  return os;
}

/**
 * Reading from a stream is taken to read a textual input, rather than
 * de-serialising the actual binary data.
 */
inline std::istream& operator>>(std::istream &is, VisionData::ColorRGB &col)
{
  readText(is, col);
  return is;
}

#endif

