/**
 * @author Michael Zillich
 * @date May 2009
 */

#ifndef VISION_UTILS_H
#define VISION_UTILS_H

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

#endif

