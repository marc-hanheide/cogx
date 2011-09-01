#ifndef GEOIMG_H_
#define GEOIMG_H_

#include "blztools.h"

class GeoImage {

public:

	// construction
	GeoImage(int width, int height, double fu, double fv, int ppu, int ppv);
	bool createMesh(double z);

	// attach routines
	bool attachScalarPropertyFromMatFiles(const char* fileNameX);
	bool attachNormalPropertyFromMatFiles(const char* fileNameX, const char* fileNameY, const char* fileNameZ);
	bool attachImageFromFile(const char* fileName);

	// access
	OpenMesh::VPropHandleT<TriangleMesh::Normal> getNormalPropertyHandle(int i){ return normalProperties_[i]; };
	OpenMesh::VPropHandleT<TriangleMesh::Scalar> getScalarPropertyHandle(int i){ return scalarProperties_[i]; };
	bool cropMesh(int uStart, int uEnd, int vStart, int vEnd);
	bool cropMesh(double xStart, double xEnd, double yStart, double yEnd);
	bool cropMesh(double min, double max, double threshold, unsigned int erodeSteps, int prop);
	TriangleMesh returnMesh() { return backProjection_; };
	bool saveScalarProperty(const char* fileName, const char* propName, int i);

	// processing
	bool laplacianHighPass(int i);
	bool normalizeScalarProperty(int i);
	bool updateDims();

protected:

	// the surface
	TriangleMesh backProjection_;

	// cam parameters
	int width_;			/*!< image dims in pxl */
	int height_;
	double fu_;			/*!< focal length in pxl */
	double fv_;
	int ppu_;			/*!< principal point coordinates in pxl */
	int ppv_;

	// optical measurement data attached to surface
	std::vector<OpenMesh::VPropHandleT<TriangleMesh::Normal> > normalProperties_;
	std::vector<OpenMesh::VPropHandleT<TriangleMesh::Scalar> > scalarProperties_;



};

#endif /*GEOIMG_H_*/
