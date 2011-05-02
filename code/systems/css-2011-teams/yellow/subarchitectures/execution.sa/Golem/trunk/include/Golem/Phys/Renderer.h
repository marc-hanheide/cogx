/** @file Renderer.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_RENDERER_H_
#define _GOLEM_PHYS_RENDERER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Math/Bounds.h>
#include <vector>
#include <GL/gl.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** RGBA colour */
class RGBA {
public:
	typedef std::vector<RGBA> Seq;

	struct rgba {
		/** RGBA struct. Little endian. */
		U8 r, g, b, a;
	};
	
	union {
		struct rgba _rgba;
		U32 _U32;
		U8 _U8[4];
	};

	RGBA() {
	}
	RGBA(const RGBA& _RGBA) : _rgba(_RGBA._rgba) {
	}
	RGBA(const rgba& _rgba) : _rgba(_rgba) {
	}
	RGBA(U32 _U32) : _U32(_U32) {
	}
	RGBA(const U8* _U8) : _U32(*(U32*)_U8) {
	}
	RGBA(U8 r, U8 g, U8 b, U8 a) : _U32(r | (g << 8) | (b << 16) | (a << 24)) {
	}

	inline void set(const RGBA& _RGBA) {
		this->_rgba = _RGBA._rgba;
	}

	inline void set(const rgba& _rgba) {
		this->_rgba = _rgba;
	}

	inline void set(U32 _U32) {
		this->_U32 = _U32;
	}

	inline void set(const U8* _U8) {
		this->_U32 = *(U32*)_U8;
	}

	inline void set(U8 r, U8 g, U8 b, U8 a) {
		this->_rgba.r = r;
		this->_rgba.g = g;
		this->_rgba.b = b;
		this->_rgba.a = a;
	}

	/** Basic RGBA colours */
	static const RGBA BLACK;
	static const RGBA RED;
	static const RGBA GREEN;
	static const RGBA YELLOW;
	static const RGBA BLUE;
	static const RGBA MAGENTA;
	static const RGBA CYAN;
	static const RGBA WHITE;
};

//------------------------------------------------------------------------------

/** Bounds Renderer */
class BoundsRenderer {
public:
	static const RGBA glWireColourDflt;
	static const RGBA glSolidColourDflt;
	static const RGBA glShadowColourDflt;
	static const Real glLineWidthDflt;

	static const ::GLfloat glMatDflt[16];
	static const ::GLfloat glShadowMatDflt[16];

	static const U32 BOUNDING_SPHERE_SLICES = 16;
	static const U32 BOUNDING_SPHERE_STACKS = 16;
	static const U32 BOUNDING_CYLINDER_SLICES = 16;

protected:
	typedef std::vector<GLfloat> Data;

	static Data cylinderTriangles;
	static Data cylinderNormals;
	static Data cylinderLines;

	::GLfloat glMat[16], glShadowMat[16];
	::GLfloat glLineWidth;
	RGBA glWireColour, glSolidColour, glShadowColour;
	Data verticesData, normalsData;
	Mat34 mat;

	void initCylinder();

	void glDrawTriangles(const void* vertices, const void* normals, GLsizei numOfTriangles) {
		::glEnableClientState(GL_VERTEX_ARRAY);
		::glVertexPointer(3, GL_FLOAT, 0, vertices);
		::glEnableClientState(GL_NORMAL_ARRAY);
		::glNormalPointer(GL_FLOAT, 0, normals);

		::glDrawArrays(GL_TRIANGLES, 0, 3*numOfTriangles);

		::glDisableClientState(GL_VERTEX_ARRAY);
		::glDisableClientState(GL_NORMAL_ARRAY);
	}
	
	void glDrawLines(const void* vertices, GLsizei numOfLines) {
		::glEnableClientState(GL_VERTEX_ARRAY);
		::glVertexPointer(3, GL_FLOAT, 0, vertices);

		::glDrawArrays(GL_LINES, 0, 2*numOfLines);

		::glDisableClientState(GL_VERTEX_ARRAY);
	}
	
	void setMat(::GLfloat data [], const Mat34& mat) {
		mat.R.getColumn44(&(data[0]));
		mat.p.get(&(data[12]));

		data[3] = data[7] = data[11] = ::GLfloat(0.0);
		data[15] = ::GLfloat(1.0);
	}

public:
	BoundsRenderer();
	
	void reset();
	
	void setMat(const Mat34& mat) {
		this->mat = mat;
		//setMat(glMat, mat);
	}

	void setShadowMat(const Mat34& mat) {
		setMat(glShadowMat, mat);
	}

	void setLineWidth(Real width) {
		this->glLineWidth = (GLfloat)width;
	}

	void setWireColour(const RGBA& colour) {
		this->glWireColour = colour;
	}

	void setSolidColour(const RGBA& colour) {
		this->glSolidColour = colour;
	}

	void setShadowColour(const RGBA& colour) {
		this->glShadowColour = colour;
	}

	
	void renderWire(const golem::BoundingPlane& bounds);

	void renderSolid(const golem::BoundingPlane& bounds);

	void renderWire(const golem::BoundingSphere& bounds);

	void renderSolid(const golem::BoundingSphere& bounds);

	void renderWire(const golem::BoundingCylinder& bounds, bool useITriangleMesh = false);

	void renderSolid(const golem::BoundingCylinder& bounds, bool useITriangleMesh = false);

	void renderWire(const golem::BoundingBox& bounds);

	void renderSolid(const golem::BoundingBox& bounds);

	void renderWire(const golem::BoundingConvexMesh& bounds);

	void renderSolid(const golem::BoundingConvexMesh& bounds);


	void renderWire(const golem::ITriangleMesh& mesh);

	void renderSolid(const golem::ITriangleMesh& mesh);


	void renderWire(const golem::Bounds& bounds);

	template <typename const_iterator> void renderWire(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (glLighting) ::glDisable(GL_LIGHTING);

		::glColor4ub(glWireColour._rgba.r, glWireColour._rgba.g, glWireColour._rgba.b, glWireColour._rgba.a);
		::glLineWidth(glLineWidth);
		for (const_iterator i = begin; i != end; i++)
			renderWire(**i);

		if (glLighting) ::glEnable(GL_LIGHTING);
	}

	void renderSolid(const golem::Bounds& bounds);

	template <typename const_iterator> void renderSolid(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (!glLighting) ::glEnable(GL_LIGHTING);
		
		::glColor4ub(glSolidColour._rgba.r, glSolidColour._rgba.g, glSolidColour._rgba.b, glSolidColour._rgba.a);
		for (const_iterator i = begin; i != end; i++)
			renderSolid(**i);

		if (!glLighting) ::glDisable(GL_LIGHTING);
	}

	void renderShadow(const golem::Bounds& bounds);
	
	template <typename const_iterator> void renderShadow(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (glLighting) ::glDisable(GL_LIGHTING);
		
		::glPushMatrix();
		::glMultMatrixf(glShadowMat);

		::glColor4ub(glShadowColour._rgba.r, glShadowColour._rgba.g, glShadowColour._rgba.b, glShadowColour._rgba.a);
		for (const_iterator i = begin; i != end; i++)
			renderShadow(**i);
		
		::glPopMatrix();

		if (glLighting) ::glEnable(GL_LIGHTING);
	}
};

//------------------------------------------------------------------------------

/** Debug buffered renderer */
class DebugRenderer {
protected:
	typedef std::vector<Real> Data;

	RGBA colour;
	Data pointData, lineData, triangleData;

	void render(const Real *pData, ::GLenum mode, ::GLsizei count) const;
	
	inline void addData(Data& data, const Vec3& vec3, const RGBA& colour) {
		data.push_back(vec3.v1);
		data.push_back(vec3.v2);
		data.push_back(vec3.v3);
		data.push_back(*(Real*)&colour);
	}
	
public:
	DebugRenderer();
	
	virtual ~DebugRenderer() {}
	
	/** Renders the object */
	virtual void render();

	/** Resets render buffers */
	virtual void reset();

	/** Adds data from DebugRenderer */
	inline void add(const DebugRenderer &dr) {
		pointData.insert(pointData.end(), dr.pointData.begin(), dr.pointData.end());
		lineData.insert(lineData.end(), dr.lineData.begin(), dr.lineData.end());
		triangleData.insert(triangleData.end(), dr.triangleData.begin(), dr.triangleData.end());
	}

	/** Sets current colour */
	inline void setColour(const RGBA &colour) {
		this->colour = colour;
	}
	
	/** Adds new point */
	inline void addPoint(const Vec3 &p, const RGBA &colour) {
		addData(pointData, p, colour);
	}
	/** Adds new point */
	inline void addPoint(const Vec3 &p) {
		addPoint(p, colour);
	}
	/** Resets points memory buffer */
	inline void resizePoints(U32 numOfPoints = 0) {
		pointData.resize(size_t(4*numOfPoints));
	}
	/** Returns number of points */
	inline U32 sizePoints() const {
		return U32(pointData.size()/4);
	}
	/** Reserve points memory buffer */
	inline void reservePoints(U32 numOfPoints) {
		pointData.reserve(size_t(4*numOfPoints));
	}
	/** Returns number of allocated points in the memory buffer */
	inline U32 capacityPoints() const {
		return U32(pointData.capacity()/4);
	}
	/** Renders points */
	virtual void renderPoints();

	/** Adds new line */
	inline void addLine(const Vec3 &p0, const Vec3 &p1, const RGBA &colour) {
		addData(lineData, p0, colour);
		addData(lineData, p1, colour);
	}
	/** Adds new line */
	inline void addLine(const Vec3 &p0, const Vec3 &p1) {
		addLine(p0, p1, colour);
	}
	/** Resets lines memory buffer */
	inline void resizeLines(U32 numOfLines = 0) {
		lineData.resize(size_t(8*numOfLines));
	}
	/** Returns number of lines */
	inline U32 sizeLines() const {
		return U32(lineData.size()/8);
	}
	/** Reserve lines memory buffer */
	inline void reserveLines(U32 numOfLines) {
		lineData.reserve(size_t(8*numOfLines));
	}
	/** Returns number of allocated lines in the memory buffer */
	inline U32 capacityLines() const {
		return U32(lineData.capacity()/8);
	}
	/** Renders lines */
	virtual void renderLines();

	/** Adds new triangle */
	inline void addTriangle(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const RGBA &colour) {
		addData(triangleData, p0, colour);
		addData(triangleData, p1, colour);
		addData(triangleData, p2, colour);
	}
	/** Adds new triangle */
	inline void addTriangle(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2) {
		addTriangle(p0, p1, p2, colour);
	}
	/** Resets triangles memory buffer */
	inline void resizeTriangles(U32 numOfTriangles = 0) {
		triangleData.resize(size_t(12*numOfTriangles));
	}
	/** Returns number of triangles */
	inline U32 sizeTriangles() const {
		return U32(triangleData.size()/12);
	}
	/** Reserve triangles memory buffer */
	inline void reserveTriangles(U32 numOfTriangles) {
		triangleData.reserve(size_t(12*numOfTriangles));
	}
	/** Returns number of allocated triangles in the memory buffer */
	inline U32 capacityTriangles() const {
		return U32(triangleData.capacity()/12);
	}
	/** Renders triangles */
	virtual void renderTriangles();

	/** Adds coordinate frame axes */
	inline void addAxes(const Mat34 &pose, const Vec3 &size) {
		Vec3 axis;
		
		axis.set(size.v1, Real(0.0), Real(0.0)); // X axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, RGBA::RED);
		
		axis.set(Real(0.0), size.v2, Real(0.0)); // Y axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, RGBA::GREEN);
		
		axis.set(Real(0.0), Real(0.0), size.v3); // Z axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, RGBA::BLUE);
	}

	/** Incrementally reserves memory buffer for the specified number of axes */
	inline void reserveAxesInc(U32 numOfAxes) {
		const U32 size = sizeLines() + 3*numOfAxes;
		if (size > capacityLines())
			reserveLines(size);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_RENDERER_H_*/
