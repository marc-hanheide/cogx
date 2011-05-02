/** @file Renderer.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Renderer.h>
#include <GL/freeglut.h>
#include <memory.h>

using namespace golem;

//------------------------------------------------------------------------------

const RGBA RGBA::BLACK		= RGBA(  0,   0,   0, 255);
const RGBA RGBA::RED		= RGBA(255,   0,   0, 255);
const RGBA RGBA::GREEN		= RGBA(  0, 255,   0, 255);
const RGBA RGBA::YELLOW		= RGBA(255, 255,   0, 255);
const RGBA RGBA::BLUE		= RGBA(  0,   0, 255, 255);
const RGBA RGBA::MAGENTA	= RGBA(255,   0, 255, 255);
const RGBA RGBA::CYAN		= RGBA(  0, 255, 255, 255);
const RGBA RGBA::WHITE		= RGBA(255, 255, 255, 255);

//------------------------------------------------------------------------------

const RGBA BoundsRenderer::glWireColourDflt = RGBA(127, 127, 127, 255);
const RGBA BoundsRenderer::glSolidColourDflt = RGBA(255, 255, 255, 75);
const RGBA BoundsRenderer::glShadowColourDflt = RGBA(12, 25, 37, 255);
const Real BoundsRenderer::glLineWidthDflt = 1.0;

const ::GLfloat BoundsRenderer::glMatDflt[16] = {
	1.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 1.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 1.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 1.0f, 
};
const ::GLfloat BoundsRenderer::glShadowMatDflt[16] = {
	1.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 1.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 1.0f, 
};

BoundsRenderer::Data BoundsRenderer::cylinderTriangles;
BoundsRenderer::Data BoundsRenderer::cylinderNormals;
BoundsRenderer::Data BoundsRenderer::cylinderLines;

BoundsRenderer::BoundsRenderer() {
	reset();
}

void BoundsRenderer::reset() {
	::memcpy(glMat, glMatDflt, sizeof(glMat));
	mat.setId();
	::memcpy(glShadowMat, glShadowMatDflt, sizeof(glShadowMat));
	glLineWidth = ::GLfloat(glLineWidthDflt);
	glWireColour = glWireColourDflt;
	glSolidColour = glSolidColourDflt;
	glShadowColour = glShadowColourDflt;
	verticesData.resize(0);
	normalsData.resize(0);

	if (cylinderTriangles.empty() || cylinderNormals.empty() || cylinderLines.empty())
		initCylinder();
}

void BoundsRenderer::initCylinder() {
	const Vec3 p0(REAL_ZERO, REAL_ZERO, -REAL_HALF);
	const Vec3 p1(REAL_ZERO, REAL_ZERO, +REAL_HALF);
	const Vec3 n0(REAL_ZERO, REAL_ZERO, -REAL_ONE);
	const Vec3 n1(REAL_ZERO, REAL_ZERO, +REAL_ONE);
	const Real a = REAL_2_PI/Real(BOUNDING_CYLINDER_SLICES);
	
	cylinderTriangles.resize(BOUNDING_CYLINDER_SLICES*3*12);
	::GLfloat *pTriangles = &cylinderTriangles.front();
	cylinderNormals.resize(BOUNDING_CYLINDER_SLICES*3*12);
	::GLfloat *pNormals = &cylinderNormals.front();
	cylinderLines.resize(BOUNDING_CYLINDER_SLICES*3*24);
	::GLfloat *pLines = &cylinderLines.front();
	
	Vec3 prev0, prev1, next0, next1;
	prev0.set(REAL_ONE, REAL_ZERO, -REAL_HALF);
	prev1.set(REAL_ONE, REAL_ZERO, +REAL_HALF);
	for (U32 i = 0; i < BOUNDING_CYLINDER_SLICES; i++) {
		Real sin, cos;
		Math::sinCos(a*Real(i + 1), sin, cos);
		const Vec3 n(cos, sin, REAL_ZERO);

		next0.add(p0, n);
		next1.add(p1, n);
		
		// bottom
		p0.get(&pTriangles[0]);
		next0.get(&pTriangles[3]);
		prev0.get(&pTriangles[6]);
		pTriangles += 9;
		n0.get(&pNormals[0]);
		n0.get(&pNormals[3]);
		n0.get(&pNormals[6]);
		pNormals += 9;
		p0.get(&pLines[0]);
		prev0.get(&pLines[3]);
		prev0.get(&pLines[6]);
		next0.get(&pLines[9]);
		next0.get(&pLines[12]);
		p0.get(&pLines[15]);
		pLines += 18;
		
		// top
		p1.get(&pTriangles[0]);
		prev1.get(&pTriangles[3]);
		next1.get(&pTriangles[6]);
		pTriangles += 9;
		n1.get(&pNormals[0]);
		n1.get(&pNormals[3]);
		n1.get(&pNormals[6]);
		pNormals += 9;
		p1.get(&pLines[0]);
		prev1.get(&pLines[3]);
		prev1.get(&pLines[6]);
		next1.get(&pLines[9]);
		next1.get(&pLines[12]);
		p1.get(&pLines[15]);
		pLines += 18;
		
		// side
		prev0.get(&pTriangles[0]);
		next0.get(&pTriangles[3]);
		prev1.get(&pTriangles[6]);
		next1.get(&pTriangles[9]);
		prev1.get(&pTriangles[12]);
		next0.get(&pTriangles[15]);
		pTriangles += 18;
		n.get(&pNormals[0]);
		n.get(&pNormals[3]);
		n.get(&pNormals[6]);
		n.get(&pNormals[9]);
		n.get(&pNormals[12]);
		n.get(&pNormals[15]);
		pNormals += 18;
		prev0.get(&pLines[0]);
		next0.get(&pLines[3]);
		next0.get(&pLines[6]);
		prev1.get(&pLines[9]);
		prev1.get(&pLines[12]);
		prev0.get(&pLines[15]);
		prev1.get(&pLines[18]);
		next1.get(&pLines[21]);
		next1.get(&pLines[24]);
		next0.get(&pLines[27]);
		next0.get(&pLines[30]);
		prev1.get(&pLines[33]);
		pLines += 36;
		
		prev0 = next0;
		prev1 = next1;
	}
}

//------------------------------------------------------------------------------

void BoundsRenderer::renderWire(const golem::BoundingPlane& bounds) {
}

void BoundsRenderer::renderSolid(const golem::BoundingPlane& bounds) {
}

void BoundsRenderer::renderWire(const golem::BoundingSphere& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Real r = bounds.getRadius();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	::glScaled(r, r, r);
	::glutWireSphere(GLdouble(1.0), BOUNDING_SPHERE_SLICES, BOUNDING_SPHERE_STACKS);

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingSphere& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Real r = bounds.getRadius();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	::glScaled(r, r, r);
	::glutSolidSphere(GLdouble(1.0), BOUNDING_SPHERE_SLICES, BOUNDING_SPHERE_STACKS);

	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingCylinder& bounds, bool useITriangleMesh) {
	if (useITriangleMesh) {
		renderWire((const golem::ITriangleMesh&)bounds);
		return;
	}

	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	
	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	Real r = bounds.getRadius();
	Real l = bounds.getLength();
	::glScaled(r, r, l);
	
	glDrawLines(&cylinderLines.front(), cylinderLines.size()/6);

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingCylinder& bounds, bool useITriangleMesh) {
	if (useITriangleMesh) {
		renderSolid((const golem::ITriangleMesh&)bounds);
		return;
	}

	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	
	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	Real r = bounds.getRadius();
	Real l = bounds.getLength();
	::glScaled(r, r, l);

	glDrawTriangles(&cylinderTriangles.front(), &cylinderNormals.front(), cylinderTriangles.size()/9);

	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingBox& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Vec3 d = bounds.getDimensions();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	::glScaled(d.v1, d.v2, d.v3);
	::glutWireCube(GLdouble(2.0));
	
	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingBox& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Vec3 d = bounds.getDimensions();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	::glScaled(d.v1, d.v2, d.v3);
	::glutSolidCube(GLdouble(2.0));
	
	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingConvexMesh& bounds) {
	renderWire((const golem::ITriangleMesh&)bounds);
}

void BoundsRenderer::renderSolid(const golem::BoundingConvexMesh& bounds) {
	renderSolid((const golem::ITriangleMesh&)bounds);
}

void BoundsRenderer::renderWire(const golem::ITriangleMesh& mesh) {
	::glPushMatrix();
	setMat(glMat, this->mat);
	::glMultMatrixf(&(glMat[0]));
	
	const U32 numOfTriangles = (U32)mesh.getTriangles().size();
	verticesData.resize(6*3*numOfTriangles);
	
	const Vec3* pVertices = &mesh.getVertices().front();
	const Triangle* pTriangles = &mesh.getTriangles().front();
	::GLfloat *pVerticesData = &verticesData.front();
	for (U32 i = 0; i < numOfTriangles; i++) {
		const Vec3 &p0 = pVertices[pTriangles->t1];
		const Vec3 &p1 = pVertices[pTriangles->t2];
		const Vec3 &p2 = pVertices[pTriangles->t3];
		
		p0.get(&pVerticesData[0]);
		p1.get(&pVerticesData[3]);
		p1.get(&pVerticesData[6]);
		p2.get(&pVerticesData[9]);
		p2.get(&pVerticesData[12]);
		p0.get(&pVerticesData[15]);
		
		pTriangles += 1;
		pVerticesData += 6*3;
	}
	
	glDrawLines(&verticesData.front(), 3*numOfTriangles);

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::ITriangleMesh& mesh) {
	::glPushMatrix();
	setMat(glMat, this->mat);
	::glMultMatrixf(&(glMat[0]));
	
	const U32 numOfTriangles = (U32)mesh.getTriangles().size();
	verticesData.resize(3*3*numOfTriangles);
	normalsData.resize(3*3*numOfTriangles);
	
	const Vec3* pVertices = &mesh.getVertices().front();
	const Vec3* pNormals = &mesh.getNormals().front();
	const Triangle* pTriangles = &mesh.getTriangles().front();
	::GLfloat *pVerticesData = &verticesData.front();
	::GLfloat *pNormalsData = &normalsData.front();
	for (U32 i = 0; i < numOfTriangles; i++) {
		const Vec3 &p0 = pVertices[pTriangles->t1];
		const Vec3 &p1 = pVertices[pTriangles->t2];
		const Vec3 &p2 = pVertices[pTriangles->t3];
		
		p0.get(&pVerticesData[0]);
		p1.get(&pVerticesData[3]);
		p2.get(&pVerticesData[6]);
		
		Vec3 n = pNormals[i];
		n.setNegative();
		
		n.get(&pNormalsData[0]);
		n.get(&pNormalsData[3]);
		n.get(&pNormalsData[6]);
		
		pTriangles += 1;
		pVerticesData += 3*3;
		pNormalsData += 3*3;
	}
	
	glDrawTriangles(&verticesData.front(), &normalsData.front(), numOfTriangles);

	::glPopMatrix();
}

//------------------------------------------------------------------------------

void BoundsRenderer::renderWire(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		renderWire(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		renderWire(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderWire(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderWire(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderWire(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

void BoundsRenderer::renderSolid(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		renderSolid(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		renderSolid(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderSolid(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderSolid(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderSolid(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

void BoundsRenderer::renderShadow(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	//case Bounds::TYPE_PLANE:
	//	renderSolid(dynamic_cast<const BoundingPlane&>(bounds));
	//	break;
	case Bounds::TYPE_SPHERE:
		renderSolid(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderSolid(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderSolid(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderSolid(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------

DebugRenderer::DebugRenderer() {
}

void DebugRenderer::render(const Real *pData, ::GLenum mode, ::GLsizei count) const {
	::glDisable(GL_LIGHTING);
	::glEnableClientState(GL_VERTEX_ARRAY);
#ifdef REAL_F32
	::glVertexPointer(3, GL_FLOAT, 4*sizeof(::GLfloat), pData);
#else // REAL_F64
	::glVertexPointer(3, GL_DOUBLE, 4*sizeof(::GLdouble), pData);
#endif
	::glEnableClientState(GL_COLOR_ARRAY);
#ifdef REAL_F32
	::glColorPointer(4, GL_UNSIGNED_BYTE, 4*sizeof(::GLfloat), pData + 3);
#else // REAL_F64
	::glColorPointer(4, GL_UNSIGNED_BYTE, 4*sizeof(::GLdouble), pData + 3);
#endif
	::glDrawArrays(mode, (::GLint)0, count/4);
	::glDisableClientState(GL_COLOR_ARRAY);
	::glDisableClientState(GL_VERTEX_ARRAY);
	::glEnable(GL_LIGHTING);
}

/** Renders the object */
void DebugRenderer::render() {
	renderPoints();
	renderLines();
	renderTriangles();
}

/** Resets render buffers */
void DebugRenderer::reset() {
	resizePoints();
	resizeLines();
	resizeTriangles();
}

/** Renders points */
void DebugRenderer::renderPoints() {
	const ::GLsizei count = (::GLsizei)pointData.size();
	if (count > 0)
		render((const Real*)&pointData.front(), GL_POINTS, count);
}

/** Renders lines */
void DebugRenderer::renderLines() {
	const ::GLsizei count = (::GLsizei)lineData.size();
	if (count > 0)
		render((const Real*)&lineData.front(), GL_LINES, count);
}

/** Renders triangles */
void DebugRenderer::renderTriangles() {
	const ::GLsizei count = (::GLsizei)triangleData.size();
	if (count > 0)
		render((const Real*)&triangleData.front(), GL_TRIANGLES, count);
}

//------------------------------------------------------------------------------

