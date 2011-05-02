/** @file TriangleMesh.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/TriangleMesh.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool ITriangleMesh::findTriangles(const std::vector<Vec3> &vertices, std::vector<Triangle> &triangles) {
	// TODO use any convex hull algorithm (e.g. quick hull)
	return false;
}

bool ITriangleMesh::findNormals(const std::vector<Vec3>& vertices, const std::vector<Triangle>& triangles, std::vector<Vec3>& normals, std::vector<Real>& distances) {
	const size_t TRIANGLES = triangles.size();
	normals.resize(TRIANGLES);
	distances.resize(TRIANGLES);
	for (size_t i = 0; i < TRIANGLES; ++i) {
		const Triangle t = triangles[i];
		const Vec3 p0 = vertices[t.t1];
		const Vec3 p1 = vertices[t.t2];
		const Vec3 p2 = vertices[t.t3];
		
		Vec3 p01, p02, normal;
		p01.subtract(p1, p0);
		p02.subtract(p2, p0);
		normal.cross(p02, p01);
		normal.normalise();
		
		normals[i] = normal;
		distances[i] = normal.dot(p0);
	}

	return true;
}

//------------------------------------------------------------------------------

bool TriangleMesh::create(const Desc& desc) {
	if (!desc.isValid())
		return false;

	// setup vertices
	vertices = desc.vertices;
	// setup triangles
	if (desc.bCook && !ITriangleMesh::findTriangles(vertices, triangles))
		return false;
	else
		triangles = desc.triangles;

	// setup normals
	(void)ITriangleMesh::findNormals(vertices, triangles, normals, distances);

	return isValid();
}

//------------------------------------------------------------------------------
