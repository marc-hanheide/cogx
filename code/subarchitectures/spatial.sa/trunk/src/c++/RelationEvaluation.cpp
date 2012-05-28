#include <cast/core/CASTUtils.hpp>
#include "RelationEvaluation.hpp"
using namespace cast;
#include <Pose3.h>
#include <iostream>
#include <set>

using namespace std;
using namespace cogx::Math;

namespace spatial {

struct ActiveFace {
	double normalExpansion;
	double lateralExpansion;
	double length;
};

RelationEvaluator::RelationEvaluator() {
	patchThreshold = 0.020;

	// Separation/interpenetration at which onness drops by half
	// IROS 2010 sim values: 0.007, 0.004
	distanceFalloffOutside = 0.007;
	distanceFalloffInside = 0.004;

	// Abruptness of transition as the COM moves out from
	// inside the (horizontal projection of) the contact patch
	supportCOMContainmentSteepness = 1;
	// Offset for point of greatest slope. Positive means
	// slope is greatest somewhere outside the patch boundary
	supportCOMContainmentOffset = -0.5;

	planeThickness = 0.05;
	circlePlaneApproximationThreshold = 0.05; //Controls number of
	//edges in polygon used to approximate circular planes
	cylinderApproximationThreshold = 0.01;
	sphereTessellationFactor = 2; //Number of latitudes and half number of
	//longitudes. 2 makes the sphere an octahedron
	boxThickness = 0.02; //Controls thickness of walls of hollow container
}

double RelationEvaluator::computePolyhedronVolume(const Polyhedron &polyhedron) {
	if (polyhedron.vertices.size() < 4)
		return 0.0;

	//Algorithm: Move a virtual plane upwards from the lowest vertex in the
	//polyhedron. Integrate the area of the cross-section polygon.

	//Maintain a list of active 3D edges (that become points in the polygon)
	//Each active edge imposes 2 active faces on each side (that become
	//2D edges in the polygon). Each active edge notifies its active faces
	//of which its neighbor face is on that side.

	//The angle between the polygon edges is computable from the face normals,
	//projected in the xy plane. The rate of extension or retraction along
	//the edge's normal, as the plane moves upward at unit speed, can also be
	//gleaned from the normal vector.
	//From the angle between edges and the extension/retraction rate, the
	//rate of edge elongation/contraction (lateral) can  be computed:
	//Lateral speed (one side) = 
	//extension_rate_this_edge*cot(angle) + 
	//extension_rate_other_edge/sin(angle)
	//(Add both sides together for the total lateral expansion/contraction rate.

	//The new area of the polygon after a displacement h is:
	//A_new = A_old + 
	// sum[over edges] normal_extension_rate*(Length_old + 0.5*h*lateral_extension_rate 
	//and the incremental volume over this displacement is:
	//V = A_0*(h) + h^3/6*sum[lateral_rate*normal_rate] + 
	//  h^2/2*sum[length_old*normal_rate]

	const double epsilon = 1e-8;
	vector<Vector3> verts = polyhedron.vertices;
	const vector<vector<Edge> > &faces = polyhedron.faces;

	bool noSameZ = true;
	int tries = 0;
	do {
		tries++;
		if (tries > 100) {
			cerr << "Warning: volume computation failed.\n";
			return 0.0;
		}
		noSameZ = true;
		for (unsigned int i = 0; i < verts.size() - 1; i++) {
			for (unsigned int j = i + 1; j < verts.size(); j++) {
				if (abs(verts[i].z - verts[j].z) < epsilon) {
					noSameZ = false;
				}
			}
		}
		if (!noSameZ) {
			Matrix33 tmp;
			fromRotX(tmp, 0.01);
			Matrix33 tmp2;
			fromRotY(tmp2, 0.003);
			Pose3 rot;
			rot.rot = tmp * tmp2;
			for (unsigned int i = 0; i < verts.size(); i++) {
				verts[i] = transform(rot, verts[i]);
			}
		}
	} while (!noSameZ);

	vector<vector<unsigned int> > vertexUpwardEdges(verts.size());
	vector<vector<unsigned int> > vertexDownwardEdges(verts.size());

	vector<pair<unsigned int, unsigned int> > edgeFaceNeighbors;
	vector<Vector3> edgeDirs;

	map<Edge, unsigned int> directedEdgeToUpwardEdgeMap;

	vector<double> faceNormalRate(faces.size());
	vector<Vector3> faceEdgeNormal(faces.size());

	for (unsigned int i = 0; i < faces.size(); i++) {
		for (unsigned int j = 0; j < faces[i].size(); j++) {
			unsigned int fromVertNo = faces[i][j].first;
			unsigned int toVertNo = faces[i][j].second;

			double zDiff = verts[toVertNo].z - verts[fromVertNo].z;

			// Each edge occurs twice; once in each of its neighbor faces.
			// If the edge points upward along the winding of the current face,
			// then that face is to the left of the upgoing edge.
			// Then set this face as neighbor 1 of that edge. Otherwise,
			// set it as number 2.

			if (directedEdgeToUpwardEdgeMap.find(faces[i][j])
					== directedEdgeToUpwardEdgeMap.end()) {
				//Edge not yet registered in upward-directed representation. Do so now.
				if (zDiff > 0) {
					edgeDirs.push_back(verts[toVertNo] - verts[fromVertNo]);
					unsigned int newEdgeID = edgeDirs.size() - 1;

					vertexUpwardEdges[fromVertNo].push_back(newEdgeID);
					vertexDownwardEdges[toVertNo].push_back(newEdgeID);

					directedEdgeToUpwardEdgeMap[Edge(fromVertNo, toVertNo)] = newEdgeID;
					directedEdgeToUpwardEdgeMap[Edge(toVertNo, fromVertNo)] = newEdgeID;
				} else {
					edgeDirs.push_back(verts[fromVertNo] - verts[toVertNo]);
					unsigned int newEdgeID = edgeDirs.size() - 1;

					vertexUpwardEdges[toVertNo].push_back(newEdgeID);
					vertexDownwardEdges[fromVertNo].push_back(newEdgeID);

					directedEdgeToUpwardEdgeMap[Edge(fromVertNo, toVertNo)] = newEdgeID;
					directedEdgeToUpwardEdgeMap[Edge(toVertNo, fromVertNo)] = newEdgeID;
				}
				edgeFaceNeighbors.push_back(pair<unsigned int, unsigned int> ());
			}

			if (zDiff > 0) {
				edgeFaceNeighbors[directedEdgeToUpwardEdgeMap[faces[i][j]]].first = i;
			} else {
				edgeFaceNeighbors[directedEdgeToUpwardEdgeMap[faces[i][j]]].second = i;
			}
		}

		//Compute normal
		Vector3 p1 = verts[faces[i][0].first];
		Vector3 p2 = verts[faces[i][0].second];
		Vector3 p3 = verts[faces[i].back().first];

		Vector3 normal = cross(p2 - p1, p3 - p1);
		normalise(normal);
		double zComp = normal.z;

		normal.z = 0.0;
		double horizontalComp = length(normal);

		faceNormalRate[i] = -zComp / horizontalComp; //-Tan of the inclination

		normal = normal / horizontalComp;
		faceEdgeNormal[i] = normal;
	}

	std::set<unsigned int> activeEdges;

	//Find vertex with lowest z-coordinate
	double minZ = FLT_MAX;
	unsigned int currentVertex = 0;
	for (unsigned int i = 0; i < verts.size(); i++) {
		if (verts[i].z < minZ) {
			currentVertex = i;
			minZ = verts[i].z;
		}
	}

	//Initialize cross-section polygon from a single point (the lowest one),
	//N edges and N polygon vertices
	double currentPolygonArea = 0.0;
	double accumulatedVolume = 0.0;
	unsigned int verticesProcessed = 0;

	vector<double> polygonEdgeLengths(faces.size());
	vector<double> leftLateralExtensionSpeed(faces.size());
	vector<double> rightLateralExtensionSpeed(faces.size());

	while (verticesProcessed < verts.size() - 1) {
		double currentZ = verts[currentVertex].z;
		//Check all down-going edges of this vertex. Deactivate them;
		//set the adjoining faces' half lateral extension speeds to 0
		for (vector<unsigned int>::iterator it =
				vertexDownwardEdges[currentVertex].begin(); it
				!= vertexDownwardEdges[currentVertex].end(); it++) {

			unsigned int leftFaceOfEdge = edgeFaceNeighbors[*it].first;
			unsigned int rightFaceOfEdge = edgeFaceNeighbors[*it].second;

			rightLateralExtensionSpeed[leftFaceOfEdge] = 0.0;
			leftLateralExtensionSpeed[rightFaceOfEdge] = 0.0;

			activeEdges.erase(*it);
		}

		//Check all upgoing edges of this vertex. Activate them;
		//set the adjoining faces' half lateral extension speeds
		for (vector<unsigned int>::iterator it =
				vertexUpwardEdges[currentVertex].begin(); it
				!= vertexUpwardEdges[currentVertex].end(); it++) {

			unsigned int leftFaceOfEdge = edgeFaceNeighbors[*it].first;
			unsigned int rightFaceOfEdge = edgeFaceNeighbors[*it].second;

			double angleAtEdgeCos = -dot(faceEdgeNormal[leftFaceOfEdge],
					faceEdgeNormal[rightFaceOfEdge]);

			double angleAtEdgeSin = sqrt(1 - angleAtEdgeCos * angleAtEdgeCos);

			if (abs(angleAtEdgeSin) < epsilon) {
				angleAtEdgeSin = epsilon;
			}

			rightLateralExtensionSpeed[leftFaceOfEdge]
					= faceNormalRate[leftFaceOfEdge] * angleAtEdgeCos / angleAtEdgeSin
							+ faceNormalRate[rightFaceOfEdge] / angleAtEdgeSin;
			leftLateralExtensionSpeed[rightFaceOfEdge]
					= faceNormalRate[rightFaceOfEdge] * angleAtEdgeCos / angleAtEdgeSin
							+ faceNormalRate[leftFaceOfEdge] / angleAtEdgeSin;

			activeEdges.insert(*it);
		}

		//Find next vertex in z-ordering
		double minZ = FLT_MAX;
		unsigned int nextVertex = 0;
		for (unsigned int i = 0; i < verts.size(); i++) {
			if (verts[i].z > currentZ && verts[i].z < minZ) {
				minZ = verts[i].z;
				nextVertex = i;
			}
		}

		//Accumulate volume
		double dz = minZ - currentZ;
		accumulatedVolume += dz * currentPolygonArea;

		for (std::set<unsigned int>::iterator it = activeEdges.begin(); it
				!= activeEdges.end(); it++) {
			//Take the face to the left of this edge

			//Accumulate volume
			unsigned int faceID = edgeFaceNeighbors[*it].first;
			accumulatedVolume += dz * dz * dz / 6 * faceNormalRate[faceID]
					* (rightLateralExtensionSpeed[faceID]
							+ leftLateralExtensionSpeed[faceID]) + dz * dz / 2
					* polygonEdgeLengths[faceID] * faceNormalRate[faceID];

			//Update cross-section area
			currentPolygonArea += dz * dz / 2 * (rightLateralExtensionSpeed[faceID]
					+ leftLateralExtensionSpeed[faceID]) * faceNormalRate[faceID] + dz
					* polygonEdgeLengths[faceID] * faceNormalRate[faceID];

			//Update face edge lengths
			polygonEdgeLengths[faceID] += (leftLateralExtensionSpeed[faceID]
					+ rightLateralExtensionSpeed[faceID]) * dz;
		}

		verticesProcessed++;
		currentVertex = nextVertex;
	}

	return accumulatedVolume;
}

void RelationEvaluator::clipPolyhedronToPlane(Polyhedron &polyhedron,
		const Vector3 &pointInPlane, const Vector3 &planeNormal) {
	if (polyhedron.vertices.size() == 0)
		return;
	double epsilon = 1e-6;
	// Cut in half any edge passing through the plane. Keep edges, faces, vertices
	// on positive side of plane

	// Find all vertices that will be pruned

	// Find all edges with one of each type of vertex
	// Find intersection point with plane, shorten edge to this point

	std::set<unsigned int> verticesToPrune;
	//Loop over vertices
	for (unsigned int i = 0; i < polyhedron.vertices.size(); i++) {
		if (dot((polyhedron.vertices[i] - pointInPlane), planeNormal) < -epsilon) {
			verticesToPrune.insert(i);
		}
	}

	unsigned int newVertsStartAt = polyhedron.vertices.size();
	vector<vector<Edge> > newFaces;

	map<Edge, unsigned int> edgeToSplitVertexMap;

	for (unsigned int faceNo = 0; faceNo < polyhedron.faces.size(); faceNo++) {
		vector<Edge> &edges = polyhedron.faces[faceNo];
		int edgeGoingOut = -1;
		int edgeGoingIn = -1;
		for (unsigned int edgeNo = 0; edgeNo < edges.size(); edgeNo++) {
			Edge &edge = edges[edgeNo];
			bool vert1Inside = (verticesToPrune.find(edges[edgeNo].first)
					== verticesToPrune.end());
			bool vert2Inside = (verticesToPrune.find(edges[edgeNo].second)
					== verticesToPrune.end());
			if (vert1Inside != vert2Inside) {
				unsigned int newVertId;
				if (edgeToSplitVertexMap.find(edge) == edgeToSplitVertexMap.end()) {
					// Edge intersects plane. Find intersection point
					Vector3 edgeVector = polyhedron.vertices[edge.second]
							- polyhedron.vertices[edge.first];
					double intersectionParam = dot(pointInPlane
							- polyhedron.vertices[edge.first], planeNormal) / dot(edgeVector,
							planeNormal);

					// Create new vertex; put it in list

					Vector3 newVertex = polyhedron.vertices[edge.first] + edgeVector
							* intersectionParam;
					polyhedron.vertices.push_back(newVertex);

					newVertId = polyhedron.vertices.size() - 1;
				} else {
					newVertId = edgeToSplitVertexMap[edge];
				}

				edgeToSplitVertexMap[Edge(edge.second, edge.first)] = newVertId;
				edgeToSplitVertexMap[Edge(edge.first, edge.second)] = newVertId;

				// Adjust the cut edge so it has the new vertex for an end/beginning
				if (vert1Inside && !vert2Inside) {
					edge.second = newVertId;
					edgeGoingOut = edgeNo;
				} else {
					edge.first = newVertId;
					edgeGoingIn = edgeNo;
				}
			}
		}

		// For all faces, check each edge if it's been cut. If an edge has been cut,
		// find the other edge that's been cut (there has to be exactly one); add new
		// edge between this vertex and that one

		if (edgeGoingOut > -1) {
			vector<Edge> newFace;
			int i = edgeGoingIn;
			for (; i != edgeGoingOut;) {
				newFace.push_back(polyhedron.faces[faceNo][i]);

				i++;
				if ((unsigned int) i >= polyhedron.faces[faceNo].size())
					i = 0;
			}
			newFace.push_back(polyhedron.faces[faceNo][edgeGoingOut]);
			newFace.push_back(Edge(polyhedron.faces[faceNo][edgeGoingOut].second,
					polyhedron.faces[faceNo][edgeGoingIn].first));
			newFaces.push_back(newFace);
		} else {
			// This face doesn't intersect the plane. Is it completely clipped?
			bool vert1Inside = (verticesToPrune.find(edges[0].first)
					== verticesToPrune.end());
			if (vert1Inside) {
				// Leave this face in
				newFaces.push_back(edges);
			}
		}
	}

	// Get convex hull of newly created vertices, add a face with those

	if (newVertsStartAt < polyhedron.vertices.size() - 1) {
		vector<unsigned int> newFace;

		// Compute convex hull of newInterestPoints
		unsigned int currentPoint = newVertsStartAt;

		std::set<int> donePoints;
		do {
			newFace.push_back(currentPoint);
			donePoints.insert(currentPoint);
			unsigned int nextPoint =
					(currentPoint == polyhedron.vertices.size() - 1 ? newVertsStartAt
							: currentPoint + 1);

			Vector3 edge = polyhedron.vertices[nextPoint]
					- polyhedron.vertices[currentPoint];
			for (unsigned int otherPoint = newVertsStartAt; otherPoint
					< polyhedron.vertices.size(); otherPoint++) {
				if (otherPoint != currentPoint && otherPoint != nextPoint) {
					double leftness = dot(-planeNormal, cross(edge,
							polyhedron.vertices[otherPoint]
									- polyhedron.vertices[currentPoint]));
					if (leftness < -epsilon) {
						nextPoint = otherPoint;
						edge = polyhedron.vertices[otherPoint]
								- polyhedron.vertices[currentPoint];
					}
				}
			}

			currentPoint = nextPoint;
		} while (donePoints.find(currentPoint) == donePoints.end());
		newFaces.push_back(vector<Edge> ());
		for (unsigned int i = 0; i < newFace.size() - 1; i++) {
			newFaces.back().push_back(Edge(newFace[i], newFace[i + 1]));
		}
		newFaces.back().push_back(Edge(newFace.back(), newFace.front()));
	}

	// Shrink vertex set
	map<int, int> oldVertexToNewVertexMap;
	vector<Vector3> newVertexSet;
	for (unsigned int i = 0; i < polyhedron.vertices.size(); i++) {
		oldVertexToNewVertexMap[i] = newVertexSet.size();
		if (verticesToPrune.find(i) == verticesToPrune.end()) {
			newVertexSet.push_back(polyhedron.vertices[i]);
		}
	}
	polyhedron.vertices = newVertexSet;
	for (unsigned int i = 0; i < newFaces.size(); i++) {
		for (unsigned int j = 0; j < newFaces[i].size(); j++) {
			newFaces[i][j].first = oldVertexToNewVertexMap[newFaces[i][j].first];
			newFaces[i][j].second = oldVertexToNewVertexMap[newFaces[i][j].second];
		}
	}
	polyhedron.faces = newFaces;
}

std::vector<Vector3> RelationEvaluator::findPolygonIntersection(
		const std::vector<Vector3> &polygon1, const std::vector<Vector3> &polygon2) {
	// Find all vertices of either polygon that is strictly inside the other,
	// and all intersection points. Then find the convex hull around these
	// interest points.
	std::vector<Vector3> interestPoints;

	double epsilon = 1e-6;
	Vector3 polygonNormal = cross(polygon2[2] - polygon2[1], polygon2[0]
			- polygon2[1]);

	// Points of polygon 1 inside polygon 2
	for (unsigned int i = 0; i < polygon1.size(); i++) {
		bool vertex_i_inside_polygon_2 = true;
		for (unsigned int j = 0; j < polygon2.size(); j++) {
			unsigned int jplus = (j == polygon2.size() - 1 ? 0 : j + 1);

			if (dot(polygonNormal, cross(polygon2[jplus] - polygon2[j], polygon1[i]
					- polygon2[j])) < 0.0) {
				vertex_i_inside_polygon_2 = false;
				break;
			}
		}
		if (vertex_i_inside_polygon_2) {
			interestPoints.push_back(polygon1[i]);
		}
	}
	// Points of polygon 2 inside polygon 1
	for (unsigned int i = 0; i < polygon2.size(); i++) {
		bool vertex_i_inside_polygon_1 = true;
		for (unsigned int j = 0; j < polygon1.size(); j++) {
			unsigned int jplus = (j == polygon1.size() - 1 ? 0 : j + 1);

			if (dot(polygonNormal, cross(polygon1[jplus] - polygon1[j], polygon2[i]
					- polygon1[j])) < 0.0) {
				vertex_i_inside_polygon_1 = false;
				break;
			}
		}
		if (vertex_i_inside_polygon_1) {
			interestPoints.push_back(polygon2[i]);
		}
	}

	for (unsigned int i = 0; i < polygon1.size(); i++) {
		unsigned int iplus = (i == polygon1.size() - 1 ? 0 : i + 1);
		Vector3 currentPoint = polygon1[i];
		Vector3 thisEdge = polygon1[iplus] - currentPoint;

		for (unsigned int j = 0; j < polygon2.size(); j++) {
			unsigned int jplus = (j == polygon2.size() - 1 ? 0 : j + 1);

			Vector3 difference;
			Vector3 otherEdgeDir;
			double otherEdgeLength;
			difference = polygon2[j] - currentPoint;
			otherEdgeDir = polygon2[jplus] - polygon2[j];

			if (abs(dot(thisEdge, otherEdgeDir)) == 1.0) {
				continue;
			}

			otherEdgeLength = length(otherEdgeDir);
			otherEdgeDir /= otherEdgeLength;

			// The normal vector from currentPoint to the other edge's line
			Vector3 normalVector = difference - otherEdgeDir * dot(difference,
					otherEdgeDir);
			double normalLength = length(normalVector);

			double intersectionParamThisEdge; // 0 - 1
			if (!equals(normalLength, 0.0, epsilon)) {
				intersectionParamThisEdge = normalLength * normalLength / (dot(
						normalVector, thisEdge));
			} else {
				intersectionParamThisEdge = 0.0;
			}

			if (intersectionParamThisEdge > -epsilon && (intersectionParamThisEdge
					< 1.0 + epsilon)) {
				Vector3 intersectionPoint = currentPoint + thisEdge
						* intersectionParamThisEdge;

				double intersectionParamOtherEdge;
				intersectionParamOtherEdge = dot(otherEdgeDir, intersectionPoint
						- polygon2[j]) / otherEdgeLength;

				if (intersectionParamOtherEdge > -epsilon && intersectionParamOtherEdge
						< 1.0 + epsilon) {
					interestPoints.push_back(intersectionPoint);
				}
			}
		}
	}

	vector<Vector3> outPolygon;

	computeConvexHull(interestPoints, polygonNormal, outPolygon);

	return outPolygon;
}

void RelationEvaluator::computeConvexHull(const std::vector<Vector3>& points,
		const Vector3 &polygonNormal, std::vector<Vector3>& hull) {
	double epsilon = 1e-6;

	std::vector<Vector3> newPoints;

	// Eliminate redundant interest points
	for (unsigned int i = 0; i < points.size(); i++) {
		bool keepThis = true;
		for (unsigned int j = i + 1; j < points.size(); j++) {
			if (vequals(points[i], points[j], epsilon)) {
				keepThis = false;
				break;
			}
		}
		if (keepThis) {
			newPoints.push_back(points[i]);
		}
	}

	if (newPoints.size() < 4) {
		hull = newPoints;
		return;
	}

	//Compute convex hull
	//Find rightmost point
	double maxX = -FLT_MAX;
	double maxY = -FLT_MAX;
	double minX = FLT_MAX;
	double minY = FLT_MAX;
	unsigned int highestXPoint;
	unsigned int highestYPoint;
	for (unsigned int i = 0; i < newPoints.size(); i++) {
		if (newPoints[i].x > maxX) {
			highestXPoint = i;
			maxX = newPoints[i].x;
		}
		if (newPoints[i].y > maxY) {
			highestYPoint = i;
			maxY = newPoints[i].y;
		}

		if (newPoints[i].x < minX) {
			minX = newPoints[i].x;
		}
		if (newPoints[i].y < minY) {
			minY = newPoints[i].y;
		}
	}
	// Avoid case where all points have the same X (or Y)
	unsigned int startPoint = (maxX - minX > maxY - minY) ? highestXPoint
			: highestYPoint;

	// Compute convex hull of points
	unsigned int currentPoint = startPoint;

	std::deque<int> outPoints;
	std::set<int> donePoints;
	do {
		outPoints.push_back(currentPoint);
		donePoints.insert(currentPoint);
		unsigned int nextPoint = (currentPoint == newPoints.size() - 1 ? 0
				: currentPoint + 1);

		Vector3 edge = newPoints[nextPoint] - newPoints[currentPoint];
		for (unsigned int otherPoint = 0; otherPoint < newPoints.size(); otherPoint++) {
			if (otherPoint != currentPoint && otherPoint != nextPoint) {
				double leftness = dot(polygonNormal, cross(edge, newPoints[otherPoint]
						- newPoints[currentPoint]));
				if (leftness < 0) {
					nextPoint = otherPoint;
					edge = newPoints[otherPoint] - newPoints[currentPoint];
				}
			}
		}

		currentPoint = nextPoint;
	} while (donePoints.find(currentPoint) == donePoints.end());
	//Shear off any false starts
	while (startPoint != currentPoint) {
		outPoints.pop_front();
		startPoint = outPoints.front();
	}

	hull.clear();

	while (!outPoints.empty()) {
		hull.push_back(newPoints[outPoints.front()]);
		outPoints.pop_front();
	}
}

double RelationEvaluator::findOverlappingArea(
		const std::vector<Vector3>& polygon, Vector3 circleCenter,
		double circleRadius, const Vector3 &circleNormal) {
	const Vector3 zeroVec = vector3(0, 0, 0);
	unsigned nextIndex = 1;
	vector<Vector3> entryEgressPoints;
	vector<Vector3> internalPolygonPoints;
	bool entryEgressPointsStartWithEntry = false;
	bool inside = true;
	Vector3 currentPoint = polygon[0];
	bool finished = false;
	while (!finished) {
		Vector3 nextPoint = polygon[nextIndex];
		Vector3 radiusVector = (currentPoint - circleCenter);

		if (nextPoint == currentPoint) {
			nextIndex++;
			if (nextIndex >= polygon.size())
				nextIndex = 0;
			continue;
		}

		Vector3 edge = nextPoint - currentPoint;
		double edgeLength = length(edge);
		Vector3 edgeDir = edge / edgeLength;

		Vector3 intersection1;
		Vector3 intersection2;

		Vector3 shortestRadius = radiusVector - edgeDir
				* dot(edgeDir, radiusVector);
		double shortestRadiusLength = length(shortestRadius);

		if (shortestRadiusLength <= circleRadius) {
			Vector3 halfChord = edgeDir * sqrt(circleRadius * circleRadius
					- shortestRadiusLength * shortestRadiusLength);

			intersection1 = circleCenter + shortestRadius - halfChord;
			intersection2 = circleCenter + shortestRadius + halfChord;

			double paramOfIntersection1 = dot(edgeDir, intersection1 - currentPoint);
			double paramOfIntersection2 = dot(edgeDir, intersection2 - currentPoint);

			if (paramOfIntersection1 == 0.0 && paramOfIntersection2 > 0.0) {
				// Edge is entering circle at currentPoint.
				// This will already have registered at the end point of the
				// last edge, but if that edge was internal, we need to reverse
				// the egress point created then.
				if (!inside) {
					entryEgressPoints.pop_back();
					if (entryEgressPoints.empty()) {
						// Need to reset this too
						entryEgressPointsStartWithEntry = false;
					}
					inside = true;
				}
			}

			if (paramOfIntersection1 == edgeLength) {
				// Edge is entering the circle at nextPoint
				if (entryEgressPoints.size() == 0) {
					entryEgressPointsStartWithEntry = true;
					internalPolygonPoints.clear(); //May have added points under the assumption
					// that we were inside the circle
				}
				entryEgressPoints.push_back(nextPoint);
				internalPolygonPoints.push_back(nextPoint);
				currentPoint = nextPoint;

				inside = true;

				if (nextIndex == 0)
					finished = true;
				nextIndex++;
				if (nextIndex >= polygon.size())
					nextIndex = 0;
			} else if (paramOfIntersection2 == edgeLength) {
				// Edge is leaving the circle at nextPoint 
				entryEgressPoints.push_back(nextPoint);
				internalPolygonPoints.push_back(nextPoint);
				inside = false;

				currentPoint = nextPoint;

				if (nextIndex == 0)
					finished = true;
				nextIndex++;
				if (nextIndex >= polygon.size())
					nextIndex = 0;
			} else if (paramOfIntersection1 == paramOfIntersection2) {
				// Edge is tangent to circle except at the end point.
				// Ignore intersection, treat as normal edge
				if (inside) {
					internalPolygonPoints.push_back(nextPoint);
				}
				currentPoint = nextPoint;

				if (nextIndex == 0)
					finished = true;
				nextIndex++;
				if (nextIndex >= polygon.size())
					nextIndex = 0;
			} else if (paramOfIntersection1 > 0.0 && paramOfIntersection1
					< edgeLength) {
				// Edge is entering the circle at intersection1 between endpoints
				if (entryEgressPoints.size() == 0) {
					entryEgressPointsStartWithEntry = true;
					internalPolygonPoints.clear(); //May have added points under the assumption
					// that we were inside the circle
				}
				entryEgressPoints.push_back(intersection1);
				internalPolygonPoints.push_back(intersection1);
				currentPoint = intersection1;

				inside = true;
			} else if (paramOfIntersection2 > 0.0 && paramOfIntersection2
					< edgeLength) {
				// Edge is leaving the circle at intersection2 between endpoints
				entryEgressPoints.push_back(intersection2);
				internalPolygonPoints.push_back(intersection2);
				inside = false;

				currentPoint = nextPoint;

				if (nextIndex == 0)
					finished = true;
				nextIndex++;
				if (nextIndex >= polygon.size())
					nextIndex = 0;
			} else {
				// Edge is not intersecting the circle. If it is inside the circle,
				// add the endpoint
				if (inside) {
					internalPolygonPoints.push_back(nextPoint);
				}
				currentPoint = nextPoint;

				if (nextIndex == 0)
					finished = true;
				nextIndex++;
				if (nextIndex >= polygon.size())
					nextIndex = 0;
			}
		} else {
			// Line entirely outside circle. just move on to the next
			// point.

			currentPoint = nextPoint;

			if (nextIndex == 0)
				finished = true;
			nextIndex++;
			if (nextIndex >= polygon.size())
				nextIndex = 0;
		}
	}

	// Add up area of polygon and circle arc segments
	double totalArea = getPolygonArea(internalPolygonPoints);

	// Segments start egress points and end at entry points
	// (i.e. egress of the polygon edge is entry of the circle and the
	// beginning of an overlapping segment)
	if (entryEgressPoints.size() > 1) { // There may be 1 invalid egress point
		// left in the vector from an internal-to-internal degenerate vertex
		unsigned int entryIndex;
		unsigned int egressIndex;
		if (entryEgressPointsStartWithEntry) {
			entryIndex = 0;
			egressIndex = entryEgressPoints.size() - 1;
		} else {
			entryIndex = 1;
			egressIndex = 0;
		}
		while (entryIndex <= entryEgressPoints.size() - 1) {
			const Vector3& entryPoint = entryEgressPoints[entryIndex];
			const Vector3& egressPoint = entryEgressPoints[egressIndex];

			double entryAngle = atan2((entryPoint - circleCenter).y / circleRadius,
					(entryPoint - circleCenter).x / circleRadius);
			double egressAngle = atan2((egressPoint - circleCenter).y / circleRadius,
					(egressPoint - circleCenter).x / circleRadius);

			double theta = entryAngle - egressAngle;
			if (theta < 0.0)
				theta += M_PI * 2;
			if (theta > M_PI * 2)
				theta -= M_PI * 2;

			totalArea += circleRadius * circleRadius * (theta / 2 - sin(theta / 2)
					* cos(theta / 2)); // Should work even for segments
			//longer than half a circle


			entryIndex += 2;
			egressIndex = entryIndex - 1;
		}
		return totalArea;
	} else {
		// Circle is either completely outside or completely inside polygon.
		// Checking one vertex is sufficient.
		if (length(polygon[0] - circleCenter) <= circleRadius) {
			// Polygon is inside circle
			return getPolygonArea(polygon);
		} else {
			// Circle is inside polygon
			return M_PI * circleRadius * circleRadius;
		}
	}
}

double RelationEvaluator::RelationEvaluator::getPolygonArea(const std::vector<
		Vector3> &polygon) {
	double ret = 0.0;
	//  for (unsigned int i = 0; i < polygon.size(); i++) {
	//    unsigned int iplus = (i == polygon.size()-1) ? 0 : i + 1;
	//    ret += polygon[i].x*polygon[iplus].y - polygon[i].y*polygon[iplus].x;
	//  }
	if (polygon.size() < 3) {
		return 0.0;
	}

	Vector3 normal = cross(polygon[1] - polygon[0], polygon.back() - polygon[0]);
	normalise(normal);
	for (unsigned int i = 0; i < polygon.size(); i++) {
		unsigned int iplus = (i == polygon.size() - 1) ? 0 : i + 1;
		ret += dot(normal, cross(polygon[i], polygon[iplus]));
	}
	return 0.5 * ret;
}

double RelationEvaluator::RelationEvaluator::getPolygonAreaAndCentroid(
		const std::vector<Vector3> &polygon, Vector3 &centroid) {
	double ret = 0.0;
	//  for (unsigned int i = 0; i < polygon.size(); i++) {
	//    unsigned int iplus = (i == polygon.size()-1) ? 0 : i + 1;
	//    ret += polygon[i].x*polygon[iplus].y - polygon[i].y*polygon[iplus].x;
	//  }
	if (polygon.size() < 3) {
		return 0.0;
	}

	centroid = vector3(0, 0, 0);
	Vector3 normal = cross(polygon[1] - polygon[0], polygon.back() - polygon[0]);
	normalise(normal);
	// The origin, as projected onto the polygon plane (in world coordinates)
	Vector3 originInPolygon = normal * dot(normal, polygon[0]);

	for (unsigned int i = 0; i < polygon.size(); i++) {
		unsigned int iplus = (i == polygon.size() - 1) ? 0 : i + 1;
		// Effectively, the area of a triangle between the origin, and the line segment
		// in question; projected down into the polygon plane, and taken negative if
		// flattened rear side up
		double weight = 0.5 * dot(normal, cross(polygon[i], polygon[iplus]));
		ret += weight;
		centroid += weight * (polygon[i] + polygon[iplus] + originInPolygon);
	}
	centroid /= (3 * ret);
	return ret;
}

bool RelationEvaluator::isIntersecting(double wr, double dr, double hr,
		const Vector3 BVertices[]) {
	const int edges[] = { 0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 4, 5, 5, 1, 5, 6, 6, 2,
			6, 7, 7, 3, 7, 4 }; //pairs of ints, indexing into BVertices

	//Check for intersection: 
	//Check if any vertex of B is inside A
	for (int vertexNo = 0; vertexNo < 8; vertexNo++) {
		Vector3 vert = BVertices[vertexNo];
		if (vert.x > -wr && vert.x < wr && vert.y > -dr && vert.y < dr && vert.z
				> -hr && vert.z < hr) {
			return true;
		}
	}
	//Check each edge of B against each face of A
	for (int edgeNo = 0; edgeNo < 12; edgeNo++) {
		const Vector3 &point1 = BVertices[edges[edgeNo * 2]];
		const Vector3 &point2 = BVertices[edges[edgeNo * 2 + 1]];
		Vector3 edge = point2 - point1;
		//Check the faces of A
		if ((point1.x > wr) != (point2.x > wr)) {
			//Check right face of A
			double intersectionParam = (wr - point1.x) / (point2.x - point1.x);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.y < dr && intersectionPoint.y > -dr
					&& intersectionPoint.z < hr && intersectionPoint.z > -hr) {
				return true;
			}
		}
		if ((point1.x > -wr) != (point2.x > -wr)) {
			//Check left face of A
			double intersectionParam = (-wr - point1.x) / (point2.x - point1.x);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.y < dr && intersectionPoint.y > -dr
					&& intersectionPoint.z < hr && intersectionPoint.z > -hr) {
				return true;
			}
		}

		if ((point1.y > dr) != (point2.y > dr)) {
			//Check rear face of A
			double intersectionParam = (dr - point1.y) / (point2.y - point1.y);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.x < wr && intersectionPoint.x > -wr
					&& intersectionPoint.z < hr && intersectionPoint.z > -hr) {
				return true;
			}
		}
		if ((point1.y > -dr) != (point2.y > -dr)) {
			//Check front face of A
			double intersectionParam = (-dr - point1.y) / (point2.y - point1.y);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.x < wr && intersectionPoint.x > -wr
					&& intersectionPoint.z < hr && intersectionPoint.z > -hr) {
				return true;
			}
		}

		if ((point1.z > hr) != (point2.z > hr)) {
			//Check top face of A
			double intersectionParam = (hr - point1.z) / (point2.z - point1.z);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.x < wr && intersectionPoint.x > -wr
					&& intersectionPoint.y < dr && intersectionPoint.y > -dr) {
				return true;
			}
		}
		if ((point1.z > -hr) != (point2.z > -hr)) {
			//Check top face of A
			double intersectionParam = (-hr - point1.z) / (point2.z - point1.z);
			Vector3 intersectionPoint = intersectionParam * edge + point1;
			if (intersectionPoint.x < wr && intersectionPoint.x > -wr
					&& intersectionPoint.y < dr && intersectionPoint.y > -dr) {
				return true;
			}
		}
	}
	return false;
}

void RelationEvaluator::getCornerWitnesses(double wr, double dr, double hr,
		const Vector3 BVertices[], const vector<Vector3> &BEdges,
		vector<Witness> &cornerWitnesses) {
	const int edges[] = { 0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 4, 5, 5, 1, 5, 6, 6, 2,
			6, 7, 7, 3, 7, 4 }; //pairs of ints, indexing into BVertices
	const Vector3 faceNormals[] = { vector3(0, 0, 1), vector3(0, 1, 0), vector3(
			-1, 0, 0), vector3(0, -1, 0), vector3(1, 0, 0), vector3(0, 0, -1) };

	//For each face in A, find the corner(s) of B at which the normal of the face
	//points into B. 
	Witness newWitness;
	newWitness.typeOnA = WITNESS_FACE;
	newWitness.typeOnB = WITNESS_VERTEX;
	// Loop over vertices in B
	for (int i = 0; i < 8; i++) {
		newWitness.idOnB = i;
		Vector3 outgoingEdges[3];
		int edgeNo = 0;
		for (int j = 0; edgeNo < 3 && j < 12; j++) {
			if (edges[j * 2] == i) {
				outgoingEdges[edgeNo] = BEdges[j];
				edgeNo++;
			} else if (edges[j * 2 + 1] == i) {
				outgoingEdges[edgeNo] = -BEdges[j];
				edgeNo++;
			}
		}
		if (edgeNo != 3) {
			exit(1);
		}

		const double epsilon = 1e-6;

		const double x = BVertices[i].x;
		const double y = BVertices[i].y;
		const double z = BVertices[i].z;
		//    if (y <= dr && y >= -dr && z <= hr && z >= -hr) {
		if (outgoingEdges[0].x >= -epsilon && outgoingEdges[1].x >= -epsilon
				&& outgoingEdges[2].x >= -epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.x = wr;
			newWitness.distance = x - wr;
			newWitness.idOnA = 4;
			newWitness.normal = faceNormals[4];
			cornerWitnesses.push_back(newWitness);
		}
		if (outgoingEdges[0].x <= epsilon && outgoingEdges[1].x <= epsilon
				&& outgoingEdges[2].x <= epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.x = -wr;
			newWitness.distance = -x - wr;
			newWitness.idOnA = 2;
			newWitness.normal = faceNormals[2];
			cornerWitnesses.push_back(newWitness);
		}
		//    }


		//    if (x <= wr && x >= -wr && z <= hr && z >= -hr) {
		if (outgoingEdges[0].y >= -epsilon && outgoingEdges[1].y >= -epsilon
				&& outgoingEdges[2].y >= -epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.y = dr;
			newWitness.distance = y - dr;
			newWitness.idOnA = 1;
			newWitness.normal = faceNormals[1];
			cornerWitnesses.push_back(newWitness);
		}
		if (outgoingEdges[0].y <= epsilon && outgoingEdges[1].y <= epsilon
				&& outgoingEdges[2].y <= epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.y = -dr;
			newWitness.distance = -y - dr;
			newWitness.idOnA = 3;
			newWitness.normal = faceNormals[3];
			cornerWitnesses.push_back(newWitness);
		}
		//    }

		//    if (y <= dr && y >= -dr && x <= wr && x >= -wr) {
		if (outgoingEdges[0].z >= -epsilon && outgoingEdges[1].z >= -epsilon
				&& outgoingEdges[2].z >= -epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.z = hr;
			newWitness.distance = z - hr;
			newWitness.idOnA = 0;
			newWitness.normal = faceNormals[0];
			cornerWitnesses.push_back(newWitness);
		}
		if (outgoingEdges[0].z <= epsilon && outgoingEdges[1].z <= epsilon
				&& outgoingEdges[2].z <= epsilon) {
			newWitness.point2 = BVertices[i];
			newWitness.point1 = BVertices[i];
			newWitness.point1.z = -hr;
			newWitness.distance = -z - hr;
			newWitness.idOnA = 5;
			newWitness.normal = faceNormals[5];
			cornerWitnesses.push_back(newWitness);
		}
		//    }
	}
}

void RelationEvaluator::getEdgeWitnesses(double wr, double dr, double hr,
		const Vector3 BVertices[], const vector<Vector3> &BEdges,
		vector<Witness> &edgeWitnesses, bool intersecting) {
	const Vector3 zeroVec = vector3(0, 0, 0);
	const int edges[] = { 0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 4, 5, 5, 1, 5, 6, 6, 2,
			6, 7, 7, 3, 7, 4 }; //pairs of ints, indexing into BVertices

	const double axisDirections[] = { 0, -1, -1, 1, 0, -1, 0, 1, -1, -1, 0, -1,
			-1, -1, 0, 0, -1, 1, 1, -1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, -1, 1, 0, -1,
			0, 1 }; // Triples of doubles, telling you for each
	// vertex which direction the axis vectors are pointing into the box

	// Loop over edges in B. For a point p1 on edge E1 on A to be of interest as a witness point 
	// vis-a-vis point p2 on edge E2 on B,
	// the normal between p1 and p2 must lie in the solid manifold around p2.
	const Vector3 axis1 = BEdges[2];
	const Vector3 axis2 = BEdges[3];
	const Vector3 axis3 = BEdges[6];

	const double edgeManifoldComponentsX[] = { 0, 1, 0, -1, -1, 0, 1, 1, 1, 0,
			-1, -1 };
	const double edgeManifoldComponentsY[] = { -1, 0, 1, 0, -1, -1, -1, 0, 1, 1,
			1, 0 };
	const double edgeManifoldComponentsZ[] = { -1, -1, -1, -1, 0, 1, 0, 1, 0, 1,
			0, 1 };
	const Vector3 AVerts[] = { vector3(wr, dr, hr), vector3(-wr, dr, hr),
			vector3(-wr, -dr, hr), vector3(wr, -dr, hr), vector3(wr, dr, -hr),
			vector3(-wr, dr, -hr), vector3(-wr, -dr, -hr), vector3(wr, -dr, -hr) };
	const Vector3 AEdges[] = { vector3(-2 * wr, 0, 0), vector3(0, -2 * dr, 0),
			vector3(2 * wr, 0, 0), vector3(0, 2 * dr, 0), vector3(0, 0, -2 * hr),
			vector3(-2 * wr, 0, 0), vector3(0, 0, 2 * hr), vector3(0, -2 * dr, 0),
			vector3(0, 0, 2 * hr), vector3(2 * wr, 0, 0), vector3(0, 0, 2 * hr),
			vector3(0, 2 * dr, 0) };
	double epsilon = 0.00001;
	for (unsigned int i = 0; i < BEdges.size(); i++) {
		// Computing the normal between E1 and E2:
		// Vector3 crossVec = cross(BEdges[i], vector3(1, 0, 0));
		// normalise(crossVec);
		// Vector3 offset = BVertices[startIndices[i]] - vector3(-wr, dr, hr);
		// Vector3 normal = dot(offset, crossVec) * crossVec;
		// ...but with A axis-aligned we can get rid of some math
		Vector3 edgeDir = BEdges[i];
		double edgeLength = length(edgeDir);
		edgeDir = edgeDir / edgeLength;
		double axisDir1 = axisDirections[i * 3];
		double axisDir2 = axisDirections[i * 3 + 1];
		double axisDir3 = axisDirections[i * 3 + 2];

		//Cross product of B edge and A edge 
		const Vector3 crossVecs[] = { vector3(0, -edgeDir.z, edgeDir.y), vector3(
				edgeDir.z, 0, -edgeDir.x), vector3(0, edgeDir.z, -edgeDir.y), vector3(
				-edgeDir.z, 0, edgeDir.x),

		vector3(-edgeDir.y, edgeDir.x, 0), vector3(0, -edgeDir.z, edgeDir.y),
				vector3(edgeDir.y, -edgeDir.x, 0), vector3(edgeDir.z, 0, -edgeDir.x),
				vector3(edgeDir.y, -edgeDir.x, 0), vector3(0, edgeDir.z, -edgeDir.y),
				vector3(edgeDir.y, -edgeDir.x, 0), vector3(-edgeDir.z, 0, edgeDir.x) };

		// Loop over edges in A
		for (int j = 0; j < 12; j++) {
			if (!vequals(crossVecs[j], zeroVec, epsilon)) {
				Vector3 offset = BVertices[edges[i * 2]] - AVerts[edges[j * 2]];
				if (!vequals(offset, zeroVec, epsilon)) {

					//Vector3 crossVec = vector3(0, edgeDir.z, -edgeDir.y);

					//Normalised cross vector
					Vector3 normCrossVec = crossVecs[j];
					normalise(normCrossVec);

					//Distance from A to B, if this witness is true, taken with sign.
					//Positive if outside,negative if inside.
					Vector3 directionOutOfA; //Vector actually leading out of a, whether
					// there's intersection or not

					if (normCrossVec.x * edgeManifoldComponentsX[j] <= epsilon
							&& normCrossVec.y * edgeManifoldComponentsY[j] <= epsilon
							&& normCrossVec.z * edgeManifoldComponentsZ[j] <= epsilon) {
						// normCrossVec points out of A
						directionOutOfA = normCrossVec;
					} else if (normCrossVec.x * edgeManifoldComponentsX[j] > -epsilon
							&& normCrossVec.y * edgeManifoldComponentsY[j] > -epsilon
							&& normCrossVec.z * edgeManifoldComponentsZ[j] > -epsilon) {
						// normCrossVec points into A
						directionOutOfA = -normCrossVec;
					} else {
						// neither normCrossVec or -normCrossVec are in the right manifold.
						// This can be skipped
						continue;
					}

					// Check that the vector out of A also points into B
					if (axisDir1 * dot(directionOutOfA, axis1) >= -epsilon && axisDir2
							* dot(directionOutOfA, axis2) >= -epsilon && axisDir3 * dot(
							directionOutOfA, axis3) >= -epsilon) {

						double distance; //Distance A to B with sign
						distance = dot(offset, directionOutOfA);

						// Find intersection point in the plane perpendicular to 
						// directionOutOfA
						Vector3 projectedOffset = offset - directionOutOfA * distance;
						Vector3 normalVector = projectedOffset - edgeDir * dot(
								projectedOffset, edgeDir);
						double normalLength = length(normalVector);

						double intersectionParamThisEdge; // 0 - 1
						if (!equals(normalLength, 0.0, epsilon)) {
							intersectionParamThisEdge = normalLength * normalLength / dot(
									normalVector, AEdges[j]);
						} else {
							intersectionParamThisEdge = 0.0;
						}
						Witness newWitness;
						newWitness.point1 = AVerts[edges[j * 2]] + AEdges[j]
								* intersectionParamThisEdge;
						newWitness.point2 = newWitness.point1 + directionOutOfA * distance;
						newWitness.typeOnA = WITNESS_EDGE;
						newWitness.typeOnB = WITNESS_EDGE;
						newWitness.idOnA = j;
						newWitness.idOnB = i;
						newWitness.paramOnA = intersectionParamThisEdge;
						newWitness.paramOnB = dot(edgeDir, newWitness.point2
								- BVertices[edges[i * 2]]) / edgeLength;
						newWitness.normal = directionOutOfA;
						newWitness.distance = distance;
						edgeWitnesses.push_back(newWitness);
					}
				} else {
					// Two vertices essentially intersecting
					// Should be handled by face-vertex case
				}
			}
		}
	}
}

double RelationEvaluator::getDistanceToPolygon(const Vector3 &ref,
		const std::vector<Vector3> &polygon) {
	// Note: assumes the polygon is in the xy-plane, positive w.r.t. z
	double bestDistSq = FLT_MAX;
	double closestInside = -FLT_MAX;
	for (unsigned int i = 0; i < polygon.size(); i++) {
		unsigned int iplus = (i == polygon.size() - 1 ? 0 : i + 1);
		Vector3 edge = polygon[iplus] - polygon[i];
		double edgeLength = length(edge);
		Vector3 offset = ref - polygon[i];
		double distance = cross(offset, edge).z;
		distance /= edgeLength;
		if (distance > 0.0) {
			double distanceSq = distance * distance;
			if (dot(offset, edge) / edgeLength > edgeLength) {
				distanceSq = (ref.x - polygon[iplus].x) * (ref.x - polygon[iplus].x)
						+ (ref.y - polygon[iplus].y) * (ref.y - polygon[iplus].y) + (ref.z
						- polygon[iplus].z) * (ref.z - polygon[iplus].z);
			} else if (dot(offset, edge) < 0) {
				distanceSq = (ref.x - polygon[i].x) * (ref.x - polygon[i].x) + (ref.y
						- polygon[i].y) * (ref.y - polygon[i].y) + (ref.z - polygon[i].z)
						* (ref.z - polygon[i].z);
			}

			if (distanceSq < bestDistSq) {
				bestDistSq = distanceSq;
			}
		} else {
			if (distance > closestInside) {
				closestInside = distance;
			}
		}
	}

	if (bestDistSq != FLT_MAX) {
		return sqrt(bestDistSq);
	} else {
		return closestInside;
	}
}

void getRandomSampleSphere(vector<Matrix33> &orientations, int n) {
	//See randomizeOrientation for credit

	orientations.resize(n * n * n);

	float x0[] = { ((float) rand()) / RAND_MAX / n, ((float) rand()) / RAND_MAX
			/ n, ((float) rand()) / RAND_MAX / n };

	double delta = 1 / (double) n;
	double deltaPhi = 2 * M_PI * delta;

	double cosdelta = cos(deltaPhi);
	double sindelta = sin(deltaPhi);

	double offsetTheta = 2 * M_PI * x0[0];
	double offsetPhi = 2 * M_PI * x0[1];
	double offsetZ = 2.0 * x0[2];

	double sintheta = sin(offsetTheta);
	double costheta = cos(offsetTheta);
	double sinphi0 = sin(offsetPhi);
	double cosphi0 = cos(offsetPhi);

	unsigned int i = 0;
	for (double theta0 = 0; theta0 < 2 * M_PI; theta0 += deltaPhi) {
		double sinphi = sinphi0;
		double cosphi = cosphi0;
		for (double phi0 = 0; phi0 < 2 * M_PI; phi0 += deltaPhi) {
			for (double z0 = 0; z0 < 2; z0 += 2 * delta) {
				double theta = theta0 + offsetTheta;
				double phi = phi0 + offsetPhi;
				double z = z0 + offsetZ;
				if (theta >= 2 * M_PI)
					theta -= 2 * M_PI;
				if (phi >= 2 * M_PI)
					phi -= 2 * M_PI;
				if (z >= 2)
					z -= 2;

				float r = sqrt(z);
				float Vx = sinphi * r;
				float Vy = cosphi * r;
				float Vz = sqrt(2.0 - z);

				/* Compute the row vector S = Transpose(V) * R, where R is a simple */
				/* rotation by theta about the z-axis.  No need to compute Sz since */
				/* it's just Vz.                                                    */

				float st = sintheta;
				float ct = costheta;
				float Sx = Vx * ct - Vy * st;
				float Sy = Vx * st + Vy * ct;

				/* Construct the rotation matrix  ( V Transpose(V) - I ) R, which   */
				/* is equivalent to V S - R.                                        */

				float rowMajor[] = { Vx * Sx - ct, Vx * Sy - st, Vx * Vz,

				Vy * Sx + st, Vy * Sy - ct, Vy * Vz,

				Vz * Sx, Vz * Sy, 1.0 - z }; /* This equals Vz * Vz - 1.0 */

				setRow33(orientations[i], rowMajor);
				i++;
			}

			double tempsin = sinphi * cosdelta + cosphi * sindelta;
			double tempcos = cosphi * cosdelta - sinphi * sindelta;
			sinphi = tempsin;
			cosphi = tempcos;
		}
		double tempsin = sintheta * cosdelta + costheta * sindelta;
		double tempcos = costheta * cosdelta - sintheta * sindelta;
		sintheta = tempsin;
		costheta = tempcos;
	}
}

void getRandomSampleCircle(vector<Matrix33> &orientations, int n) {
	orientations.resize(n);

	float offset = ((float) rand()) / RAND_MAX / n;

	for (int i = 0; i < n; i++) {
		fromRotZ(orientations[i], offset);
		offset += (2 * M_PI) / n;
		if (offset > 2 * M_PI)
			offset -= 2 * M_PI;
	}
}

void randomizeOrientation(Pose3 &pose) {
	/*======================================================================*
	 *  R A N D _ R O T A T I O N      Author: Jim Arvo, 1991               *
	 *                                                                      *
	 *  This routine maps three values (x[0], x[1], x[2]) in the range      *
	 *  [0,1] into a 3x3 rotation matrix M.  Uniformly distributed random   *
	 *  variables x0, x1, and x2 create uniformly distributed random        *
	 *  rotation matrices.  To create small uniformly distributed           *
	 *  "perturbations", supply samples in the following ranges             *
	 *                                                                      *
	 *      x[0] in [ 0, d ]                                                *
	 *      x[1] in [ 0, 1 ]                                                *
	 *      x[2] in [ 0, d ]                                                *
	 *                                                                      *
	 * where 0 < d < 1 controls the size of the perturbation.  Any of the   *
	 * random variables may be stratified (or "jittered") for a slightly    *
	 * more even distribution.                                              *
	 *                                                                      *
	 *======================================================================*/
	// Adapted by Kristoffer Sjöö
	float x[] = { ((float) rand()) / RAND_MAX, ((float) rand()) / RAND_MAX,
			((float) rand()) / RAND_MAX };
	float theta = x[0] * M_PI * 2; /* Rotation about the pole (Z).      */
	float phi = x[1] * M_PI * 2; /* For direction of pole deflection. */
	float z = x[2] * 2.0; /* For magnitude of pole deflection. */

	/* Compute a vector V used for distributing points over the sphere  */
	/* via the reflection I - V Transpose(V).  This formulation of V    */
	/* will guarantee that if x[1] and x[2] are uniformly distributed,  */
	/* the reflected points will be uniform on the sphere.  Note that V */
	/* has length sqrt(2) to eliminate the 2 in the Householder matrix. */

	float r = sqrt(z);
	float Vx = sin(phi) * r;
	float Vy = cos(phi) * r;
	float Vz = sqrt(2.0 - z);

	/* Compute the row vector S = Transpose(V) * R, where R is a simple */
	/* rotation by theta about the z-axis.  No need to compute Sz since */
	/* it's just Vz.                                                    */

	float st = sin(theta);
	float ct = cos(theta);
	float Sx = Vx * ct - Vy * st;
	float Sy = Vx * st + Vy * ct;

	/* Construct the rotation matrix  ( V Transpose(V) - I ) R, which   */
	/* is equivalent to V S - R.                                        */

	float rowMajor[] = { Vx * Sx - ct, Vx * Sy - st, Vx * Vz,

	Vy * Sx + st, Vy * Sy - ct, Vy * Vz,

	Vz * Sx, Vz * Sy, 1.0 - z }; /* This equals Vz * Vz - 1.0 */

	setRow33(pose.rot, rowMajor);
}

void RelationEvaluator::mergeAnyOverlappingVertices(Polyhedron &polyhedron,
		double eps) {
	if (polyhedron.vertices.size() < 2)
		return;
	vector<Vector3> newVerts;

	map<int, int> oldVertexToNewVertexMap;
	for (unsigned int i = 0; i < polyhedron.vertices.size(); i++) {
		if (oldVertexToNewVertexMap.find(i) == oldVertexToNewVertexMap.end()) {
			// i is a keeper.
			newVerts.push_back(polyhedron.vertices[i]);
			// old index i -> new index last in newVerts
			oldVertexToNewVertexMap[i] = newVerts.size() - 1;
		}

		for (unsigned int j = i + 1; j < polyhedron.vertices.size(); j++) {
			if (vequals(polyhedron.vertices[i], polyhedron.vertices[j], eps)) {
				// j is merged with i, begin last in newVerts
				oldVertexToNewVertexMap[j] = newVerts.size() - 1;
			}
		}
	}

	for (unsigned int i = 0; i < polyhedron.faces.size();) {
		for (unsigned int j = 0; j < polyhedron.faces[i].size();) {
			polyhedron.faces[i][j].first
					= oldVertexToNewVertexMap[polyhedron.faces[i][j].first];
			polyhedron.faces[i][j].second
					= oldVertexToNewVertexMap[polyhedron.faces[i][j].second];
			if (polyhedron.faces[i][j].first == polyhedron.faces[i][j].second) {
				polyhedron.faces[i].erase(polyhedron.faces[i].begin() + j);
			} else {
				j++;
			}
		}
		if (polyhedron.faces[i].size() < 3) {
			polyhedron.faces.erase(polyhedron.faces.begin() + i);
		} else {
			i++;
		}
	}
	polyhedron.vertices = newVerts;
}

Vector3 RelationEvaluator::computeAttentionVectorSumForPatch(const vector<
		Vector3> patch, const Vector3 &focus, const Vector3 &trajector,
		double falloff) {
	double totalArea = 0;

	const int subdivisionFactor = 5; // TODO: adapt this to triangle size compared to
	// falloff

	Vector3 vectorSum = vector3(0, 0, 0);

	//Divide polygon into triangles
	int apexIndex = 0;
	for (int index2 = apexIndex + 1; index2 < patch.size() - 1; index2++) {
		Vector3 v1 = patch[index2] - patch[apexIndex];
		Vector3 v2 = patch[index2 + 1] - patch[apexIndex];

		Vector3 vectorSumThisTriangle = vector3(0, 0, 0);

		double triangleArea = 0.5 * length(cross(v1, v2));

		Vector3 vStep1 = v1 / subdivisionFactor;
		Vector3 vStep2 = v2 / subdivisionFactor;

		// Divide triangle into parallelogram tessellation
		for (int k = 0; k < subdivisionFactor - 1; k++) {
			Vector3 firstCenter = patch[apexIndex] + (k + 0.5) * vStep1;
			for (int l = 0; l < subdivisionFactor - 1 - k; l++) {
				Vector3 center = firstCenter + (l + 0.5) * vStep2;

				double distanceToFocus = length(center - focus);
				double factor = exp(-distanceToFocus / falloff);

				vectorSumThisTriangle += 2 * factor * (trajector - center);
			}
		}

		// Add the final fringe of triangles at the far edge
		Vector3 firstCenter = patch[apexIndex] + vStep1 * (subdivisionFactor - 1
				+ 1.0 / 3) + vStep2 * (1.0 / 3);
		Vector3 vStep3 = vStep2 - vStep1;
		for (int k = 0; k < subdivisionFactor; k++) {
			Vector3 center = firstCenter + k * vStep3;

			double distanceToFocus = length(center - focus);
			double factor = exp(-distanceToFocus / falloff);

			vectorSumThisTriangle += factor * (trajector - center);
		}

		vectorSumThisTriangle *= triangleArea / (subdivisionFactor
				* subdivisionFactor);
		vectorSum += vectorSumThisTriangle;
		totalArea += triangleArea;
	}

	vectorSum /= totalArea;
	return vectorSum;
}

Vector3 RelationEvaluator::computeAttentionVectorSumForSolid(const Object *obj,
		const Vector3 &focus, const Vector3 &trajector, double falloff) {

	const int subdivisionFactor = 5; // TODO: adapt this to triangle size compared to
	// falloff

	Vector3 vectorSum = vector3(0, 0, 0);

	if (obj->type == OBJECT_BOX) {
		// Loop over elements of box
		Vector3 v1 = vector3(1, 0, 0);
		Vector3 v2 = vector3(0, 1, 0);
		Vector3 v3 = vector3(0, 0, 1);
		v1 = obj->pose.rot * v1; //transform(obj->pose.rot, v1);
		v2 = obj->pose.rot * v2; //transform(obj->pose.rot, v2);
		v3 = obj->pose.rot * v3; //transform(obj->pose.rot, v3);
		Vector3 vStep1 = v1 / subdivisionFactor;
		Vector3 vStep2 = v2 / subdivisionFactor;
		Vector3 vStep3 = v3 / subdivisionFactor;
		Vector3 firstCenter = obj->pose.pos - 0.5 * v1 + vStep1 * 0.5 - 0.5 * v2
				+ vStep2 * 0.5 - 0.5 * v3 + vStep3 * 0.5;

		for (int i = 0; i < subdivisionFactor; i++) {
			for (int j = 0; j < subdivisionFactor; j++) {
				for (int k = 0; k < subdivisionFactor; k++) {
					Vector3 center = firstCenter + i * vStep1 + j * vStep2 + k * vStep3;

					double distanceToFocus = length(center - focus);
					double factor = exp(-distanceToFocus / falloff);

					vectorSum += factor * (trajector - center);
				}
			}
		}

		vectorSum /= (subdivisionFactor * subdivisionFactor * subdivisionFactor);
	} else {
		cerr << "Error! Attention vector sum not implemented for this object!\n";
		return vector3(0, 0, 0);
	}

	return vectorSum;
}

spatial::Object *
generateNewObjectModel(const std::string &label) {
	//  log("generateNewObjectModel %s", label.c_str());
	if (label == "bookcase" || label == "box") {
		HollowBoxObject *newBoxObject = new HollowBoxObject;
		newBoxObject->type = OBJECT_HOLLOW_BOX;
		newBoxObject->thickness = 0.051;
		newBoxObject->sideOpen = 5; //Positive z
		if (label == "bookcase") {
			newBoxObject->radius1 = 0.155;
			newBoxObject->radius2 = 0.40;
			newBoxObject->radius3 = 0.75;
		} else if (label == "box") {
			newBoxObject->radius1 = 0.27;
			newBoxObject->radius2 = 0.165;
			newBoxObject->radius3 = 0.135;
		} else {
			newBoxObject->radius1 = 0.1;
			newBoxObject->radius2 = 0.1;
			newBoxObject->radius3 = 0.1;
		}
		newBoxObject->pose.pos.x = -FLT_MAX;
		newBoxObject->pose.pos.y = -FLT_MAX;
		newBoxObject->pose.pos.z = -FLT_MAX;
		return newBoxObject;
	} else {
		BoxObject *newBoxObject = new BoxObject;
		newBoxObject->type = OBJECT_BOX;
		if (label == "cornflakes") {
			newBoxObject->radius1 = 0.097;
			newBoxObject->radius2 = 0.064;
			newBoxObject->radius3 = 0.146;
		} else if (label == "mug") {
			newBoxObject->radius1 = 0.1;
			newBoxObject->radius2 = 0.1;
			newBoxObject->radius3 = 0.1;
		} else if (label == "paperclip") {
			newBoxObject->radius1 = 0.05;
			newBoxObject->radius2 = 0.085;
			newBoxObject->radius3 = 0.05;
		} else if (label == "stapler") {
			newBoxObject->radius1 = 0.0305;
			newBoxObject->radius2 = 0.0185;
			newBoxObject->radius3 = 0.077;
		} else if (label == "marker") {
			newBoxObject->radius1 = 0.019;
			newBoxObject->radius2 = 0.07;
			newBoxObject->radius3 = 0.048;
		} else if (label == "book") {
			newBoxObject->radius1 = 0.55;
			newBoxObject->radius2 = 0.45;
			newBoxObject->radius3 = 0.275;
		} else if (label == "table") {
			newBoxObject->radius1 = 1.70;
			newBoxObject->radius2 = 0.45;
			newBoxObject->radius3 = 0.275;
		} else if (label == "counter") {
			newBoxObject->radius1 = 1.70;
			newBoxObject->radius2 = 0.45;
			newBoxObject->radius3 = 0.275;
		} else {
			newBoxObject->radius1 = 0.1;
			newBoxObject->radius2 = 0.1;
			newBoxObject->radius3 = 0.1;
		}
		newBoxObject->pose.pos.x = -FLT_MAX;
		newBoxObject->pose.pos.y = -FLT_MAX;
		newBoxObject->pose.pos.z = -FLT_MAX;

		return newBoxObject;
	}
}

}
;
