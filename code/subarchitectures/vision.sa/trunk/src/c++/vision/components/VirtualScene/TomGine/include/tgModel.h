 /**
 * @file tgModel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining a model for rendering.
 */

#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>

#include "tgMathlib.h"

namespace TomGine{

/**
* @brief Class tgModel
*/
class tgModel{	
public:

	struct Vertex{
		vec3 pos;				///< 3D position of vertex
		vec3 normal;		///< normal vector of vertex
		vec2 texCoord;	///< texture coordinate of vertex
	};
	struct Face{
		std::vector<int> vertices;		///< list of vertex-indices
		vec3 normal;									///< normal vector of face
	};
	struct Line{
		vec3 start;
		vec3 end;
	};
	struct LineLoop{
		std::vector<vec3> points;
	};

	std::vector<Vertex>			m_vertices;		///< list of vertices
	std::vector<Face>				m_faces;			///< list of faces
	std::vector<Face>				m_polygons;		///< list of triangle fans (=polygons)
	std::vector<Face>				m_quadstrips;	///< list of quad strips (e.g. for cylinders)
	std::vector<Line>				m_lines;			///< list of lines
	std::vector<LineLoop>		m_lineloop;		///< list of line strip (=polygon contours)
	std::vector<vec3>				m_points;			///< list of points
	
	/** @brief draws triangles and quadrangles given by m_faces */
	virtual void DrawFaces();
	
	/** @brief draws triangulated polygons given by m_polygon where the vector of vertices defines the contour */
	virtual void DrawPolygons();
	
	virtual void DrawQuadstrips();
	
	/** @brief draws single lines given by start and end points in m_lines */
	virtual void DrawLines();
	
	/** @brief draws closed line loop given by the point sequence m_lineloop */
	virtual void DrawLineLoops();
	
	/** @brief draws single points given by m_points */
	virtual void DrawPoints();
	
	/** @brief draws normals of vertices in m_faces */
	virtual void DrawNormals(float normal_length);
	
	/** @brief computes normals of vertices of m_faces */
	void ComputeNormals();
	
	void PrintInfo();
};

} // namespace TomGine

#endif
