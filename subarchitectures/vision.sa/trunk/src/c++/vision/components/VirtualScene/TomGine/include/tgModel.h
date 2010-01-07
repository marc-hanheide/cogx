//HEADER:
//			Title:			class tgModel
//			File:				tgModel.h
//
//			Function:		Header file for geometrical representation
//
//			Author:			Thomas Mörwald
//			Date:				15.12.2009
// ----------------------------------------------------------------------------


#ifndef TG_MODEL
#define TG_MODEL

#include <stdio.h>
#include <vector>
#include "mathlib.h"
#include "tgPose.h"


using namespace std;


class tgModel{	
public:
	tgModel();
	
	struct Vertex{
		vec3 pos;
		vec3 normal;
		vec2 texCoord;
	};
	struct Face{
		vector<int> vertices;		// vertex-indexlist
		vec3 normal;
	};
	struct Material{
	vec4 ambient;
	vec4 diffuse;
	vec4 specular;
	float shininess;
	};
	
	vector<Vertex>			m_vertices;
	vector<Face>				m_faces;
	
	Material 						m_material;
	tgPose							m_pose;
	
	void DrawFaces();
	void DrawNormals(float normal_length);
	void ComputeNormals();
	void PrintInfo();
private:
	void ApplyMaterial(Material mat);

};

#endif
