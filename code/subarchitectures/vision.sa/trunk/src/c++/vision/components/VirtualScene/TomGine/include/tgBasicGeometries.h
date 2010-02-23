 /**
 * @file tgBasicGeometries.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Geomitry factory for generating basic shapes like cubes, cylinders, etc.
 */

#include "tgModel.h"

namespace TomGine{

void GenCube( tgModel &model,
							float x, float y, float z)
{
	float x2 = x * 0.5;
	float y2 = y * 0.5;
	float z2 = z * 0.5;
	
	tgModel::Vertex v;
	tgModel::Face f;
	
	printf("[BasicGeometries.h - GenCube()] Generating cube not implemented yet\n");

}

void GenCylinder(	tgModel &model,
									float radius, float height,
									int slices, int stacks, bool closed=true)
{
	int x,z, i=0;
	float alpha = 2*PI / slices;
	float deltaH = height / stacks;
	tgModel::Vertex v[4];
	tgModel::Face f;
	
	for(z=0; z<stacks; z++){
		for(x=0; x<slices; x++){
			f.vertices.clear();
			
			v[0].pos.x = radius * cos(alpha*x);
			v[0].pos.y = radius * sin(alpha*x);
			v[0].pos.z = deltaH*z - height*0.5;
			v[0].normal.x = v[0].pos.x;
			v[0].normal.y = v[0].pos.y;
			v[0].normal.z = 0.0;
			v[0].normal = normalize(v[0].normal);
			f.vertices.push_back(i++);
			model.m_vertices.push_back(v[0]);
			
			v[1].pos.x = radius * cos(alpha*(x+1));
			v[1].pos.y = radius * sin(alpha*(x+1));
			v[1].pos.z = deltaH*z - height*0.5;
			v[1].normal.x = v[1].pos.x;
			v[1].normal.y = v[1].pos.y;
			v[1].normal.z = 0.0;
			v[1].normal = normalize(v[1].normal);
			f.vertices.push_back(i++);
			model.m_vertices.push_back(v[1]);
			
			v[2].pos.x = radius * cos(alpha*(x+1));
			v[2].pos.y = radius * sin(alpha*(x+1));
			v[2].pos.z = deltaH*(z+1) - height*0.5;
			v[2].normal.x = v[2].pos.x;
			v[2].normal.y = v[2].pos.y;
			v[2].normal.z = 0.0;
			v[2].normal = normalize(v[2].normal);
			f.vertices.push_back(i++);
			model.m_vertices.push_back(v[2]);
			
			v[3].pos.x = radius * cos(alpha*x);
			v[3].pos.y = radius * sin(alpha*x);
			v[3].pos.z = deltaH*(z+1) - height*0.5;
			v[3].normal.x = v[3].pos.x;
			v[3].normal.y = v[3].pos.y;
			v[3].normal.z = 0.0;
			v[3].normal = normalize(v[3].normal);
			f.vertices.push_back(i++);
			model.m_vertices.push_back(v[3]);
			model.m_faces.push_back(f);
		}
	}
	
	if(closed){
		for(z=0; z<2; z++){
			deltaH = height*z - height*0.5;
			
			for(x=0; x<slices; x++){
				f.vertices.clear();
				
				v[0].pos = vec3(0.0,0.0,deltaH);
				v[0].normal = normalize(v[0].pos);
				f.vertices.push_back(i++);
				model.m_vertices.push_back(v[0]);
				
				v[1].pos.x = radius * cos(alpha*x);
				v[1].pos.y = radius * sin(alpha*x);
				v[1].pos.z = deltaH;
				v[1].normal = v[0].normal;
				f.vertices.push_back(i++);
				model.m_vertices.push_back(v[1]);
				
				v[2].pos.x = radius * cos(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.y = radius * sin(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.z = deltaH;
				v[2].normal = v[0].normal;
				f.vertices.push_back(i++);
				model.m_vertices.push_back(v[2]);
				model.m_faces.push_back(f);
			}
		}
	}
}

} // namespace TomGine
