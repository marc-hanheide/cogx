
#include <VisionData.hpp>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

using namespace VisionData;

void genBox(	VisionData::GeometryModelPtr model,
							float x, float y, float z)
{
	printf("[ObjectDetector::BasicShapeGeometries.hpp::genCube] Warning not implemented yet");
	
	
	
	
}

void genCylinder(	VisionData::GeometryModelPtr model,
									float radius, float height,
									int slices, int stacks,
									bool closed=true)
{
	int x,z, i=0;
	float alpha = 2*PI / slices;
	float deltaH = height / stacks;
	VisionData::Vertex v[4];
	VisionData::Face f;
	
	for(z=0; z<stacks; z++){
		for(x=0; x<slices; x++){
			f.vertices.clear();
			
			v[0].pos.x = radius * cos(alpha*x);
			v[0].pos.y = radius * sin(alpha*x);
			v[0].pos.z = deltaH*z - height*0.5;
			v[0].normal.x = v[0].pos.x;
			v[0].normal.y = v[0].pos.y;
			v[0].normal.z = 0.0;
			normalise(v[0].normal);
			f.vertices.push_back(i++);
			model->vertices.push_back(v[0]);
			
			v[1].pos.x = radius * cos(alpha*(x+1));
			v[1].pos.y = radius * sin(alpha*(x+1));
			v[1].pos.z = deltaH*z - height*0.5;
			v[1].normal.x = v[1].pos.x;
			v[1].normal.y = v[1].pos.y;
			v[1].normal.z = 0.0;
			normalise(v[1].normal);
			f.vertices.push_back(i++);
			model->vertices.push_back(v[1]);
			
			v[2].pos.x = radius * cos(alpha*(x+1));
			v[2].pos.y = radius * sin(alpha*(x+1));
			v[2].pos.z = deltaH*(z+1) - height*0.5;
			v[2].normal.x = v[2].pos.x;
			v[2].normal.y = v[2].pos.y;
			v[2].normal.z = 0.0;
			normalise(v[2].normal);
			f.vertices.push_back(i++);
			model->vertices.push_back(v[2]);
			
			v[3].pos.x = radius * cos(alpha*x);
			v[3].pos.y = radius * sin(alpha*x);
			v[3].pos.z = deltaH*(z+1) - height*0.5;
			v[3].normal.x = v[3].pos.x;
			v[3].normal.y = v[3].pos.y;
			v[3].normal.z = 0.0;
			normalise(v[3].normal);
			f.vertices.push_back(i++);
			model->vertices.push_back(v[3]);
			model->faces.push_back(f);
		}
	}
	
	if(closed){
		for(z=0; z<2; z++){
			deltaH = height*z - height*0.5;
			
			for(x=0; x<slices; x++){
				f.vertices.clear();
				
				v[0].pos.x = 0.0;
				v[0].pos.y = 0.0;
				v[0].pos.z = deltaH;
				v[0].normal = v[0].pos;
				normalise(v[0].normal);
				f.vertices.push_back(i++);
				model->vertices.push_back(v[0]);
				
				v[1].pos.x = radius * cos(alpha*x);
				v[1].pos.y = radius * sin(alpha*x);
				v[1].pos.z = deltaH;
				v[1].normal = v[0].normal;
				f.vertices.push_back(i++);
				model->vertices.push_back(v[1]);
				
				v[2].pos.x = radius * cos(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.y = radius * sin(alpha*(x+(float(z)-0.5)*2.0));
				v[2].pos.z = deltaH;
				v[2].normal = v[0].normal;
				f.vertices.push_back(i++);
				model->vertices.push_back(v[2]);
				model->faces.push_back(f);
			}
		}
	}
}
