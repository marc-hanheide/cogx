/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#include <VisionData.hpp>
#include "tgModel.h"

using namespace cast;
using namespace cogx;
using namespace Math;



// converts a VisionData::GeometryModel to a Scene Model
bool convertGeometryModel(VisionData::GeometryModelPtr geom, tgModel* model){
	unsigned int i;
	
	// Check if model structure is empty
	if(!geom){
		printf("[GeometryModel_Converter] no geometry found\n");
		return false;
	}

	// Parse through vertices and store content in Model
	tgModel::Vertex v;
	for(i=0; i<geom->vertices.size(); i++){
		v.pos.x = geom->vertices[i].pos.x;
		v.pos.y = geom->vertices[i].pos.y;
		v.pos.z = geom->vertices[i].pos.z;
		v.normal.x = geom->vertices[i].normal.x;
		v.normal.y = geom->vertices[i].normal.y;
		v.normal.z = geom->vertices[i].normal.z;
		v.texCoord.x = geom->vertices[i].texCoord.x;
		v.texCoord.y = geom->vertices[i].texCoord.y;
		model->m_vertices.push_back(v);
	}
	
	// Parse through faces and store content in Model
	tgModel::Face f;
	for(i=0; i<geom->faces.size(); i++){	
		f.vertices = geom->faces[i].vertices;
		model->m_faces.push_back(f);
// 		printf("Face: %i %i %i %i\n", f.vertices[0], f.vertices[1], f.vertices[2], f.vertices[3]);
	}	
	
	return true;
}


// SDL - Keyboard and Mouse input control
bool inputsControl(){
 
	SDL_Event event;
	while(SDL_PollEvent(&event)){
		switch(event.type){
		case SDL_KEYDOWN:
            switch(event.key.keysym.sym){
				case SDLK_ESCAPE:
					return false;
					break;
        default:
					break;
			}
			break;
		case SDL_QUIT:
			return false;
			break;
		default:
			break;
		}
	}
	return true;
}






