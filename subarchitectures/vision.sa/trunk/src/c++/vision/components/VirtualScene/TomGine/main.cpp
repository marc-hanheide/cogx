 /**
 * @file main.cpp
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file for standalone version of TomGine rendering engine.
 */
 
#include <stdio.h>

#include "tgEngine.h"
#include "tgRenderModel.h"
#include "tgBasicGeometries.h"

using namespace TomGine;

int main(int argc, char *argv[])
{
	float fTime;

	tgEngine render;
	render.Init(640,480,1.0,"TomGine Render Engine",true);
	
	// Load Model
	// for more materials visit: http://wiki.delphigl.com/index.php/Materialsammlung
	tgRenderModel::Material matSilver;
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
		
	tgRenderModel cylinder;
	GenCylinder(cylinder, 0.05, 0.2, 32, 1);
	cylinder.m_material = matSilver;
	
	// Rendering loop
	while(render.Update(fTime)){
		cylinder.DrawFaces();
	}
	
	return 0;
}



