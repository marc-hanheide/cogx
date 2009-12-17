
#include <stdio.h>

#include "tgEngine.h"
#include "BasicGeometries.h"

int main(int argc, char *argv[])
{
	tgEngine render;
	render.Init(640,480,1.0);
	
	// Load Model
	// for more materials visit: http://wiki.delphigl.com/index.php/Materialsammlung
	tgModel::Material matSilver;
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
		
	tgModel cylinder = genCylinder(0.05, 0.2, 64, 1);
	cylinder.m_material = matSilver;
	
	// Rendering loop
	while(render.Update()){
		cylinder.drawFaces();				
	}
	
	return 0;
}




