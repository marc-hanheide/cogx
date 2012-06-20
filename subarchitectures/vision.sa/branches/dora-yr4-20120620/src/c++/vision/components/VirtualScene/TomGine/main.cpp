 /**
 * @file main.cpp
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file for standalone version of TomGine rendering engine.
 */
 
#include <stdio.h>

#include "tgEngine.h"
#include "tgFont.h"
#include "tgLabel.h"
#include "tgRenderModel.h"
#include "tgModelLoader.h"
#include "tgShapeCreator.h"

using namespace TomGine;
using namespace std;

typedef vector<vec3> PointList;

int main(int argc, char *argv[])
{
	float fTime;
	
	int i;
	PointList m_points;
	vec3 v;

	tgEngine render;
	render.Init(640,480, 1.0, 0.01, "TomGine Render Engine", true);

	tgFont m_font("/usr/share/fonts/truetype/freefont/FreeSerifBold.ttf");

	// Load Model
	// for more materials visit: http://wiki.delphigl.com/index.php/Materialsammlung
	tgRenderModel::Material matSilver;
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
	
	tgRenderModel::Material matRed;
	matRed.ambient = vec4(0.3,0.3,0.3,1.0);
	matRed.diffuse = vec4(1.0,0.0,0.0,1.0);
	matRed.specular = vec4(0.5,0.5,0.5,1.0);
	matRed.shininess = 10.0;
	matRed.color = vec4(1.0,0.0,0.0,1.0);
	
	tgRenderModel::Material matBlueBlend;
	matBlueBlend.ambient = vec4(0.3,0.3,0.3,0.5);
	matBlueBlend.diffuse = vec4(0.0,0.0,1.0,0.5);
	matBlueBlend.specular = vec4(0.5,0.5,0.5,0.5);
	matBlueBlend.shininess = 10.0;
	matBlueBlend.color = vec4(0.0,0.0,1.0,0.5);
		
	tgRenderModel camera;
	tgModelLoader loader;
	loader.LoadPly(camera, "resources/camera.ply");
	camera.m_material = matRed;
	
	tgRenderModel shape;
	tgShapeCreator shape_creator;
	shape_creator.CreateSphere(shape, 0.05, 3, ICOSAHEDRON);
// 	shape_creator.CreateBox(shape, 0.1,0.1,0.1);
// 	shape_creator.CreateCylinder(shape, 0.1, 0.2, 16, 2, true);
	shape.m_material = matBlueBlend;
	
	tgLabel label("/usr/share/fonts/truetype/freefont/FreeSerifBold.ttf");
	label.AddText("Camera");
	label.AddText("Logitech");
	
	// Rendering loop
	while(render.Update(fTime)){
		
		camera.DrawFaces();
		
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		shape.DrawFaces();
		glDisable(GL_BLEND);
		
		label.Draw();
		
// 		render.Activate2D();
// 		m_font.Print("TomGine Render Engine", 18, 5, 5);
	}
	
	return 0;
}



