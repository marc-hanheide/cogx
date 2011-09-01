/**
* @file main_bsplinesurface.cpp
* @author Thomas Mörwald
* @date May 2011
* @version 0.1
* @brief TomGine Demo: Drawing a B-Spline surface using GPU shader
*/

#include <stdio.h>
#include <v4r/TomGine/tgTomGine.h>

using namespace TomGine;
using namespace std;

void printIntro(){
	printf("\n B-Spline Surface Demo\n\n");
    printf(" TomGine control\n");
    printf(" -------------------------------------------\n");
    printf(" [Left Mouse Button] Rotate\n");
    printf(" [Right Mouse Button] Move\n");
    printf(" [Scroll Wheel] Zoom\n");
    printf(" [w] Switch to wireframe mode\n");
    printf(" [f] Switch to flat/smooth shading\n");
    printf(" [Escape] Quit demo\n");
    printf(" \n\n");
}

int main(int argc, char *argv[])
{
    printIntro();
    srand(time(NULL));

    // Initialize
	unsigned width = 800;
    unsigned height = 600;
    char charbuffer[128];
    float fTime;

	tgEngine render(width, height, 10.0f, 0.01f, "NURBS Surface", false);
    glClearColor(0.1, 0.1, 0.1, 1.0);
    tgTimer timer;

    // ***************************************
    // NURBS surface
    TomGine::tgNurbsSurfacePatch nurbs;

	nurbs.orderU = 2;
	nurbs.orderV = 2;

	nurbs.knotsU.push_back(0.0);
	nurbs.knotsU.push_back(0.0);
	nurbs.knotsU.push_back(0.0);
	nurbs.knotsU.push_back(0.5);
	nurbs.knotsU.push_back(1.0);
	nurbs.knotsU.push_back(1.0);
	nurbs.knotsU.push_back(1.0);
	nurbs.knotsV = nurbs.knotsU;

	nurbs.cps.push_back(vec4(-1.0,-1.0,1.0,1.0));	nurbs.cps.push_back(vec4(0.0,-1.0,0.0,1.0));	nurbs.cps.push_back(vec4(1.0,-1.0,0.0,1.0));	nurbs.cps.push_back(vec4(2.0,-1.0,0.0,1.0));
	nurbs.cps.push_back(vec4(-1.0, 0.0,1.0,1.0));	nurbs.cps.push_back(vec4(0.0, 0.0,2.0,1.0));	nurbs.cps.push_back(vec4(1.0, 0.0,0.0,1.0));	nurbs.cps.push_back(vec4(2.0, 0.0,0.0,1.0));
	nurbs.cps.push_back(vec4(-1.0, 1.0,1.0,1.0));	nurbs.cps.push_back(vec4(0.0, 1.0,0.0,1.0));	nurbs.cps.push_back(vec4(1.0, 1.0,-2.0,1.0));	nurbs.cps.push_back(vec4(2.0, 1.0,0.0,1.0));
	nurbs.cps.push_back(vec4(-1.0, 2.0,1.0,1.0));	nurbs.cps.push_back(vec4(0.0, 2.0,0.0,1.0));	nurbs.cps.push_back(vec4(1.0, 2.0,0.0,1.0));	nurbs.cps.push_back(vec4(2.0, 2.0,0.0,1.0));
	nurbs.ncpsU = 4;
	nurbs.ncpsV = 4;
	nurbs.resU = 32;
	nurbs.resV = 32;
	// ***************************************

	tgRenderModel sphere;
	tgShapeCreator::CreateSphere(sphere, 0.5, 2, tgShapeCreator::ICOSAHEDRON);

	tgNurbsSurface surf(nurbs);

    // Rendering loop
    while( render.ProcessEvents()) {
    	fTime = timer.Update();
    	float t = timer.GetApplicationTime();

        // animate surface by moving control points
        vec4 v1 = surf.GetCP(1,1);
    	vec4 v2 = surf.GetCP(2,2);
    	v1.z = sinf(t);
    	v2.z = -sinf(t);
    	surf.SetCP(1,1,v1);
    	surf.SetCP(2,2,v2);

    	// draw NURBS surface using shader
    	surf.DrawFaces();

    	// draw control points
    	glPointSize(5.0);
    	glColor3f(0.0,1.0,0.0);
    	surf.DrawCPs();

        render.DrawCoordinates(1.0, 2.0);

        // print FPS
        render.DrawFPS();

		render.Update(fTime);
        usleep(5000);   // not to overload GPU

    }


    return 0;
}
