/**
* @file main_nurbsvolume.cpp
* @author Thomas Mörwald
* @date June 2011
* @version 0.1
* @brief TomGine NURBS Demo - Dancing Pipe
*/

#include <stdio.h>

#include <map>

#include <v4r/TomGine/tgTomGine.h>
#include <v4r/TomGine/tgNurbsVolume.h>

using namespace TomGine;
using namespace std;

void printIntro(){
	printf("\n TomGine NURBS Demo - Dancing Pipe\n\n");
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

class SmoothFilter	// first order smooth filter to make FPS rate readable
{
public:
    SmoothFilter(){ a = 0.0f; b = 1.f - a; z = 0; };
    SmoothFilter(float a){ this->a = a; b = 1.f - a; z = 0; };
    inline float Process(float in) { z = (in * b) + (z * a); return z; }
	inline float Set(const float &z) { this->z = z; }
	inline float Set(const float &z, float delay) { this->z = z; a = delay; b = 1.f - delay; }
	inline void SetDelay(float d) { a = d; b = 1.f- d; }
private:
    float a, b, z;
};

void addRing(std::vector<vec4> &cps, float x, float y, float z){
	float w = 1.0 / sqrt(2.0);

	cps.push_back(vec4( x,   0.0, z, 1.0));
	cps.push_back(vec4( x,   y,   z, w));
	cps.push_back(vec4( 0.0, y,   z, 1.0));
	cps.push_back(vec4(-x,   y,   z, w));
	cps.push_back(vec4(-x,   0.0, z, 1.0));
	cps.push_back(vec4(-x,  -y,   z, w));
	cps.push_back(vec4( 0.0,-y,   z, 1.0));
	cps.push_back(vec4( x,  -y,   z, w));
	cps.push_back(vec4( x,   0.0, z, 1.0));
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

    tgEngine render(width, height, 10.0f, 0.01f, "NURBS Volume - dancing pipe", false);
    glClearColor(0.2, 0.2, 0.2, 1.0);
    tgTimer timer;
    SmoothFilter filter(0.995f);

    // ***************************************
	// Nurbs Volume (pipe)
    tgNurbsVolumePatch nurbsData;
    nurbsData.orderU = 2;
    nurbsData.orderV = 1;
    nurbsData.orderW = 2;
    nurbsData.knotsU.push_back(0.0);
	nurbsData.knotsU.push_back(0.0);
	nurbsData.knotsU.push_back(0.0);
	nurbsData.knotsU.push_back(0.25);
	nurbsData.knotsU.push_back(0.25);
	nurbsData.knotsU.push_back(0.5);
	nurbsData.knotsU.push_back(0.5);
	nurbsData.knotsU.push_back(0.75);
	nurbsData.knotsU.push_back(0.75);
	nurbsData.knotsU.push_back(1.0);
	nurbsData.knotsU.push_back(1.0);
	nurbsData.knotsU.push_back(1.0);

	nurbsData.knotsV.push_back(0.0);
	nurbsData.knotsV.push_back(0.0);
	nurbsData.knotsV.push_back(1.0);
	nurbsData.knotsV.push_back(1.0);

	nurbsData.knotsW.push_back(0.0);
	nurbsData.knotsW.push_back(0.0);
	nurbsData.knotsW.push_back(0.0);
	nurbsData.knotsW.push_back(1.0);
	nurbsData.knotsW.push_back(1.0);
	nurbsData.knotsW.push_back(1.0);

    float r1 = 0.25;
    float r2 = 0.1;
    float x,y,z;

    x = r1;		y = r1;		z = 0.0;
    addRing(nurbsData.cps, x, y, z);
	x = r2;		y = r2;		z = 0.0;
	addRing(nurbsData.cps, x, y, z);

	x = r1;		y = r1;		z = 0.5;
	addRing(nurbsData.cps, x, y, z);
	x = r2;		y = r2;		z = 0.5;
	addRing(nurbsData.cps, x, y, z);

	x = r1;		y = r1;		z = 1.0;
	addRing(nurbsData.cps, x, y, z);
	x = r2;		y = r2;		z = 1.0;
	addRing(nurbsData.cps, x, y, z);

	nurbsData.ncpsU = 9; nurbsData.ncpsV = 2, nurbsData.ncpsW = 3;

	nurbsData.resU = 32; nurbsData.resV = 1; nurbsData.resW = 16;
	// ***************************************

	tgNurbsVolume nurbs_volume(	nurbsData );

    // Rendering loop
	fTime=timer.Update();
    while( render.ProcessEvents() ) {


    	float t = timer.GetApplicationTime();

    	for(unsigned i=18; i<36; i++){
    		vec4 cp = nurbs_volume.GetCP(i);
    		cp.x = nurbsData.cps[i].x + 0.5 * sinf(3.0*t);
    		cp.y = nurbsData.cps[i].y + 0.1 * cosf(3.0*t);
    		nurbs_volume.SetCP(i, cp);
    	}

    	for(unsigned i=36; i<54; i++){
    		vec4 cp = nurbs_volume.GetCP(i);
    		cp.z = nurbsData.cps[i].z + 0.2 * nurbsData.cps[i].x * sinf(3.0*t) + 0.2 * nurbsData.cps[i].y * cosf(3.0*t);
    		nurbs_volume.SetCP(i, cp);
    	}

    	nurbs_volume.DrawFaces();
    	glPointSize(5.0);
    	glColor3f(0.0,1.0,0.0);
    	nurbs_volume.DrawCPs();

        render.DrawCoordinates(0.9, 2.0);

        // print FPS
        sprintf(charbuffer, "%d", (int)filter.Process(1.0/fTime));
		render.PrintText2D(std::string(charbuffer), vec2(10,10), 20);
        usleep(1000);   // not to overload GPU

        render.Update(fTime=timer.Update());
    }

    return 0;
}
