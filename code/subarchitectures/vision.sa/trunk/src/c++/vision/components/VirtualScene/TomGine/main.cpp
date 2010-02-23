 /**
 * @file main.cpp
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file for standalone version of TomGine rendering engine.
 */
 
#include <stdio.h>
#include <opencv/highgui.h>


#include "tgEngine.h"
#include "tgFont.h"
#include "tgImageProcessor.h"
#include "tgRenderModel.h"
#include "tgBasicGeometries.h"

using namespace TomGine;
using namespace std;

typedef vector<vec3> PointList;

int main(int argc, char *argv[])
{
	float fTime;
	
	int i;
	int w=640, h=480;
	PointList m_points;
	vec3 v;
	
// 	CvCapture*	m_capture = cvCreateCameraCapture(CV_CAP_ANY);
// 	IplImage*		m_image = cvQueryFrame(m_capture);
	
	tgEngine render;
	render.Init("TomGine Render Engine");
	
// 	tgImageProcessor m_ip;
// 	m_ip.init(w,h);
// 	tgTexture m_tex;
// 	m_tex.Load((unsigned char*)m_image->imageData, m_image->width, m_image->height);

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
		
	tgRenderModel cylinder;
	GenCylinder(cylinder, 0.05, 0.2, 32, 1);
	cylinder.m_material = matRed;
	
	tgFont myFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf");
	
	
	// Rendering loop
	while(render.Update(fTime)){
		
		cylinder.DrawFaces();
		
		render.Activate2D();
		myFont.print("TomGine Render Engine", 16, 5, 7);
		
	}
	
// 	cvReleaseCapture(&m_capture);
	
	return 0;
}



