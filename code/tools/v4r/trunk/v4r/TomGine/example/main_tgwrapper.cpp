/**
* @file main_tomgine.cpp
* @author Thomas Mörwald
* @date October 2009
* @version 0.1
* @brief TomGine rendering demo
*/

#include <stdio.h>

#include <v4r/TomGine/tgTomGine.h>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>

using namespace std;
using namespace TomGine;

class myModel : public tgRenderModel
{
public:
	myModel(){
		// NO OpenGL commands here (object is constructed in main thread)
	}
	~myModel(){
		// NO OpenGL commands here (object is destroyed in main thread)
	}

	virtual void Draw(){
		// OpenGL commands are allowed here

//		m_material.Activate();

		tgRenderModel::DrawFaces();

		glColor3f(0.0f, 0.2f, 0.8f);
		tgModel::DrawNormals(0.1f);
	}
};

int main(int argc, char *argv[])
{
    // NO OpenGL commands in main thread!

	unsigned width = 800;
    unsigned height = 600;

    printf("\n Demo TomGineWrapper\n\n");

    printf(" TomGine control\n");
    printf(" -------------------------------------------\n");
    printf(" [Left Mouse Button] Rotate\n");
    printf(" [Right Mouse Button] Move\n");
    printf(" [Scroll Wheel] Zoom\n");
    printf(" [w] Switch to wireframe mode\n");
    printf(" [f] Switch to flat/smooth shading\n");
    printf("\n");
    printf(" TomGineThread control\n");
    printf(" -------------------------------------------\n");
    printf(" [c] Clear scene.\n");
    printf(" [i] Draw background image.\n");
    printf(" [p] Draw point-cloud.\n");
    printf(" [l] Draw labels.\n");
    printf(" [m] Draw models.\n");
    printf(" [n] Draw NURBS objects.\n");
    printf(" \n\n");

    std::string path_smiley = std::string(V4R_DIR) + "/v4r/TomGine/example/Resources/smiley.jpg";
    cv::Mat cv_img = cv::imread(path_smiley.c_str());

    tgTomGineThread tgThread(width, height);
    tgTimer timer;

    tgNurbsSurfacePatch nurbsData;
    nurbsData.knotsU.push_back(0.0);
    nurbsData.knotsU.push_back(0.0);
    nurbsData.knotsU.push_back(1.0);
    nurbsData.knotsU.push_back(1.0);

    nurbsData.knotsV.push_back(0.0);
    nurbsData.knotsV.push_back(0.0);
    nurbsData.knotsV.push_back(1.0);
    nurbsData.knotsV.push_back(1.0);

    nurbsData.cps.push_back(vec4(0.0, 0.0, 1.0, 1.0));
    nurbsData.cps.push_back(vec4(1.0, 0.0, 0.5, 1.0));
    nurbsData.cps.push_back(vec4(0.0, 1.0, 0.5, 1.0));
    nurbsData.cps.push_back(vec4(1.0, 1.0, 1.0, 1.0));

    nurbsData.ncpsU = 2;
    nurbsData.ncpsV = 2;

    nurbsData.orderU = 1;
    nurbsData.orderV = 1;

    int nurbs_id = tgThread.AddNurbsSurface(nurbsData);

    // Attention: OpenGL Calls (like tgModel::DrawFaces() wont work here, since there is no OpenGL context in this thread
    myModel model;
    model.m_pose.Translate(0.5, 0.0, 0.0);
    tgShapeCreator::CreateSphere(model, 0.5, 2, tgShapeCreator::ICOSAHEDRON);
    tgThread.AddModel(&model);

    tgModel pcl;
    tgColorPoint cpt;
    for(unsigned i=0; i<1000; i++){
		cpt.pos = vec3(	double(rand())/RAND_MAX*2.0-1.0,
						double(rand())/RAND_MAX*2.0-1.0,
						double(rand())/RAND_MAX*2.0-1);
		cpt.color[0] = char( double(rand())/RAND_MAX*255.0 );
		cpt.color[1] = char( double(rand())/RAND_MAX*255.0 );
		cpt.color[2] = char( double(rand())/RAND_MAX*255.0 );
		pcl.m_colorpoints.push_back(cpt);
    }
    tgThread.AddPointCloud(pcl);

    tgThread.SetImage(cv_img);

    while(!tgThread.Stopped()){

    	timer.Update();
    	float t = timer.GetApplicationTime();
    	float z = sinf(t);
    	nurbsData.cps[0] = vec4(0.0,0.0,z,1.0);
    	tgThread.SetNurbsSurface(nurbs_id, nurbsData);
    	tgThread.Update();

    	usleep(10000); // update nurbs every 10 ms
    }
    return 0;
}



