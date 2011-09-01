/**
* @file main_tests4tomgine.cpp
* @author Thomas Mörwald
* @date May 2011
* @version 0.1
* @brief Test environment for TomGine
*/

#include <stdio.h>

#include <map>

#include <v4r/TomGine/tgTomGine.h>

#include "DataLoading.h"
#include "PatchFitting.h"
#include "NurbsTools.h"

using namespace TomGine;
using namespace std;

void printIntro(){
	printf("\n TomGine Testing Environment\n\n");
    printf(" TomGine control\n");
    printf(" -------------------------------------------\n");
    printf(" [Left Mouse Button] Rotate\n");
    printf(" [Right Mouse Button] Move\n");
    printf(" [Scroll Wheel] Zoom\n");
    printf(" [w] Switch to wireframe mode\n");
    printf(" [f] Switch to flat/smooth shading\n");
    printf(" \n\n");
}

int main(int argc, char *argv[])
{
    printIntro();
    srand(time(NULL));
    float fTime;

    // Load data
    std::string file_path = std::string("/home/tm/svn-repositories/workspace/data/DataGuteLaune/");
    std::string pc_file_yml;
	std::string mask_file;
	std::string camera_file;
	std::string pose_file;

	// Load segmented point cloud
    DataManager data_set;
    data_set.getDataSetFilenames( 0, file_path, pc_file_yml, mask_file, camera_file, pose_file);
    data_set.loadDataKinect( pc_file_yml, mask_file, camera_file, pose_file, false);

    unsigned width = data_set[0]->depth.cols;
    unsigned height = data_set[0]->depth.rows;


    tgEngine render(width, height, 10.0f, 0.01f, "TomGine Testing Environment", false);
    tgTimer timer;
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // Camera settings
    ON_BoundingBox bb = data_set[0]->interior.BoundingBox();
    ON_3dPoint cor = (bb.Max() + bb.Min()) * 0.5;
    render.SetCenterOfRotation(cor.x, cor.y, cor.z);
    printf("%f %f %f, %f %f %f\n", bb.Min().x, bb.Min().y, bb.Min().z, bb.Max().x, bb.Max().y, bb.Max().z);

    cv::Mat_<float> camR = cv::Mat::eye(3,3,CV_32F);
    cv::Mat camT = cv::Mat::zeros(3,1,CV_32F);
    render.SetCamera(data_set[0]->camera, width, height, camR, camT);

    glDepthRange(0.01, 0.5);
	float dr[2];
	glGetFloatv(GL_DEPTH_RANGE, dr);
	printf("depth-range: %f %f\n", dr[0], dr[1]);


    render.Update();

//    float minY = 10.0;
//    float maxY = -10.0;
//    for(unsigned i=0; i<data_set[0]->boundary.PointCount(); i++){
//    	if(data_set[0]->boundary[i].y < minY)
//    		minY = data_set[0]->boundary[i].y;
//    	if(data_set[0]->boundary[i].y > maxY)
//    		maxY = data_set[0]->boundary[i].y;
//    }
//    printf("minY: %f maxY: %f\n", minY, maxY);


    // ***************************************
	// Patch Fitting
    // ***************************************
    int order = 2;
    PatchFitting patchFit(order, data_set[0]);

//    ON_NurbsSurface patch = NurbsTools::InitNurbsPatch(3);
//    NurbsTools::Refine(0, patch);
//    NurbsTools::Refine(1, patch);
//    ON_TextLog out;
//    patch.Dump(out);
//
    tgNurbsSurfacePatch nurbsData;
    nurbsData = NurbsTools::Convert(*patchFit.m_patch);
    nurbsData.resU = 1; nurbsData.resV = 1;
    tgNurbsSurface nurbs(nurbsData);


	std::string patchfit_vert = std::string(V4R_DIR) + "/apps/tests4tomgine/PatchFitting.vert";
	std::string patchfit_frag = std::string(V4R_DIR) + "/apps/tests4tomgine/PatchFitting.frag";
	tgShader shPatchFit(patchfit_vert.c_str(), patchfit_frag.c_str(), NULL);
	shPatchFit.bind();
	shPatchFit.setUniform("texNurb", 0);
	shPatchFit.unbind();

	std::string path_gauss = std::string(V4R_DIR) + "/v4r/TomGine/shader/ipGauss.frag";
	std::string path_sobel = std::string(V4R_DIR) + "/v4r/TomGine/shader/ipSobel.frag";
	std::string path_thinning = std::string(V4R_DIR) + "/v4r/TomGine/shader/ipThinning.frag";
	std::string path_spreading = std::string(V4R_DIR) + "/v4r/TomGine/shader/ipSpreading.frag";
	std::string path_smiley = std::string(V4R_DIR) + "/v4r/TomGine/example/Resources/smiley.jpg";

	tgFrameBufferObject fbo(width, height, GL_RGBA32F, GL_DEPTH_COMPONENT32);

	fbo.Bind();
	fbo.Clear();
	render.Activate3D();
	nurbs.DrawFaces();
	fbo.Unbind();

	fbo.SaveColor("color.jpg");
	fbo.SaveDepth("depth.jpg");

	{

//		std::vector<float> img;
//		unsigned w = fbo.texDepth.GetWidth();
//		unsigned h = fbo.texDepth.GetHeight();
//		img.assign(w*h, 0.0f);
//		tgCheckError("[main] A");
//		fbo.texDepth.Bind(0);
//		glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &img[0]);
//		tgCheckError("[main] B");
//
//		for(unsigned i=0; i<370; i++){
//			printf("%f ", img[i*h+330]);
//		}
//		printf("\n");
	}

    // Rendering loop
    while( render.ProcessEvents() ) {

    	float t = timer.GetApplicationTime();

    	nurbs.DrawFaces();
    	glPointSize(5.0); glColor3f(0.0,1.0,0.0);
    	nurbs.DrawCPs();

    	fbo.texDepth.Bind(0);
    	shPatchFit.bind();
    	glPointSize(1.0);
    	glDisable(GL_LIGHTING);
    	glBegin(GL_POINTS);
			for(int i=0; i<data_set[0]->interior.PointCount(); i++){
				glColor3f(1.0,0.0,0.0);
				glVertex3f(data_set[0]->interior[i].x, data_set[0]->interior[i].y, data_set[0]->interior[i].z);
			}
    	glEnd();
    	shPatchFit.unbind();


        render.DrawCoordinates(0.9, 2.0);

        // print FPS
        render.DrawFPS();
        render.Update();

        usleep(1000);   // not to overload GPU
    }

    return 0;
}
