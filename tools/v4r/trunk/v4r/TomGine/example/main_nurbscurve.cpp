/**
* @file main_bsplinecurve.cpp
* @author Thomas Mörwald
* @date May 2011
* @version 0.1
* @brief TomGine Demo drawing a B-Spline surface
*/

#include <stdio.h>

#include <v4r/TomGine/tgTomGine.h>

using namespace TomGine;
using namespace std;

#ifndef CALLBACK
#define CALLBACK
#endif

void printIntro(){
	printf("\n NURBS Demo\n\n");
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

// [1] Isogeometric Analysis, Toward Integration of CAD and FEA; J.Austin Cottrell, Thomas J.R. Hughes, Yuri Bazilevs

    int order = 2;
	std::vector<float> knots;
	std::vector<vec4> cps;
	int nknots;
	int ncps;
	float N[25];


int getSpan(float xi)
{
	int knotSpan = -1;

	// find knot span
	for(int s=0; s<nknots; s++){
		if( xi>=knots[s] && xi<knots[s+1] ){
			knotSpan = s;
			break;
		}else if(xi==knots[nknots-1]){
			knotSpan = nknots-order-1;
		}
	}

	return knotSpan;
}



void cox(float xi, int knotSpan)
{
	int nsupKnot = 2*order+1;
	float supKnot[9];

	// get supporting knots
	for(int s=0; s<nsupKnot; s++){
		supKnot[s] = knots[knotSpan-order+s];
	}

	for(int p=0; p<(order+1); p++){	// loop from lower order to higher -> unwrapped recursion

		for(int s=0; s<(order+1); s++){	// evaluate the basis N for each knotspan s

			if(p==0){

				// Equation (2.1) in [1]
				if(xi>=supKnot[s] && xi<supKnot[s+1])
					N[s] = 1.0;
				else if(s==order && xi==supKnot[s+1])
					N[s] = 1.0;
				else
					N[s] = 0.0;

			}else{
				// Equation (2.2) in [1]
				float A=0.0;
				float B=0.0;
				if( (xi-supKnot[s])!=0.0 && (supKnot[s+p]-supKnot[s])!=0.0 )
					A = (xi-supKnot[s]) / (supKnot[s+p]-supKnot[s]);

				if( (supKnot[s+p+1]-xi)!=0.0 && (supKnot[s+p+1]-supKnot[s+1])!=0.0 && s<order ) // (s<order) because N(s+i,p-1) does not support
					B = (supKnot[s+p+1]-xi) / (supKnot[s+p+1]-supKnot[s+1]);

				N[s + p*(order+1)] = A * N[s + (p-1)*(order+1)] + B * N[(s+1) + (p-1)*(order+1)];

			}
		}
	}

	float Nd[5];
	int p=order;

	for(int s=0; s<(order+1); s++){	// evaluate the basis N for each knotspan s
		float A=0.0;
		float B=0.0;
		if( (supKnot[s+p]-supKnot[s])!=0.0 )
			A = p / (supKnot[s+p]-supKnot[s]);

		if( (supKnot[s+p+1]-supKnot[s+1])!=0.0 && s<order ) // (s<order) because N(s+i,p-1) does not support
			B = p / (supKnot[s+p+1]-supKnot[s+1]);

		Nd[s] = A * N[s + (p-1)*(order+1)] - B * N[(s+1) + (p-1)*(order+1)];
	}

	for(int s=0; s<(order+1); s++){
		N[s + (p-1)*(order+1)] = Nd[s];
	}



	for(unsigned i=0; i<9; i++){
		if(i%3==0)
			printf("\n");
		printf("%f ", N[i]);
	}
	printf("\n");
//	for(unsigned i=0; i<3; i++){
//		printf("%f ", Nd[i]);
//	}
//	printf("\n");

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

    // files
	std::string nurbs_vert = std::string(V4R_DIR) + "/v4r/TomGine/shader/nurbscurve.vert";
	std::string cox_head = std::string(V4R_DIR) + "/v4r/TomGine/shader/coxdeboor.c";
	std::string color_frag = std::string(V4R_DIR) + "/v4r/TomGine/shader/color.frag";

	// Setup TomGine
    tgEngine render(width, height, 10.0f, 0.01f, "NURBS Curves", false);
    glClearColor(0.2, 0.2, 0.2, 1.0);
    tgTimer timer;

    // *****************************************
    // NURBS
	knots.push_back(0.0);
	knots.push_back(0.0);
	knots.push_back(0.0);
	knots.push_back(0.5);
	knots.push_back(1.0);
	knots.push_back(1.0);
	knots.push_back(1.0);
	nknots = knots.size();

	cps.push_back(vec4(0.0,0.0,0.0,1.0));
	cps.push_back(vec4(0.33,0.0,-1.0,1.0));
	cps.push_back(vec4(0.66,0.0,1.0,1.0));
	cps.push_back(vec4(1.0,0.0,0.0,1.0));
	ncps = cps.size();
	// *****************************************

	// Use textures as dynamic arrays to store knots and control points
    tgTexture1D texKnotsU, texCPs;
    texKnotsU.Load(&knots[0], (int)knots.size(), GL_R32F, GL_RED, GL_FLOAT);
    texCPs.Load(&cps[0].x, (int)cps.size(), GL_RGBA32F, GL_RGBA, GL_FLOAT);

	// Set up shader
    tgShader shNurbsCurve(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());
    shNurbsCurve.bind();
    shNurbsCurve.setUniform("g_knots", 0);
    shNurbsCurve.setUniform("g_cps", 1);
    shNurbsCurve.setUniform("g_nknots", nknots);
    shNurbsCurve.setUniform("g_order", order);
    shNurbsCurve.unbind();

    // Generate curve to draw, calculate B-Spline using functions above
    std::vector<tgVertex> curve;
    int  nsteps = 100;
    for(int i=0; i<nsteps; i++){
    	float xi = float(i) * 1.0/(nsteps-1);
    	if(xi==1.0)
    		xi=0.999999;
    	int knotSpan = getSpan(xi);
    	cox(xi, knotSpan);
    	tgVertex result;
    	result.pos = vec3(0,0,0);
    	result.normal = vec3(0,0,0);
    	result.texCoord.x = xi;
    	for(int s=0; s<(order+1); s++){
    		result.pos = result.pos + cps[knotSpan-order+s] * N[s+order*(order+1)];
    		result.normal = result.normal + cps[knotSpan-order+s] * N[s+(order-1)*(order+1)];
    	}
    	printf("%f %f %f\n", result.normal.x, result.normal.y, result.normal.z);
    	curve.push_back(result);
    }

    // Rendering loop
    while( render.ProcessEvents()) {
    	fTime = timer.Update();
        render.Activate3D();

        // Draw NURBS using gpu-shader
//        shNurbsCurve.bind();
//			texKnotsU.Bind(0);
//			texCPs.Bind(1);
//			glBegin(GL_LINE_STRIP);
//				for(unsigned i=0; i<nsteps; i++){
//					glTexCoord2f(curve[i].texCoord.x, 0.0);
//					glVertex3f(curve[i].texCoord.x, 0.0, 0.0);	// Note that vertex pos is set by the shader using B-Spline algorithm (cox-de-boor)
//				}
//			glEnd();
//        shNurbsCurve.unbind();
//        glDisable(GL_TEXTURE_1D);

        // Draw NURBS using functions above
        glDisable(GL_LIGHTING);
        glColor3f(0.0,0.5,0.5);
        glBegin(GL_LINE_STRIP);
        	for(int i=0; i<nsteps; i++){
        		glVertex3f(curve[i].pos.x, curve[i].pos.y, curve[i].pos.z);
        	}
        glEnd();

        glColor3f(0.0,0.0,1.0);
        glBegin(GL_LINE_STRIP);
			for(int i=0; i<nsteps; i++){
        		glVertex3f(curve[i].normal.x, curve[i].normal.y, curve[i].normal.z);
        	}
        glEnd();
//        glBegin(GL_LINE_STRIP);
//			for(int i=0; i<nsteps; i++){
//        		glVertex3f(curve[i].pos.x, curve[i].pos.y, curve[i].pos.z);
//        		glVertex3f(	curve[i].pos.x+curve[i].normal.x,
//        					curve[i].pos.y+curve[i].normal.y,
//        					curve[i].pos.z+curve[i].normal.z);
//        	}
//        glEnd();

        // draw control points
        glDisable(GL_LIGHTING);
        glPointSize(5.0);
        glColor3f(0.0,1.0,0.0);
        glBegin(GL_POINTS);
        for(unsigned i=0; i<cps.size(); i++){
        	glVertex3f(cps[i].x, cps[i].y, cps[i].z);

        }
        glEnd();

        render.DrawCoordinates(1.0, 2.0);

        // print FPS
        sprintf(charbuffer, "%d", (int)(1.0/fTime));
		render.PrintText2D(std::string(charbuffer), vec2(10,10), 20);

		render.Update();
        usleep(5000);   // not to overload GPU

    }


    return 0;
}










//    // GLU NURBS
//    GLfloat knots[8] = {0.0,0.0,0.0,0.0, 1.0,1.0,1.0,1.0};
//    GLfloat ctrlpoints[4][4][3];
//    //initsurface
//    {
//    	int u,v;
//    	for(u=0; u<4; u++){
//    		for(v=0; v<4; v++){
//    			ctrlpoints[u][v][0] = 2.0*((GLfloat)u-1.5);
//    			ctrlpoints[u][v][1] = 2.0*((GLfloat)v-1.5);
//
//    			if((u==1 || u==2) & (v==1 || v==2))
//    				ctrlpoints[u][v][2] = 3.0;
//    			else
//    				ctrlpoints[u][v][2] = -3.0;
//
//    		}
//    	}
//
//    }
//    GLUnurbsObj* theNurb = gluNewNurbsRenderer();
//    gluNurbsProperty(theNurb, GLU_DISPLAY_MODE, GLU_FILL);
//    gluNurbsCallback(theNurb, GLU_NURBS_ERROR, nurbsError);

//        gluBeginSurface(theNurb);
//        gluNurbsSurface(theNurb,
//        		8, knots, 8, knots,
//        		4 * 3, 3, &ctrlpoints[0][0][0],
//        		4, 4, GL_MAP2_VERTEX_3);
//        gluEndSurface(theNurb);
//        tgCheckError("Nurbs: ");


