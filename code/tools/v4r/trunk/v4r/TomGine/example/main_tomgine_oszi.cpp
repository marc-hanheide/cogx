/**
* @file main_tomgine_oszi.cpp
* @author Thomas Mörwald
* @date October 2009
* @version 0.1
* @brief TomGine demo for data visualisation with osziloscope style.
*/

#include <stdio.h>

#include <v4r/TomGine/tgPlot2D.h>
#include <v4r/TomGine/tgEngine.h>
#include <v4r/TomGine/tgTimer.h>
#include <v4r/TomGine/tgShapeCreator.h>

#include <time.h>

using namespace TomGine;
using namespace std;

int main(int argc, char *argv[])
{
    unsigned width = 800;
    unsigned height = 600;

    printf("\n Demo TomGine Oszi\n\n");
    printf(" [Escape] Quit demo\n");

    srand(time(NULL));
    float fTime;
    tgTimer timer;

    // create container for oszi-plots
    tgEngine render1(width, height);
    tgEngine render2(width, height);

    // cleate 2 plots
    tgPlot2D myplot1(50, 25, 700, 250);
    tgPlot2D myplot2(50, 325, 700, 250);

    // set axis
    myplot1.Axis(0.0, 1.0, -1.0, 1.0);
    myplot2.Axis(0.0, 10.0, -1.0, 1.0);

    myplot2.SetBufferSize(100);

    tgRenderModel shape;
    tgShapeCreator::CreateBox(shape, 0.3,0.3,0.3);

    // Rendering loop
    bool run = true;
    while(  run  ){

    	// *** render 2 ***
    	render2.m_window->Activate();
    	render2.Activate2D();
    	timer.Update();

    	// one plot can display up to 4 signals
    	float y1 = cos( timer.GetApplicationTime() );
    	float y2 = sin( timer.GetApplicationTime() );
    	float y3 = 0.1 * cos( 5.0 * timer.GetApplicationTime() + 0.5 );
    	float y4 = 0.3 * sin( 1.0 * timer.GetApplicationTime() + 0.5 ) - 0.2 * double(rand()) / RAND_MAX;

    	myplot1.DrawAxis();
    	myplot1.Push( y1, y2, y3, y4 );
    	myplot1.DrawLegend(750, 200, "y1", "y2", "y3", "y4");

    	// second plot
    	myplot2.DrawAxis();
    	myplot2.Push( y4 );
    	myplot1.DrawLegend(750, 450, "y4");

    	run = run && render2.ProcessEvents();
    	render2.Update();

    	// *** render 1 ***
    	render1.m_window->Activate();

    	shape.m_pose.t.x = y1;
     	shape.m_pose.t.y = y2;
     	shape.m_pose.Rotate(0,y3,0);
    	shape.DrawFaces();

    	render1.DrawCoordinates(1.0, 2.0);

    	run = run && render1.ProcessEvents();
    	render1.Update();

#ifdef LINUX
        usleep(5000);   // not to overload GPU
#endif

#ifdef WIN32
        Sleep(10);
#endif

    }

    return 0;
}



