// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include <cvd/colourspace_convert.h>

using namespace CVD;
using namespace std;
using namespace GVars3;


System::System()
  : defaultvideosize(640, 480), mGLWindow(defaultvideosize, "PTAM")
{
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  
  mimFrameBW.resize(defaultvideosize);
  mimFrameRGB.resize(defaultvideosize);


  mpCamera = new ATANCamera("Camera");

  mpMap = new Map;
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(defaultvideosize, *mpCamera, *mpMap, *mpMapMaker);
  mpARDriver = new ARDriver(*mpCamera, defaultvideosize, mGLWindow);
  //mpMapViewer = new MapViewer(*mpMap, mGLWindow);
  
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  //GUI.ParseLine("DrawAR=0");
  //GUI.ParseLine("DrawMap=0");
  //GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  //GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");
  
  mbDone = false;
  bReset_Tracker = false;
  bReset_Component = false;

  para_A = 0.0;
  para_B = 0.0;
  para_C = 0.0;
  para_D = 0.0;


};

void System::Run(IplImage *iplImage)
{
  if (!mbDone)
    {
      
      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

  CVD::Image<CVD::Rgb<CVD::byte> > p(defaultvideosize);
  IplImage &temp_img = *iplImage;
  ConvertImageFromIpl(temp_img, p);
  mimFrameRGB = p;
  convert_image(p, mimFrameBW);
 

      static bool bFirstFrame = true;
      if(bFirstFrame)
	{
	  mpARDriver->Init();
	  bFirstFrame = false;
	}
      
      mGLWindow.SetupViewport();
      mGLWindow.SetupVideoOrtho();
      mGLWindow.SetupVideoRasterPosAndZoom();
      
      if(!mpMap->IsGood())
	mpARDriver->Reset();
      
      //static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
      //static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);
      
      bool bDrawMap = false;
      bool bDrawAR = false;
      
      mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap);
      para_A = mpTracker->para_a;
      para_B = mpTracker->para_b;
      para_C = mpTracker->para_c;
      para_D = mpTracker->para_d;
      if (mpTracker->bTReset)  bReset_Tracker = true;
      if (bReset_Component)  mpTracker->bTReset = false;

      WMcenter = mpTracker->v3center;
      WMsize = mpTracker->v3size;
      WMradius = mpTracker->vdradius;

/*      
      if(bDrawMap)
	mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
      else if(bDrawAR)
	mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());
*/
      //      mGLWindow.GetMousePoseUpdate();
      string sCaption;
/*      if(bDrawMap)
	sCaption = mpMapViewer->GetMessageForUser();
      else*/
	if(!bDrawMap)
	sCaption = mpTracker->GetMessageForUser();
      mGLWindow.DrawCaption(sCaption);
      mGLWindow.DrawMenus();
      mGLWindow.swap_buffers();
      mGLWindow.HandlePendingEvents();
    }
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}

void System::ConvertImageFromIpl(IplImage &fromIpl, Image<Rgb<byte> > &toImgRGB)
{
	if(fromIpl.depth == (int)IPL_DEPTH_8U || fromIpl.depth == (int)IPL_DEPTH_8S)
	{
	int x, y;
	for(y = 0; y < fromIpl.height; y++)
	    for(x = 0; x < fromIpl.width; x++)
	    {
		int i = y*fromIpl.widthStep + 3*x;
		unsigned char red = (int) fromIpl.imageData[i];
		unsigned char green = (int) fromIpl.imageData[i+1];
		unsigned char blue = (int) fromIpl.imageData[i+2];

		toImgRGB[y][x].red = red;
		toImgRGB[y][x].green = green;
		toImgRGB[y][x].blue = blue;
	    }
	}
}






