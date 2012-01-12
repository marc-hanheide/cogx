// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "GLWindow2.h"
#include <opencv2/highgui/highgui.hpp>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;

class System
{
public:
  System();
  void Run(IplImage *iplImage);
/////////////parameters of dominant plane, Ax+By+Cz+D = 0////////////////
  double para_A;
  double para_B;
  double para_C;
  double para_D;
  bool bReset_Tracker;
  bool bReset_Component;

  std::vector< Vector<3> > WMcenter;
  std::vector< Vector<3> > WMsize;
  std::vector<double> WMradius;

  
private:
  CVD::ImageRef defaultvideosize;

  GLWindow2 mGLWindow;
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
  CVD::Image<CVD::byte> mimFrameBW;
  
  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker; 
  ATANCamera *mpCamera;
  ARDriver *mpARDriver;
  MapViewer *mpMapViewer;
  
  bool mbDone;

void ConvertImageFromIpl(IplImage &fromIpl, CVD::Image<CVD::Rgb<CVD::byte> > &toImgRGB);

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};



#endif
