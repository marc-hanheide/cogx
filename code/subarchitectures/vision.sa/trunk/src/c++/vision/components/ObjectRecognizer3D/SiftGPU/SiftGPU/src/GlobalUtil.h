////////////////////////////////////////////////////////////////////////////
//	File:		GlobalUtil.h
//	Author:		Changchang Wu
//	Description : 
//		GlobalParam:	Global parameters
//		ClockTimer:		Timer 
//		GlobalUtil:		Global Function wrapper
//
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty. 
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////


#ifndef _GLOBAL_UTILITY_H
#define _GLOBAL_UTILITY_H

//
#ifdef _WIN32
	#define HIGH_RESOLUTION_TIME
	//use timeGetTime instead of clock for timing
#endif

#include <stdio.h>
#include <string.h>

//wrapper for some shader function
class ProgramGPU;

class GlobalParam
{
public:
	static GLuint	_texTarget;
	static GLuint   _iTexFormat;
	static int		_texMaxDim;
	static int		_texMaxDimGL; 
	static int		_MaxDrawBuffers;
	static int		_verbose;
	static int		_timingS;
	static int		_timingO;
	static int		_timingL;
	static int		_usePackedTex;
	static int		_BetaFilter;
	static int		_IsATI;
	static int		_UseGLSL;
	static int		_UseDynamicIndexing; 
	static int		_debug;
	static int		_MaxFilterWidth;
	static float	_FilterWidthFactor;
	static float    _OrientationWindowFactor;
	static float	_DescriptorWindowFactor; 
	static int		_ExtraDownSample;
	static int		_MaxOrientation;
	static int      _OrientationPack2;
	static int		_ListGenGPU;
	static int		_ListGenSkipGPU;
	static int		_SupportFP40; //previously named _SupportAdvanced
	static int		_SupportNVFloat;
	static int		_FullSupported;
	static int		_SupportGP4GP;
	static float	_MaxFeaturePercent;
	static int		_MaxLevelFeatureNum;
	static int		_DescriptorPPT; //pixel per texture for one descriptor
	static int		_DescriptorVPC; //values per color channel( 1 for float, 4 for unsigned buyte)
	static int		_FeatureTexBlock;
	static int		_NarrowFeatureTex; //implemented but, no performance improvement
	static int		_SubpixelLocalization;
	static int		_ProcessOBO; //not implemented yet (could save 30% of memory)
	static int		_PreciseBorder; //implemented
	static int		_UseSiftGPUEX;
	static int		_ForceTightPyramid;
	static int		_octave_min_default;
	static int		_octave_num_default;
	static int		_InitPyramidWidth;
	static int		_InitPyramidHeight;
	static int		_PreProcessOnCPU;
	static int		_HaveGlSupport;
	static int		_FixedOrientation;
	static int		_LoweOrigin;
	static int		_GradientLevelOffset;
	static int		_ExitAfterSIFT; 
	static int		_NormalizedSIFT;
	static int		_BinarySIFT;
	static int		_KeepExtremumSign;
	//for compatable with old version:
	static float	_OrientationExtraFactor;
	static float	_OrientationGaussianFactor;
};


class ClockTimer
{
private:
	char _current_event[256];
	int  _time_start;
	int  _time_stop;
public:
	ClockTimer(char * event){StartTimer(event);};
	void StopTimer(int verb = 1);
	void StartTimer(char * event, int verb=0);
	float  GetElapsedTime();
};

class GlobalUtil:public GlobalParam
{
    static ClockTimer _globalTimer;                             
public:
	static void CheckFragmentProgramError();
	static void StopTimer()				{	_globalTimer.StopTimer(_timingS);}
	static void StartTimer(char * event){	_globalTimer.StartTimer(event, _timingO);	}
	static float GetElapsedTime(){return _globalTimer.GetElapsedTime();}
	static void FitViewPort(int width, int height);
	static void SetTextureParameter();
	static void SetTextureParameterUS();
#ifdef _DEBUG
	static void CheckErrorsGL(const char* location = NULL);
#else
	static void inline CheckErrorsGL(const char* location = NULL){};
#endif
	static bool CheckFramebufferStatus();
	//initialize Opengl parameters
	static void InitGLParam();
	static int  CreateWindowGLUT();

};


#endif

