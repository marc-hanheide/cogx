////////////////////////////////////////////////////////////////////////////
//	File:		SiftGPU.h
//	Author:		Changchang Wu
//	Description :	interface for the SIFTGPU class.
//					SiftGPU:	The SiftGPU Tool.  
//					SiftGPUEX:	SiftGPU + viewer
//					SiftParam:	Sift Parameters
//					SiftMatchGPU: GPU SIFT Matcher;
//
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


#ifndef GPU_SIFT_H
#define GPU_SIFT_H

#if  defined(_WIN32) 
	#ifdef SIFTGPU_DLL
		#ifdef DLL_EXPORT
			#define SIFTGPU_EXPORT __declspec(dllexport)
		#else
			#define SIFTGPU_EXPORT __declspec(dllimport)
		#endif
	#else
		#define SIFTGPU_EXPORT
	#endif

	#if _MSC_VER > 1000
		#pragma once
	#endif
#else
	#define SIFTGPU_EXPORT
#endif

#if !defined(_MAX_PATH)
	#if defined (MAX_PATH)
		#define _MAX_PATH MAX_PATH
	#else
		#define _MAX_PATH 512
	#endif
#endif
#include <cstddef>
///////////////////////////////////////////////////////////////////
//clss SiftParam
//description: SIFT parameters
////////////////////////////////////////////////////////////////////
class GlobalUtil;
class SiftParam
{
public:
	float*		_sigma;
	float		_sigma_skip0; // 
	float		_sigma_skip1; //
	//sigma of the first level
	float		_sigma0;
	float		_sigman;
	int			_sigma_num;

	//how many dog_level in an octave
	int			_dog_level_num;
	int			_level_num;


	//starting level in an octave
	int			_level_min;
	int			_level_max;
	int			_level_ds;
	//dog threshold
	float		_dog_threshold;
	//edge elimination
	float		_edge_threshold;
	void		 ParseSiftParam();
public:
	float GetLevelSigma(int lev);
	float GetInitialSmoothSigma(int octave_min);
	SIFTGPU_EXPORT SiftParam();
};


class GLTexInput;
class ShaderMan;
class SiftPyramid;
class ImageList;
////////////////////////////////////////////////////////////////
//class SIftGPU
//description: Interface of SiftGPU lib
////////////////////////////////////////////////////////////////
class SiftGPU:public SiftParam
{
public:
	enum
	{
		SIFTGPU_NOT_SUPPORTED = 0,
		SIFTGPU_PARTIAL_SUPPORTED = 1, // detction works, but not orientation/descriptor
		SIFTGPU_FULL_SUPPORTED = 2
	};
	typedef struct SiftKeypoint
	{
		float x, y, s, o; //x, y, scale, orientation.
	}SiftKeypoint;
protected: 
	//when more than one images are specified
	//_current indicates the active one
	int		_current;
	//_initialized indicates if the shaders and OpenGL/SIFT parameters are initialized
	//they are initialized only once for one SiftGPU inistance
	//that is, SIFT parameters will not be changed
	int		_initialized;
	//_image_loaded indicates if the current images are loaded
	int		_image_loaded;
	//the name of current input image
	char	_imgpath[_MAX_PATH];
	//_outpath containes the name of the output file
	char	_outpath[_MAX_PATH];
	//the list of image filenames
	ImageList *    _list;
	//the texture that holds loaded input image
	GLTexInput *   _texImage;
	//the SiftPyramid
	SiftPyramid *  _pyramid;
	SiftPyramid ** _pyramids;
	int			   _nPyramid;

	//print out the command line options
	static void PrintUsage();
	//Initialize OpenGL and SIFT paremeters, and create the shaders accordingly
	void InitSiftGPU();
	//load the image list from a file
	void LoadImageList(char *imlist);
public:
	//timing results for 10 steps
	float			_timing[10];
public:
	//set the image list for processing
	SIFTGPU_EXPORT virtual void SetImageList(int nimage, const char** filelist);
	//get the number of SIFT features in current image
	SIFTGPU_EXPORT virtual int	GetFeatureNum();
	//save the SIFT result as a ANSCII/BINARY file 
	SIFTGPU_EXPORT virtual void SaveSIFT(const char * szFileName);
	//Copy the SIFT result to two vectors
	SIFTGPU_EXPORT virtual void GetFeatureVector(SiftKeypoint * keys, float * descriptors);
	//Set keypoint list before running sift to get descriptors
	SIFTGPU_EXPORT virtual void SetKeypointList(int num, const SiftKeypoint * keys, int keys_have_orientation = 1);
	//Enable downloading results to CPU. 
	//create a new OpenGL context for processing
	//call VerifyContextGL instead if you want to crate openGL context yourself, or your are 
	//mixing mixing siftgpu with other openGL code
	SIFTGPU_EXPORT virtual int CreateContextGL();
	//verify the current opengl context..
	//(for example, you call wglmakecurrent yourself and verify the current context)
	SIFTGPU_EXPORT virtual int VerifyContextGL();
	//check if all siftgpu functions are supported
	SIFTGPU_EXPORT virtual int IsFullSupported();
	//set verbose mode
	SIFTGPU_EXPORT virtual void SetVerbose(int verbose = 4);
	//set SiftGPU to brief display mode, which is faster
	inline void SetVerboseBrief(){SetVerbose(2);};
	//parse SiftGPU parameters
	SIFTGPU_EXPORT virtual void ParseParam(int argc, char **argv);
	//retrieve the size of current input image
	SIFTGPU_EXPORT virtual void GetImageDimension(int &w, int&h);
	//run SIFT on a new image given filename
	SIFTGPU_EXPORT virtual int  RunSIFT(char * imgpath);
	//run SIFT on an image in the image list given the file index
	SIFTGPU_EXPORT virtual int	RunSIFT(int index);
	//run SIFT on a new image given the pixel data and format/type;
	//gl_format (e.g. GL_LUMINANCE, GL_RGB) is the format of the pixel data
	//gl_type (e.g. GL_UNSIGNED_BYTE, GL_FLOAT) is the data type of the pixel data;
	//Check glTexImage2D(...format, type,...) for the accepted values
	//Using image data of GL_LUMINANCE + GL_UNSIGNED_BYTE can minimize transfer time
	SIFTGPU_EXPORT virtual int  RunSIFT(int width, int height,	const void * data, 
										unsigned int gl_format, unsigned int gl_type);
	//run SIFT on current image (specified by arguments), or processing the current image again
	SIFTGPU_EXPORT virtual int  RunSIFT();
	//run SIFT with keypoints on current image again. 
	SIFTGPU_EXPORT virtual int  RunSIFT(int num, const SiftKeypoint * keys, int keys_have_orientation = 1);
	//constructor, (np is the number of pyramids)
	SIFTGPU_EXPORT SiftGPU(int np = 1);
	//destructor
	SIFTGPU_EXPORT virtual ~SiftGPU();
	//set the active pyramid
	SIFTGPU_EXPORT virtual void  SetActivePyramid(int index);
	//retrieve the number of images in the image list
	SIFTGPU_EXPORT virtual int GetImageCount();
	//set parameter GlobalUtil::_ForceTightPyramid
	SIFTGPU_EXPORT virtual void SetTightPyramid(int tight = 1);
	//allocate pyramid for a given size of image
	SIFTGPU_EXPORT virtual int AllocatePyramid(int width, int height);
	//none of the texture in processing can be larger
	//automatic down-sample is used if necessary. 
	SIFTGPU_EXPORT virtual void SetMaxDimension(int sz);
	///
public:
	//overload the new operator because delete operator is virtual
	//and it is operating on the heap inside the dll (due to the 
	//compiler setting of /MT and /MTd). Without the overloaded operator
	//deleting a SiftGPU object will cause a heap corruption in the 
	//static link case (but not for the runtime dll loading).
	// SIFTGPU_EXPORT void* operator new (size_t size); 
};



////////////////////////////////////////////////////////////////
//class SIftGPUEX
//description: add viewing extension to Interface of SiftGPU
////////////////////////////////////////////////////////////////

class SiftGPUEX:public SiftGPU
{
	//view mode
	int	_view;
	//sub view mode
	int _sub_view;
	//whether display a debug view
	int _view_debug;
	//colors for SIFT feature display
	enum{COLOR_NUM = 36};
	float _colors[COLOR_NUM*3];
	//display functions
	void DisplayInput();	//display gray level image of input image	
	void DisplayDebug();	//display debug view
	void DisplayFeatureBox(int i);	//display SIFT features
	void DisplayLevel(void (*UseDisplayShader)(), int i);		//display one level image
	void DisplayOctave(void (*UseDisplayShader)(), int i);		//display all images in one octave
	//display different content of Pyramid by specifying different data and display shader
	//the first nskip1 levels and the last nskip2 levels are skiped in display
	void DisplayPyramid( void (*UseDisplayShader)(), int dataName, int nskip1 = 0, int nskip2 = 0);
	//use HSVtoRGB to generate random colors
	static void HSVtoRGB(float hsv[3],float rgb[3]);
public:
	SIFTGPU_EXPORT SiftGPUEX();
	//change view mode
	SIFTGPU_EXPORT void SetView(int view, int sub_view, char * title);
	//display current view
	SIFTGPU_EXPORT void DisplaySIFT();
	//toggle debug mode on/off
	SIFTGPU_EXPORT void ToggleDisplayDebug();
	//randomize the display colors
	SIFTGPU_EXPORT void RandomizeColor();
};

///matcher export
//This is a gpu-based sift match implementation. 
class SiftMatchGPU
{
public:
	enum SIFTMATCH_LANGUAGE	{
		SIFTMATCH_SAME_AS_SIFTGPU = 0, //when siftgpu already initialized.
		SIFTMATCH_CG = 1,
		SIFTMATCH_GLSL = 2,
		SIFTMATCH_CUDA = 3
	};
private:
	int				__max_sift;
	int				__language;
	SiftMatchGPU *	__matcher;
	virtual void   InitSiftMatch(){}

public:
	//OpenGL Context creation/verification, initialization is done automatically inside
	SIFTGPU_EXPORT virtual int  CreateContextGL();
	SIFTGPU_EXPORT virtual int  VerifyContextGL();

	//Consructor, the argument specifies the maximum number of features to match
	SIFTGPU_EXPORT SiftMatchGPU(int max_sift = 4096);

	//chagne gpu_language : 0(same as siftgpu), 1 (cg), 2 (glsl), 3 (cuda)
	//they can the four in SIFTMATCH_LANGUAGE.
	SIFTGPU_EXPORT virtual void SetLanguage(int gpu_language);

	//change the maximum of features to match whenever you want
	SIFTGPU_EXPORT virtual void SetMaxSift(int max_sift);
	//desctructor
	SIFTGPU_EXPORT virtual ~SiftMatchGPU();

	//Specifiy descriptors to match, index = [0/1] for two features sets respectively
	//Option1, use float descriptors, and they be already normalized to 1.0
	SIFTGPU_EXPORT virtual void SetDescriptors(int index, int num, const float* descriptors, int id  = -1);
	//Option 2 unsigned char descriptors. They must be already normalized to 512
	SIFTGPU_EXPORT virtual void SetDescriptors(int index, int num, const unsigned char * descriptors, int id = -1);

	//match two sets of features, the function RETURNS the number of matches.
	//Given two normalized descriptor d1,d2, the distance here is acos(d1 *d2);
	SIFTGPU_EXPORT virtual int  GetSiftMatch(
				int max_match,	// the length of the match_buffer.
				int match_buffer[][2], //buffer to receive the matched feature indices
				float distmax = 0.7,	//maximum distance of sift descriptor
				float ratiomax = 0.8,	//maximum distance ratio
				int mutual_best_match = 1); //mutual best match or one way

	//two functions for guded matching, two constraints can be used 
	//one homography and one fundamental matrix, the use is as follows
	//1. for each image, first call SetDescriptor then call SetFeatureLocation
	//2. Call GetGuidedSiftMatch
	//input feature location is a vector of [float x, float y, float skip[gap]]
	SIFTGPU_EXPORT virtual void SetFeautreLocation(int index, const float* locations, int gap = 0);
	inline void SetFeatureLocation(int index, const SiftGPU::SiftKeypoint * keys)
	{
		SetFeautreLocation(index, (const float*) keys, 2);
	}

	//use a guiding Homography H and a guiding Fundamental Matrix F to compute feature matches
	//the function returns the number of matches.
	SIFTGPU_EXPORT virtual int  GetGuidedSiftMatch(
					int max_match, int match_buffer[][2], //buffer to recieve 
					float H[3][3],			//homography matrix,  (Set NULL to skip)
					float F[3][3],			//fundamental matrix, (Set NULL to skip)
					float distmax = 0.7,	//maximum distance of sift descriptor
					float ratiomax = 0.8,   //maximum distance ratio
					float hdistmax = 32,    //threshold for |H * x1 - x2|_1 
					float fdistmax = 16,    //threshold for sampson error of x2'FX1
					int mutual_best_match = 1); //mutual best or one way

public:
	//overload the new operator, the same reason as SiftGPU above
	// SIFTGPU_EXPORT void* operator new (size_t size);
};


//Two exported global functions used to create SiftGPU and SiftMatchGPU
SIFTGPU_EXPORT SiftGPU * CreateNewSiftGPU(int np =1);
SIFTGPU_EXPORT SiftMatchGPU * CreateNewSiftMatchGPU(int max_sift = 4096);
#endif 
