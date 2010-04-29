////////////////////////////////////////////////////////////////////////////
//	File:		SiftPyramid.h
//	Author:		Changchang Wu
//	Description : interface for the SiftPyramid class.
//		SiftPyramid:			data storage for SIFT
//		|---PyramidGL:			OpenGL based implementation
//		|   |--PyramidNaive:	Unpacked version 
//		|   |--PyramidPacked:	packed version 
//		|--PyramidCU:			CUDA-based implementation
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



#ifndef _SIFT_PYRAMID_H
#define _SIFT_PYRAMID_H


class GLTexImage;
class GLTexInput;
class SiftParam;
class GlobalUtil;

/////////////////////////////////////////////////////////////////////////////
//class SiftPyramid
//description: virutal class of SIFT data pyramid
//			   provides functions for SiftPU to run steps of GPU SIFT
//			   class PyramidNaive is the first implementation
//			   class PyramidPacked is a better OpenGL implementation
//			   class PyramidCu is a CUDA based implementation
/////////////////////////////////////////////////////////////////////////////

#define NO_DUPLICATE_DOWNLOAD

class SiftPyramid : public GlobalUtil
{
public:
	enum{
		DATA_GAUSSIAN	= 0,
		DATA_DOG		= 1,
		DATA_KEYPOINT	= 2,
		DATA_GRAD		= 3,
		DATA_ROT		= 4,
		DATA_NUM		= 5
	};
	enum{
		SIFT_SKIP_FILTERING		= 0x01,
		SIFT_SKIP_DETECTION		= 0x02,
		SIFT_SKIP_ORIENTATION	= 0x04
	};
protected:
	SiftParam&	param;
	int			_hpLevelNum;
	int*		_levelFeatureNum;
	int			_featureNum;
	float*		_histo_buffer;
	int			_existing_keypoints;
	vector<int>	_keypoint_index;
	//display vbo
	GLuint*	    _featureDisplayVBO;
	GLuint*	 	_featurePointVBO;
public:
	//
	float		_timing[8];
	//image size related
	//first octave
	int			_octave_min;
	//how many octaves
	int			_octave_num;
	//pyramid storage
	int			_pyramid_octave_num;
	int			_pyramid_octave_first;
	int			_pyramid_width;
	int			_pyramid_height;
	int			_down_sample_factor;
	int			_allocated; 
	int		    _alignment;
public:
	vector<float>	_keypoint_buffer;
	vector<float>	_descriptor_buffer;
private:
	inline  void PrepareBuffer();
	inline  void LimitFeatureCount();
public:
	//shared by all implementations
	virtual void RunSIFT(GLTexInput*input);
	virtual void SaveSIFT(const char * szFileName);
	virtual void CopyFeatureVector(float*keys, float *descriptors);
	virtual void SetKeypointList(int num, const float * keys, int run_on_current, int skip_orientation);
	//implementation-dependent functions
	virtual void GetFeatureDescriptors() = 0;
	virtual void GenerateFeatureListTex() =0;
	virtual void ReshapeFeatureListCPU() =0;
	virtual void GenerateFeatureDisplayVBO() =0;
	virtual void DownloadKeypoints() = 0;
	virtual void GenerateFeatureListCPU()=0;
	virtual void GenerateFeatureList()=0;
	virtual GLTexImage* GetLevelTexture(int octave, int level)=0;
	virtual GLTexImage* GetLevelTexture(int octave, int level, int dataName) = 0;
	virtual void BuildPyramid(GLTexInput * input)=0;
	virtual void ResizePyramid(int w, int h) = 0;
	virtual void InitPyramid(int w, int h, int ds = 0)=0;
	virtual void DetectKeypointsEX() = 0;
	virtual void ComputeGradient() = 0;
	virtual void GetFeatureOrientations() = 0;
	virtual void GetSimplifiedOrientation() = 0;

	///inline functions, shared by all implementations
	int	GetFeatureNum(){return _featureNum;}
	int GetHistLevelNum(){return _hpLevelNum;}
	const GLuint * GetFeatureDipslayVBO(){return _featureDisplayVBO;}
	const GLuint * GetPointDisplayVBO(){return _featurePointVBO;}
	const int * GetLevelFeatureNum(){return _levelFeatureNum;}
	void	GetPyramidTiming(float * timing){	for(int i = 0; i < 8; i++) timing[i] = _timing[i];	}
	SiftPyramid(SiftParam&sp):param(sp)
	{
		_featureNum = 0;
		_featureDisplayVBO = 0;
		_featurePointVBO = 0;
		_levelFeatureNum = NULL;
		_histo_buffer = NULL;
		_hpLevelNum = 0;

		//image size
		_octave_num = 0;
		_octave_min = 0;
		_alignment = 1;
		_pyramid_octave_num = _pyramid_octave_first = 0;
		_pyramid_width = _pyramid_height = 0;
		_allocated = 0;
		_down_sample_factor = 0;

		/////
		_existing_keypoints = 0;
	}
	virtual ~SiftPyramid() {};	

#ifdef DEBUG_SIFTGPU
private:
	void StopDEBUG();
	void BeginDEBUG(const char* imagepath);
	void WriteTextureForDEBUG(GLTexImage * tex, const char * namet, ...);
#endif
};
#endif 
