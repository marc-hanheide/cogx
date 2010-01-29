////////////////////////////////////////////////////////////////////////////
//	File:		SiftPyramid.h
//	Author:		Changchang Wu
//	Description : interface for the SiftPyramid class.
//		SiftPyramid:	data storage for SIFT
//						base class of PyramidNaive and PyramidPacked
//		PyramidNaive:	Unpacked version of SIFT storage
//		PyramidPacked:	packed version of SIFT storage
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
class SiftParam;
class ProgramGPU;
class FilterProgram;
class ShaderMan;
class GlobalUtil;

/////////////////////////////////////////////////////////////////////////////
//class SiftPyramid
//description: virutal class of SIFT data pyramid
//			   provides functions for SiftPU to run steps of GPU SIFT
//			   class PyramidNaive is the first implementation
//			   class PyramidPacked is a new implementation
/////////////////////////////////////////////////////////////////////////////


class SiftPyramid : public GlobalUtil
{
public:
	enum{
		DATA_GAUSSIAN	= 0,
		DATA_DOG		= 1,
		DATA_GRAD		= 2,
		DATA_ROT		= 3,
		DATA_KEYPOINT	= 4,
		DATA_NUM		= 5
	};
	enum{
		SIFT_SKIP_FILTERING		= 0x01,
		SIFT_SKIP_DETECTION		= 0x02,
		SIFT_SKIP_ORIENTATION	= 0x04
	};
protected:
	SiftParam&	 param;
	int			 _hpLevelNum;
	GLTexImage * _histoPyramidTex;
	GLTexImage * _featureTex;
	GLTexImage * _descriptorTex;
	GLTexImage * _vertexTex;
	GLTexImage * _orientationTex;
	int*		_levelFeatureNum;
	int			_featureNum;
	GLuint* _descriptorPBO;
	GLuint* _featureDisplayVBO;
	GLuint* _featurePointVBO;
	GLuint	_vBufferID;
	float*		 _histo_buffer;
	int			 _existing_keypoints;
	vector<int>	 _keypoint_index;
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
public:
	virtual void RunSIFT(GLTexImage*input);
	void DestroyPerLevelData();
	void DestroySharedData();
	virtual void SaveSIFT(std::ostream & out);
	virtual void CopyFeatureVector(float*keys, float *descriptors);
	virtual void SetKeypointList(int num, const float * keys, int run_on_current, int skip_orientation);
	int			 GetFeatureNum(){return _featureNum;}
	virtual void DownloadKeypoints();
	virtual void PrepareBuffer();
	virtual void BuildVertexBuffer(int count);
	virtual void DrawVertexBuffer(int count);
	virtual void GetFeatureDescriptors();
	virtual void GenerateFeatureListCPU()=0;
	virtual void GenerateFeatureListTex();
	virtual void ReshapeFeatureListCPU();
	virtual void GenerateFeatureList()=0;
	virtual void GenerateFeatureDisplayVBOonCPU(){};//CPU version
	virtual void GenerateFeatureDisplayVBO();
	void SetLevelFeatureNum(int idx, int num);
	void GetTextureStorageSize(int num, int &fw, int& fh);
	void GetAlignedStorageSize(int num, int align, int &fw, int &fh);
	virtual GLTexImage* GetLevelTexture(int octave, int level)=0;
	virtual GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN)=0;
	virtual void BuildPyramid(GLTexImage * input)=0;
	virtual void ResizePyramid(int w, int h) = 0;
	virtual int ResizeFeatureStorage();
	virtual void InitPyramid(int w, int h, int ds = 0)=0;
	virtual void DetectKeypointsEX() = 0;
	virtual void ComputeGradient() = 0;
	virtual void GetFeatureOrienations() = 0;
	virtual void GetSimplifiedOrientation() = 0;
	int GetHistLevelNum(){return _hpLevelNum;}
	const GLuint * GetFeatureDipslayVBO(){return _featureDisplayVBO;}
	const GLuint * GetPointDisplayVBO(){return _featurePointVBO;}
	const int * GetLevelFeatureNum(){return _levelFeatureNum;}
	SiftPyramid(SiftParam&sp):param(sp)
	{
		_featureNum = 0;
		_featureDisplayVBO = 0;
		_featurePointVBO = 0;
		_descriptorPBO = 0;
		_levelFeatureNum = NULL;
		_featureTex = NULL;
		_orientationTex = NULL;
		_descriptorTex = NULL;
		_vertexTex = NULL;
		_histoPyramidTex = NULL;
		_histo_buffer = NULL;
		_vBufferID = 0;
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
	virtual ~SiftPyramid();	
	void		GetPyramidTiming(float * timing)
	{
		for(int i = 0; i < 8; i++) timing[i] = _timing[i];
	}
#ifdef DEBUG_SIFTGPU
private:
	void StopDEBUG();
	void BeginDEBUG(const char* imagepath);
	void WriteTextureForDEBUG(GLTexImage * tex, const char * namet, ...);
#endif
};

class PyramidNaive:public SiftPyramid, public ShaderMan  
{
protected:
	GLTexImage * _texPyramid;
	GLTexImage * _auxPyramid;
public:
	void DestroyPyramidData();
	void GetSimplifiedOrientation();
	void GenerateFeatureListCPU();
	virtual void GetFeatureOrienations();
	virtual void GenerateFeatureList();
	void DetectKeypointsEX();
	void ComputeGradient();
	GLTexImage* GetLevelTexture(int octave, int level);
	GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN);
	void BuildPyramid(GLTexImage * input);
	void InitPyramid(int w, int h, int ds);
	void FitPyramid(int w, int h);
	void ResizePyramid(int w, int h);
	void FitHistogramPyramid();
	PyramidNaive(SiftParam & sp);
	~PyramidNaive();
};



class PyramidPacked:public SiftPyramid, public ShaderMan
{
	GLTexPacked * _allPyramid;
public:
	PyramidPacked(SiftParam& sp);
	~PyramidPacked();
	void DestroyPyramidData();
	void DetectKeypointsEX();
	void ComputeGradient();
	void BuildPyramid(GLTexImage * input);
	void InitPyramid(int w, int h, int ds);
	void FitPyramid(int w, int h);
	void ResizePyramid(int w, int h);
	void FitHistogramPyramid();
	void GenerateFeatureListCPU();
	void GenerateFeatureList();
	void GenerateFeatureListV2();
	void GetSimplifiedOrientation();
	void GetFeatureOrienations();
	GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN);
	GLTexImage* GetLevelTexture(int octave, int level);
};
#endif 
