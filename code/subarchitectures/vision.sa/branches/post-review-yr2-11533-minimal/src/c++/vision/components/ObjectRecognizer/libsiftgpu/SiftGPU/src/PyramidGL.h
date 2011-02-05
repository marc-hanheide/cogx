////////////////////////////////////////////////////////////////////////////
//	File:		PyramidGL.h
//	Author:		Changchang Wu
//	Description : interface for the PyramdGL
//		class PyramidNaive and PyramidPacked are derived from PyramidGL
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



#ifndef _PYRAMID_GL_H
#define _PYRAMID_GL_H

class GLTexImage;
class SiftParam;
class ProgramGPU;
class FilterProgram;
class ShaderMan;
class GlobalUtil;
class SiftPyramid;

class PyramidGL:public SiftPyramid
{
protected:
	GLTexImage* _histoPyramidTex;
	GLTexImage* _featureTex;
	GLTexImage* _descriptorTex;
	GLTexImage* _orientationTex;
public:
	void SetLevelFeatureNum(int idx, int num);
	void GetTextureStorageSize(int num, int &fw, int& fh);
	void GetAlignedStorageSize(int num, int align, int &fw, int &fh);
	static void InterlaceDescriptorF2(int w, int h, float* buf, float* pd, int step);
	static void NormalizeDescriptor(int num, float*pd);
	virtual void DownloadKeypoints();
	virtual int ResizeFeatureStorage();
	////////////////////////////
	virtual void DestroyPerLevelData();
	virtual void DestroySharedData();
	virtual void GetFeatureDescriptors();
	virtual void GenerateFeatureListTex();
	virtual void ReshapeFeatureListCPU();
	virtual void GenerateFeatureDisplayVBO();

	virtual GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN)=0;
public:
	PyramidGL(SiftParam&sp):SiftPyramid(sp)
	{
		_featureTex = NULL;
		_orientationTex = NULL;
		_descriptorTex = NULL;
		_histoPyramidTex = NULL;	
	}
	virtual ~PyramidGL();
};

class PyramidNaive:public PyramidGL, public ShaderMan  
{
protected:
	GLTexImage * _texPyramid;
	GLTexImage * _auxPyramid;
public:
	void DestroyPyramidData();
	void GetSimplifiedOrientation();
	void GenerateFeatureListCPU();
	virtual void GetFeatureOrientations();
	virtual void GenerateFeatureList();
	void DetectKeypointsEX();
	void ComputeGradient();
	GLTexImage* GetLevelTexture(int octave, int level);
	GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN);
	GLTexImage* GetLevelTexture(int octave, int level, int dataName);
	void BuildPyramid(GLTexInput * input);
	void InitPyramid(int w, int h, int ds);
	void FitPyramid(int w, int h);
	void ResizePyramid(int w, int h);
	void FitHistogramPyramid();
	PyramidNaive(SiftParam & sp);
	~PyramidNaive();
};


class PyramidPacked:public PyramidGL, public ShaderMan
{
	GLTexPacked * _allPyramid;
public:
	PyramidPacked(SiftParam& sp);
	~PyramidPacked();
	void DestroyPyramidData();
	void DetectKeypointsEX();
	void ComputeGradient();
	void BuildPyramid(GLTexInput * input);
	void InitPyramid(int w, int h, int ds);
	void FitPyramid(int w, int h);
	void ResizePyramid(int w, int h);
	void FitHistogramPyramid();
	void GenerateFeatureListCPU();
	void GenerateFeatureList();
	void GenerateFeatureListV2();
	void GetSimplifiedOrientation();
	void GetFeatureOrientations();
	GLTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN);
	GLTexImage* GetLevelTexture(int octave, int level);
	GLTexImage* GetLevelTexture(int octave, int level, int dataName);
};

#endif
