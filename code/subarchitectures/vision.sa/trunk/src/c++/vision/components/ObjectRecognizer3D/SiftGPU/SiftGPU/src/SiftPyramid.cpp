////////////////////////////////////////////////////////////////////////////
//	File:		SiftPyramid.cpp
//	Author:		Changchang Wu
//	Description :	Implementation of the SiftPyramid/PyramidNaive class.
//					
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


#include "GL/glew.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>
using namespace std;

#include "GlobalUtil.h"
//#include "IL/il.h"
#include "/usr/include/IL/il.h"
#include "GLTexImage.h"
#include "SiftGPU.h"
#include "ShaderMan.h"
#include "SiftPyramid.h"
#include "math.h"
#include "FrameBufferObject.h"

#ifdef _WIN32
//#include "imdebug/imdebuggl.h"
//#pragma comment (lib, "../lib/imdebug.lib")
#endif

#ifdef DEBUG_SIFTGPU
#ifdef _WIN32
#include "direct.h"
#include "io.h"
#include <sys/stat.h> 
#else
// linux headers
#endif
#endif

#define DEBUG_DESCRIPTOR_BUG


#if defined(HIGH_RESOLUTION_TIME) && defined(_WIN32)
	#include <windows.h>
	#include "mmsystem.h"
	#define clock	timeGetTime
	#define CLOCKS_PER_SEC 1000.0
#else
	#include "time.h"
#endif
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
PyramidNaive::PyramidNaive(SiftParam& sp): SiftPyramid(sp)
{
	_texPyramid = NULL;
	_auxPyramid = NULL;
}

PyramidNaive::~PyramidNaive()
{
	DestroyPyramidData();
}

//align must be 2^i
void SiftPyramid::	GetAlignedStorageSize(int num, int align,  int &fw, int &fh)
{
	if(num <=0)
	{
		fw = fh = 0;
	}else if(num < align*align)
	{
		fw = align;
		fh = (int)ceil(double(num) / fw);
/*	}else if(GlobalUtil::_NarrowFeatureTex)
	{
		double dn = double(num);
		int nb = (int) ceil(dn/GlobalUtil::_texMaxDim/align);	
		fw = align * nb;	
		fh = (int)ceil(dn /fw);*/
	}else
	{
		double dn = double(num);
		int nb = (int) ceil(dn/GlobalUtil::_texMaxDim/align);
		fh = align * nb;
		if(nb <=1)
		{
			fw = (int)ceil(dn / fh);
			//align this dimension to blocksize
			fw = ((int) ceil(double(fw) /align))*align;
		}else
		{
			fw = GlobalUtil::_texMaxDim;
		}

	}


}

void SiftPyramid::GetTextureStorageSize(int num, int &fw, int& fh)
{
	if(num <=0)
	{
		fw = fh = 0;
	}else if(num <= GlobalUtil::_FeatureTexBlock)
	{
		fw = num;
		fh = 1;
	}else if(GlobalUtil::_NarrowFeatureTex)
	{
		double dn = double(num);
		int nb = (int) ceil(dn/GlobalUtil::_texMaxDim/GlobalUtil::_FeatureTexBlock);	
		fw = GlobalUtil::_FeatureTexBlock * nb;	
		fh = (int)ceil(dn /fw);
	}else
	{
		double dn = double(num);
		int nb = (int) ceil(dn/GlobalUtil::_texMaxDim/GlobalUtil::_FeatureTexBlock);
		fh = GlobalUtil::_FeatureTexBlock * nb;
		if(nb <=1)
		{
			fw = (int)ceil(dn / fh);

			//align this dimension to blocksize

			//
			if( fw < fh)
			{
				int temp = fh;
				fh = fw;
				fw = temp;
			}
		}else
		{
			fw = GlobalUtil::_texMaxDim;
		}
	}
}

void PyramidNaive::DestroyPyramidData()
{
	if(_texPyramid)
	{
		delete [] _texPyramid;
		_texPyramid = NULL;
	}
	if(_auxPyramid)
	{
		delete [] _auxPyramid;  
		_auxPyramid = NULL;
	}
}

SiftPyramid::~SiftPyramid()
{
	DestroyPerLevelData();
	DestroySharedData();
	_allocated = 0;
}

void SiftPyramid::DestroyPerLevelData()
{
	//integers vector to store the feature numbers.
	if(_levelFeatureNum)
	{
		delete [] _levelFeatureNum;
		_levelFeatureNum = NULL;
	}
	//texture used to store features
	if(	_featureTex)
	{
		delete [] _featureTex;
		_featureTex =	NULL;
	}
	//texture used for multi-orientation 
	if(_orientationTex)
	{
		delete [] _orientationTex;
		_orientationTex = NULL;
	}
	int no = _octave_num* param._dog_level_num;

	//not used
	if( _descriptorPBO)
	{
		glDeleteBuffers(no, _descriptorPBO);
		delete [] _descriptorPBO;
		_descriptorPBO = NULL;
	}

	//two sets of vbos used to display the features
	if(_featureDisplayVBO)
	{
		glDeleteBuffers(no, _featureDisplayVBO);
		delete [] _featureDisplayVBO;
		_featureDisplayVBO = NULL;
	}
	if( _featurePointVBO)
	{
		glDeleteBuffers(no, _featurePointVBO);
		delete [] _featurePointVBO;
		_featurePointVBO = NULL;
	}

}

void SiftPyramid::DestroySharedData()
{
	//histogram reduction
	if(_histoPyramidTex)
	{
		delete[]	_histoPyramidTex;
		_hpLevelNum = 0;
		_histoPyramidTex = NULL;
	}
	//not used
	if(_vertexTex != _descriptorTex && _vertexTex)
	{
		delete _vertexTex;
		_vertexTex = NULL;
	}
	//descriptor storage shared by all levels
	if(_descriptorTex)
	{
		delete [] _descriptorTex;
		_descriptorTex = NULL;
	}
	// vertext buffer used in one of the descriptor algorithm
	if(_vBufferID)
	{
		glDeleteBuffers(1, &_vBufferID);
		_vBufferID = 0;
	}
	//cpu reduction buffer.
	if(_histo_buffer)
	{
		delete[] _histo_buffer;
		_histo_buffer = 0;
	}
}

void PyramidNaive::FitHistogramPyramid()
{
	GLTexImage * tex, *htex;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 

	tex = GetBaseLevel(_octave_min , DATA_KEYPOINT) + 2;
	htex = _histoPyramidTex + hist_level_num - 1;
	int w = tex->GetImgWidth() >> 1;
	int h = tex->GetImgHeight() >> 1;

	for(int k = 0; k <hist_level_num -1; k++, htex--)
	{
		if(htex->GetImgHeight()!= h || htex->GetImgWidth() != w)
		{	
			htex->SetImageSize(w, h);
			htex->ZeroHistoMargin();
		}

		w = (w + 1)>>1; h = (h + 1) >> 1;
	}
}

void PyramidNaive::FitPyramid(int w, int h)
{
	//(w, h) <= (_pyramid_width, _pyramid_height);

	_pyramid_octave_first = 0;
	//
	_octave_num  = GlobalUtil::_octave_num_default;

	int _octave_num_max = max(1, (int) floor (log ( double(min(w, h)))/log(2.0))  -3 );

	if(_octave_num < 1 || _octave_num > _octave_num_max) 
	{
		_octave_num = _octave_num_max;
	}


	int pw = _pyramid_width>>1, ph = _pyramid_height>>1;
	while(_pyramid_octave_first + _octave_num < _pyramid_octave_num &&  
		pw >= w && ph >= h)
	{
		_pyramid_octave_first++;
		pw >>= 1;
		ph >>= 1;
	}

	for(int i = 0; i < _octave_num; i++)
	{
		GLTexImage * tex = GetBaseLevel(i + _octave_min);
		GLTexImage * aux = GetBaseLevel(i + _octave_min, DATA_KEYPOINT);
		for(int j = param._level_min; j <= param._level_max; j++, tex++, aux++)
		{
			tex->SetImageSize(w, h);
			aux->SetImageSize(w, h);
		}
		w>>=1;
		h>>=1;
	}
}
void PyramidNaive::InitPyramid(int w, int h, int ds)
{
	int wp, hp, toobig = 0;
	if(ds == 0)
	{
		_down_sample_factor = 0;
		if(GlobalUtil::_octave_min_default>=0)
		{
			wp = w >> GlobalUtil::_octave_min_default;
			hp = h >> GlobalUtil::_octave_min_default;
		}else 
		{
			wp = w << (-GlobalUtil::_octave_min_default);
			hp = h << (-GlobalUtil::_octave_min_default);
		}
		_octave_min = _octave_min_default;
	}else
	{
		//must use 0 as _octave_min; 
		_octave_min = 0;
		_down_sample_factor = ds;
		w >>= ds;
		h >>= ds;
		wp = w;
		hp = h; 

	}

	while(wp > GlobalUtil::_texMaxDim || hp > GlobalUtil::_texMaxDim)
	{
		_octave_min ++;
		wp >>= 1;
		hp >>= 1;
		toobig = 1;
	}

	if(toobig && GlobalUtil::_verbose)
	{

		std::cout<< "**************************************************************\n"
					"Image larger than allowed dimension, data will be downsampled!\n"
					"use -maxd to change the settings\n"
					"***************************************************************\n";
	}

	if( wp == _pyramid_width && hp == _pyramid_height && _allocated )
	{
		FitPyramid(wp, hp);
	}else if(GlobalUtil::_ForceTightPyramid || _allocated ==0)
	{
		ResizePyramid(wp, hp);
	}
	else if( wp > _pyramid_width || hp > _pyramid_height )
	{
		ResizePyramid(max(wp, _pyramid_width), max(hp, _pyramid_height));
		if(wp < _pyramid_width || hp < _pyramid_height)  FitPyramid(wp, hp);
	}
	else
	{
		//try use the pyramid allocated for large image on small input images
		FitPyramid(wp, hp);
	}

	//select the initial smoothing filter according to the new _octave_min
	ShaderMan::SelectInitialSmoothingFilter(_octave_min, param);
}

void PyramidNaive::ResizePyramid( int w,  int h)
{
	//
	unsigned int totalkb = 0;
	int _octave_num_new, input_sz;
	int i, j;
	GLTexImage * tex, *aux;
	//

	if(_pyramid_width == w && _pyramid_height == h && _allocated) return;

	if(w > GlobalUtil::_texMaxDim || h > GlobalUtil::_texMaxDim) return ;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<w<<"x"<<h<<endl;
	//first octave does not change
	_pyramid_octave_first = 0;

	
	//compute # of octaves

	input_sz = min(w,h) ;


	_pyramid_width =  w;
	_pyramid_height =  h;



	//reset to preset parameters

	_octave_num_new  = GlobalUtil::_octave_num_default;

	if(_octave_num_new < 1) 
	{
		_octave_num_new = (int) floor (log ( double(input_sz))/log(2.0)) -3 ;
		if(_octave_num_new<1 ) _octave_num_new = 1;
	}

	if(_pyramid_octave_num != _octave_num_new)
	{
		//destroy the original pyramid if the # of octave changes
		if(_octave_num >0)
		{
			DestroyPerLevelData();
			DestroyPyramidData();
		}
		_pyramid_octave_num = _octave_num_new;
	}

	_octave_num = _pyramid_octave_num;

	int noct = _octave_num;
	int nlev = param._level_num;

	//	//initialize the pyramid
	if(_texPyramid==NULL)	_texPyramid = new GLTexImage[ noct* nlev ];
	if(_auxPyramid==NULL)	_auxPyramid = new GLTexImage[ noct* nlev ];


	tex = GetBaseLevel(_octave_min, DATA_GAUSSIAN);
	aux = GetBaseLevel(_octave_min, DATA_KEYPOINT);
	for(i = 0; i< noct; i++)
	{
		totalkb += (nlev * w * h * 16 / 1024);
		for( j = 0; j< nlev; j++, tex++)
		{
			tex->InitTexture(w, h);
			//tex->AttachToFBO(0);
		}
		//several auxilary textures are not actually required
		totalkb += ((nlev - 3) * w * h * 16 /1024);
		for( j = 0; j< nlev ; j++, aux++)
		{
			if(j < 2) continue;
			if(j >= nlev - 1) continue;
			aux->InitTexture(w, h, 0);
			//aux->AttachToFBO(0);
		}

		w>>=1;
		h>>=1;
	}

	totalkb += ResizeFeatureStorage();

	//build the vertex buffer for features??
	if(GlobalUtil::_DescriptorPPT ==32)
	{
		BuildVertexBuffer(_featureTex->GetImgWidth()*_featureTex->GetImgHeight());
	}

	//
	_allocated = 1;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<(totalkb/1024)<<"MB\n";

}


int SiftPyramid::ResizeFeatureStorage()
{
	int totalkb = 0;
	if(_levelFeatureNum==NULL)	_levelFeatureNum = new int[_octave_num * param._dog_level_num];
	std::fill(_levelFeatureNum, _levelFeatureNum+_octave_num * param._dog_level_num, 0); 

	int wmax = GetBaseLevel(_octave_min)->GetDrawWidth();
	int hmax = GetBaseLevel(_octave_min)->GetDrawHeight();
	int w ,h, i;

	//use a fbo to initialize textures..
	FrameBufferObject fbo;
	
	//
	if(_histo_buffer == NULL) _histo_buffer = new float[1 << (2 + 2 * GlobalUtil::_ListGenSkipGPU)];
	//histogram for feature detection

	int num = (int)ceil(log(double(max(wmax, hmax)))/log(2.0));

	if( _hpLevelNum != num)
	{
		_hpLevelNum = num;
		if(GlobalUtil::_ListGenGPU)
		{
			if(_histoPyramidTex ) delete [] _histoPyramidTex;
			_histoPyramidTex = new GLTexImage[_hpLevelNum];
			w = h = 1 ;
			for(i = 0; i < _hpLevelNum; i++)
			{
				totalkb += (w * h * 16 / 1024);
				_histoPyramidTex[i].InitTexture(w, h, 0);
				_histoPyramidTex[i].AttachToFBO(0);
				w<<=1;
				h<<=1;
			}
		}
	}



	//initialize the feature texture

	int idx = 0, n = _octave_num * param._dog_level_num;
	if(_featureTex==NULL)	_featureTex = new GLTexImage[n];
	if(GlobalUtil::_MaxOrientation >1 && GlobalUtil::_OrientationPack2==0 && _orientationTex== NULL)
		_orientationTex = new GLTexImage[n];




	for(i = 0; i < _octave_num; i++)
	{
		GLTexImage * tex = GetBaseLevel(i+_octave_min);
		int fmax = int(tex->GetImgWidth()*tex->GetImgHeight()*GlobalUtil::_MaxFeaturePercent);
		int fw, fh;
		//
		if(fmax > GlobalUtil::_MaxLevelFeatureNum) fmax = GlobalUtil::_MaxLevelFeatureNum;
		else if(fmax < 32) fmax = 32;	//give it at least a space of 32 feature

		GetTextureStorageSize(fmax, fw, fh);
		
		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{

			_featureTex[idx].InitTexture(fw, fh, 0);
			_featureTex[idx].AttachToFBO(0);
			totalkb += fw * fh * 16 /1024;
			//
			if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
			{
				_orientationTex[idx].InitTexture(fw, fh, 0);
				_orientationTex[idx].AttachToFBO(0);
				totalkb += fw * fh * 16 /1024;
			}
		}


	}


	if(_descriptorTex==NULL)
	{
		//initialize feature texture pyramid
		wmax = _featureTex->GetImgWidth();
		hmax = _featureTex->GetImgHeight();

		int nf, ns;
		if(GlobalUtil::_DescriptorPPT)
		{
			//32*4 = 128. 
			nf = 32 / GlobalUtil::_DescriptorPPT / GlobalUtil::_DescriptorVPC;	// how many textures we need
			ns = max(4, GlobalUtil::_DescriptorPPT);		    // how many point in one texture for one descriptor
		}else
		{
			nf = 1; ns = 4;
		}
		//
		_alignment = ns;
		//
		_descriptorTex = new GLTexImage[nf];

		int fw, fh;
		GetAlignedStorageSize(hmax*wmax* max(ns, 10), _alignment, fw, fh);

		if(fh < hmax ) fh = hmax;
		if(fw < wmax ) fw = wmax;

		if(_vertexTex == NULL) _vertexTex = _descriptorTex;

		totalkb += ( fw * fh * nf * 16 /1024);
		for(i =0; i < nf; i++)
		{
			_descriptorTex[i].InitTexture(fw, fh);
		}
	}
	return totalkb;
}

#define USE_TIMING()		double t, t0, tt;
#define OCTAVE_START()		if(GlobalUtil::_timingO){	t = t0 = clock();		cout<<"#"<<i+_down_sample_factor<<"\t";	}
#define LEVEL_FINISH()		if(GlobalUtil::_timingL){	glFinish();	tt = clock();cout<<(tt-t)/CLOCKS_PER_SEC<<"\t";	t = clock();}
#define OCTAVE_FINISH()		if(GlobalUtil::_timingO)cout<<"|\t"<<(clock()-t0)/CLOCKS_PER_SEC<<endl;


void PyramidNaive::BuildPyramid(GLTexImage *input)
{

	//
	USE_TIMING();
	int i, j, k;
	GLTexPacked * tex;
	FilterProgram ** filter;
	FrameBufferObject fbo;

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	input->FitTexViewPort();

	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{

		tex = (GLTexPacked*)GetBaseLevel(i);

		j = param._level_min + 1;
		filter = ShaderMan::f_gaussian_step;

		OCTAVE_START();

		if( i == _octave_min )
		{
			if(i < 0)
			{

				TextureUpSample(tex, input, 1<<(-i)	);			
			}else
			{
				//image might have been already down-sampled by cpu code
				TextureDownSample(tex, input, 1<<i);
			}


			//
			if(ShaderMan::f_gaussian_skip0)
			{
				ShaderMan::f_gaussian_skip0->RunFilter(tex, tex, NULL);
			}
	
			LEVEL_FINISH();

		}else
		{
			TextureDownSample(tex, GetLevelTexture(i-1, param._level_ds)); 
	
			LEVEL_FINISH();

			if(ShaderMan::f_gaussian_skip1)
			{
				ShaderMan::f_gaussian_skip1->RunFilter(tex, tex, NULL);
				LEVEL_FINISH();
			}else if (GlobalUtil::_MaxFilterWidth == -1 && GlobalUtil::_ExtraDownSample)
			{
				//copy and downsample other extra levels...
				//according to definition, we can down sample more than one level
				for(k = param._level_ds +1 ;k <= param._level_max; k++, tex++, j++, filter++)
				{

					TextureDownSample(tex+1, GetLevelTexture(i-1, k));
					LEVEL_FINISH();
				}
				//this won't work well if the gussian kernel size is truncated for large size
			}
		}



		for( ; j <=  param._level_max ; j++, tex++, filter++)
		{
			// filtering
			(*filter)->RunFilter(tex+1, tex, NULL);
			LEVEL_FINISH();
		}
		OCTAVE_FINISH();

	}
	if(GlobalUtil::_timingS)	glFinish();
	UnloadProgram();
}


void SiftPyramid::RunSIFT(GLTexImage*input)
{
	//
	if(_existing_keypoints & SIFT_SKIP_FILTERING)
	{
		_timing[0] = 0;
	}else
	{
		GlobalUtil::StartTimer("Build    Pyramid");
		BuildPyramid(input);
		GlobalUtil::StopTimer();
		_timing[0] = GetElapsedTime();
	}

	if(_existing_keypoints)
	{
		//existing keypoint list should at least have the locations and scale
		GlobalUtil::StartTimer("Upload Feature List");
		ComputeGradient();
		GenerateFeatureListTex();
		GlobalUtil::StopTimer();
		_timing[2] = GetElapsedTime();
		_timing[1] = 0;
	}else
	{

		GlobalUtil::StartTimer("Detect Keypoints");
		DetectKeypointsEX();
		GlobalUtil::StopTimer();
		_timing[1] = GetElapsedTime();


		if(GlobalUtil::_ListGenGPU ==1)
		{
			GlobalUtil::StartTimer("Get Feature List");
			GenerateFeatureList();
			GlobalUtil::StopTimer();

		}else
		{
			GlobalUtil::StartTimer("Transfer Feature List");
			GenerateFeatureListCPU();
			GlobalUtil::StopTimer();
		}

		_timing[2] = GetElapsedTime();
	}

	if((_existing_keypoints& SIFT_SKIP_ORIENTATION)  || GlobalUtil::_FixedOrientation)
	{
		//use exisitng feature orientation or
		//use 0 as orientation for all features
		_timing[3] = _timing[4] =0;
	}else 	if(GlobalUtil::_MaxOrientation>0)
	{
		//some extra tricks are done to handle existing keypoint list
		GlobalUtil::StartTimer("Feature Orientations");
		GetFeatureOrienations();
		GlobalUtil::StopTimer();
		_timing[3] = GetElapsedTime();

		//for existing keypoint list, only the best orientation is kept.
		if(GlobalUtil::_MaxOrientation >1 && !_existing_keypoints)
		{
			GlobalUtil::StartTimer("MultiO Feature List");
			ReshapeFeatureListCPU();
			GlobalUtil::StopTimer();	
			_timing[4] = GetElapsedTime();
		}else
		{
			_timing[4] = 0;
		}
	}else
	{
		GlobalUtil::StartTimer("Feature Orientations");
		GetSimplifiedOrientation();
		GlobalUtil::StopTimer();
		_timing[3] = GetElapsedTime();
		_timing[4] = 0; 
	}

	PrepareBuffer();

	if(_existing_keypoints & SIFT_SKIP_ORIENTATION)
	{
		//no need to read back feature if all fields of keypoints are already specified
		_timing[5] = 0;
	}else
	{
		GlobalUtil::StartTimer("Download Keypoints");
		DownloadKeypoints();
		GlobalUtil::StopTimer();
		_timing[5] =  GetElapsedTime(); 
	}



	if(GlobalUtil::_DescriptorPPT)
	{
		//desciprotrs are downloaded in descriptor computation of each level
		GlobalUtil::StartTimer("Descriptor");
		GetFeatureDescriptors();
		GlobalUtil::StopTimer();
		_timing[6] =  GetElapsedTime(); 
	}else
	{
		_timing[6] = 0;
	}

	//reset the existing keypoints
	_existing_keypoints = 0;
	_keypoint_index.resize(0);

	if(GlobalUtil::_UseSiftGPUEX)
	{
		GlobalUtil::StartTimer("Gen. Display VBO");
		GenerateFeatureDisplayVBO();
		GlobalUtil::StopTimer();
		_timing[7] = GlobalUtil::GetElapsedTime();
	}else
	{
		_timing[7] = 0;
	} 
}





GLTexImage*  PyramidNaive::GetLevelTexture(int octave, int level)
{
	return _texPyramid+ (_pyramid_octave_first + octave - _octave_min) * param._level_num 
		+ (level - param._level_min);
}

//in the packed implementation( still in progress)
// DATA_GAUSSIAN, DATA_DOG, DATA_GAD will be stored in different textures.

GLTexImage*  PyramidNaive::GetBaseLevel(int octave, int dataName)
{
	if(octave <_octave_min || octave > _octave_min + _octave_num) return NULL;
	switch(dataName)
	{
		case DATA_GAUSSIAN:
		case DATA_DOG:
		case DATA_GRAD:
		case DATA_ROT:
			return _texPyramid+ (_pyramid_octave_first + octave - _octave_min) * param._level_num;
		case DATA_KEYPOINT:
			return _auxPyramid + (_pyramid_octave_first + octave - _octave_min) * param._level_num;
		default:
			return NULL;
	}
}



void SiftPyramid::BuildVertexBuffer(int count)
{
	//UseShaderCopyTexCoord
	int w = _vertexTex->GetTexWidth();
	int h = (int)ceil(count / double(w));
	_vertexTex->AttachToFBO(0);
	_vertexTex->FitRealTexViewPort();
	ShaderMan::UseShaderCopyTexCoord();
	glBegin(GL_QUADS);
	for(int i = 0, c = 0; i < h; i++)
	{
		glTexCoord2i(c, 0);			glVertex2i( 0, i);
		glTexCoord2i(c, 1);			glVertex2i( 0, i +1);
		c+=w;
		glTexCoord2i(c, 1);			glVertex2i( w, i +1);
		glTexCoord2i(c, 0);			glVertex2i( w, i);
	}
	glEnd();

	if(_vBufferID ==0)		glGenBuffers(1, &_vBufferID);
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _vBufferID);
	glBufferData(GL_PIXEL_PACK_BUFFER_ARB, w*h*4*sizeof(float),	NULL, GL_STATIC_DRAW_ARB);
	glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, 0);
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
}


void SiftPyramid::DrawVertexBuffer(int count)
{
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glBindBuffer( GL_ARRAY_BUFFER, _vBufferID );
	glVertexPointer( 4, GL_FLOAT, 0, (char *) NULL );	
	glTexCoordPointer(4, GL_FLOAT, 0, (char*) NULL);
	glDrawArrays( GL_POINTS, 0, count);
	glFlush();
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

/*	glBegin(GL_POINTS);
	for(int i = 0; i < count; i++)
	{
		glVertex2f(i+0.5, 0.5);
	}
	glEnd();	*/
	GlobalUtil::CheckErrorsGL();
}






void PyramidNaive::ComputeGradient()
{

	int i, j;
	double  ts, t1;
	GLTexImage * tex;
	FrameBufferObject fbo;


	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = clock();
	
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		for( j = param._level_min + GlobalUtil::_GradientLevelOffset ; j < param._level_max ; j++)
		{
			tex = GetLevelTexture(i, j);
			tex->FitTexViewPort();
			tex->AttachToFBO(0);
			tex->BindTex();
			ShaderMan::UseShaderGradientPass();
			tex->DrawQuadMT4();
		}
	}		

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		glFinish();
		t1 = clock();	
		std::cout<<"<Compute Gradient>\t"<<(t1-ts)/CLOCKS_PER_SEC<<"\n";
	}

	UnloadProgram();
	GLTexImage::UnbindMultiTex(3);
	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);
}


//keypoint detection with subpixel localization
void PyramidNaive::DetectKeypointsEX()
{
	//

	int i, j;
	double t0, t, ts, t1, t2;
	GLTexImage * tex, *aux;
	FrameBufferObject fbo;


	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = clock();
	
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);


	//extra gradient data required for visualization or special parameter
	if(GlobalUtil::_UseSiftGPUEX || GlobalUtil::_GradientLevelOffset ==1)
	{
		int levels[2] = {param._level_min +1, param._level_max};
		int nlevel = GlobalUtil::_UseSiftGPUEX ? 2 : 1;
		for ( i = _octave_min; i < _octave_min + _octave_num; i++)
		{
			for( j =0; j < nlevel ; j++)
			{
				tex = GetLevelTexture(i, levels[j]);
				tex->FitTexViewPort();
				tex->AttachToFBO(0);
				tex->BindTex();
				ShaderMan::UseShaderGradientPass();
				tex->DrawQuadMT4();
			}
		}		

	}

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		glFinish();
		t1 = clock();
	}


	GLenum buffers[] = { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
	glDrawBuffers(2, buffers);
	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		if(GlobalUtil::_timingO)
		{
			t0 = clock();
			std::cout<<"#"<<(i + _down_sample_factor)<<"\t";
		}
		tex = GetBaseLevel(i) + 2;
		aux = GetBaseLevel(i, DATA_KEYPOINT) +2;
		aux->FitTexViewPort();

		for( j = param._level_min +2; j <  param._level_max ; j++, aux++, tex++)
		{
			if(GlobalUtil::_timingL)t = clock();		
			tex->AttachToFBO(0);
			aux->AttachToFBO(1);
			glActiveTexture(GL_TEXTURE0);
			tex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			(tex+1)->BindTex();
			glActiveTexture(GL_TEXTURE2);
			(tex-1)->BindTex();
			ShaderMan::UseShaderKeypoint((tex+1)->GetTexID(), (tex-1)->GetTexID());
			aux->DrawQuadMT8();
	
			if(GlobalUtil::_timingL)
			{
				glFinish();
				std::cout<<(clock()-t)/CLOCKS_PER_SEC<<"\t";
			}
			tex->DetachFBO(0);
			aux->DetachFBO(1);
		}
		if(GlobalUtil::_timingO)
		{
			std::cout<<"|\t"<<(clock()-t0)/CLOCKS_PER_SEC<<"\n";
		}
	}

	if(GlobalUtil::_timingS)
	{
		glFinish();
		t2 = clock();
		if(GlobalUtil::_verbose) 
			std::cout	<<"<Get Keypoints ..  >\t"<<(t2-t1)/CLOCKS_PER_SEC<<"\n"
						<<"<Extra Gradient..  >\t"<<(t1-ts)/CLOCKS_PER_SEC<<"\n";
	}
	UnloadProgram();
	GLTexImage::UnbindMultiTex(3);
	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);


}
//generate feature list on GPU
void PyramidNaive::GenerateFeatureList()
{
	//generate the histogram0pyramid
	FrameBufferObject fbo;
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	GLTexImage * htex, * ftex, * tex;
	double t1, t2, t3, ot1, ot2, ts1 = 0, ts2 = 0; 
	int ocount, idx = 0;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 
	int hist_skip_gpu = GlobalUtil::_ListGenSkipGPU; 
	_featureNum = 0;

	FitHistogramPyramid();

	for(int i = 0; i < _octave_num; i++)
	{
		tex = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + 2;
		//output

		if(GlobalUtil::_timingO)
		{
			ot1 = ot2 = 0; 
			ocount = 0;
			std::cout<<"#"<<i+_octave_min + _down_sample_factor<<":\t";
		}
		for(int j = 0; j < param._dog_level_num; j++, tex++, idx++)
		{
			float fcount = 0.0f;
			ftex = _featureTex + idx;
			htex = _histoPyramidTex + hist_level_num - 1 - i;
			if(GlobalUtil::_timingL) t1= clock();
			///
			glActiveTexture(GL_TEXTURE0);
			tex->BindTex();
			htex->AttachToFBO(0);
			int tight = ((htex->GetImgWidth() * 2 == tex->GetImgWidth() -1 || tex->GetTexWidth() == tex->GetImgWidth()) &&
						 (htex->GetImgHeight() *2 == tex->GetImgHeight()-1 || tex->GetTexHeight() == tex->GetImgHeight()));
			ShaderMan::UseShaderGenListInit(tex->GetImgWidth(), tex->GetImgHeight(), tight);
			htex->FitTexViewPort();
			//this uses the fact that no feature is on the edge.
			htex->DrawQuadReduction();

			//reduction..
			htex--;
	
			//this part might have problems on several GPUS
			//because the output of one pass is the input of the next pass
			//need to call glFinish to make it right
			//but too much glFinish makes it slow
			for(int k = 0; k <hist_level_num - i - 1 - hist_skip_gpu; k++, htex--)
			{
				htex->AttachToFBO(0);
				htex->FitTexViewPort();
				(htex+1)->BindTex();
				ShaderMan::UseShaderGenListHisto();
				htex->DrawQuadReduction();					
			}
			//glFinish();
			//
			if(GlobalUtil::_timingL)		t2= clock();
			//
			if(hist_skip_gpu == 0)
			{	
				//read back one pixel
				float fn[4];
				glReadPixels(0, 0, 1, 1, GL_RGBA , GL_FLOAT, fn);
				fcount = (fn[0] + fn[1] + fn[2] + fn[3]);
				if(fcount < 1) fcount = 0;


				_levelFeatureNum[ idx] = (int)(fcount);
				SetLevelFeatureNum(idx, (int)fcount);

				//save  number of features
				ocount+=int(fcount);
				_featureNum += int(fcount);

				//
				if(fcount < 1.0) 
				{
					ot1 += (t2 - t1); 
					if(GlobalUtil::_timingL) std::cout<<"0\t";
					continue;
				}
			

				///generate the feature texture

				htex=  _histoPyramidTex;

				/*if(GlobalUtil::_IsATI)
				{
					unsigned char zeros[4] = {0, 0, 0, 0};
					ftex->BindTex();
					glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, zeros);
					//something need to be done to activate the texture on my ATI..

				}*/
				htex->BindTex();

				
				/*if(GlobalUtil::_IsATI) 
				{
					glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, 1, 1, GL_RGBA, GL_FLOAT, fn);
					//something need to be done to activate the texture on my ATI..
				}*/
				//first pass
				ftex->AttachToFBO(0);
				if(GlobalUtil::_MaxOrientation>1)
				{
					//this is very important...
					ftex->FitRealTexViewPort();
					glClear(GL_COLOR_BUFFER_BIT);
					glFinish();
				}else
				{
					
					ftex->FitTexViewPort();
					
				}


				ShaderMan::UseShaderGenListStart((float)ftex->GetImgWidth(), htex->GetTexID());

				ftex->DrawQuad();
				//make sure it finishes before the next step
				ftex->DetachFBO(0);

				//pass on each pyramid level
				htex++;
			}else
			{

				int tw = htex[1].GetDrawWidth(), th = htex[1].GetDrawHeight();
				int fc = 0;
				glReadPixels(0, 0, tw, th, GL_RGBA , GL_FLOAT, _histo_buffer);	
				_keypoint_buffer.resize(0);
				for(int y = 0, pos = 0; y < th; y++)
				{
					for(int x= 0; x < tw; x++)
					{
						for(int c = 0; c < 4; c++, pos++)
						{
							int ss =  (int) _histo_buffer[pos]; 
							if(ss == 0) continue;
							float ft[4] = {2 * x + (c%2? 1.5f:  0.5f), 2 * y + (c>=2? 1.5f: 0.5f), 0, 1 };
							for(int t = 0; t < ss; t++)
							{
								ft[2] = (float) t; 
								_keypoint_buffer.insert(_keypoint_buffer.end(), ft, ft+4);
							}
							fc += (int)ss; 
						}
					}
				}
				_levelFeatureNum[ idx] = fc;
				SetLevelFeatureNum(idx, fc);
				if(fc == 0) 
				{
					ot1 += (t2 - t1); 
					if(GlobalUtil::_timingL) std::cout<<"0\t";
					continue;
				}

				//////
				fcount = (float) fc; 	
				ocount += fc;		_featureNum += fc;
				/////////////////////
				ftex->AttachToFBO(0);
				if(GlobalUtil::_MaxOrientation>1)
				{
					ftex->FitRealTexViewPort();
					glClear(GL_COLOR_BUFFER_BIT);
				}else
				{					
					ftex->FitTexViewPort();
				}
				_keypoint_buffer.resize(ftex->GetDrawWidth() * ftex->GetDrawHeight()*4, 0);
				///////////
				glActiveTexture(GL_TEXTURE0);
				ftex->BindTex();
				glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, ftex->GetDrawWidth(),
					ftex->GetDrawHeight(), GL_RGBA, GL_FLOAT, &_keypoint_buffer[0]);
				htex += 2;
			}

			for(int lev = 1 + hist_skip_gpu; lev < hist_level_num  - i; lev++, htex++)
			{

				glActiveTexture(GL_TEXTURE0);
				ftex->BindTex();
				ftex->AttachToFBO(0);
				glActiveTexture(GL_TEXTURE1);
				htex->BindTex();
				ShaderMan::UseShaderGenListStep(ftex->GetTexID(), htex->GetTexID());
				ftex->DrawQuad();
				ftex->DetachFBO(0);	
			}
			if(GlobalUtil::_timingL)
			{
				glFinish();
				t3 = clock();
				ot1 += (t2 - t1); ot2 += ( t3 - t2);
				std::cout<<int(fcount)<<"\t";
			}
			GLTexImage::UnbindMultiTex(2);


		}
		if(GlobalUtil::_timingO)
		{	
			ot1 /= CLOCKS_PER_SEC; ot2/= CLOCKS_PER_SEC;
			ts1 += ot1; ts2 += ot2; 
			std::cout << "| \t" << int(ocount) << " :\t(" << ot1 <<",\t" << ot2 << ")\n";
		}
	}
	if(GlobalUtil::_timingS)glFinish();
	if(GlobalUtil::_verbose)
	{
		//std::cout<<"("<<(ts1)/CLOCKS_PER_SEC<<",\t"<<(ts2)/CLOCKS_PER_SEC<<")\t";
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}
}


void SiftPyramid::GenerateFeatureDisplayVBO()
{
	//use a big VBO to save all the SIFT box vertices
	int w, h, esize; GLint bsize;
	int nvbo = _octave_num * param._dog_level_num;
	if(_featureDisplayVBO==NULL)
	{
		//initialize the vbos
		_featureDisplayVBO = new GLuint[nvbo];
		_featurePointVBO = new GLuint[nvbo];
		glGenBuffers( nvbo, _featureDisplayVBO );	
		glGenBuffers(nvbo, _featurePointVBO);
	}

	FrameBufferObject fbo;
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glActiveTexture(GL_TEXTURE0);
	
	//
	GLTexImage & tempTex = * _vertexTex;
	//
	for(int i = 0, idx = 0; i < _octave_num; i++)
	{
		for(int j = 0; j < param._dog_level_num; j ++, idx++)
		{
			GLTexImage * ftex  = _featureTex + idx;

			if(_levelFeatureNum[idx]<=0)continue;
			//box display vbo
			int count = _levelFeatureNum[idx]* 10;
			GetAlignedStorageSize(count, _alignment, w, h);
			w = (int)ceil(double(count)/ h);

			//input
			fbo.BindFBO();
			ftex->BindTex();

			//output
			tempTex.AttachToFBO(0);
			GlobalUtil::FitViewPort(w, h);
			//shader
			ShaderMan::UseShaderGenVBO(	(float)ftex->GetImgWidth(),  (float) w, 
				param.GetLevelSigma(j + param._level_min + 1));
			GLTexImage::DrawQuad(0,  (float)w, 0, (float)h);
		
			//
			glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _featureDisplayVBO[ idx]);
			glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
			esize = w*h * sizeof(float)*4;
			//increase size when necessary
			if(bsize < esize) glBufferData(GL_PIXEL_PACK_BUFFER_ARB, esize*3/2,	NULL, GL_STATIC_DRAW_ARB);
			
			//read back if we have enough buffer	
			glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
			if(bsize >= esize)	glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, 0);
			else glBufferData(GL_PIXEL_PACK_BUFFER_ARB, 0,	NULL, GL_STATIC_DRAW_ARB);
			glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);


			//copy the texture into vbo
			fbo.BindFBO();
			tempTex.AttachToFBO(0);

			ftex->BindTex();
			ftex->FitTexViewPort();
			ShaderMan::UseShaderCopyKeypoint();
			ftex->DrawQuad();

			glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB,  _featurePointVBO[ idx]);
			glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
			esize = ftex->GetImgHeight() * ftex->GetImgWidth()*sizeof(float) *4;

			//increase size when necessary
			if(bsize < esize)	glBufferData(GL_PIXEL_PACK_BUFFER_ARB, esize*3/2 ,	NULL, GL_STATIC_DRAW_ARB);

			//read back if we have enough buffer
			glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
			if(bsize >= esize) glReadPixels(0, 0, ftex->GetImgWidth(), ftex->GetImgHeight(), GL_RGBA, GL_FLOAT, 0);
			else  glBufferData(GL_PIXEL_PACK_BUFFER_ARB, 0,	NULL, GL_STATIC_DRAW_ARB);

			glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
			
		}
	}
	glReadBuffer(GL_NONE);
	glFinish();

}





void PyramidNaive::GetFeatureOrienations()
{
	GLTexImage * gtex;
	GLTexImage * stex = NULL;
	GLTexImage * ftex = _featureTex;
	GLTexImage * otex = _orientationTex;
	int sid = 0; 
	int * count	 = _levelFeatureNum;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num);
	FrameBufferObject fbo;
	if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
	{
		GLenum buffers[] = { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
		glDrawBuffers(2, buffers);
	}else
	{
		glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	}
	for(int i = 0; i < _octave_num; i++)
	{
		gtex = GetLevelTexture(i+_octave_min, param._level_min + GlobalUtil::_GradientLevelOffset);
		if(GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)
			stex = GetBaseLevel(i+_octave_min, DATA_KEYPOINT) + 2;

		for(int j = 0; j < param._dog_level_num; j++, ftex++, otex++, count++, gtex++, stex++)
		{
			if(*count<=0)continue;

			sigma = param.GetLevelSigma(j+param._level_min+1);

			//
			ftex->FitTexViewPort();

			glActiveTexture(GL_TEXTURE0);
			ftex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			gtex->BindTex();
			//
			ftex->AttachToFBO(0);
			if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 ==0)
				otex->AttachToFBO(1);
			if(!_existing_keypoints && (GlobalUtil::_SubpixelLocalization|| GlobalUtil::_KeepExtremumSign))
			{
				glActiveTexture(GL_TEXTURE2);
				stex->BindTex();
				sid = * stex;
			}
			ShaderMan::UseShaderOrientation(gtex->GetTexID(),
				gtex->GetImgWidth(), gtex->GetImgHeight(),
				_existing_keypoints? 0 : sigma,  
				sid, sigma_step);
			ftex->DrawQuad();
	//		glFinish();
			
		}
	}

	GLTexImage::UnbindMultiTex(3);
	if(GlobalUtil::_timingS)glFinish();

	if(GlobalUtil::_MaxOrientation>1&& GlobalUtil::_OrientationPack2 ==0)	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);

}



//to compare with GPU feature list generation
void PyramidNaive::GenerateFeatureListCPU()
{

	FrameBufferObject fbo;
	_featureNum = 0;
	GLTexImage * tex = GetBaseLevel(_octave_min);
	float * mem = new float [tex->GetTexWidth()*tex->GetTexHeight()];
	vector<float> list;
	int idx = 0;
	for(int i = 0; i < _octave_num; i++)
	{
		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			tex = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + j + 2;
			tex->BindTex();
			glGetTexImage(GlobalUtil::_texTarget, 0, GL_RED, GL_FLOAT, mem);
			//tex->AttachToFBO(0);
			//tex->FitTexViewPort();
			//glReadPixels(0, 0, tex->GetTexWidth(), tex->GetTexHeight(), GL_RED, GL_FLOAT, mem);
			//
			//make a list of 
			list.resize(0);
			float * p = mem;
			int fcount = 0 ;
			for(int k = 0; k < tex->GetTexHeight(); k++)
			{
				for( int m = 0; m < tex->GetTexWidth(); m ++, p++)
				{
					if(*p==0)continue;
					if(m ==0 || k ==0 || k >= tex->GetImgHeight() -1 || m >= tex->GetImgWidth() -1 ) continue;
					list.push_back(m+0.5f);
					list.push_back(k+0.5f);
					list.push_back(0);
					list.push_back(1);
					fcount ++;


				}
			}
			if(fcount==0)continue;


			
			GLTexImage * ftex = _featureTex+idx;
			_levelFeatureNum[idx] = (fcount);
			SetLevelFeatureNum(idx, fcount);

			_featureNum += (fcount);


			int fw = ftex->GetImgWidth();
			int fh = ftex->GetImgHeight();

			list.resize(4*fh*fw);

			ftex->BindTex();
			ftex->AttachToFBO(0);
	//		glTexImage2D(GlobalUtil::_texTarget, 0, GlobalUtil::_iTexFormat, fw, fh, 0, GL_BGRA, GL_FLOAT, &list[0]);
			glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, fw, fh, GL_RGBA, GL_FLOAT, &list[0]);
			//
		}
	}
	GLTexImage::UnbindTex();
	delete mem;
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}
}


void SiftPyramid::ReshapeFeatureListCPU()
{
	//make a compact feature list
	//each wit only one orientation

	///download orientations list
	///download featue list
	//reshape it
	//upload it

	FrameBufferObject fbo;
	int i, szmax =0, sz;
	int n = param._dog_level_num*_octave_num;
	for( i = 0; i < n; i++)
	{
		sz = _featureTex[i].GetImgWidth() * _featureTex[i].GetImgHeight();
		if(sz > szmax ) szmax = sz;
	}
	float * buffer = new float[szmax*24];
	float * buffer1 = buffer;
	float * buffer2 = buffer + szmax*4;
	float * buffer3 = buffer + szmax*8;

	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);


	_featureNum = 0;

	for(i = 0; i < n; i++)
	{
		if(_levelFeatureNum[i]==0)continue;

		_featureTex[i].AttachToFBO(0);
		_featureTex[i].FitTexViewPort();
		glReadPixels(0, 0, _featureTex[i].GetImgWidth(), _featureTex[i].GetImgHeight(),GL_RGBA, GL_FLOAT, buffer1);
		
		int fcount =0, ocount;
		float * src = buffer1;
		float * orientation  = buffer2;
		float * des = buffer3;
		if(GlobalUtil::_OrientationPack2 == 0)
		{	
			//read back orientations from another texture
			_orientationTex[i].AttachToFBO(0);
			glReadPixels(0, 0, _orientationTex[i].GetImgWidth(), _orientationTex[i].GetImgHeight(),GL_RGBA, GL_FLOAT, buffer2);
			//make the feature list
			for(int j = 0; j < _levelFeatureNum[i]; j++, src+=4, orientation+=4)
			{
				if(_existing_keypoints) 
				{
					des[0] = src[0];
					des[1] = src[1];
					des[2] = orientation[0];
					des[3] = src[3];			
					fcount++;
					des += 4;
				}else
				{
					ocount = (int)src[2];
					for(int k = 0 ; k < ocount; k++, des+=4)
					{
						des[0] = src[0];
						des[1] = src[1];
						des[2] = orientation[k];
						des[3] = src[3];			
						fcount++;
					}
				}
			}
		}else
		{
			_featureTex[i].DetachFBO(0);
			const static double factor  = 2.0*3.14159265358979323846/65535.0;
			for(int j = 0; j < _levelFeatureNum[i]; j++, src+=4)
			{
				unsigned short * orientations = (unsigned short*) (&src[2]);
				if(_existing_keypoints) 
				{
					des[0] = src[0];
					des[1] = src[1];
					des[2] = float( factor* orientations[0]);
					des[3] = src[3];			
					fcount++;
					des += 4;
				}else
				{
					if(orientations[0] != 65535)
					{
						des[0] = src[0];
						des[1] = src[1];
						des[2] = float( factor* orientations[0]);
						des[3] = src[3];			
						fcount++;
						des += 4;

						if(orientations[1] != 65535)
						{
							des[0] = src[0];
							des[1] = src[1];
							des[2] = float(factor* orientations[1]);
							des[3] = src[3];			
							fcount++;
							des += 4;
						}
					}
				}
			}
		}
		//texture size
		SetLevelFeatureNum(i, fcount);
		//
		int nfw = _featureTex[i].GetImgWidth();
		int nfh = _featureTex[i].GetImgHeight();
		int sz = nfh * nfw;
		for(int u = fcount; u < sz; u++, des+=4)
		{
			des[0] = des[1] = des[2] = des[3] = 0;
		}
		//glDrawPixels or gltexSubImage2D
		//_featureTex[i].AttachToFBO(0);
		//_featureTex[i].FitTexViewPort();
		//glRasterPos2i(0,0);
		//glDrawPixels(nfw, nfh, GL_RGBA, GL_FLOAT, buffer3);
		//glFinish();

		_featureTex[i].BindTex();
		glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, nfw, nfh, GL_RGBA, GL_FLOAT, buffer3);
		_featureTex[i].UnbindTex();

		_levelFeatureNum[i] = fcount;
		_featureNum += fcount;
	}

	delete[] buffer;
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features MO:\t"<<_featureNum<<endl;
	}

}


inline void SiftPyramid::SetLevelFeatureNum(int idx, int fcount)
{
	int fw, fh;
	GLTexImage * ftex = _featureTex + idx;
	//set feature texture size. normally fh will be one
	GetTextureStorageSize(fcount, fw, fh);
	if(fcount >  ftex->GetTexWidth()*ftex->GetTexHeight())
	{
		if(GlobalUtil::_verbose)
			std::cout<<"Too many features, reallocate texture\n";

		ftex->InitTexture(fw, fh, 0);
		if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
		{
			_orientationTex[idx].InitTexture(fw, fh, 0);
		}

	}
	if(GlobalUtil::_NarrowFeatureTex)
		fh = fcount ==0? 0:(int)ceil(double(fcount)/fw);
	else
		fw = fcount ==0? 0:(int)ceil(double(fcount)/fh);
	ftex->SetImageSize(fw, fh);
	if(GlobalUtil::_MaxOrientation > 1 && GlobalUtil::_OrientationPack2 == 0)
		_orientationTex[idx].SetImageSize(fw, fh);
}

void PyramidNaive::GetSimplifiedOrientation()
{
	//
	int idx = 0;
//	int n = _octave_num  * param._dog_level_num;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num); 
	GLTexImage * ftex = _featureTex;

	FrameBufferObject fbo;
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	for(int i = 0; i < _octave_num; i++)
	{
		GLTexImage *gtex = GetLevelTexture(i+_octave_min, 2+param._level_min);
		for(int j = 0; j < param._dog_level_num; j++, ftex++,  gtex++, idx ++)
		{
			if(_levelFeatureNum[idx]<=0)continue;
			sigma = param.GetLevelSigma(j+param._level_min+1);
//			imdebugTexImagef(GlobalUtil::_texTarget, ftex->GetTexID(), GL_RGBA);
			//
			ftex->AttachToFBO(0);
			ftex->FitTexViewPort();

			glActiveTexture(GL_TEXTURE0);
			ftex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			gtex->BindTex();

			ShaderMan::UseShaderSimpleOrientation(gtex->GetTexID(), sigma, sigma_step);
			ftex->DrawQuad();
			
//			imdebugTexImagef(GlobalUtil::_texTarget, ftex->GetTexID(), GL_RGBA);
		}
	}

	GLTexImage::UnbindMultiTex(2);

}

void SiftPyramid::GetFeatureDescriptors()
{
	//descriptors...
	float sigma;
	int idx, i, j, k,   w, h;
	int ndf = 32 / GlobalUtil::_DescriptorPPT / GlobalUtil::_DescriptorVPC; //number of textures

	float* pd =  &_descriptor_buffer[0];
	float* pbuf  = NULL, *ppd;
	vector<float>read_buffer, descriptor_buffer2;

	//use another buffer, if we need to re-order the descriptors
	if(_keypoint_index.size() > 0)
	{
		descriptor_buffer2.resize(_descriptor_buffer.size());
		pd = &descriptor_buffer2[0];
	}
	FrameBufferObject fbo;

	GLTexImage * gtex, *otex, * ftex;
	GLenum buffers[8] = { 
		GL_COLOR_ATTACHMENT0_EXT,		GL_COLOR_ATTACHMENT1_EXT ,
		GL_COLOR_ATTACHMENT2_EXT,		GL_COLOR_ATTACHMENT3_EXT ,
		GL_COLOR_ATTACHMENT4_EXT,		GL_COLOR_ATTACHMENT5_EXT ,
		GL_COLOR_ATTACHMENT6_EXT,		GL_COLOR_ATTACHMENT7_EXT ,
	};
	ShaderMan::UnloadProgram();

	glDrawBuffers(ndf, buffers);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);


	for( i = 0, idx = 0, ftex = _featureTex; i < _octave_num; i++)
	{
		gtex = GetBaseLevel(i + _octave_min, DATA_GRAD) + GlobalUtil::_GradientLevelOffset;
		otex = GetBaseLevel(i + _octave_min, DATA_ROT)  + GlobalUtil::_GradientLevelOffset;
		for( j = 0; j < param._dog_level_num; j++, ftex++, idx++, gtex++, otex++)
		{
			if(_levelFeatureNum[idx]==0)continue;

			sigma = param.GetLevelSigma(j+param._level_min+1);



			if(GlobalUtil::_DescriptorPPT ==1)
			{
				w = ftex->GetImgWidth();
				h = ftex->GetImgHeight();
			}else if(GlobalUtil::_DescriptorPPT == 16)
			{
				int count = _levelFeatureNum[idx] * 8;
				GetAlignedStorageSize(count, 8, w, h);
				h = (int)ceil(double(count) / w);
				h *= 2;
			}
			else
			{
				int count = _levelFeatureNum[idx]*GlobalUtil::_DescriptorPPT;
				GetAlignedStorageSize( count, _alignment, w, h);
				h = (int)ceil(double(count) / w);
			}
			//not enought space for holding the descriptor data
			if(w > _descriptorTex[0].GetTexWidth() || h > _descriptorTex[0].GetTexHeight())
			{
				for(k = 0; k < ndf; k++)_descriptorTex[k].InitTexture(w, h);
			}
			for(k = 0; k < ndf; k++)
				_descriptorTex[k].AttachToFBO(k);
			GlobalUtil::FitViewPort(w, h);
			glActiveTexture(GL_TEXTURE0);
			ftex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			gtex->BindTex();
			if(otex!=gtex)
			{
				glActiveTexture(GL_TEXTURE2);
				otex->BindTex();
			}

			ShaderMan::UseShaderDescriptor(gtex->GetTexID(), otex->GetTexID(), 
				w, ftex->GetImgWidth(), gtex->GetImgWidth(), gtex->GetImgHeight(), sigma);
			if(GlobalUtil::_DescriptorPPT ==32)
			{
				//glClear(GL_COLOR_BUFFER_BIT);
				//drawing point vertices when using geometry shader 
				DrawVertexBuffer(_levelFeatureNum[idx]);
			}else
			{
				GLTexImage::DrawQuad(0, (float)w, 0, (float)h);
			}

			//download descriptor
			if(ndf ==1)
			{
				glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
				//WHEN geometry shader is used...simply read back
				glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, pd);
				pd += 128*_levelFeatureNum[idx];
			}else if(GlobalUtil::_DescriptorPPT == 1)
			{
				//GlobalUtil::_DescriptorVPC == 4;
				int step = w * h * 4;
				read_buffer.resize(step * ndf);
				pbuf = (&read_buffer[0]);
				for(k = 0; k < ndf; k++)
				{
					glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + k);
					glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, pbuf + k * step);
				}
				//

				for(int v = 0; v < ndf; v++, pbuf += step)
				{
					unsigned char * ubuf = (unsigned char*) pbuf;
					float * pdx = pd + v * 16;
					for(k = 0; k < _levelFeatureNum[idx]; k++, pdx += 128)
					{
						for(int u = 0; u < 16; ) pdx[u++] = *ubuf++ /512.0f;
					}
				}
				pd += 128*_levelFeatureNum[idx];
			}else //read back float format descriptors and do normalization on CPU
			{
				int step = w*h*4;
				if((unsigned int)step*ndf > read_buffer.size())
				{
					read_buffer.resize(ndf*step);
				}
				pbuf = &read_buffer[0];
				
				//read back
				for(k = 0; k < ndf; k++, pbuf+=step)
				{
					glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + k);
					glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, pbuf);
				}
				GlobalUtil::CheckErrorsGL();
				pbuf = &read_buffer[0];
				ppd = pd;

				//interlace
				if(ndf ==4)
				{
					int np = GlobalUtil::_DescriptorPPT * _levelFeatureNum[idx];
					float * pp;
					for(k = 0; k < 4; k++)
					{
						pp = pbuf + k * step;
						ppd = pd + k * 4;
						for(int v = 0; v < np; v++)
						{
							ppd[0] = pp[0];
							ppd[1] = pp[1];
							ppd[2] = pp[2];
							ppd[3] = pp[3];
							ppd += 16;
							pp+= 4;
						}
					}
				}else if(ndf ==8)
				{
					float * pp;
					static const int offset[8] = {0, 1, 2, 3, 8, 9, 10, 11};
					static const int jumps[4]= {16, 48, 16, 48};
					int np =  4 * _levelFeatureNum[idx];
					for(k = 0; k < 8; k++)
					{	
						pp = pbuf + k * step;
						ppd = pd + offset[k] * 4;
						for(int v = 0; v < np; v++)
						{
							ppd[0] = pp[0];
							ppd[1] = pp[1];
							ppd[2] = pp[2];
							ppd[3] = pp[3];
							ppd += jumps[v%4];
							pp += 4;
						}
					}

				}else if(ndf ==2)
				{
					w /= 8; h /= 2;
					float * pp;
					for(k = 0; k < 2; k++)
					{
						pp = pbuf + k * step;
						ppd = pd + k * 4;
						for(int u = 0; u < h ; u++)
						{
							int v; 
							for(v= 0; v < w; v++)
							{
								for(int t = 0; t < 8; t++)
								{
									ppd[0] = pp[0];
									ppd[1] = pp[1];
									ppd[2] = pp[2];
									ppd[3] = pp[3];
									ppd += 8;
									pp+= 4;
								}
								ppd += 64;
							}
							ppd += ( 64 - 128 * w );
							for(v= 0; v < w; v++)
							{
								for(int t = 0; t < 8; t++)
								{
									ppd[0] = pp[0];
									ppd[1] = pp[1];
									ppd[2] = pp[2];
									ppd[3] = pp[3];
									ppd += 8;
									pp+= 4;
								}
								ppd += 64;
							}
							ppd -=64;
						}
					}

				}
				//need to do normalization
		
				if(GlobalUtil::_NormalizedSIFT)
				{
					ppd = pd;
					int debug_descriptor = 1;
					for(k = 0; k < _levelFeatureNum[idx]; k++)
					{
						float sq = 0;
						int v;
						//normalize
						for( v = 0; v < 128; v++, ppd++)	sq += (*ppd)*(*ppd);
						sq = 1.0f / sqrtf(sq);
						ppd -= 128;
						//truncate to .2
						for(v = 0; v < 128; v ++, ppd++)	*ppd = min(*ppd*sq, 0.2f);
						ppd -= 128;
						//renormalize
						sq = 0;
						for( v = 0; v < 128; v++, ppd++)	sq += (*ppd)*(*ppd);
						sq = 1.0f / sqrtf(sq);
						ppd -= 128;
						for(v = 0; v < 128; v ++, ppd++)	*ppd = *ppd*sq;
					}
				}
				pd += 128*_levelFeatureNum[idx];
			}
			glReadBuffer(GL_NONE);
			GlobalUtil::CheckErrorsGL();

		}
	}
	GLTexImage::UnbindMultiTex(3); 
	glDrawBuffer(GL_NONE);

	ShaderMan::UnloadProgram();
	if(GlobalUtil::_timingS)glFinish();
	for(i = 0; i < ndf; i++)
	{
		fbo.UnattachTex(GL_COLOR_ATTACHMENT0_EXT +i);
	}


	//finally, put the descriptor back to their original order for existing keypoint list.
	if(_keypoint_index.size() > 0)
	{
		for(i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_descriptor_buffer[index*128], &descriptor_buffer2[i*128], 128 * sizeof(float));
		}
	}
}


void SiftPyramid::PrepareBuffer()
{
	//int align = max(16, GlobalUtil::_FeatureTexBlock);

	//when there is no existing keypoint list, the feature list need to be downloaded
	//when an existing keypoint list does not have orientaiton, we need to download them again.
	if(!(_existing_keypoints & SIFT_SKIP_ORIENTATION)) 
		//_keypoint_buffer.resize(4 * (_featureNum +align));
		_keypoint_buffer.resize(4 * (_featureNum + GlobalUtil::_texMaxDim)); //11/19/2008
	if(GlobalUtil::_DescriptorPPT)
	{
		//_descriptor_buffer.resize(128*(_featureNum + align)); //11/19/2008
		_descriptor_buffer.resize(128 * _featureNum + 16 * GlobalUtil::_texMaxDim);
	}

}

void SiftPyramid::DownloadKeypoints()
{
	const double twopi = 2.0*3.14159265358979323846;
	int idx = 0;
	float * buffer = &_keypoint_buffer[0];
	vector<float> keypoint_buffer2;
	//use a different keypoint buffer when processing with an exisint features list
	//without orientation information. 
	if(_keypoint_index.size() > 0)
	{
		keypoint_buffer2.resize(_keypoint_buffer.size());
		buffer = &keypoint_buffer2[0];
	}
	float * p = buffer, *ps, sigma;
	GLTexImage * ftex = _featureTex;
	FrameBufferObject fbo;
	ftex->FitRealTexViewPort();
	/////////////////////
	float os = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	if(_down_sample_factor>0) os *= float(1<<_down_sample_factor); 
	float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f;
	/////////////////////
	for(int i = 0; i < _octave_num; i++, os *= 2.0f)
	{
		
		for(int j = 0; j  < param._dog_level_num; j++, idx++, ftex++)
		{

			if(_levelFeatureNum[idx]>0)
			{	
				ftex->AttachToFBO(0);
				glReadPixels(0, 0, ftex->GetImgWidth(), ftex->GetImgHeight(),GL_RGBA, GL_FLOAT, p);
				ps = p;
				for(int k = 0;  k < _levelFeatureNum[idx]; k++, ps+=4)
				{
					ps[0] = os*(ps[0]-0.5f) + offset;	//x
					ps[1] = os*(ps[1]-0.5f) + offset;	//y
					sigma = os*ps[3]; 
					ps[3] = (float)fmod(twopi-ps[2], twopi);	//orientation, mirrored
					ps[2] = sigma;  //scale
				}
				p+= 4* _levelFeatureNum[idx];
			}
		}
	}

	//put the feature into their original order

	if(_keypoint_index.size() > 0)
	{
		for(int i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_keypoint_buffer[index*4], &keypoint_buffer2[i*4], 4 * sizeof(float));
		}
	}
}

void SiftPyramid::CopyFeatureVector(float*keys, float *descriptors)
{
	if(keys)		memcpy(keys, &_keypoint_buffer[0], 4*_featureNum*sizeof(float));
	if(descriptors)	memcpy(descriptors, &_descriptor_buffer[0], 128*_featureNum*sizeof(float));
}

void SiftPyramid:: SetKeypointList(int num, const float * keys, int run_on_current, int skip_orientation)
{
	//for each input keypoint
	//sort the key point list by size, and assign them to corresponding levels
	if(num <=0) return;
	_featureNum = num;
	///copy the keypoints
	_keypoint_buffer.resize(num * 4);
	memcpy(&_keypoint_buffer[0], keys, 4 * num * sizeof(float));
	//location and scale can be skipped
	_existing_keypoints = SIFT_SKIP_DETECTION;
	//filtering is skipped if it is running on the same image
	if(run_on_current) _existing_keypoints |= SIFT_SKIP_FILTERING;
	//orientation can be skipped if specified
	if(skip_orientation) _existing_keypoints |= SIFT_SKIP_ORIENTATION;
}


void SiftPyramid::GenerateFeatureListTex()
{
	//generate feature list texture from existing keypoints
	//do feature sorting in the same time?

	FrameBufferObject fbo;
	vector<float> list;
	int idx = 0;
	const double twopi = 2.0*3.14159265358979323846;
	float sigma_half_step = powf(2.0f, 0.5f / param._dog_level_num);
	float octave_sigma = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f; 
	if(_down_sample_factor>0) octave_sigma *= float(1<<_down_sample_factor); 

	_keypoint_index.resize(0); // should already be 0
	for(int i = 0; i < _octave_num; i++, octave_sigma*= 2.0f)
	{
		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			list.resize(0);
			float level_sigma = param.GetLevelSigma(j + param._level_min + 1) * octave_sigma;
			float sigma_min = level_sigma / sigma_half_step;
			float sigma_max = level_sigma * sigma_half_step;
			int fcount = 0 ;
			for(int k = 0; k < _featureNum; k++)
			{
				float * key = &_keypoint_buffer[k*4];
				if(   (key[2] >= sigma_min && key[2] < sigma_max)
					||(key[2] < sigma_min && i ==0 && j == 0)
					||(key[2] > sigma_max && i == _octave_num -1 && j == param._dog_level_num - 1))
				{
					//add this keypoint to the list
					list.push_back((key[0] - offset) / octave_sigma + 0.5f);
					list.push_back((key[1] - offset) / octave_sigma + 0.5f);
					list.push_back((float)fmod(twopi-key[3], twopi));
					list.push_back(key[2] / octave_sigma);
					fcount ++;
					//save the index of keypoints
					_keypoint_index.push_back(k);
				}

			}

			_levelFeatureNum[idx] = fcount;
			if(fcount==0)continue;
			GLTexImage * ftex = _featureTex+idx;

			SetLevelFeatureNum(idx, fcount);

			int fw = ftex->GetImgWidth();
			int fh = ftex->GetImgHeight();

			list.resize(4*fh*fw);

			ftex->BindTex();
			ftex->AttachToFBO(0);
			glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, fw, fh, GL_RGBA, GL_FLOAT, &list[0]);
		}
	}
	GLTexImage::UnbindTex();
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}
}

void SiftPyramid::SaveSIFT(std::ostream & out)
{
	if (_featureNum <=0) return;
	float * pk = &_keypoint_buffer[0];

	if(GlobalUtil::_BinarySIFT)
	{
	
		out.write((char* )(&_featureNum), sizeof(int));

		if(GlobalUtil::_DescriptorPPT)
		{
			int dim = 128;
			out.write((char* )(&dim), sizeof(int));
			float * pd = &_descriptor_buffer[0] ;
			for(int i = 0; i < _featureNum; i++, pk+=4, pd +=128)
			{
				out.write((char* )(pk +1), sizeof(float));
				out.write((char* )(pk), sizeof(float));
				out.write((char* )(pk+2), 2 * sizeof(float));
				out.write((char* )(pd), 128 * sizeof(float));
			}
		}else
		{
			int dim = 0;
			out.write((char* )(&dim), sizeof(int));
			for(int i = 0; i < _featureNum; i++, pk+=4)
			{
				out.write((char* )(pk +1), sizeof(float));
				out.write((char* )(pk), sizeof(float));
				out.write((char* )(pk+2), 2 * sizeof(float));
			}
		}
	}else
	{
		out.flags(ios::fixed);

		if(GlobalUtil::_DescriptorPPT)
		{
			float * pd = &_descriptor_buffer[0] ;
			out<<_featureNum<<" 128"<<endl;

			for(int i = 0; i < _featureNum; i++)
			{
				//in y, x, scale, orientation order
				out<<setprecision(2) << pk[1]<<" "<<setprecision(2) << pk[0]<<" "
					<<setprecision(3) << pk[2]<<" " <<setprecision(3) <<  pk[3]<< endl; 

				////out << setprecision(12) << pk[1] <<  " " << pk[0] << " " << pk[2] << " " << pk[3] << endl;
				pk+=4;
				for(int k = 0; k < 128; k ++, pd++) 
				{
					if(GlobalUtil::_NormalizedSIFT)
						out<< ((unsigned int)floor(0.5+512.0f*(*pd)))<<" ";
					else
						out << setprecision(8) << pd[0] << " ";

					if ( (k+1)%20 == 0 ) out<<endl; //suggested by Martin Schneider

				}
				out<<endl;

			}
		
		}else
		{
			out<<_featureNum<<" 0"<<endl;
			for(int i = 0; i < _featureNum; i++, pk+=4)
			{
				out<<pk[1]<<" "<<pk[0]<<" "<<pk[2]<<" " << pk[3]<<endl;
			}
		}
	}
}

#ifdef DEBUG_SIFTGPU
void SiftPyramid::BeginDEBUG(const char *imagepath)
{
	if(imagepath && imagepath[0])
	{
		strcpy(_debug_path, imagepath);
		strcat(_debug_path, ".debug");
	}else
	{
		strcpy(_debug_path, ".debug");
	}

	mkdir(_debug_path);
	chmod(_debug_path, _S_IREAD | _S_IWRITE);
}

void SiftPyramid::StopDEBUG()
{
	_debug_path[0] = 0;
}


void SiftPyramid::WriteTextureForDEBUG(GLTexImage * tex, const char *namet, ...)
{
	char name[_MAX_PATH];
	char * p = name, * ps = _debug_path;
	while(*ps) *p++ = *ps ++;
	*p++ = '/';
	va_list marker;
	va_start(marker, namet);
	vsprintf(p, namet, marker);
	va_end(marker);
	unsigned int imID;
	int width = tex->GetImgWidth();
	int height = tex->GetImgHeight();
	float* buffer1 = new float[ width * height  * 4];
	float* buffer2 = new float[ width * height  * 4];

	//read data back
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	tex->AttachToFBO(0);
	tex->FitTexViewPort();
	glReadPixels(0, 0, width, height, GL_RGBA , GL_FLOAT, buffer1);

	//Tiffs saved with IL are flipped
	for(int i = 0; i < height; i++)
	{
		memcpy(buffer2 + i * width * 4, 
			buffer1 + (height - i - 1) * width * 4,  
			width * 4 * sizeof(float));
	}

	//save data as floating point tiff file
	ilGenImages(1, &imID);
	ilBindImage(imID); 
	ilEnable(IL_FILE_OVERWRITE);
	ilTexImage(width, height, 1, 4, IL_RGBA, IL_FLOAT, buffer2);
	ilSave(IL_TIF, name); 
	ilDeleteImages(1, &imID); 

	delete buffer1;
	delete buffer2;
	glReadBuffer(GL_NONE);
}


#endif

PyramidPacked::PyramidPacked(SiftParam& sp): SiftPyramid(sp)
{
	_allPyramid = NULL;
}

PyramidPacked::~PyramidPacked()
{
	DestroyPyramidData();
}


//build the gaussian pyrmaid

void PyramidPacked::BuildPyramid(GLTexImage * input)
{
	//
	USE_TIMING();
	int i, j, k;
	GLTexImage * tex, *tmp;
	FilterProgram ** filter;
	FrameBufferObject fbo;

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	input->FitTexViewPort();

	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{

		tex = GetBaseLevel(i);
		tmp = GetBaseLevel(i, DATA_DOG) + 2; //use this as a temperory texture

		j = param._level_min + 1;
		filter = ShaderMan::f_gaussian_step;

		OCTAVE_START();

		if( i == _octave_min )
		{
			if(i < 0)
			{
				TextureUpSample(tex, input, 1<<(-i-1));			
			}else
			{
				//image might have been already down-sampled by cpu code
				TextureDownSample(tex, input, 1<<(i+1));
			}
			//
			if(ShaderMan::f_gaussian_skip0)
			{
				ShaderMan::f_gaussian_skip0->RunFilter(tex, tex, tmp);
			}
			//tex->FillMargin(0, 1);	
			LEVEL_FINISH();
		}else
		{
			TextureDownSample(tex, GetLevelTexture(i-1, param._level_ds)); 
	
			LEVEL_FINISH();

			if(ShaderMan::f_gaussian_skip1)
			{
				ShaderMan::f_gaussian_skip1->RunFilter(tex, tex, tmp);
				//tex->FillMargin(0, 1);
				LEVEL_FINISH();
			}else if (GlobalUtil::_MaxFilterWidth == -1 && GlobalUtil::_ExtraDownSample)
			{
				//copy and downsample other extra levels...
				//according to definition, we can down sample more than one level
				for(k = param._level_ds +1 ;k <= param._level_max; k++, tex++, j++, filter++)
				{
					TextureDownSample(tex+1, GetLevelTexture(i-1, k));
					//(tex+1)->FillMargin(1, 1);
					LEVEL_FINISH();
				}
				//this won't work well if the gussian kernel size is truncated for large size
			}
		}
		

		for( ; j <=  param._level_max ; j++, tex++, filter++)
		{
			// filtering
			(*filter)->RunFilter(tex+1, tex, tmp);
			//(tex+1)->FillMargin(0, 1);
			LEVEL_FINISH();


		}
		//tex->FillMargin(1, 0);
		OCTAVE_FINISH();

	}
	if(GlobalUtil::_timingS)	glFinish();
	UnloadProgram();	
}

void PyramidPacked::ComputeGradient()
{
	
	//first pass, compute dog, gradient, orientation
	GLenum buffers[4] = { 
		GL_COLOR_ATTACHMENT0_EXT,		GL_COLOR_ATTACHMENT1_EXT ,
		GL_COLOR_ATTACHMENT2_EXT,		GL_COLOR_ATTACHMENT3_EXT
	};

	int i, j;
	double ts, t1;
	FrameBufferObject fbo;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = clock();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		GLTexImage * gus = GetBaseLevel(i) +  GlobalUtil::_GradientLevelOffset;
		GLTexImage * dog = GetBaseLevel(i, DATA_DOG) +  GlobalUtil::_GradientLevelOffset;
		GLTexImage * grd = GetBaseLevel(i, DATA_GRAD) +  GlobalUtil::_GradientLevelOffset;
		GLTexImage * rot = GetBaseLevel(i, DATA_ROT) +  GlobalUtil::_GradientLevelOffset;
		glDrawBuffers(3, buffers);
		gus->FitTexViewPort();
		//compute the gradient
		for(j = 0; j <  param._dog_level_num ; j++, gus++, dog++, grd++, rot++)
		{
			//gradient, dog, orientation
			glActiveTexture(GL_TEXTURE0);
			gus->BindTex();
			glActiveTexture(GL_TEXTURE1);
			(gus-1)->BindTex();
			//output
			dog->AttachToFBO(0);
			grd->AttachToFBO(1);
			rot->AttachToFBO(2);
			ShaderMan::UseShaderGradientPass((gus-1)->GetTexID());
			//compuate
			dog->DrawQuadMT4();
		}
	}
	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		glFinish();
		t1 = clock();
		std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)/CLOCKS_PER_SEC<<"\n";
	}
	GLTexImage::DetachFBO(1);
	GLTexImage::DetachFBO(2);
	UnloadProgram();
	GLTexImage::UnbindMultiTex(3);
	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);
}

void PyramidPacked::DetectKeypointsEX()
{

	//first pass, compute dog, gradient, orientation
	GLenum buffers[4] = { 
		GL_COLOR_ATTACHMENT0_EXT,		GL_COLOR_ATTACHMENT1_EXT ,
		GL_COLOR_ATTACHMENT2_EXT,		GL_COLOR_ATTACHMENT3_EXT
	};

	int i, j;
	double t0, t, ts, t1, t2;
	FrameBufferObject fbo;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = clock();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		GLTexImage * gus = GetBaseLevel(i) + 1;
		GLTexImage * dog = GetBaseLevel(i, DATA_DOG) + 1;
		GLTexImage * grd = GetBaseLevel(i, DATA_GRAD) + 1;
		GLTexImage * rot = GetBaseLevel(i, DATA_ROT) + 1;
		glDrawBuffers(3, buffers);
		gus->FitTexViewPort();
		//compute the gradient
		for(j = param._level_min +1; j <=  param._level_max ; j++, gus++, dog++, grd++, rot++)
		{
			//gradient, dog, orientation
			glActiveTexture(GL_TEXTURE0);
			gus->BindTex();
			glActiveTexture(GL_TEXTURE1);
			(gus-1)->BindTex();
			//output
			dog->AttachToFBO(0);
			grd->AttachToFBO(1);
			rot->AttachToFBO(2);
			ShaderMan::UseShaderGradientPass((gus-1)->GetTexID());
			//compuate
			dog->DrawQuadMT4();
		}
	}
	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		glFinish();
		t1 = clock();
	}
	GLTexImage::DetachFBO(1);
	GLTexImage::DetachFBO(2);
	glDrawBuffers(1, buffers);
	

	GlobalUtil::CheckErrorsGL();

	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		if(GlobalUtil::_timingO)
		{
			t0 = clock();
			std::cout<<"#"<<(i + _down_sample_factor)<<"\t";
		}
		GLTexImage * dog = GetBaseLevel(i, DATA_DOG) + 2;
		GLTexImage * key = GetBaseLevel(i, DATA_KEYPOINT) +2;
		key->FitTexViewPort();

		for( j = param._level_min +2; j <  param._level_max ; j++, dog++, key++)
		{
			if(GlobalUtil::_timingL)t = clock();		
			key->AttachToFBO(0);
			glActiveTexture(GL_TEXTURE0);
			dog->BindTex();
			glActiveTexture(GL_TEXTURE1);
			(dog+1)->BindTex();
			glActiveTexture(GL_TEXTURE2);
			(dog-1)->BindTex();
			ShaderMan::UseShaderKeypoint((dog+1)->GetTexID(), (dog-1)->GetTexID());
			key->DrawQuadMT8();
			if(GlobalUtil::_timingL)
			{
				glFinish();
				std::cout<<(clock()-t)/CLOCKS_PER_SEC<<"\t";
			}
		}
		if(GlobalUtil::_timingO)
		{
			glFinish();
			std::cout<<"|\t"<<(clock()-t0)/CLOCKS_PER_SEC<<"\n";
		}
	}

	if(GlobalUtil::_timingS)
	{
		glFinish();
		if(GlobalUtil::_verbose) 
		{	
			t2 = clock();
			std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)/CLOCKS_PER_SEC<<"\n"
						<<"<Get Keypoints  >\t"<<(t2-t1)/CLOCKS_PER_SEC<<"\n";
		}
						
	}
	UnloadProgram();
	GLTexImage::UnbindMultiTex(3);
	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);
}

void PyramidPacked::GenerateFeatureList()
{
	//generate the histogram0pyramid
	FrameBufferObject fbo;
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	GLTexImage * htex, * ftex, * tex;
	double t1, t2, t3, ot1, ot2, ts1 = 0, ts2 = 0; 
	int ocount= 0, idx = 0;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 
	int hist_skip_gpu = GlobalUtil::_ListGenSkipGPU; 
	_featureNum = 0;

	FitHistogramPyramid();

	for(int i = 0; i < _octave_num; i++)
	{
		tex = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + 2;
		//output

		if(GlobalUtil::_timingO)
		{
			ot1 = ot2 = 0; 
			ocount = 0;
			std::cout<<"#"<<i+_octave_min + _down_sample_factor<<":\t";
		}
		for(int j = 0; j < param._dog_level_num; j++, tex++, idx++)
		{
			float fcount = 0.0f; 
			htex = _histoPyramidTex + hist_level_num - 1 - i;
			ftex = _featureTex + idx;
			if(GlobalUtil::_timingL) t1= clock();
			//fill zero to an extra row/col if the height/width is odd
			glActiveTexture(GL_TEXTURE0);
			tex->BindTex();
			htex->AttachToFBO(0);
			int tight = (htex->GetImgWidth() * 4 == tex->GetImgWidth() -1 && htex->GetImgHeight() *4 == tex->GetImgHeight()-1 );
			ShaderMan::UseShaderGenListInit(tex->GetImgWidth(), tex->GetImgHeight(), tight);
			htex->FitTexViewPort();
			//this uses the fact that no feature is on the edge.
			htex->DrawQuadReduction();
			//reduction..
			htex--;
	
			//this part might have problems on several GPUS
			//because the output of one pass is the input of the next pass
			//need to call glFinish to make it right
			//but too much glFinish makes it slow
			for(int k = 0; k <hist_level_num - i-1 - hist_skip_gpu; k++, htex--)
			{
				htex->AttachToFBO(0);
				htex->FitTexViewPort();
				(htex+1)->BindTex();
				ShaderMan::UseShaderGenListHisto();
				htex->DrawQuadReduction();		
			}
			//
			if(GlobalUtil::_timingL)	t2= clock();

			if(hist_skip_gpu == 0)
			{		
				//read back one pixel
				float fn[4];
				glReadPixels(0, 0, 1, 1, GL_RGBA , GL_FLOAT, fn);
				fcount = (fn[0] + fn[1] + fn[2] + fn[3]);
				if(fcount < 1) fcount = 0;

				_levelFeatureNum[ idx] = (int)(fcount);
				SetLevelFeatureNum(idx, (int)fcount);

				//save  number of features
				ocount+=int(fcount);
				_featureNum += int(fcount);

				//
				if(fcount < 1.0) 
				{
					ot1 += (t2 - t1); 
					if(GlobalUtil::_timingL) std::cout<<"0\t";
					continue;
				}
			

				///generate the feature texture

				htex=  _histoPyramidTex;

				htex->BindTex();

				//first pass
				ftex->AttachToFBO(0);
				if(GlobalUtil::_MaxOrientation>1)
				{
					//this is very important...
					ftex->FitRealTexViewPort();
					glClear(GL_COLOR_BUFFER_BIT);
					glFinish();
				}else
				{	
					ftex->FitTexViewPort();
				}


				ShaderMan::UseShaderGenListStart((float)ftex->GetImgWidth(), htex->GetTexID());

				ftex->DrawQuad();
				//make sure it finishes before the next step
				ftex->DetachFBO(0);
				//pass on each pyramid level
				htex++;
			}else
			{

				int tw = htex[1].GetDrawWidth(), th = htex[1].GetDrawHeight();
				int fc = 0;
				glReadPixels(0, 0, tw, th, GL_RGBA , GL_FLOAT, _histo_buffer);	
				_keypoint_buffer.resize(0);
				for(int y = 0, pos = 0; y < th; y++)
				{
					for(int x= 0; x < tw; x++)
					{
						for(int c = 0; c < 4; c++, pos++)
						{
							int ss =  (int) _histo_buffer[pos]; 
							if(ss == 0) continue;
							float ft[4] = {2 * x + (c%2? 1.5f:  0.5f), 2 * y + (c>=2? 1.5f: 0.5f), 0, 1 };
							for(int t = 0; t < ss; t++)
							{
								ft[2] = (float) t; 
								_keypoint_buffer.insert(_keypoint_buffer.end(), ft, ft+4);
							}
							fc += (int)ss; 
						}
					}
				}
				_levelFeatureNum[ idx] = fc;
				SetLevelFeatureNum(idx, fc);
				if(fc == 0) 
				{
					ot1 += (t2 - t1); 
					if(GlobalUtil::_timingL) std::cout<<"0\t";
					continue;
				}
				fcount = (float) fc; 	
				ocount += fc;	
				_featureNum += fc;
				/////////////////////
				ftex->AttachToFBO(0);
				if(GlobalUtil::_MaxOrientation>1)
				{
					ftex->FitRealTexViewPort();
					glClear(GL_COLOR_BUFFER_BIT);
				}else
				{					
					ftex->FitTexViewPort();
				}
				_keypoint_buffer.resize(ftex->GetDrawWidth() * ftex->GetDrawHeight()*4, 0);
				///////////
				glActiveTexture(GL_TEXTURE0);
				ftex->BindTex();
				glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, ftex->GetDrawWidth(),
					ftex->GetDrawHeight(), GL_RGBA, GL_FLOAT, &_keypoint_buffer[0]);
				htex += 2; 
			}

			for(int lev = 1 + hist_skip_gpu; lev < hist_level_num  - i; lev++, htex++)
			{
				glActiveTexture(GL_TEXTURE0);
				ftex->BindTex();
				ftex->AttachToFBO(0);
				glActiveTexture(GL_TEXTURE1);
				htex->BindTex();
				ShaderMan::UseShaderGenListStep(ftex->GetTexID(), htex->GetTexID());
				ftex->DrawQuad();
				ftex->DetachFBO(0);	
			}

			ftex->AttachToFBO(0);
			glActiveTexture(GL_TEXTURE1);
			tex->BindTex();
			ShaderMan::UseShaderGenListEnd(tex->GetTexID());
			ftex->DrawQuad();

			if(GlobalUtil::_timingL)
			{
				glFinish();
				t3 = clock();
				ot1 += (t2 - t1); ot2 += ( t3 - t2);
				std::cout<<int(fcount)<<"\t";
			}
			GLTexImage::UnbindMultiTex(2);


		}
		if(GlobalUtil::_timingO)
		{	
			ot1 /= CLOCKS_PER_SEC; ot2/= CLOCKS_PER_SEC;
			ts1 += ot1; ts2 += ot2; 
			std::cout << "| \t" << int(ocount) << " :\t(" << ot1 <<",\t" << ot2 << ")\n";
		}
	}
	if(GlobalUtil::_timingS)glFinish();
	if(GlobalUtil::_verbose)
	{
		//std::cout<<"("<<(ts1)/CLOCKS_PER_SEC<<",\t"<<(ts2)/CLOCKS_PER_SEC<<")\t";
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}

}


void PyramidPacked::GenerateFeatureListV2()
{
}


void PyramidPacked::GenerateFeatureListCPU()
{
	FrameBufferObject fbo;
	_featureNum = 0;
	GLTexImage * tex = GetBaseLevel(_octave_min);
	float * mem = new float [tex->GetTexWidth()*tex->GetTexHeight()*4];
	vector<float> list;
	int idx = 0;
	for(int i = 0; i < _octave_num; i++)
	{
		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			tex = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + j + 2;
			tex->BindTex();
			glGetTexImage(GlobalUtil::_texTarget, 0, GL_RGBA, GL_FLOAT, mem);
			//tex->AttachToFBO(0);
			//tex->FitTexViewPort();
			//glReadPixels(0, 0, tex->GetTexWidth(), tex->GetTexHeight(), GL_RED, GL_FLOAT, mem);
			//
			//make a list of 
			list.resize(0);
			float *pl = mem;
			int fcount = 0 ;
			for(int k = 0; k < tex->GetDrawHeight(); k++)
			{
				float * p = pl; 
				pl += tex->GetTexWidth() * 4;
				for( int m = 0; m < tex->GetDrawWidth(); m ++, p+=4)
				{
				//	if(m ==0 || k ==0 || k == tex->GetDrawHeight() -1 || m == tex->GetDrawWidth() -1) continue;
				//	if(*p == 0) continue;
					unsigned char * keys = (unsigned char*) p;
					for(int t = 0; t < 4; t++)
					{
						if(keys[t] > 0)
						{
							int xx = m + m + ( (t %2)? 1 : 0);
							int yy = k + k + ( (t <2)? 0 : 1);
							if(xx ==0 || yy == 0) 
							{
								break;
							}
							if(xx >= tex->GetImgWidth() - 1 || yy >= tex->GetImgHeight() - 1)
							{
								break;
							}
							list.push_back(xx + 0.5f + p[1]);
							list.push_back(yy + 0.5f + p[2]);
							list.push_back(GlobalUtil::_KeepExtremumSign && keys[t] < 0.6f ? -1.0f : 1.0f);
							list.push_back(p[3]);
							fcount ++;
							break;
						}
					}
				}
			}
			if(fcount==0)continue;

			if(GlobalUtil::_timingL) std::cout<<fcount<<".";
			
			GLTexImage * ftex = _featureTex+idx;
			_levelFeatureNum[idx] = (fcount);
			SetLevelFeatureNum(idx, fcount);

			_featureNum += (fcount);


			int fw = ftex->GetImgWidth();
			int fh = ftex->GetImgHeight();

			list.resize(4*fh*fw);

			ftex->BindTex();
			ftex->AttachToFBO(0);
	//		glTexImage2D(GlobalUtil::_texTarget, 0, GlobalUtil::_iTexFormat, fw, fh, 0, GL_BGRA, GL_FLOAT, &list[0]);
			glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, fw, fh, GL_RGBA, GL_FLOAT, &list[0]);
			//
		}
	}
	GLTexImage::UnbindTex();
	delete mem;
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}
}



void PyramidPacked::GetFeatureOrienations()
{
	GLTexImage * gtex;
	GLTexImage * otex;
	GLTexImage * ftex = _featureTex;
	GLTexImage * fotex = _orientationTex; 
	int * count	 = _levelFeatureNum;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num);
	FrameBufferObject fbo;
	if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
	{
		GLenum buffers[] = { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
		glDrawBuffers(2, buffers);
	}else
	{
		glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	}
	for(int i = 0; i < _octave_num; i++)
	{
		gtex = GetBaseLevel(i+_octave_min, DATA_GRAD) + GlobalUtil::_GradientLevelOffset;
		otex = GetBaseLevel(i+_octave_min, DATA_ROT) + GlobalUtil::_GradientLevelOffset;


		for(int j = 0; j < param._dog_level_num; j++, ftex++, otex++, count++, gtex++, fotex++)
		{
			if(*count<=0)continue;

			sigma = param.GetLevelSigma(j+param._level_min+1);

			ftex->FitTexViewPort();

			glActiveTexture(GL_TEXTURE0);
			ftex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			gtex->BindTex();
			glActiveTexture(GL_TEXTURE2);
			otex->BindTex();
			//
			ftex->AttachToFBO(0);
			if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 ==0)
				fotex->AttachToFBO(1);

			ShaderMan::UseShaderOrientation(gtex->GetTexID(),
				gtex->GetImgWidth(), gtex->GetImgHeight(),
				_existing_keypoints? 0 : sigma,  
				otex->GetTexID(), sigma_step);
			ftex->DrawQuad();

			//imdebugTexImagef(GlobalUtil::_texTarget, ftex->GetTexID(), GL_RGBA);
		}
	}

	GLTexImage::UnbindMultiTex(3);
	if(GlobalUtil::_timingS)glFinish();

	if(GlobalUtil::_MaxOrientation>1&& GlobalUtil::_OrientationPack2 ==0)	fbo.UnattachTex(GL_COLOR_ATTACHMENT1_EXT);

}


void PyramidPacked::GetSimplifiedOrientation()
{
	//
	int idx = 0;
//	int n = _octave_num  * param._dog_level_num;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num); 
	GLTexImage * ftex = _featureTex;

	FrameBufferObject fbo;
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	for(int i = 0; i < _octave_num; i++)
	{
		GLTexImage *otex = GetBaseLevel(i + _octave_min, DATA_ROT) + 2;
		for(int j = 0; j < param._dog_level_num; j++, ftex++,  otex++, idx ++)
		{
			if(_levelFeatureNum[idx]<=0)continue;
			sigma = param.GetLevelSigma(j+param._level_min+1);
			//
			ftex->AttachToFBO(0);
			ftex->FitTexViewPort();

			glActiveTexture(GL_TEXTURE0);
			ftex->BindTex();
			glActiveTexture(GL_TEXTURE1);
			otex->BindTex();

			ShaderMan::UseShaderSimpleOrientation(otex->GetTexID(), sigma, sigma_step);
			ftex->DrawQuad();
		}
	}
	GLTexImage::UnbindMultiTex(2);
}

void PyramidPacked::InitPyramid(int w, int h, int ds)
{
	int wp, hp, toobig = 0;
	if(ds == 0)
	{
		_down_sample_factor = 0;
		if(GlobalUtil::_octave_min_default>=0)
		{
			wp = w >> GlobalUtil::_octave_min_default;
			hp = h >> GlobalUtil::_octave_min_default;
		}else 
		{
			wp = w << (-GlobalUtil::_octave_min_default);
			hp = h << (-GlobalUtil::_octave_min_default);
		}
		_octave_min = _octave_min_default;
	}else
	{
		//must use 0 as _octave_min; 
		_octave_min = 0;
		_down_sample_factor = ds;
		w >>= ds;
		h >>= ds;
		wp = w;
		hp = h; 

	}

	while(wp > GlobalUtil::_texMaxDim  || hp > GlobalUtil::_texMaxDim )
	{
		_octave_min ++;
		wp >>= 1;
		hp >>= 1;
		toobig = 1;
	}
	if(toobig && GlobalUtil::_verbose)
	{

		std::cout<< "**************************************************************\n"
					"Image larger than allowed dimension, data will be downsampled!\n"
					"use -maxd to change the settings\n"
					"***************************************************************\n";
	}
	//make them odd;
	//wp |=  0x1 ;	hp |=  0x1 ;
	

	if( wp == _pyramid_width && hp == _pyramid_height && _allocated )
	{
		FitPyramid(wp, hp);
	}else if(GlobalUtil::_ForceTightPyramid || _allocated ==0)
	{
		ResizePyramid(wp, hp);
	}
	else if( wp > _pyramid_width || hp > _pyramid_height )
	{
		ResizePyramid(max(wp, _pyramid_width), max(hp, _pyramid_height));
		if(wp < _pyramid_width || hp < _pyramid_height)  FitPyramid(wp, hp);
	}
	else
	{
		//try use the pyramid allocated for large image on small input images
		FitPyramid(wp, hp);
	}

	//select the initial smoothing filter according to the new _octave_min
	ShaderMan::SelectInitialSmoothingFilter(_octave_min, param);
}



void PyramidPacked::FitPyramid(int w, int h)
{
	//(w, h) <= (_pyramid_width, _pyramid_height);

	_pyramid_octave_first = 0;
	//
	_octave_num  = GlobalUtil::_octave_num_default;

	int _octave_num_max = max(1, (int) floor (log ( double(min(w, h)))/log(2.0))  -3 );

	if(_octave_num < 1 || _octave_num > _octave_num_max) 
	{
		_octave_num = _octave_num_max;
	}


	int pw = _pyramid_width>>1, ph = _pyramid_height>>1;
	while(_pyramid_octave_first + _octave_num < _pyramid_octave_num &&  
		pw >= w && ph >= h)
	{
		_pyramid_octave_first++;
		pw >>= 1;
		ph >>= 1;
	}

	for(int i = 0; i < _octave_num; i++)
	{
		GLTexImage * tex = GetBaseLevel(i + _octave_min);
		GLTexImage * dog = GetBaseLevel(i + _octave_min, DATA_DOG);
		GLTexImage * grd = GetBaseLevel(i + _octave_min, DATA_GRAD);
		GLTexImage * rot = GetBaseLevel(i + _octave_min, DATA_ROT);
		GLTexImage * key = GetBaseLevel(i + _octave_min, DATA_KEYPOINT);
		for(int j = param._level_min; j <= param._level_max; j++, tex++, dog++, grd++, rot++, key++)
		{
			tex->SetImageSize(w, h);
			if(j == param._level_min) continue;
			dog->SetImageSize(w, h);
			grd->SetImageSize(w, h);
			rot->SetImageSize(w, h);
			if(j == param._level_min + 1 || j == param._level_max) continue;
			key->SetImageSize(w, h);
		}
		w>>=1;
		h>>=1;
	}
}


void PyramidPacked::ResizePyramid( int w,  int h)
{
	//
	unsigned int totalkb = 0;
	int _octave_num_new, input_sz, i, j;
	//

	if(_pyramid_width == w && _pyramid_height == h && _allocated) return;

	if(w > GlobalUtil::_texMaxDim || h > GlobalUtil::_texMaxDim) return ;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<w<<"x"<<h<<endl;
	//first octave does not change
	_pyramid_octave_first = 0;

	
	//compute # of octaves

	input_sz = min(w,h) ;


	_pyramid_width =  w;
	_pyramid_height =  h;



	//reset to preset parameters

	_octave_num_new  = GlobalUtil::_octave_num_default;

	if(_octave_num_new < 1) 
	{
		_octave_num_new = (int) floor (log ( double(input_sz))/log(2.0)) -3 ;
		if(_octave_num_new<1 ) _octave_num_new = 1;
	}

	if(_pyramid_octave_num != _octave_num_new)
	{
		//destroy the original pyramid if the # of octave changes
		if(_octave_num >0) 
		{
			DestroyPerLevelData();
			DestroyPyramidData();
		}
		_pyramid_octave_num = _octave_num_new;
	}

	_octave_num = _pyramid_octave_num;

	int noct = _octave_num;
	int nlev = param._level_num;

	//	//initialize the pyramid
	if(_allPyramid==NULL)	_allPyramid = new GLTexPacked[ noct* nlev * DATA_NUM];


	GLTexPacked * gus = (GLTexPacked *) GetBaseLevel(_octave_min, DATA_GAUSSIAN);
	GLTexPacked * dog = (GLTexPacked *) GetBaseLevel(_octave_min, DATA_DOG);
	GLTexPacked * grd = (GLTexPacked *) GetBaseLevel(_octave_min, DATA_GRAD);
	GLTexPacked * rot = (GLTexPacked *) GetBaseLevel(_octave_min, DATA_ROT);
	GLTexPacked * key = (GLTexPacked *) GetBaseLevel(_octave_min, DATA_KEYPOINT);


	////////////there could be "out of memory" happening during the allocation

	for(i = 0; i< noct; i++)
	{
		for( j = 0; j< nlev; j++, gus++, dog++, grd++, rot++, key++)
		{
			gus->InitTexture(w, h);
			if(j==0)continue;
			dog->InitTexture(w, h);
			grd->InitTexture(w, h, 0);
			rot->InitTexture(w, h);
			if(j<=1 || j >=nlev -1) continue;
			key->InitTexture(w, h, 0);
		}
		int tsz = (gus -1)->GetTexPixelCount() * 16;
		totalkb += ((nlev *5 -6)* tsz / 1024);
		//several auxilary textures are not actually required
		w>>=1;
		h>>=1;
	}

	totalkb += ResizeFeatureStorage();

	_allocated = 1;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<(totalkb/1024)<<"MB\n";

}

void PyramidPacked::DestroyPyramidData()
{
	if(_allPyramid)
	{
		delete [] _allPyramid;
		_allPyramid = NULL;
	}
}


GLTexImage*  PyramidPacked::GetLevelTexture(int octave, int level)
{
	return _allPyramid+ (_pyramid_octave_first + octave - _octave_min) * param._level_num 
		+ (level - param._level_min);
}

//in the packed implementation( still in progress)
// DATA_GAUSSIAN, DATA_DOG, DATA_GAD will be stored in different textures.

GLTexImage*  PyramidPacked::GetBaseLevel(int octave, int dataName)
{
	if(octave <_octave_min || octave > _octave_min + _octave_num) return NULL;
	int offset = (_pyramid_octave_first + octave - _octave_min) * param._level_num;
	int num = param._level_num * _pyramid_octave_num;
	return _allPyramid + num *dataName + offset;
}


void PyramidPacked::FitHistogramPyramid()
{
	GLTexImage * tex, *htex;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 

	tex = GetBaseLevel(_octave_min , DATA_KEYPOINT) + 2;
	htex = _histoPyramidTex + hist_level_num - 1;
	int w = (tex->GetImgWidth() + 2) >> 2;
	int h = (tex->GetImgHeight() + 2)>> 2;


	//4n+1 -> n; 4n+2,2, 3 -> n+1
	for(int k = 0; k <hist_level_num -1; k++, htex--)
	{
		if(htex->GetImgHeight()!= h || htex->GetImgWidth() != w)
		{	
			htex->SetImageSize(w, h);
			htex->ZeroHistoMargin();
		}

		w = (w + 1)>>1; h = (h + 1) >> 1;
	}
}

