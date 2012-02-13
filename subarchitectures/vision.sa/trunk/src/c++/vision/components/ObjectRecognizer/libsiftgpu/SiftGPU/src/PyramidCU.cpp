////////////////////////////////////////////////////////////////////////////
//	File:		PyramidCU.cpp
//	Author:		Changchang Wu
//	Description : implementation of the PyramidCU class.
//				CUDA-based implementation of SiftPyramid
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

#if defined(CUDA_SIFTGPU_ENABLED)


#include "GL/glew.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
using namespace std;

#include "GlobalUtil.h"
#include "GLTexImage.h"
#include "CuTexImage.h" 
#include "SiftGPU.h"
#include "SiftPyramid.h"
#include "ProgramCU.h"
#include "PyramidCU.h"


//#include "imdebug/imdebuggl.h"
//#pragma comment (lib, "../lib/imdebug.lib")



#define USE_TIMING()		double t, t0, tt;
#define OCTAVE_START()		if(GlobalUtil::_timingO){	t = t0 = CLOCK();	cout<<"#"<<i+_down_sample_factor<<"\t";	}
#define LEVEL_FINISH()		if(GlobalUtil::_timingL){	ProgramCU::FinishCUDA();	tt = CLOCK();cout<<(tt-t)<<"\t";	t = CLOCK();}
#define OCTAVE_FINISH()		if(GlobalUtil::_timingO)cout<<"|\t"<<(CLOCK()-t0)<<endl;


PyramidCU::PyramidCU(SiftParam& sp) : SiftPyramid(sp)
{
	_allPyramid = NULL;
	_histoPyramidTex = NULL;
	_featureTex = NULL;
	_descriptorTex = NULL;
	_orientationTex = NULL;
	_bufferPBO = 0;
	_inputTex = new CuTexImage();
}

PyramidCU::~PyramidCU()
{
	DestroyPerLevelData();
	DestroySharedData();
	DestroyPyramidData();
	if(_inputTex) delete _inputTex;
}

void PyramidCU::InitPyramid(int w, int h, int ds)
{
	int wp, hp, toobig = 0;
	if(ds == 0)
	{
		//
		TruncateWidth(w);
		////
		_down_sample_factor = 0;
		if(GlobalUtil::_octave_min_default>=0)
		{
			wp = w >> _octave_min_default;
			hp = h >> _octave_min_default;
		}else
		{
			//can't upsample by more than 8
			_octave_min_default = max(-3, _octave_min_default);
			//
			wp = w << (-_octave_min_default);
			hp = h << (-_octave_min_default);
		}
		_octave_min = _octave_min_default;
	}else
	{
		//must use 0 as _octave_min; 
		_octave_min = 0;
		_down_sample_factor = ds;
		w >>= ds;
		h >>= ds;
		/////

		TruncateWidth(w);

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
	ResizePyramid(wp, hp);

	if(_bufferPBO == 0) glGenBuffers(1, &_bufferPBO);

}

void PyramidCU::ResizePyramid(int w, int h)
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
	if(_allPyramid==NULL)	_allPyramid = new CuTexImage[ noct* nlev * DATA_NUM];

	CuTexImage * gus =  GetBaseLevel(_octave_min, DATA_GAUSSIAN);
	CuTexImage * dog =  GetBaseLevel(_octave_min, DATA_DOG);
	CuTexImage * got =  GetBaseLevel(_octave_min, DATA_GRAD);
	CuTexImage * key =  GetBaseLevel(_octave_min, DATA_KEYPOINT);

	////////////there could be "out of memory" happening during the allocation

	for(i = 0; i< noct; i++)
	{
		int wa = ((w + 3) / 4) * 4;

		totalkb += ((nlev *8 -19)* (wa * h) * 4 / 1024);
		for( j = 0; j< nlev; j++, gus++, dog++, got++, key++)
		{
			gus->InitTexture(wa, h); //nlev
			if(j==0)continue;
			dog->InitTexture(wa, h);  //nlev -1
			if(	j >= GlobalUtil::_GradientLevelOffset && j < GlobalUtil::_GradientLevelOffset + param._dog_level_num)
			{
				got->InitTexture(wa, h, 2); //2 * nlev - 6
				got->InitTexture2D();
			}
			if(j > 1 && j < nlev -1)	key->InitTexture(wa, h, 4); // nlev -3 ; 4 * nlev - 12
		}
		w>>=1;
		h>>=1;
	}

	totalkb += ResizeFeatureStorage();

	ProgramCU::CheckErrorCUDA("ResizePyramid");

	_allocated = 1;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<(totalkb/1024)<<"MB\n";

}

int PyramidCU::IsCudaSupported()
{
	return ProgramCU::IsCudaSupported();
}

void PyramidCU::SetLevelFeatureNum(int idx, int fcount)
{
	_featureTex[idx].InitTexture(fcount, 1, 4);
	_levelFeatureNum[idx] = fcount;
}

int PyramidCU::ResizeFeatureStorage()
{
	int totalkb = 0;
	if(_levelFeatureNum==NULL)	_levelFeatureNum = new int[_octave_num * param._dog_level_num];
	std::fill(_levelFeatureNum, _levelFeatureNum+_octave_num * param._dog_level_num, 0); 

	int wmax = GetBaseLevel(_octave_min)->GetImgWidth();
	int hmax = GetBaseLevel(_octave_min)->GetImgHeight();
	int whmax = max(wmax, hmax);
	int w,  i;

	//
	int num = (int)ceil(log(double(whmax))/log(4.0));

	if( _hpLevelNum != num)
	{
		_hpLevelNum = num;
		if(_histoPyramidTex ) delete [] _histoPyramidTex;
		_histoPyramidTex = new CuTexImage[_hpLevelNum];
	}

	for(i = 0, w = 1; i < _hpLevelNum; i++)
	{
		_histoPyramidTex[i].InitTexture(w, whmax, 4);
		w<<=2;
	}

	// (4 ^ (_hpLevelNum) -1 / 3) pixels
	totalkb += (((1 << (2 * _hpLevelNum)) -1) / 3 * 16 / 1024);

	//initialize the feature texture
	int idx = 0, n = _octave_num * param._dog_level_num;
	if(_featureTex==NULL)	_featureTex = new CuTexImage[n];
	if(GlobalUtil::_MaxOrientation >1 && GlobalUtil::_OrientationPack2==0 && _orientationTex== NULL)
		_orientationTex = new CuTexImage[n];


	for(i = 0; i < _octave_num; i++)
	{
		CuTexImage * tex = GetBaseLevel(i+_octave_min);
		int fmax = int(tex->GetImgWidth() * tex->GetImgHeight()*GlobalUtil::_MaxFeaturePercent);
		//
		if(fmax > GlobalUtil::_MaxLevelFeatureNum) fmax = GlobalUtil::_MaxLevelFeatureNum;
		else if(fmax < 32) fmax = 32;	//give it at least a space of 32 feature

		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			_featureTex[idx].InitTexture(fmax, 1, 4);
			totalkb += fmax * 16 /1024;
			//
			if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
			{
				_orientationTex[idx].InitTexture(fmax, 1, 4);
				totalkb += fmax * 16 /1024;
			}
		}
	}


	//this just need be initialized once
	if(_descriptorTex==NULL)
	{
		//initialize feature texture pyramid
		int fmax = _featureTex->GetImgWidth();
		_descriptorTex = new CuTexImage;
		totalkb += ( fmax /2);
		_descriptorTex->InitTexture(fmax *128, 1, 1);

	}else
	{
		totalkb +=  _descriptorTex->GetDataSize()/1024;
	}
	return totalkb;
}

void PyramidCU::GetFeatureDescriptors() 
{
	//descriptors...
	int idx, i, j;
	float* pd =  &_descriptor_buffer[0];
	vector<float>read_buffer, descriptor_buffer2;

	//use another buffer, if we need to re-order the descriptors
	if(_keypoint_index.size() > 0)
	{
		descriptor_buffer2.resize(_descriptor_buffer.size());
		pd = &descriptor_buffer2[0];
	}

	CuTexImage * got, * ftex;
	for( i = 0, idx = 0, ftex = _featureTex; i < _octave_num; i++)
	{
		got = GetBaseLevel(i + _octave_min, DATA_GRAD) + GlobalUtil::_GradientLevelOffset;
		for( j = 0; j < param._dog_level_num; j++, ftex++, idx++, got++)
		{
			if(_levelFeatureNum[idx]==0)continue;

			//process
			ProgramCU::ComputeDescriptor(ftex, got, _descriptorTex);
			
			//readback
			_descriptorTex->CopyToHost(pd);
			//download descriptor
			pd += 128*_levelFeatureNum[idx];
		}
	}

	if(GlobalUtil::_timingS)ProgramCU::FinishCUDA();

	//finally, put the descriptor back to their original order for existing keypoint list.
	if(_keypoint_index.size() > 0)
	{
		for(i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_descriptor_buffer[index*128], &descriptor_buffer2[i*128], 128 * sizeof(float));
		}
	}

	ProgramCU::CheckErrorCUDA("PyramidCU::GetFeatureDescriptors");
}

void PyramidCU::GenerateFeatureListTex() 
{

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
					list.push_back(key[2] / octave_sigma);
					list.push_back((float)fmod(twopi-key[3], twopi));
					fcount ++;
					//save the index of keypoints
					_keypoint_index.push_back(k);
				}

			}

			_levelFeatureNum[idx] = fcount;
			if(fcount==0)continue;
			CuTexImage * ftex = _featureTex+idx;

			SetLevelFeatureNum(idx, fcount);
			ftex->CopyFromHost(&list[0]);
		}
	}

	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}

}

void PyramidCU::ReshapeFeatureListCPU() 
{
	int i, szmax =0, sz;
	int n = param._dog_level_num*_octave_num;
	for( i = 0; i < n; i++) 
	{
		sz = _levelFeatureNum[i];
		if(sz > szmax ) szmax = sz;
	}
	float * buffer = new float[szmax*16];
	float * buffer1 = buffer;
	float * buffer2 = buffer + szmax*4;



	_featureNum = 0;

#ifdef NO_DUPLICATE_DOWNLOAD
	const double twopi = 2.0*3.14159265358979323846;
	_keypoint_buffer.resize(0);
	float os = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	if(_down_sample_factor>0) os *= float(1<<_down_sample_factor); 
	float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f;
#endif


	for(i = 0; i < n; i++)
	{
		if(_levelFeatureNum[i]==0)continue;

		_featureTex[i].CopyToHost(buffer1);
		
		int fcount =0;
		float * src = buffer1;
		float * des = buffer2;
		const static double factor  = 2.0*3.14159265358979323846/65535.0;
		for(int j = 0; j < _levelFeatureNum[i]; j++, src+=4)
		{
			unsigned short * orientations = (unsigned short*) (&src[3]);
			if(orientations[0] != 65535)
			{
				des[0] = src[0];
				des[1] = src[1];
				des[2] = src[2];
				des[3] = float( factor* orientations[0]);
				fcount++;
				des += 4;
				if(orientations[1] != 65535 && orientations[1] != orientations[0])
				{
					des[0] = src[0];
					des[1] = src[1];
					des[2] = src[2];
					des[3] = float(factor* orientations[1]);	
					fcount++;
					des += 4;
				}
			}
		}
		//texture size
		SetLevelFeatureNum(i, fcount);
		_featureTex[i].CopyFromHost(buffer2);


#ifdef NO_DUPLICATE_DOWNLOAD
		float oss = os * (1 << (i / param._dog_level_num));
		_keypoint_buffer.resize((_featureNum + fcount) * 4);
		float* ds = &_keypoint_buffer[_featureNum * 4];
		float* fs = buffer2;
		for(int k = 0;  k < fcount; k++, ds+=4, fs+=4)
		{
			ds[0] = oss*(fs[0]-0.5f) + offset;	//x
			ds[1] = oss*(fs[1]-0.5f) + offset;	//y
			ds[2] = oss*fs[2];  //scale
			ds[3] = (float)fmod(twopi-fs[3], twopi);	//orientation, mirrored
		}
#endif
		_featureNum += fcount;
	}

	delete[] buffer;
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features MO:\t"<<_featureNum<<endl;
	}
}

void PyramidCU::GenerateFeatureDisplayVBO() 
{
	//it is weried that this part is very slow.
	//use a big VBO to save all the SIFT box vertices
	int nvbo = _octave_num * param._dog_level_num;
	if(_featureDisplayVBO==NULL)
	{
		//initialize the vbos
		_featureDisplayVBO = new GLuint[nvbo];
		_featurePointVBO = new GLuint[nvbo];
		glGenBuffers(nvbo, _featureDisplayVBO);	
		glGenBuffers(nvbo, _featurePointVBO);
	}
	for(int i = 0; i < nvbo; i++)
	{
		if(_levelFeatureNum[i]<=0)continue;
		CuTexImage * ftex  = _featureTex + i;
		CuTexImage texPBO1( _levelFeatureNum[i]* 10, 1, 4, _featureDisplayVBO[i]);
		CuTexImage texPBO2(_levelFeatureNum[i], 1, 4, _featurePointVBO[i]);
		ProgramCU::DisplayKeyBox(ftex, &texPBO1);
		ProgramCU::DisplayKeyPoint(ftex, &texPBO2);	
	}
}

void PyramidCU::DestroySharedData() 
{
	//histogram reduction
	if(_histoPyramidTex)
	{
		delete[]	_histoPyramidTex;
		_hpLevelNum = 0;
		_histoPyramidTex = NULL;
	}
	//descriptor storage shared by all levels
	if(_descriptorTex)
	{
		delete _descriptorTex;
		_descriptorTex = NULL;
	}
	//cpu reduction buffer.
	if(_histo_buffer)
	{
		delete[] _histo_buffer;
		_histo_buffer = 0;
	}
}

void PyramidCU::DestroyPerLevelData() 
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

void PyramidCU::DestroyPyramidData()
{
	if(_allPyramid)
	{
		delete [] _allPyramid;
		_allPyramid = NULL;
	}
}

void PyramidCU::DownloadKeypoints() 
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
	float * p = buffer, *ps;
	CuTexImage * ftex = _featureTex;
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
				ftex->CopyToHost(ps = p);
				for(int k = 0;  k < _levelFeatureNum[idx]; k++, ps+=4)
				{
					ps[0] = os*(ps[0]-0.5f) + offset;	//x
					ps[1] = os*(ps[1]-0.5f) + offset;	//y
					ps[2] = os*ps[2]; 
					ps[3] = (float)fmod(twopi-ps[3], twopi);	//orientation, mirrored
				}
				p+= 4* _levelFeatureNum[idx];
			}
		}
	}

	//put the feature into their original order for existing keypoint 
	if(_keypoint_index.size() > 0)
	{
		for(int i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_keypoint_buffer[index*4], &keypoint_buffer2[i*4], 4 * sizeof(float));
		}
	}
}

void PyramidCU::GenerateFeatureListCPU()
{
	//no cpu version provided
	GenerateFeatureList();
}

void PyramidCU::GenerateFeatureList()
{
	CuTexImage * htex, * ftex, * tex, *got;
	double t1, t2, t3, ot1, ot2, ts1 = 0, ts2 = 0; 
	int ocount = 0, idx = 0, reduction_count;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 
	//int hist_skip_gpu = GlobalUtil::_ListGenSkipGPU; 
	int ii, k, len; 
	vector<int> hbuffer;
	_featureNum = 0;


	for(int i = 0; i < _octave_num; i++)
	{
		tex = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + 2;
		got = GetBaseLevel(_octave_min + i, DATA_GRAD) + 2;
		//output
		reduction_count = FitHistogramPyramid(tex);

		if(GlobalUtil::_timingO)
		{
			ot1 = ot2 = 0; 
			ocount = 0;
			std::cout<<"#"<<i+_octave_min + _down_sample_factor<<":\t";
		}
		for(int j = 0; j < param._dog_level_num; j++, tex++, got++, idx++)
		{
			int fcount = 0;
			ftex = _featureTex + idx;
			htex = _histoPyramidTex + hist_level_num -1;
			if(GlobalUtil::_timingL) t1= CLOCK();
			ProgramCU::InitHistogram(tex, htex);
			for(k = 0; k < reduction_count - 1; k++, htex--)
			{
				ProgramCU::ReduceHistogram(htex, htex -1);	
			}
			if(GlobalUtil::_timingL)		t2= CLOCK();
			
			//htex has the row reduction result
			len = htex->GetImgHeight() * 4;
			hbuffer.resize(len);
			htex->CopyToHost(&hbuffer[0]);
			//
			for(ii = 0; ii < len; ++ii)		fcount += hbuffer[ii];
			SetLevelFeatureNum(idx, fcount);
			//build the feature list
#ifdef _DEBUG
//			ProgramCU::CheckErrorCUDA("Generate List Level");
#endif
			///
			if(fcount > 0)
			{
				_featureNum += fcount;
				_keypoint_buffer.resize(fcount * 4);
				//vector<int> ikbuf(fcount*4);
				int* ibuf = (int*) (&_keypoint_buffer[0]);

				for(ii = 0; ii < len; ++ii)
				{
					int x = ii%4, y = ii / 4;
					for(int jj = 0 ; jj < hbuffer[ii]; ++jj, ibuf+=4)
					{
						ibuf[0] = x; ibuf[1] = y; ibuf[2] = jj; ibuf[3] = 0;
					}
				}
				_featureTex[idx].CopyFromHost(&_keypoint_buffer[0]);
			
				////////////////////////////////////////////
				ProgramCU::GenerateList(_featureTex + idx, ++htex);
				for(k = 2; k < reduction_count; k++)
				{
					ProgramCU::GenerateList(_featureTex + idx, ++htex);
				}
			}

			/////////////////////////////
			if(GlobalUtil::_timingL)
			{
				ProgramCU::FinishCUDA();
				t3 = CLOCK();
				ot1 += (t2 - t1); ot2 += ( t3 - t2);
				std::cout<<int(fcount)<<"\t";
			}
			ocount += fcount;
		}
		if(GlobalUtil::_timingO)
		{	
			ts1 += ot1; ts2 += ot2; 
			std::cout << "| \t" << int(ocount) << " :\t(" << ot1 <<",\t" << ot2 << ")\n";
		}
	}
	/////
	CopyGradientTex();
	/////
	if(GlobalUtil::_timingS)ProgramCU::FinishCUDA();

	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}

	ProgramCU::CheckErrorCUDA("PyramidCU::GenerateFeatureList");
}

GLTexImage* PyramidCU::GetLevelTexture(int octave, int level)
{
	return GetLevelTexture(octave, level, DATA_GAUSSIAN);
}

GLTexImage* PyramidCU::ConvertTexCU2GL(CuTexImage* tex, int dataName)
{
	static GLTexImage texGL;
	GLenum format = GL_LUMINANCE;
	int convert_done = 1;
	switch(dataName)
	{
	case DATA_GAUSSIAN:
		{
			convert_done = tex->CopyToPBO(_bufferPBO);
			break;
		}
	case DATA_DOG:
		{
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 1, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertDOG(tex, &texPBO);
			break;
		}
	case DATA_GRAD:
		{
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 1, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertGRD(tex, &texPBO);
			break;
		}
	case DATA_KEYPOINT:
		{
			CuTexImage * dog = tex - param._level_num * _pyramid_octave_num;
			format = GL_RGBA;
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 4, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertKEY(tex, dog, &texPBO);
			break;
		}
	default:
			convert_done = 0;
			break;
	}

	if(convert_done)
	{
		texGL.InitTexture(max(texGL.GetTexWidth(), tex->GetImgWidth()), max(texGL.GetTexHeight(), tex->GetImgHeight()));
		texGL.CopyFromPBO(_bufferPBO, tex->GetImgWidth(), tex->GetImgHeight(), format);
	}else
	{
		texGL.SetImageSize(0, 0);
	}

	return &texGL;
}

GLTexImage* PyramidCU::GetLevelTexture(int octave, int level, int dataName) 
{
	CuTexImage* tex = GetBaseLevel(octave, dataName) + (level - param._level_min);
	//CuTexImage* gus = GetBaseLevel(octave, DATA_GAUSSIAN) + (level - param._level_min); 
	return ConvertTexCU2GL(tex, dataName);
}

void PyramidCU::ConvertInputToCU(GLTexInput* input)
{
	int ws = input->GetImgWidth(), hs = input->GetImgHeight();
	TruncateWidth(ws);
	//copy the input image to pixel buffer object
	if(input->CopyToPBO(_bufferPBO, ws, hs))
	{
		CuTexImage texPBO(ws, hs, 4, _bufferPBO);
		_inputTex->InitTexture(ws, hs, 1);
		ProgramCU::ReduceToSingleChannel(_inputTex, &texPBO, !input->_rgb_converted);

		//delete original texture to save some memory
		//glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, _bufferPBO);
		//glBufferData(GL_PIXEL_PACK_BUFFER_ARB, 1,	NULL, GL_STATIC_DRAW_ARB);
		//glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
	
	}else
	{
		std::cerr<< "Unable To Convert Intput\n";
	}
}

void PyramidCU::BuildPyramid(GLTexInput * input)
{

	USE_TIMING();

	int i, j;
	
	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{

		float* filter_sigma = param._sigma;
		CuTexImage *tex = GetBaseLevel(i);
		CuTexImage *buf = GetBaseLevel(i, DATA_KEYPOINT) +2;
		j = param._level_min + 1;

		OCTAVE_START();

		if( i == _octave_min )
		{	
			ConvertInputToCU(input);

			if(i == 0)
			{
				ProgramCU::FilterImage(tex, _inputTex, buf, param.GetInitialSmoothSigma(_octave_min));
			}else
			{
				if(i < 0)	ProgramCU::SampleImageU(tex, _inputTex, -i);			
				else		ProgramCU::SampleImageD(tex, _inputTex, i);
				ProgramCU::FilterImage(tex, tex, buf, param.GetInitialSmoothSigma(_octave_min));
			}
			LEVEL_FINISH();
		}else
		{
			ProgramCU::SampleImageD(tex, GetBaseLevel(i - 1) + param._level_ds - param._level_min); 
	
			LEVEL_FINISH();

			if(param._sigma_skip1 > 0)
			{
				ProgramCU::FilterImage(tex, tex, buf, param._sigma_skip1);
				LEVEL_FINISH();
			}
		}
		for( ; j <=  param._level_max ; j++, tex++, filter_sigma++)
		{
			// filtering
			ProgramCU::FilterImage(tex + 1, tex, buf, *filter_sigma);
			LEVEL_FINISH();
		}
		OCTAVE_FINISH();
	}
	if(GlobalUtil::_timingS) ProgramCU::FinishCUDA();

	ProgramCU::CheckErrorCUDA("PyramidCU::BuildPyramid");
}

void PyramidCU::DetectKeypointsEX()
{


	int i, j;
	double t0, t, ts, t1, t2;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = CLOCK();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		CuTexImage * gus = GetBaseLevel(i) + 1;
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) + 1;
		CuTexImage * got = GetBaseLevel(i, DATA_GRAD) + 1;
		//compute the gradient
		for(j = param._level_min +1; j <=  param._level_max ; j++, gus++, dog++, got++)
		{
			//input: gus and gus -1
			//output: gradient, dog, orientation
			ProgramCU::ComputeDOG(gus, dog, got);
		}
	}
	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		ProgramCU::FinishCUDA();
		t1 = CLOCK();
	}


	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		if(GlobalUtil::_timingO)
		{
			t0 = CLOCK();
			std::cout<<"#"<<(i + _down_sample_factor)<<"\t";
		}
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) + 2;
		CuTexImage * key = GetBaseLevel(i, DATA_KEYPOINT) +2;


		for( j = param._level_min +2; j <  param._level_max ; j++, dog++, key++)
		{
			if(GlobalUtil::_timingL)t = CLOCK();
			//input, dog, dog + 1, dog -1
			//output, key
			ProgramCU::ComputeKEY(dog, key, param._dog_threshold, param._edge_threshold);
			if(GlobalUtil::_timingL)
			{
				std::cout<<(CLOCK()-t)<<"\t";
			}
		}
		if(GlobalUtil::_timingO)
		{
			std::cout<<"|\t"<<(CLOCK()-t0)<<"\n";
		}
	}

	if(GlobalUtil::_timingS)
	{
		ProgramCU::FinishCUDA();
		if(GlobalUtil::_verbose) 
		{	
			t2 = CLOCK();
			std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)<<"\n"
						<<"<Get Keypoints  >\t"<<(t2-t1)<<"\n";
		}				
	}
}

void PyramidCU::CopyGradientTex()
{
	double ts, t1;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = CLOCK();

	for(int i = 0, idx = 0; i < _octave_num; i++)
	{
		CuTexImage * got = GetBaseLevel(i + _octave_min, DATA_GRAD) +  GlobalUtil::_GradientLevelOffset;
		//compute the gradient
		for(int j = 0; j <  param._dog_level_num ; j++, got++, idx++)
		{
			if(_levelFeatureNum[idx] > 0)	got->CopyToTexture2D();
		}
	}
	if(GlobalUtil::_timingS)
	{
		ProgramCU::FinishCUDA();
		if(GlobalUtil::_verbose)
		{
			t1 = CLOCK();
			std::cout	<<"<Copy Grad/Orientation>\t"<<(t1-ts)<<"\n";
		}
	}
}

void PyramidCU::ComputeGradient() 
{

	int i, j;
	double ts, t1;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = CLOCK();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		CuTexImage * gus = GetBaseLevel(i) +  GlobalUtil::_GradientLevelOffset;
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) +  GlobalUtil::_GradientLevelOffset;
		CuTexImage * got = GetBaseLevel(i, DATA_GRAD) +  GlobalUtil::_GradientLevelOffset;

		//compute the gradient
		for(j = 0; j <  param._dog_level_num ; j++, gus++, dog++, got++)
		{
			ProgramCU::ComputeDOG(gus, dog, got);
		}
	}
	if(GlobalUtil::_timingS)
	{
		ProgramCU::FinishCUDA();
		if(GlobalUtil::_verbose)
		{
			t1 = CLOCK();
			std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)<<"\n";
		}
	}
}

int PyramidCU::FitHistogramPyramid(CuTexImage* tex)
{
	CuTexImage *htex;
	int hist_level_num = _hpLevelNum - _pyramid_octave_first; 
	htex = _histoPyramidTex + hist_level_num - 1;
	int w = (tex->GetImgWidth() + 2) >> 2;
	int h = tex->GetImgHeight();
	int count = 0; 
	for(int k = 0; k < hist_level_num; k++, htex--)
	{
		htex->SetImageSize(w, h);		++count;
		if(w == 1) break;
		w = (w + 3)>>2; 
	}
	return count;
}

void PyramidCU::GetFeatureOrientations() 
{

	CuTexImage * ftex = _featureTex;
	int * count	 = _levelFeatureNum;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num);

	for(int i = 0; i < _octave_num; i++)
	{
		CuTexImage* got = GetBaseLevel(i + _octave_min, DATA_GRAD) + GlobalUtil::_GradientLevelOffset;
		CuTexImage* key = GetBaseLevel(i + _octave_min, DATA_KEYPOINT) + 2;

		for(int j = 0; j < param._dog_level_num; j++, ftex++, count++, got++, key++)
		{
			if(*count<=0)continue;

			//if(ftex->GetImgWidth() < *count) ftex->InitTexture(*count, 1, 4);

			sigma = param.GetLevelSigma(j+param._level_min+1);

			ProgramCU::ComputeOrientation(ftex, got, key, sigma, sigma_step, _existing_keypoints);		
		}
	}

	if(GlobalUtil::_timingS)ProgramCU::FinishCUDA();
	ProgramCU::CheckErrorCUDA("PyramidCU::GetFeatureOrientations");

}

void PyramidCU::GetSimplifiedOrientation() 
{
	//no simplified orientation
	GetFeatureOrientations();
}

CuTexImage* PyramidCU::GetBaseLevel(int octave, int dataName)
{
	if(octave <_octave_min || octave > _octave_min + _octave_num) return NULL;
	int offset = (_pyramid_octave_first + octave - _octave_min) * param._level_num;
	int num = param._level_num * _pyramid_octave_num;
	if (dataName == DATA_ROT) dataName = DATA_GRAD;
	return _allPyramid + num * dataName + offset;
}

#endif
