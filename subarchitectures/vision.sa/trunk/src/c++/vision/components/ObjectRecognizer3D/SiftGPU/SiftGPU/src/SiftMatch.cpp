////////////////////////////////////////////////////////////////////////////
//	File:		SiftMatch.cpp
//	Author:		Changchang Wu
//	Description :	implementation of SiftMatchGPU and SiftMatchCG
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
#include <strstream>
#include <algorithm>
using namespace std;
#include "GlobalUtil.h"
#include "ProgramCG.h"
#include "GLTexImage.h"
#include "SiftGPU.h"
#include "SiftMatch.h"
#include "FrameBufferObject.h"


SiftMatchCG::SiftMatchCG(int max_sift)
{
	s_multiply = s_col_max = s_row_max = s_guided_mult = NULL;
	_num_sift[0] = _num_sift[1] = 0;
	_id_sift[0] = _id_sift[1] = 0;
	_have_loc[0] = _have_loc[1] = 0;
	_max_sift = max_sift <=0 ? 4096 : ((max_sift + 31)/ 32 * 32) ; 
	_use_packed_des = 1;
	_pixel_per_sift = _use_packed_des? 8 : 32;
	_sift_num_stripe = 1; 
	_sift_per_stripe = 1;
	_sift_per_row = _sift_per_stripe * _sift_num_stripe;
	_initialized = 0;

}
SiftMatchCG::~SiftMatchCG()
{
	if(s_multiply) delete s_multiply;
	if(s_guided_mult) delete s_guided_mult;
	if(s_col_max) delete s_col_max;
	if(s_row_max) delete s_row_max;
}

void SiftMatchCG::SetMaxSift(int max_sift)
{
	
	max_sift = ((max_sift + 31)/32)*32;
	if(max_sift > GlobalUtil::_texMaxDimGL) max_sift = GlobalUtil::_texMaxDimGL;
	if(max_sift > _max_sift)
	{
		_max_sift = max_sift;
		AllocateSiftMatch();
		_have_loc[0] = _have_loc[1] = 0;
		_id_sift[0] = _id_sift[1] = -1;
		_num_sift[0] = _num_sift[1] = 1;
	}

}

void SiftMatchCG::AllocateSiftMatch()
{
	//parameters, number of sift is limited by the texture size
	if(_max_sift > GlobalUtil::_texMaxDimGL) _max_sift = GlobalUtil::_texMaxDimGL;	
	///
	int h = _max_sift / _sift_per_row; 
	int n = (GlobalUtil::_texMaxDimGL + h - 1) / GlobalUtil::_texMaxDimGL; 
	if ( n > 1) {_sift_num_stripe *= n; _sift_per_row *= n; }

	//initialize

	_texDes[0].InitTexture(_sift_per_row * _pixel_per_sift, _max_sift / _sift_per_row, 0);
	_texDes[1].InitTexture(_sift_per_row * _pixel_per_sift, _max_sift / _sift_per_row, 0);
	_texLoc[0].InitTexture(_sift_per_row , _max_sift / _sift_per_row, 0);
	_texLoc[1].InitTexture(_sift_per_row , _max_sift / _sift_per_row, 0);

	if(GlobalUtil::_SupportNVFloat)
	{
		_texDot.InitTexture(_max_sift, _max_sift, 0, GL_FLOAT_R_NV);
		_texMatch[0].InitTexture(16, _max_sift / 16, 0, GL_FLOAT_R_NV);
		_texMatch[1].InitTexture(16, _max_sift / 16, 0, GL_FLOAT_R_NV);
	}else
	{
		_texDot.InitTexture(_max_sift, _max_sift, 0);
		_texMatch[0].InitTexture(16, _max_sift / 16, 0);
		_texMatch[1].InitTexture(16, _max_sift / 16, 0);

	}

}
void SiftMatchCG::InitSiftMatchCG()
{
	if(_initialized) return;
	GlobalUtil::InitGLParam();
	if(GlobalUtil::_HaveGlSupport == 0) return;
	ProgramCG::InitContext();
	AllocateSiftMatch();
	LoadSiftMatchShaders();
	_initialized = 1; 
}


void SiftMatchCG::SetDescriptors(int index, int num, const unsigned char* descriptors, int id)
{	
	if(_initialized == 0) return;
	if (index > 1) index = 1;
	if (index < 0) index = 0;
	_have_loc[index] = 0;
	//the same feature is already set
	if(id !=-1 && id == _id_sift[index]) return ;
	_id_sift[index] = id;
	if(num > _max_sift) num = _max_sift;
	sift_buffer.resize(num * 128 /4);
	memcpy(&sift_buffer[0], descriptors, 128 * num);
	_num_sift[index] = num; 
	int w = _sift_per_row * _pixel_per_sift;
	int h = (num + _sift_per_row  - 1)/ _sift_per_row; 
	sift_buffer.resize(w * h * 4, 0);
	_texDes[index].SetImageSize(w , h);
	_texDes[index].BindTex(); 
	if(_sift_num_stripe == 1)
	{
		glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, w, h, GL_RGBA, _use_packed_des? GL_FLOAT : GL_UNSIGNED_BYTE, &sift_buffer[0]);
	}else
	{
		for(int i = 0; i < _sift_num_stripe; ++i)
		{
			int ws = _sift_per_stripe * _pixel_per_sift;
			int x = i * ws;
			int pos = i * ws * h * 4; 
			glTexSubImage2D(GlobalUtil::_texTarget, 0, x, 0, ws, h, GL_RGBA, _use_packed_des? GL_FLOAT : GL_UNSIGNED_BYTE, &sift_buffer[pos]);
		}
	}
	_texDes[index].UnbindTex();

}

void SiftMatchCG::SetFeautreLocation(int index, const float* locations, int gap)
{
	if(_num_sift[index] <=0) return;
	int w = _sift_per_row ;
	int h = (_num_sift[index] + _sift_per_row  - 1)/ _sift_per_row; 
	sift_buffer.resize(_num_sift[index] * 2);
	if(gap == 0)
	{
		memcpy(&sift_buffer[0], locations, _num_sift[index] * 2 * sizeof(float));
	}else
	{
		for(int i = 0; i < _num_sift[index]; ++i)
		{
			sift_buffer[i*2] = *locations++;
			sift_buffer[i*2+1]= *locations ++;
			locations += gap;
		}
	}
	sift_buffer.resize(w * h * 2, 0);
	_texLoc[index].SetImageSize(w , h);
	_texLoc[index].BindTex(); 
	if(_sift_num_stripe == 1)
	{
		glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, w, h, GL_LUMINANCE_ALPHA , GL_FLOAT , &sift_buffer[0]);
	}else
	{
		for(int i = 0; i < _sift_num_stripe; ++i)
		{
			int ws = _sift_per_stripe;
			int x = i * ws;
			int pos = i * ws * h * 2; 
			glTexSubImage2D(GlobalUtil::_texTarget, 0, x, 0, ws, h, GL_LUMINANCE_ALPHA , GL_FLOAT, &sift_buffer[pos]);
		}
	}
	_texLoc[index].UnbindTex();
	_have_loc[index] = 1;
}

void SiftMatchCG::SetDescriptors(int index, int num, const float* descriptors, int id)
{	
	if(_initialized == 0) return;
	if (index > 1) index = 1;
	if (index < 0) index = 0;
	_have_loc[index] = 0;
	//the same feature is already set
	if(id !=-1 && id == _id_sift[index]) return ;
	_id_sift[index] = id; 
	if(num > _max_sift) num = _max_sift;

	sift_buffer.resize(num * 128 /4);
	unsigned char * pub = (unsigned char*) &sift_buffer[0];
	for(int i = 0; i < 128 * num; ++i)
	{
		pub[i] = int(512 * descriptors[i] + 0.5);
	}
	_num_sift[index] = num; 
	int w = _sift_per_row * _pixel_per_sift;
	int h = (num + _sift_per_row  - 1)/ _sift_per_row; 
	sift_buffer.resize(w * h * 4, 0);
	_texDes[index].SetImageSize(w, h);
	_texDes[index].BindTex();
	if(_sift_num_stripe == 1)
	{
		glTexSubImage2D(GlobalUtil::_texTarget, 0, 0, 0, w, h, GL_RGBA, _use_packed_des? GL_FLOAT : GL_UNSIGNED_BYTE, &sift_buffer[0]);
	}else
	{
		for(int i = 0; i < _sift_num_stripe; ++i)
		{
			int ws = _sift_per_stripe * _pixel_per_sift;
			int x = i * ws;
			int pos = i * ws * h * 4; 
			glTexSubImage2D(GlobalUtil::_texTarget, 0, x, 0, ws, h, GL_RGBA, _use_packed_des? GL_FLOAT : GL_UNSIGNED_BYTE, &sift_buffer[pos]);
		}
	}
	_texDes[index].UnbindTex();
}



void SiftMatchCG::LoadSiftMatchShaders()
{
	char buffer[10240];
	ostrstream out(buffer, 10240);
	out <<	"#define SIFT_PER_STRIPE " << _sift_per_stripe << "\n" 
			"#define UNSIGNED_BYTE_SIFT " <<_use_packed_des << "\n"
			"#define PIXEL_PER_SIFT " << _pixel_per_sift << "\n"
			"#define USE_BIT_OPERATION 0\n"
			"void main(uniform samplerRECT tex1,		\n"
			"uniform samplerRECT tex2,			\n"
			"uniform float2	size,  \n"
			"in		float2	pos : WPOS,	\n"
			"out	float4  result:COLOR0)		\n"
		    "{\n"
		<<	"#if UNSIGNED_BYTE_SIFT == 0\n"
			"   float4 val = float4(0, 0, 0, 0), data1, buf;\n"
			"#elif USE_BIT_OPERATION ==0\n "
			"	float4 val = float4(0, 0, 0, 0); half4 data2, data1; float4 buf1, buf2;\n "
			"#else\n"
			"	int4 val = int4(0, 0, 0, 0), data2, data1; int4 temp1, temp2; float4 buf;\n "
			"#endif\n"
			"   float2 index = pos.yx; \n"
			"   float2 stripe_size = size.xy * SIFT_PER_STRIPE;\n"
			"   float2 stripe_index = floor(index /stripe_size);\n"
			"   index = floor(fmod(index,  stripe_size));\n"
			"   float2 index_v = floor(index / SIFT_PER_STRIPE) + 0.5;\n "
			"   float2 index_h = fmod(index, SIFT_PER_STRIPE);\n"
			"   float2 tx = (index_h + stripe_index * SIFT_PER_STRIPE)* PIXEL_PER_SIFT + 0.5;\n"
			"   float2 tpos1, tpos2; \n"
			"	float4 tpos = float4(tx, index_v);\n"
			"   for(int i = 0; i < PIXEL_PER_SIFT; ++i){\n"
			"#if UNSIGNED_BYTE_SIFT == 1\n"
			"#if USE_BIT_OPERATION == 0\n"
			"		buf2 = texRECT(tex2, tpos.yw);\n"
			"		buf1 = texRECT(tex1, tpos.xz);\n"
			"		for(int k = 0; k < 4; ++k) {\n"
			"			data2 = unpack_4ubyte(buf2[k]);\n"
			"			data1 = unpack_4ubyte(buf1[k]);\n"
			"			val += float4(data1 * data2);\n"
			"		}\n"
			"#else\n"
			"		buf = texRECT(tex2, tpos.yw);\n"
			"		temp1 = floatToRawIntBits(buf);\n"
			"		buf = texRECT(tex1, tpos.xz);\n"
			"		temp2 = floatToRawIntBits(buf);\n"
			"		data1 = temp1 & 0xff;\n"
			"		data2 = temp2 & 0xff;\n"
			"		val +=  (data1 * data2);\n"
			"		data1 = (temp1 >> 8) & 0xff;\n"
			"		data2 = (temp2 >> 8)  & 0xff;\n"
			"		val +=  (data1 * data2);\n"
			"		data1 = (temp1 >> 16)  & 0xff;\n"
			"		data2 = (temp2 >> 16)  & 0xff;\n"
			"		val +=  (data1 * data2);\n"
			"		data1 = (temp1 >> 24)  & 0xff;\n"
			"		data2 = (temp2 >> 24)  & 0xff;\n"
			"		val += (data1 * data2);\n"
			"#endif\n"
			"#else\n"
			"		buf = texRECT(tex2, tpos.yw);\n"
			"		data1 = texRECT(tex1, tpos.xz);\n"
			"		val += data1 * buf;\n"
			"#endif\n"
			"		tpos.xy = tpos.xy + float2(1.0, 1.0);\n"
			"	}\n"
			"#if USE_BIT_OPERATION == 0 || UNSIGNED_BYTE_SIFT == 0\n"
			"	const float factor = 0.248050689697265625; \n"
			"	result =float4(dot(val, factor.xxxx), index,  0);\n"
			"#else\n"
			"	const float factor = 0.000003814697265625; \n"
			"	result = float4(dot(float4(val), factor.xxxx), index, 0); \n"
			"#endif\n"
			"}"
		<<	'\0';

	s_multiply = new ProgramCG(buffer); 

	_param_multiply_tex1 = cgGetNamedParameter(*s_multiply, "tex1");
	_param_multiply_tex2 = cgGetNamedParameter(*s_multiply, "tex2");
	_param_multiply_size = cgGetNamedParameter(*s_multiply, "size");


	out.seekp(ios::beg);
	out <<	"#define SIFT_PER_STRIPE " << _sift_per_stripe << "\n" 
			"#define UNSIGNED_BYTE_SIFT " <<_use_packed_des << "\n"
			"#define PIXEL_PER_SIFT " << _pixel_per_sift << "\n"
			"void main(uniform samplerRECT tex1,		\n"
			"uniform samplerRECT tex2,			\n"
			"uniform samplerRECT texL1, \n"
			"uniform samplerRECT texL2, \n"
			"uniform float3x3 H, \n"
			"uniform float3x3 F, \n"
			"uniform float4	size,  \n"
			"in		float2	pos : WPOS,	\n"
			"out	float4  result:COLOR0)		\n"
		    "{\n"
		<<	"#if UNSIGNED_BYTE_SIFT == 0\n"
			"   float4 val = float4(0, 0, 0, 0), data1, buf;\n"
			"#else\n "
			"	float4 val = float4(0, 0, 0, 0); half4 data2, data1; float4 buf1, buf2;\n "
			"#endif\n"
			"   float2 index = pos.yx; \n"
			"   float2 stripe_size = size.xy * SIFT_PER_STRIPE;\n"
			"   float2 stripe_index = floor(index /stripe_size);\n"
			"   index = floor(fmod(index,  stripe_size));\n"
			"   float2 index_v = floor(index / SIFT_PER_STRIPE) + 0.5;\n "
			"   float2 index_h = fmod(index, SIFT_PER_STRIPE);\n"

			//read feature location data
			"   float4 tlpos = float4((index_h + stripe_index * SIFT_PER_STRIPE) + 0.5, index_v);\n"
			"   float3 loc1 = float3(texRECT(texL1, tlpos.xz).xw, 1);\n"
			"   float3 loc2 = float3(texRECT(texL2, tlpos.yw).xw, 1);\n"
			//check the guiding homography
			"   float3 hxloc1 = mul(H, loc1);\n"
			"   float2 diff = abs(loc2.xy- hxloc1.xy/hxloc1.z);\n"
			"   float disth = max(diff.x, diff.y);\n"
			"   if(disth > size.z ) {result = float4(0, index, 0); return;}\n"

			//check the guiding fundamental 
			"   float3 fx1 = mul(F, loc1), ftx2 = mul(loc2, F);\n"
			"   float x2tfx1 = dot(loc2, fx1);\n"
			"   float4 temp = float4(fx1.xy, ftx2.xy); \n"
			"   float sampson_error = (x2tfx1 * x2tfx1) / dot(temp, temp);\n"
			"   if(sampson_error > size.w) {result = float4(0, index, 0); return;}\n"
/**/
			//compare feature descriptor
			"   float2 tx = (index_h + stripe_index * SIFT_PER_STRIPE)* PIXEL_PER_SIFT + 0.5;\n"
			"   float2 tpos1, tpos2; \n"
			"	float4 tpos = float4(tx, index_v);\n"
			"   for(int i = 0; i < PIXEL_PER_SIFT; ++i){\n"
			"#if UNSIGNED_BYTE_SIFT == 1\n"
			"		buf2 = texRECT(tex2, tpos.yw);\n"
			"		buf1 = texRECT(tex1, tpos.xz);\n"
			"		for(int k = 0; k < 4; ++k) {\n"
			"			data2 = unpack_4ubyte(buf2[k]);\n"
			"			data1 = unpack_4ubyte(buf1[k]);\n"
			"			val += float4(data1 * data2);\n"
			"		}\n"
			"#else\n"
			"		buf = texRECT(tex2, tpos.yw);\n"
			"		data1 = texRECT(tex1, tpos.xz);\n"
			"		val += data1 * buf;\n"
			"#endif\n"
			"		tpos.xy = tpos.xy + float2(1.0, 1.0);\n"
			"	}\n"
			"	const float factor = 0.248050689697265625; \n"
			"	result =float4(dot(val, factor.xxxx), index,  0);\n"
			"}"
		<<	'\0';

	s_guided_mult = new ProgramCG(buffer);

	_param_guided_mult_tex1 = cgGetNamedParameter(*s_guided_mult, "tex1");
	_param_guided_mult_tex2= cgGetNamedParameter(*s_guided_mult, "tex2");
	_param_guided_mult_texl1 = cgGetNamedParameter(*s_guided_mult, "texL1");
	_param_guided_mult_texl2 = cgGetNamedParameter(*s_guided_mult, "texL2");
	_param_guided_mult_h = cgGetNamedParameter(*s_guided_mult, "H");
	_param_guided_mult_f = cgGetNamedParameter(*s_guided_mult, "F");
	_param_guided_mult_param = cgGetNamedParameter(*s_guided_mult, "size");


	//row max
	out.seekp(ios::beg);
	out <<	"#define BLOCK_WIDTH "  << 16 << "\n"
			"void main (uniform samplerRECT tex, \n"
			"uniform float3 param, in float2 pos : WPOS, \n"
			"out float4 result: COLOR0)\n"
			"{\n"
			"	float index = pos.x + floor(pos.y) * BLOCK_WIDTH; \n"
			"	float2 bestv = -1; float imax = -1;\n"
			"	for(float i = 0; i < param.x; i ++){\n "
			"		float v = texRECT(tex, float2(i + 0.5, index)).r; \n"
			"		imax = v > bestv.r ? i : imax; \n "
			"		bestv  = v > bestv.r? float2(v, bestv.r) : max(bestv, v.xx);\n "
			"	}\n"
			"	bestv = acos(min(bestv, 1.0));\n"
			"	if(bestv.x >= param.y || bestv.x >= param.z * bestv.y) imax = -1;\n"
			"	result = float4(imax, bestv, index);\n"
			"}"
		<<  '\0';
	s_row_max = new ProgramCG(buffer); 
	_param_rowmax_param = cgGetNamedParameter(*s_row_max, "param");

	out.seekp(ios::beg);
	out <<	"#define BLOCK_WIDTH "  << 16 << "\n"
			"void main (uniform samplerRECT tex, \n"
			"uniform float3 param, in float2 pos : WPOS, \n"
			"out float4 result: COLOR0)\n"
			"{\n"
			"	float index = pos.x + floor(pos.y) * BLOCK_WIDTH; \n"
			"	float2 bestv = -1; float imax = -1;\n"
			"	for(float i = 0; i < param.x; i ++){\n "
			"		float v = texRECT(tex, float2(index, i + 0.5)).r; \n"
			"		imax = (v > bestv.r)? i : imax; \n "
			"		bestv  = v > bestv.r? float2(v, bestv.r) : max(bestv, v.xx);\n "
			"	}\n"
			"	bestv = acos(min(bestv, 1.0));\n"
			"	if(bestv.x >= param.y || bestv.x >= param.z * bestv.y) imax = -1;\n"
			"	result = float4(imax, bestv, index);\n"
			"}"
		<<  '\0';
	s_col_max = new ProgramCG(buffer); 
	_param_colmax_param = cgGetNamedParameter(*s_col_max, "param");
}

int  SiftMatchCG::GetGuidedSiftMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax,
	float H[3][3], float hdistmax, float F[3][3], float fdistmax, int mbm)
{

	int dw = _num_sift[1];
	int dh = _num_sift[0]; 
	if(_initialized ==0) return 0;
	if(dw <= 0 || dh <=0) return 0;
	if(_have_loc[0] == 0 || _have_loc[1] == 0) return 0;

	FrameBufferObject fbo;
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	_texDot.SetImageSize(dw, dh);


	//data
	_texDot.AttachToFBO(0);
	_texDot.FitTexViewPort();
	glActiveTexture(GL_TEXTURE0);
	_texDes[0].BindTex();
	glActiveTexture(GL_TEXTURE1);
	_texDes[1].BindTex();
	glActiveTexture(GL_TEXTURE2);
	_texLoc[1].BindTex();
	glActiveTexture(GL_TEXTURE3);
	_texLoc[2].BindTex();
	//set parameters
	cgGLSetTextureParameter(_param_guided_mult_tex1, _texDes[0]);	
	cgGLEnableTextureParameter(_param_guided_mult_tex1);
	cgGLSetTextureParameter(_param_guided_mult_tex2, _texDes[1]);	
	cgGLEnableTextureParameter(_param_guided_mult_tex2);
	cgGLSetTextureParameter(_param_guided_mult_texl1, _texLoc[0]);	
	cgGLEnableTextureParameter(_param_guided_mult_texl1);
	cgGLSetTextureParameter(_param_guided_mult_texl2, _texLoc[1]);	
	cgGLEnableTextureParameter(_param_guided_mult_texl2);
	cgSetMatrixParameterfr(_param_guided_mult_h, H[0]);
	cgSetMatrixParameterfr(_param_guided_mult_f, F[0]);
	float dot_param[4] = {(float)_texDes[0].GetDrawHeight(), (float) _texDes[1].GetDrawHeight(), hdistmax, fdistmax};
	cgGLSetParameter4fv(_param_guided_mult_param, dot_param);

	//multiply the descriptor matrices
	s_guided_mult->UseProgram();
	_texDot.DrawQuad();

	GLTexImage::UnbindMultiTex(4);

	return GetBestMatch(max_match, match_buffer, distmax, ratiomax, mbm);
}

int SiftMatchCG::GetBestMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax, int mbm)
{

	glActiveTexture(GL_TEXTURE0);
	_texDot.BindTex();


	//readback buffer
	sift_buffer.resize(_num_sift[0] + _num_sift[1] + 16);
	float * buffer1 = &sift_buffer[0], * buffer2 = &sift_buffer[_num_sift[0]];

	//row max
	_texMatch[0].AttachToFBO(0);
	_texMatch[0].SetImageSize(16, ( _num_sift[0] + 15) / 16);
	_texMatch[0].FitTexViewPort();
	cgGLSetParameter3f(_param_rowmax_param, (float)_num_sift[1], distmax, ratiomax);
	s_row_max->UseProgram();
	_texMatch[0].DrawQuad();
	glReadPixels(0, 0, 16, (_num_sift[0] + 15)/16, GL_RED, GL_FLOAT, buffer1);


	//col max
	if(mbm)
	{
		_texMatch[1].AttachToFBO(0);
		_texMatch[1].SetImageSize(16, (_num_sift[1] + 15) / 16);
		_texMatch[1].FitTexViewPort();
		cgGLSetParameter3f(_param_colmax_param, (float)_num_sift[0], distmax, ratiomax);
		s_col_max->UseProgram();
		_texMatch[1].DrawQuad();
		glReadPixels(0, 0, 16, (_num_sift[1] + 15) / 16, GL_RED, GL_FLOAT, buffer2);
	}

	//unload
	cgGLUnbindProgram(ProgramCG::_FProfile);
	cgGLDisableProfile(ProgramCG::_FProfile);
	GLTexImage::UnbindMultiTex(2);


	//write back the matches
	int nmatch = 0, j ;
	for(int i = 0; i < _num_sift[0] && nmatch < max_match; ++i)
	{
		j = int(buffer1[i]);
		if( j>= 0 && (!mbm ||int(buffer2[j]) == i))
		{
			match_buffer[nmatch][0] = i;
			match_buffer[nmatch][1] = j;
			nmatch++;
		}
	}
	return nmatch;

}

int  SiftMatchCG::GetSiftMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax, int mbm)
{
	int dw = _num_sift[1];
	int dh = _num_sift[0]; 
	if(_initialized ==0) return 0;
	if(dw <= 0 || dh <=0) return 0;

	FrameBufferObject fbo;
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	_texDot.SetImageSize(dw, dh);


	//data
	_texDot.AttachToFBO(0);
	_texDot.FitTexViewPort();
	glActiveTexture(GL_TEXTURE0);
	_texDes[0].BindTex();
	glActiveTexture(GL_TEXTURE1);
	_texDes[1].BindTex();

	//set parameters
	cgGLSetTextureParameter(_param_multiply_tex1, _texDes[0]);	
	cgGLEnableTextureParameter(_param_multiply_tex1);
	cgGLSetTextureParameter(_param_multiply_tex2, _texDes[1]);	
	cgGLEnableTextureParameter(_param_multiply_tex2);
	float heights[2] = {(float)_texDes[0].GetDrawHeight(), (float)_texDes[1].GetDrawHeight()};
	cgGLSetParameter2fv(_param_multiply_size, heights);

	//multiply the descriptor matrices
	s_multiply->UseProgram();
	_texDot.DrawQuad();


	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GlobalUtil::_texTarget, 0);

	return GetBestMatch(max_match, match_buffer, distmax, ratiomax, mbm);
}


int SiftMatchGPU::CreateContextGL()
{
	//use GLUT to create an OpenGL Context?
	if(!GlobalUtil::CreateWindowGLUT()) return 0;
	return VerifyContextGL();
}


int SiftMatchGPU::VerifyContextGL()
{
	matcher->InitSiftMatchCG();
	return GlobalUtil::_HaveGlSupport;
}

void* SiftMatchGPU::operator new (size_t  size){
  void * p = malloc(size);
  if (p == 0)  
  {
	  const std::bad_alloc ba;
	  throw ba; 
  }
  return p; 
}


SiftMatchGPU::SiftMatchGPU(int max_sift)
{
	matcher = new SiftMatchCG(max_sift);
}


void SiftMatchGPU::SetMaxSift(int max_sift)
{
	matcher->SetMaxSift(max_sift);
}

SiftMatchGPU::~SiftMatchGPU()
{
	delete matcher;
}

void SiftMatchGPU::SetDescriptors(int index, int num, const unsigned char* descriptors, int id)
{
	matcher->SetDescriptors(index, num,  descriptors, id);
}

void SiftMatchGPU::SetDescriptors(int index, int num, const float* descriptors, int id)
{
	matcher->SetDescriptors(index, num, descriptors, id);
}

void SiftMatchGPU::SetFeautreLocation(int index, const float* locations, int gap)
{
	matcher->SetFeautreLocation(index, locations, gap);

}
int  SiftMatchGPU::GetGuidedSiftMatch(int max_match, int match_buffer[][2], float H[3][3], float F[3][3], 
				float distmax, float ratiomax, float hdistmax, float fdistmax, int mutual_best_match)
{
	if(H == NULL && F == NULL)
	{
		return matcher->GetSiftMatch(max_match, match_buffer, distmax, ratiomax, mutual_best_match);
	}else
	{
		float Z[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}}, ti = (float)GlobalUtil::_texMaxDimGL;
		return matcher->GetGuidedSiftMatch(max_match, match_buffer, distmax, ratiomax, 
		H? H : Z, H? hdistmax: ti, F? F : Z, F? fdistmax: ti, mutual_best_match);
	}
}
int  SiftMatchGPU::GetSiftMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax, int mutual_best_match)
{
	//ClockTimer timer("Sift Match 100");
	//for(int i = 0;i < 100; ++i)matcher->GetSiftMatch(max_match, match_buffer, distmax, ratiomax);
	//timer.StopTimer();
	return matcher->GetSiftMatch(max_match, match_buffer, distmax, ratiomax, mutual_best_match);
}

SiftMatchGPU* CreateNewSiftMatchGPU(int max_sift)
{
	return new SiftMatchGPU(max_sift);
}
