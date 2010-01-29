//////////////////////////////////////////////////////////////////////////////
//	File:		ProgramCG.cpp
//	Author:		Changchang Wu
//	Description :	implementation of cg related class.
//		class ProgramCG			A simple wrapper of Cg programs
//		class ShaderBagCG		cg shaders for SIFT
//		class FilterCGGL		cg gaussian filters for SIFT
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
#include "ShaderMan.h"
#include "FrameBufferObject.h"

#include "stdlib.h"
#include "math.h"

#if _MSC_VER == 1200
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif

#ifdef _WIN32
//	#include "imdebug/imdebuggl.h"
//	#pragma comment (lib, "../lib/imdebug.lib")
	#pragma comment (lib, "../lib/cg.lib")
	#pragma comment (lib, "../lib/cggl.lib")
#endif

CGcontext	ProgramCG::_Context	=0;
CGprofile	ProgramCG::_FProfile;
CGprofile	ProgramCG::_GProfile = CG_PROFILE_UNKNOWN;

//ALTERNATIVE CG.v.s. GLSL
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ProgramCG::ProgramCG()
{
	_programID = NULL;
}

ProgramCG::~ProgramCG()
{
	if(_programID) cgDestroyProgram(_programID);
}

ProgramCG::ProgramCG(const char *code, const char** cg_compile_args, CGprofile profile)
{
	_profile = profile;
	GLint epos;
	const char* ati_args[] = {"-po", "ATI_draw_buffers",0}; 
	const char* fp40_args[] = {"-ifcvt", "none","-unroll", "all", 0};
	if(cg_compile_args == NULL) cg_compile_args = GlobalUtil::_IsATI? ati_args : (GlobalUtil::_SupportFP40? fp40_args:NULL);
	_programID = ::cgCreateProgram(_Context, CG_SOURCE, code, profile, NULL, cg_compile_args);
	if(_programID)
	{
		cgGLLoadProgram(_programID );
		_texParamID = cgGetNamedParameter(_programID, "tex");


		glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &epos);
		if(epos >=0)
		{
			std::cout<<cgGetProgramString(_programID, CG_COMPILED_PROGRAM)<<endl;
			std::cerr<<glGetString(GL_PROGRAM_ERROR_STRING_ARB)<<endl;
		}
	}else
	{
		std::cerr<<code<<endl;
		glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &epos);
		if(epos >=0)
		{
			std::cout<<cgGetProgramString(_programID, CG_COMPILED_PROGRAM)<<endl;
			std::cerr<<glGetString(GL_PROGRAM_ERROR_STRING_ARB)<<endl;
		}else
		{
			std::cout<<glGetString(GL_PROGRAM_ERROR_STRING_ARB)<<endl;
		}
	}

}
void ProgramCG::InitContext()
{
	if(_Context == 0)
	{
		_Context	= cgCreateContext();
 
		/////////////
		_FProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);
		cgGLSetOptimalOptions(_FProfile);

		//_GProfile = cgGLGetLatestProfile(CG_GL_GEOMETRY);
		//just assume CG_GL_GEOMETRY = CG_GL_FRAGMENT + 1
		//for compatible with old version of cg header files

		if(GlobalUtil::_DescriptorPPT == 32 && GlobalUtil::_SupportGP4GP)
		{
			_GProfile = cgGLGetLatestProfile(CGGLenum(CG_GL_FRAGMENT+1));
			cgGLSetOptimalOptions(_GProfile);
		}else
		{	
			GlobalUtil::_DescriptorPPT = min(GlobalUtil::_DescriptorPPT, 16);
		}

		const char * profilename = cgGetProfileString(_FProfile);
		if(
			strcmp(profilename, "fp40")==0 ||
			strcmp(profilename, "gp4fp")==0	||
			strcmp(profilename, "gp5fp")==0		)GlobalUtil::_SupportFP40 = 1;
		
		std::cout<<"Shader Profile: "<<cgGetProfileString(_FProfile)
			<<",\t"<<(_GProfile!=CG_PROFILE_UNKNOWN? cgGetProfileString(_GProfile): " ")<<endl;
	}
}

void ProgramCG::DestroyContext()
{
	cgDestroyContext(_Context);
}

ShaderBagCG::ShaderBagCG()
{
	cgSetErrorCallback(ErrorCallback);
	ProgramCG::InitContext();

}


int ProgramCG::UseProgram()
{
	if(_programID)
	{
		cgGLEnableProfile(_profile);
		cgGLBindProgram(_programID);

		return 1;
	}else
	{
		return 0;
	}
}

void ShaderBagCG::UnloadProgram()
{

	cgGLUnbindProgram(ProgramCG::_FProfile);
	cgGLDisableProfile(ProgramCG::_FProfile);
	if(GlobalUtil::_DescriptorPPT == 32 && ProgramCG::_GProfile!=CG_PROFILE_UNKNOWN)
	{
		cgGLUnbindProgram(ProgramCG::_GProfile);
		cgGLDisableProfile(ProgramCG::_GProfile);
	}

}

void ShaderBagCG::ErrorCallback()
{
	CGerror err = cgGetError();
	if(err)
	{
		std::cerr<< cgGetErrorString(err)<<endl;
	}
}
void ShaderBagCG::LoadFixedShaders()
{

	/*
	if(GlobalUtil::_SupportGP4GP && GlobalUtil::_DescriptorPPT == 32)
	{
		//debug geometry shader now
/		s_debug = new ProgramCG( "POINT POINT_OUT void main(AttribArray<float4> point: POSITION )\n\
		{float4 pos = point[0];\n\
		for(float i = 0.0; i < 32.0; i+=1.0){\n\
		emitVertex(pos:POSITION, float4(0.0, 1.0, 0.0, 1.0): COLOR0);\n\
		pos.x+=1.0;}}", 
			ProgramCG::_GProfile);

	}
*/
	char * display_gaussian_code=
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float r = texRECT(tex, TexCoord0.xy).r;\n\
	FragColor = float4(r, r, r, 1.0);}";

	s_display_gaussian =  new ProgramCG( display_gaussian_code);





	char *rgb2gray_code =
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float intensity = dot(float3(0.299, 0.587, 0.114), texRECT(tex,TexCoord0.xy ).rgb);\n\
	FragColor= float4(intensity, intensity, intensity, 1.0);}";//

	s_gray = new ProgramCG( rgb2gray_code);


	char* copy_rg_code=
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy);	FragColor = float4(cc.rg, 0.0, 0.0);	}";
	s_sampling = new ProgramCG(copy_rg_code);



	s_copy_key = new ProgramCG(
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	FragColor.rg= texRECT(tex, TexCoord0.xy).rg; FragColor.ba = float2(0,1);	}");
	
	s_texcoord = new ProgramCG(
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0){FragColor = TexCoord0;	}");
	
	s_display_dog =  new ProgramCG(
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float g = (0.5+20.0*texRECT(tex, TexCoord0.xy).g);\n\
	FragColor = float4(g, g, g, 1.0);}" );


	s_display_grad = new ProgramCG(
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy); FragColor = float4(5.0 * cc.bbb, 1.0); }");


	s_display_keys= new ProgramCG(
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy);\n\
	if(cc.r ==1.0) FragColor = float4(1.0, 0, 0,1.0); \n\
	else {if (cc.r ==0.5) FragColor = float4(0.0,1.0,0.0,1.0);	else discard;}}");		


	s_zero_pass = new ProgramCG("void main(out float4 FragColor : COLOR0){FragColor = 0;}");
/*

	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy);\n\
	if(cc.r > 0.0) FragColor = float4(cc.r, 0, 0, 1.0);	else discard;}");
*/

	//shader used to write a vertex buffer object
	//which is used to draw the quads of each feature
	ProgramCG * program;
	s_vertex_list = program = new ProgramCG(
	"void main(in float4 TexCoord0: TEXCOORD0,\n\
	uniform float4 sizes, \n\
	uniform samplerRECT tex, \n\
	out float4 FragColor: COLOR0){\n\
	float fwidth = sizes.y; \n\
	float twidth = sizes.z; \n\
	float rwidth = sizes.w; \n\
	float size = sizes.x; \n\
	float index = 0.1*(fwidth*floor(TexCoord0.y) + TexCoord0.x);\n\
	float px = fmod(index, twidth);\n\
	float2 tpos= floor(float2(px, index*rwidth))+0.5;\n\
	float4 cc = texRECT(tex, tpos );\n\
	FragColor.zw = float2(0.0, 1.0);\n\
	if(any(cc.xy <=0)) {FragColor.xy = cc.xy;}else \n\
	{\n\
		float type = frac(px);\n\
		float2 dxy; float s, c;\n\
		dxy.x = type < 0.1 ? 0 : ((type <0.5 || type > 0.9)? size : -size);\n\
		dxy.y = type < 0.2 ? 0 : ((type < 0.3 || type > 0.7 )? -size :size); \n\
		sincos(cc.b, s, c);\n\
		FragColor.x = cc.x + c*dxy.x-s*dxy.y;\n\
		FragColor.y = cc.y + c*dxy.y+s*dxy.x;}\n\
	}\n\0");
	/*FragColor = float4(tpos, 0.0, 1.0);}\n\0");*/

	_param_genvbo_size = cgGetNamedParameter(*program, "sizes");

	s_margin_copy = program = new ProgramCG(
	"void main(float4 texCoord0: TEXCOORD0, out float4 FragColor: COLOR0, \n\
	uniform samplerRECT tex, uniform float2 truncate){\n\
	FragColor = texRECT(tex, min(texCoord0.xy, truncate)); }");

	_param_margin_copy_truncate = cgGetNamedParameter(*program, "truncate");


	s_grad_pass = new ProgramCG(
	"void main (\
	float4 TexCC : TEXCOORD0, float4 TexLC : TEXCOORD1,\n\
	float4 TexRC : TEXCOORD2, float4 TexCD : TEXCOORD3, float4 TexCU : TEXCOORD4,\n\
	out float4 FragData0 : COLOR0, uniform samplerRECT tex)\n\
	{\n\
		float4 v1, v2, gg;\n\
		float4 cc  = texRECT(tex, TexCC.xy);\n\
		gg.x = texRECT(tex, TexLC.xy).r;\n\
		gg.y = texRECT(tex, TexRC.xy).r;\n\
		gg.z = texRECT(tex, TexCD.xy).r;\n\
		gg.w = texRECT(tex, TexCU.xy).r;\n\
		float2 dxdy = (gg.yw - gg.xz); \n\
		float grad = 0.5*length(dxdy);\n\
		float theta = grad==0? 0: atan2(dxdy.y, dxdy.x);\n\
		FragData0 = float4(cc.rg, grad, theta);\n\
	}\n\0");


	if(GlobalUtil::_SupportFP40)
	{
		//use the packing mode for cpu list reshape and two orientations
		if(GlobalUtil::_MaxOrientation != 2) GlobalUtil::_OrientationPack2 = 0;

		LoadOrientationShader();

		if(GlobalUtil::_DescriptorPPT == 32)
		{
			GlobalUtil::_DescriptorVPC = 1;
		}
		else if(GlobalUtil::_DescriptorPPT == 1 && GlobalUtil::_MaxDrawBuffers >= 8 &&
			strcmp(cgGetProfileString(ProgramCG::_FProfile), "gp4fp")==0) 
		{
			//this implementation requires dynamic indexing of gp4fp
			GlobalUtil::_DescriptorVPC = 4;
		}else if(GlobalUtil::_DescriptorPPT && GlobalUtil::_MaxDrawBuffers >= 32 / GlobalUtil::_DescriptorPPT)
		{
			//4 -> 8, 8 -> 4, 16 -> 2
			GlobalUtil::_DescriptorVPC = 1;
		}else if(GlobalUtil::_DescriptorPPT && GlobalUtil::_MaxDrawBuffers >=2)
		{
			GlobalUtil::_DescriptorPPT = 16;
			GlobalUtil::_DescriptorVPC = 1;
		}else
		{
			GlobalUtil::_DescriptorPPT = 0;
			GlobalUtil::_DescriptorVPC = 1;
		}

		if(GlobalUtil::_DescriptorPPT)		LoadDescriptorShader();

	}else
	{
		s_orientation = program =  new ProgramCG(
		"void main(out float4 FragColor : COLOR0, \n\
		uniform samplerRECT fTex, uniform samplerRECT oTex, \n\
		uniform float size, \n\
		in float2 tpos : TEXCOORD0){\n\
		float4 cc = texRECT(fTex, tpos);\n\
		float4 oo = texRECT(oTex, cc.rg);\n\
		FragColor = float4(cc.rg, oo.a, size);}");  
		_param_orientation_gtex= cgGetNamedParameter(*program, "oTex");
		_param_orientation_size= cgGetNamedParameter(*program, "size");


		///
		GlobalUtil::_FullSupported = 0;
		GlobalUtil::_MaxOrientation = 0;  //0 for simplified version
		GlobalUtil::_DescriptorPPT = 0;
		std::cerr<<"Orientation simplified on this hardware"<<endl;
		std::cerr<<"Descriptor ignored on this hardware"<<endl;
	}

}


void ShaderBagCG::SetMarginCopyParam(int xmax, int ymax)
{
	float truncate[2] = {xmax - 0.5f , ymax - 0.5f};
	cgGLSetParameter2fv(_param_margin_copy_truncate, truncate);
}


int ShaderBagCG::LoadKeypointShaderMR(float threshold, float edge_threshold)
{
	char buffer[10240];
	float threshold0 = threshold * 0.8f;
	float threshold1 = threshold;
	float threshold2 = (edge_threshold+1)*(edge_threshold+1)/edge_threshold;
	int   max_refine = max(2, GlobalUtil::_SubpixelLocalization);
	ostrstream out(buffer, 10240);
	out<<
	"void main (\
	float4 TexCC : TEXCOORD0, float4 TexLC : TEXCOORD1,\n\
	float4 TexRC : TEXCOORD2, float4 TexCD : TEXCOORD3, \n\
	float4 TexCU : TEXCOORD4, float4 TexLD : TEXCOORD5, \n\
	float4 TexLU : TEXCOORD6, float4 TexRD : TEXCOORD7,\n\
	out float4 FragData0 : COLOR0, out float4 FragData1 : COLOR1, \n\
	uniform samplerRECT tex, uniform samplerRECT texU, uniform samplerRECT texD)\n\
	{\n\
		float4 v1, v2, gg;\n\
		float2 TexRU = float2(TexRC.x, TexCU.y); \n\
		float4 cc  = texRECT(tex, TexCC.xy);\n\
		v1.x = texRECT(tex, TexLC.xy).g;\n\
		gg.x = texRECT(tex, TexLC.xy).r;\n\
		v1.y = texRECT(tex, TexRC.xy).g;\n\
		gg.y = texRECT(tex, TexRC.xy).r;\n\
		v1.z = texRECT(tex, TexCD.xy).g;\n\
		gg.z = texRECT(tex, TexCD.xy).r;\n\
		v1.w = texRECT(tex, TexCU.xy).g;\n\
		gg.w = texRECT(tex, TexCU.xy).r;\n\
		v2.x = texRECT(tex, TexLD.xy).g;\n\
		v2.y = texRECT(tex, TexLU.xy).g;\n\
		v2.z = texRECT(tex, TexRD.xy).g;\n\
		v2.w = texRECT(tex, TexRU.xy).g;\n\
		float2 dxdy = 0.5*(gg.yw - gg.xz); \n\
		float grad = length(dxdy);\n\
		float theta = grad==0? 0: atan2(dxdy.y, dxdy.x);\n\
		FragData0 = float4(cc.rg, grad, theta);\n"
	<<"\
		float dog = 0.0; \n\
		FragData1 = float4(0, 0, 0, 0); \n\
		float2 v3; float4 v4, v5, v6;\n"
	<<"\
		if( cc.g > "<<threshold0<<" && all(cc.gggg > max(v1, v2)))\n\
		{\n\
			v3.x = texRECT(texU, TexCC.xy).g;\n\
			v4.x = texRECT(texU, TexLC.xy).g;\n\
			v4.y = texRECT(texU, TexRC.xy).g;\n\
			v4.z = texRECT(texU, TexCD.xy).g;\n\
			v4.w = texRECT(texU, TexCU.xy).g;\n\
			v6.x = texRECT(texU, TexLD.xy).g;\n\
			v6.y = texRECT(texU, TexLU.xy).g;\n\
			v6.z = texRECT(texU, TexRD.xy).g;\n\
			v6.w = texRECT(texU, TexRU.xy).g;\n\
			if(cc.g < v3.x || any(cc.gggg<v4.xyzw || cc.gggg<v6.xyzw))return; \n\
			v3.y = texRECT(texD, TexCC.xy).g;\n\
			v5.x = texRECT(texD, TexLC.xy).g;\n\
			v5.y = texRECT(texD, TexRC.xy).g;\n\
			v5.z = texRECT(texD, TexCD.xy).g;\n\
			v5.w = texRECT(texD, TexCU.xy).g;\n\
			v6.x = texRECT(texD, TexLD.xy).g;\n\
			v6.y = texRECT(texD, TexLU.xy).g;\n\
			v6.z = texRECT(texD, TexRD.xy).g;\n\
			v6.w = texRECT(texD, TexRU.xy).g;\n\
			if(cc.g < v3.y || any(cc.gggg<v5.xyzw || cc.gggg<v6.xyzw))return; \n\
			dog = 1.0; \n\
		}\n"
	//the minimum case
	<<"\
	  else if(cc.g < "<<-threshold0<<" && all(cc.gggg < min(v1, v2)))\n\
	  {\n\
			v3.x = texRECT(texU, TexCC.xy).g;\n\
			v4.x = texRECT(texU, TexLC.xy).g;\n\
			v4.y = texRECT(texU, TexRC.xy).g;\n\
			v4.z = texRECT(texU, TexCD.xy).g;\n\
			v4.w = texRECT(texU, TexCU.xy).g;\n\
			v6.x = texRECT(texU, TexLD.xy).g;\n\
			v6.y = texRECT(texU, TexLU.xy).g;\n\
			v6.z = texRECT(texU, TexRD.xy).g;\n\
			v6.w = texRECT(texU, TexRU.xy).g;\n\
			if(cc.g > v3.x || any(cc.gggg>v4.xyzw || cc.gggg>v6.xyzw))return; \n\
			v3.y = texRECT(texD, TexCC.xy).g;\n\
			v5.x = texRECT(texD, TexLC.xy).g;\n\
			v5.y = texRECT(texD, TexRC.xy).g;\n\
			v5.z = texRECT(texD, TexCD.xy).g;\n\
			v5.w = texRECT(texD, TexCU.xy).g;\n\
			v6.x = texRECT(texD, TexLD.xy).g;\n\
			v6.y = texRECT(texD, TexLU.xy).g;\n\
			v6.z = texRECT(texD, TexRD.xy).g;\n\
			v6.w = texRECT(texD, TexRU.xy).g;\n\
			if(cc.g > v3.y || any(cc.gggg>v5.xyzw || cc.gggg>v6.xyzw))return; \n\
			dog = 0.5 ; \n\
		}\n\
		else\n\
			return;\n"
	<<"\
	  int i = 0; \n\
	  float2 offset = float2(0, 0);\n\
	  float2 offsets = float2(0, 0);\n\
	  float3 dxys;		bool key_moved;	\n\
	  float fx, fy, fs; \n\
	  float fxx, fyy, fxy; \n\
	  float fxs, fys, fss; \n\
	  do\n\
	  {\n\
		dxys = float3(0, 0, 0);\n\
		offset = float2(0, 0);\n\
		float4 D2 = v1.xyzw - cc.gggg;\n\
		fxx = D2.x + D2.y;\n\
		fyy = D2.z + D2.w;\n\
		float2 D4 = v2.xw - v2.yz;\n\
		fxy = 0.25*(D4.x + D4.y);\n\
		float2 D5 = 0.5*(v1.yw-v1.xz); \n\
		fx = D5.x;\n\
		fy = D5.y ; \n\
		fs = 0.5*( v3.x - v3.y ); \n\
		fss = v3.x + v3.y - cc.g - cc.g;\n\
		fxs = 0.25 * ( v4.y + v5.x - v4.x - v5.y);\n\
		fys = 0.25 * ( v4.w + v5.z - v4.z - v5.w);\n\
		float4 A0, A1, A2 ;			\n\
		A0 = float4(fxx, fxy, fxs, -fx);	\n\
		A1 = float4(fxy, fyy, fys, -fy);	\n\
		A2 = float4(fxs, fys, fss, -fs);	\n\
		float3 x3 = abs(float3(fxx, fxy, fxs));		\n\
		float maxa = max(max(x3.x, x3.y), x3.z);	\n\
		if(maxa > 1e-10 )							\n\
		{\n\
			if(x3.y ==maxa )							\n\
			{											\n\
				float4 TEMP = A1; A1 = A0; A0 = TEMP;	\n\
			}else if( x3.z == maxa )					\n\
			{											\n\
				float4 TEMP = A2; A2 = A0; A0 = TEMP;	\n\
			}											\n\
			A0 /= A0.x;									\n\
			A1 -= A1.x * A0;							\n\
			A2 -= A2.x * A0;							\n\
			float2 x2 = abs(float2(A1.y, A2.y));		\n\
			if( x2.y > x2.x )							\n\
			{											\n\
				float3 TEMP = A2.yzw;					\n\
				A2.yzw = A1.yzw;						\n\
				A1.yzw = TEMP;							\n\
				x2.x = x2.y;							\n\
			}											\n\
			if(x2.x > 1e-10)							\n\
			{\n\
				A1.yzw /= A1.y;							\n\
				A2.yzw -= A2.y * A1.yzw;				\n\
				if(abs(A2.z) > 1e-10)					\n\
				{\n"
	// compute dx, dy, ds: 
	<<"\
					dxys.z = A2.w /A2.z;						\n\
					dxys.y = A1.w - dxys.z*A1.z;			    \n\
					dxys.x = A0.w - dxys.z*A0.z - dxys.y*A0.y;	\n\
				}\n\
			}\n\
		}\n\
		offset.x = dxys.x > 0.6 ? 1 : 0 + dxys.x < -0.6 ? -1 : 0;\n\
		offset.y = dxys.y > 0.6 ? 1 : 0 + dxys.y < - 0.6? -1 : 0;\n\
		i++; key_moved = i < "<<max_refine<<" && any(abs(offset)>0) ;	\n\
		if(key_moved)\n\
		{\n\
			offsets += offset; \n\
		    cc  =  texRECT(tex, TexCC.xy  + offsets);\n\
			v1.x = texRECT(tex , TexLC.xy + offsets).g;\n\
			v1.y = texRECT(tex , TexRC.xy + offsets).g;\n\
			v1.z = texRECT(tex , TexCD.xy + offsets).g;\n\
			v1.w = texRECT(tex , TexCU.xy + offsets).g;\n\
			v2.x = texRECT(tex , TexLD.xy + offsets).g;\n\
			v2.y = texRECT(tex , TexLU.xy + offsets).g;\n\
			v2.z = texRECT(tex , TexRD.xy + offsets).g;\n\
			v2.w = texRECT(tex , TexRU.xy + offsets).g;\n\
			v3.x = texRECT(texU, TexCC.xy + offsets).g;\n\
			v4.x = texRECT(texU, TexLC.xy + offsets).g;\n\
			v4.y = texRECT(texU, TexRC.xy + offsets).g;\n\
			v4.z = texRECT(texU, TexCD.xy + offsets).g;\n\
			v4.w = texRECT(texU, TexCU.xy + offsets).g;\n\
			v3.y = texRECT(texD, TexCC.xy + offsets).g;\n\
			v5.x = texRECT(texD, TexLC.xy + offsets).g;\n\
			v5.y = texRECT(texD, TexRC.xy + offsets).g;\n\
			v5.z = texRECT(texD, TexCD.xy + offsets).g;\n\
			v5.w = texRECT(texD, TexCU.xy + offsets).g;\n\
		}\n\
	  }while(key_moved);\n"
	  <<"\
	  bool test1 = (abs(cc.g + 0.5*dot(float3(fx, fy, fs), dxys ))>"<<threshold1<<") ;\n\
	  float test2_v1= fxx*fyy - fxy *fxy; \n\
	  float test2_v2 = (fxx+fyy); \n\
	  test2_v2 = test2_v2*test2_v2;\n\
	  bool test2 = test2_v1>0 && test2_v2 < "<<threshold2<<"*test2_v1; \n "
    //keep the point when the offset is less than 1
	<<"\
	  FragData1 = test1 && test2 && all( abs(dxys) < 1)? float4( dog, dxys.xy+offsets, dxys.z) : float4(0, 0, 0, 0); \n\
	}\n"	
	<<'\0';

	ProgramCG * program; 
	s_keypoint = program = new ProgramCG(buffer);
	//parameter
	_param_dog_texu = cgGetNamedParameter(*program, "texU");
	_param_dog_texd = cgGetNamedParameter(*program, "texD");

	return 1;

}

//keypoint detection shader
//1. compare with 26 neighbours
//2. sub-pixel sub-scale localization
//3. output: [dog, offset(x,y,s)]

void ShaderBagCG:: LoadKeypointShader(float threshold, float edge_threshold)
{
	char buffer[10240];
	float threshold0 = threshold* (GlobalUtil::_SubpixelLocalization?0.8f:1.0f);
	float threshold1 = threshold;
	float threshold2 = (edge_threshold+1)*(edge_threshold+1)/edge_threshold;
	ostrstream out(buffer, 10240);
	out<<setprecision(8);
	//tex(X)(Y)
	//X: (CLR) (CENTER 0, LEFT -1, RIGHT +1)  
	//Y: (CDU) (CENTER 0, DOWN -1, UP    +1) 

	out<<
	"void main (\
	float4 TexCC : TEXCOORD0, float4 TexLC : TEXCOORD1,\n\
	float4 TexRC : TEXCOORD2, float4 TexCD : TEXCOORD3, \n\
	float4 TexCU : TEXCOORD4, float4 TexLD : TEXCOORD5, \n\
	float4 TexLU : TEXCOORD6, float4 TexRD : TEXCOORD7,\n\
	out float4 FragData0 : COLOR0, out float4 FragData1 : COLOR1, \n\
	uniform samplerRECT tex, uniform samplerRECT texU, uniform samplerRECT texD)\n\
	{\n\
		float4 v1, v2, gg;\n\
		float2 TexRU = float2(TexRC.x, TexCU.y); \n\
		float4 cc  = texRECT(tex, TexCC.xy);\n\
		v1.x = texRECT(tex, TexLC.xy).g;\n\
		gg.x = texRECT(tex, TexLC.xy).r;\n\
		v1.y = texRECT(tex, TexRC.xy).g;\n\
		gg.y = texRECT(tex, TexRC.xy).r;\n\
		v1.z = texRECT(tex, TexCD.xy).g;\n\
		gg.z = texRECT(tex, TexCD.xy).r;\n\
		v1.w = texRECT(tex, TexCU.xy).g;\n\
		gg.w = texRECT(tex, TexCU.xy).r;\n\
		v2.x = texRECT(tex, TexLD.xy).g;\n\
		v2.y = texRECT(tex, TexLU.xy).g;\n\
		v2.z = texRECT(tex, TexRD.xy).g;\n\
		v2.w = texRECT(tex, TexRU.xy).g;\n\
		float2 dxdy = (gg.yw - gg.xz); \n\
		float grad = 0.5*length(dxdy);\n\
		float theta = grad==0? 0: atan2(dxdy.y, dxdy.x);\n\
		FragData0 = float4(cc.rg, grad, theta);\n"

	//test against 8 neighbours
	//use variable to identify type of extremum
	//1.0 for local maximum and 0.5 for minimum
	<<"\
		float dog = 0.0; \n\
		FragData1 = float4(0, 0, 0, 0); \n\
		dog = cc.g > "<<threshold0 <<" && all(cc.gggg > max(v1, v2))?1.0: 0.0;\n\
		dog = cc.g < "<<-threshold0<<" && all(cc.gggg < min(v1, v2))?0.5: dog;\n\
		if(dog == 0.0) return;\n"

	//do edge supression first.. 
	//vector v1 is < (-1, 0), (1, 0), (0,-1), (0, 1)>
	//vector v2 is < (-1,-1), (-1,1), (1,-1), (1, 1)>

	<<"\
		float fxx, fyy, fxy; \n\
		float4 D2 = v1.xyzw - cc.gggg;\n\
		float2 D4 = v2.xw - v2.yz;\n\
		fxx = D2.x + D2.y;\n\
		fyy = D2.z + D2.w;\n\
		fxy = 0.25*(D4.x + D4.y);\n\
		float fxx_plus_fyy = fxx + fyy;\n\
		float score_up = fxx_plus_fyy*fxx_plus_fyy; \n\
		float score_down = (fxx*fyy - fxy*fxy);\n\
		if( score_down <= 0 || score_up > "<<threshold2<<" * score_down)return;\n"
	//...
	<<" \
		float2 D5 = 0.5*(v1.yw-v1.xz); \n\
		float fx = D5.x, fy = D5.y ; \n\
		float fs, fss , fxs, fys ; \n\
		float2 v3; float4 v4, v5, v6;\n"
	//read 9 pixels of upper level
	<<"\
		v3.x = texRECT(texU, TexCC.xy).g;\n\
		v4.x = texRECT(texU, TexLC.xy).g;\n\
		v4.y = texRECT(texU, TexRC.xy).g;\n\
		v4.z = texRECT(texU, TexCD.xy).g;\n\
		v4.w = texRECT(texU, TexCU.xy).g;\n\
		v6.x = texRECT(texU, TexLD.xy).g;\n\
		v6.y = texRECT(texU, TexLU.xy).g;\n\
		v6.z = texRECT(texU, TexRD.xy).g;\n\
		v6.w = texRECT(texU, TexRU.xy).g;\n"
	//compare with 9 pixels of upper level
	//read and compare with 9 pixels of lower level
	//the maximum case
	<<"\
		if(dog == 1.0)\n\
		{\n\
			bool4 test = cc.gggg<v4 || cc.gggg<v6; \n\
			if(cc.g < v3.x || any(test.xy||test.zw))return; \n\
			v3.y = texRECT(texD, TexCC.xy).g;\n\
			v5.x = texRECT(texD, TexLC.xy).g;\n\
			v5.y = texRECT(texD, TexRC.xy).g;\n\
			v5.z = texRECT(texD, TexCD.xy).g;\n\
			v5.w = texRECT(texD, TexCU.xy).g;\n\
			v6.x = texRECT(texD, TexLD.xy).g;\n\
			v6.y = texRECT(texD, TexLU.xy).g;\n\
			v6.z = texRECT(texD, TexRD.xy).g;\n\
			v6.w = texRECT(texD, TexRU.xy).g;\n\
			test = cc.gggg<v5 || cc.gggg<v6; \n\
			if(cc.g < v3.y || any(test.xy||test.zw))return; \n\
		}\n"
	//the minimum case
	<<"\
		else{\n\
			bool4 test = cc.gggg>v4 || cc.gggg>v6; \n\
			if(cc.g > v3.x || any(test.xy||test.zw))return; \n\
			v3.y = texRECT(texD, TexCC.xy).g;\n\
			v5.x = texRECT(texD, TexLC.xy).g;\n\
			v5.y = texRECT(texD, TexRC.xy).g;\n\
			v5.z = texRECT(texD, TexCD.xy).g;\n\
			v5.w = texRECT(texD, TexCU.xy).g;\n\
			v6.x = texRECT(texD, TexLD.xy).g;\n\
			v6.y = texRECT(texD, TexLU.xy).g;\n\
			v6.z = texRECT(texD, TexRD.xy).g;\n\
			v6.w = texRECT(texD, TexRU.xy).g;\n\
			test = cc.gggg>v5 || cc.gggg>v6; \n\
			if(cc.g > v3.y || any(test.xy||test.zw))return; \n\
		}\n";

	if(GlobalUtil::_SubpixelLocalization)

	// sub-pixel localization FragData1 = float4(dog, 0, 0, 0); return;
	out <<" \
		fs = 0.5*( v3.x - v3.y ); //bug fix 9/12/2007 \n\
		fss = v3.x + v3.y - cc.g - cc.g;\n\
		fxs = 0.25 * ( v4.y + v5.x - v4.x - v5.y);\n\
		fys = 0.25 * ( v4.w + v5.z - v4.z - v5.w);\n"
	
	///////////////////////////////////////////////////////////////// 
	// let dog difference be quatratic function  of dx, dy, ds; 
	// df(dx, dy, ds) = fx * dx + fy*dy + fs * ds + 
	//				  + 0.5 * ( fxx * dx * dx + fyy * dy * dy + fss * ds * ds)
	//				  + (fxy * dx * dy + fxs * dx * ds + fys * dy * ds)
	// (fx, fy, fs, fxx, fyy, fss, fxy, fxs, fys are the derivatives)
	
	//the local extremum satisfies
	// df/dx = 0, df/dy = 0, df/dz = 0
	
	//that is 
	// |-fx|     | fxx fxy fxs |   |dx|
	// |-fy|  =  | fxy fyy fys | * |dy|
	// |-fs|     | fxs fys fss |   |ds|
	// need to solve dx, dy, ds

	// Use Gauss elimination to solve the linear system
    <<"\
		FragData1 = float4(dog, 0, 0, 0);	\n\
		float4 A0, A1, A2 ;			\n\
		A0 = float4(fxx, fxy, fxs, -fx);	\n\
		A1 = float4(fxy, fyy, fys, -fy);	\n\
		A2 = float4(fxs, fys, fss, -fs);	\n\
		float3 x3 = abs(float3(fxx, fxy, fxs));		\n\
		float maxa = max(max(x3.x, x3.y), x3.z);	\n\
		if(maxa < 1e-10 ) return;					\n\
		if(x3.y ==maxa )							\n\
		{											\n\
			float4 TEMP = A1; A1 = A0; A0 = TEMP;	\n\
		}else if( x3.z == maxa )					\n\
		{											\n\
			float4 TEMP = A2; A2 = A0; A0 = TEMP;	\n\
		}											\n\
		A0 /= A0.x;									\n\
		A1 -= A1.x * A0;							\n\
		A2 -= A2.x * A0;							\n\
		float2 x2 = abs(float2(A1.y, A2.y));		\n\
		if( x2.y > x2.x )							\n\
		{											\n\
			float3 TEMP = A2.yzw;					\n\
			A2.yzw = A1.yzw;						\n\
			A1.yzw = TEMP;							\n\
			x2.x = x2.y;							\n\
		}											\n\
		if(x2.x < 1e-10) return;					\n\
		A1.yzw /= A1.y;								\n\
		A2.yzw -= A2.y * A1.yzw;					\n\
		if(abs(A2.z) < 1e-10) return;\n"
	// compute dx, dy, ds: 
	<<"\
	    float3 dxys;							\n\
		dxys.z = A2.w /A2.z;				    \n\
		dxys.y = A1.w - dxys.z*A1.z;			    \n\
		dxys.x = A0.w - dxys.z*A0.z - dxys.y*A0.y;	\n"

	//one more threshold which I forgot in  versions prior to 286
	<<"\
	  bool bugfix_test = (abs(cc.g + 0.5*dot(float3(fx, fy, fs), dxys ))>"<<threshold1<<") ;\n"
    //keep the point when the offset is less than 1
	<<"\
		FragData1 = bugfix_test && all( abs(dxys) < 1.0)? float4( dog, dxys) : float4(0, 0, 0, 0); \n\
	}\n"	<<'\0';

	else		out<<"FragData1 =  float4( dog, 0, 0, 0) ;	}\n"	<<'\0';

	ProgramCG * program; 
	s_keypoint = program = new ProgramCG(buffer);
	//parameter
	_param_dog_texu = cgGetNamedParameter(*program, "texU");
	_param_dog_texd = cgGetNamedParameter(*program, "texD");




}


void ShaderBagCG::SetDogTexParam(int texU, int texD)
{
	cgGLSetTextureParameter(_param_dog_texu, texU);
	cgGLEnableTextureParameter(_param_dog_texu);
	cgGLSetTextureParameter(_param_dog_texd, texD);
	cgGLEnableTextureParameter(_param_dog_texd);
}

void ShaderBagCG::SetGenListStepParam(int tex, int tex0)
{
	cgGLSetTextureParameter(_param_genlist_step_tex, tex);
	cgGLEnableTextureParameter(_param_genlist_step_tex);
	cgGLSetTextureParameter(_param_genlist_step_tex0, tex0);
	cgGLEnableTextureParameter(_param_genlist_step_tex0);
}

void ShaderBagCG::SetGenVBOParam(float width, float fwidth, float size)
{
	float sizes[4] = {size*3.0f, fwidth, width, 1.0f/width};
	cgGLSetParameter4fv(_param_genvbo_size, sizes);
}


ProgramGPU* FilterGLCG::CreateFilterH(float kernel[], float offset[], int width)
{


	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	if(GlobalUtil::_BetaFilter)
	{
		out<< "void main(uniform samplerRECT tex,";
		out<<"\n\tin float4 TexCoord0: TEXCOORD0,";
		out<<"\n\tout float4 FragColor : COLOR0 )";
		out<<"\n{\n\tfloat4 intensity4 = float4(0, 0, 0, 0), data;\n";
		out<<"float or = texRECT(tex, TexCoord0.xy).r, intensity;\n";

		for(int i = 0; i< width; i+=4)
		{
			out <<"data = float4(";
			for(int j = i; j < i + 4; j++)
			{
				if(j != i) out <<", \n";
				if(j >= width)
				{
					out<<"0";
				}else if(offset[j]==0.0)
				{
					out<<"or";
				}else
				{
					out<<"texRECT(tex, TexCoord0.xy + float2(float("<<offset[j] <<") , 0)).r";
				}
			}
			out << ");\n";
			out << "intensity4 += data * float4(";
			for(int k = i; k < i + 4; k++)
			{
				if(k != i) out <<", ";
				if(k >= width)	out<<"0";
				else			out<<kernel[k];
			}
			out << ");\n";

 		}
		out << "intensity4.xy += intensity4.zw;\n";
		out << "intensity = intensity4.x + intensity4.y;\n";
	}else
	{
		out<< "void main(uniform samplerRECT tex,";
		out<<"\n\tin float4 TexCoord0: TEXCOORD0,";
		out<<"\n\tout float4 FragColor : COLOR0 )";
		out<<"\n{\n\tfloat intensity = 0.0 ;  float2 pos;\n";

		for(int i = 0; i< width; i++)
		{
			if(offset[i]==0.0)
			{
				out<<"float or = texRECT(tex, TexCoord0.xy).r;\n";
				out<<"intensity+= or * "<<kernel[i]<<";\n";

			}else
			{
				out<<"pos = TexCoord0.xy + float2(float("<<offset[i] <<") , 0);\n";
				out<<"intensity+= "<<kernel[i]<<"*texRECT(tex, pos).r;\n";
			}
		}
	}
	//copy original data to red channel
	out<<"FragColor.r = or;\n"; 
	out<<"FragColor.b  = intensity;}\n"<<'\0';

	return new ProgramCG( buffer);
}


ProgramGPU* FilterGLCG::CreateFilterV(float kernel[], float offset[], int height)
{
	char buffer[10240];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	if(GlobalUtil::_BetaFilter)
	{
		out<< "void main(uniform samplerRECT tex,";
		out<<"\n\tin float4 TexCoord0: TEXCOORD0,";
		out<<"\n\tout float4 FragColor : COLOR0 )";
		out<<"\n{\n\tfloat4 intensity4 = float4(0, 0, 0, 0), data;\n";
		out<<"float2 orb = texRECT(tex, TexCoord0.xy).rb; float intensity;\n";

		for(int i = 0; i< height; i+=4)
		{
			out <<"data = float4(";
			for(int j = i; j < i + 4; j++)
			{
				if(j != i) out <<", \n";
				if(j >= height)
				{
					out<<"0";
				}else if(offset[j]==0.0)
				{
					out<<"orb.y";
				}else
				{
					out<<"texRECT(tex, TexCoord0.xy + float2(0, float("<<offset[j] <<"))).b";
				}
			}
			out << ");\n";
			out << "intensity4 += data * float4(";
			for(int k = i; k < i + 4; k++)
			{
				if(k != i) out <<", ";
				if(k >= height)	out<<"0";
				else			out<<kernel[k];
			}
			out << ");\n";

 		}
		out << "intensity4.xy += intensity4.zw;\n";
		out << "intensity = intensity4.x + intensity4.y;\n";
	}else
	{
		out<< "void main(uniform samplerRECT tex,";
		out<<"\n\tin float4 TexCoord0: TEXCOORD0,";
		out<<"\n\tout float4 FragColor : COLOR0 )";
		out<<"\n{\n\tfloat intensity = 0.0 ;  float2 pos;\n";

		for(int i = 0; i< height; i++)
		{
			if(offset[i]==0.0)
			{
				out<<"float2 orb = texRECT(tex, TexCoord0.xy).rb;\n";
				out<<"intensity+= orb.y * "<<kernel[i]<<";\n";

			}else
			{
				out<<"pos = TexCoord0.xy + float2(0, float("<<offset[i] <<"));\n";
				out<<"intensity+= "<<kernel[i]<<"*texRECT(tex, pos).b;\n";
			}
		}
	}
	out<<"FragColor.b = orb.y;\n";
	out<<"FragColor.g = intensity - orb.x;\n"; // difference of gaussian..
	out<<"FragColor.r = intensity;}\n"<<'\0';
	
	return new ProgramCG( buffer);
}

void ProgramCG::SetTexParameter(unsigned int texID)
{
	cgGLSetTextureParameter(_texParamID, texID);	
	cgGLEnableTextureParameter(_texParamID);
}

ProgramGPU* FilterGLCG::CreateFilterHPK(float kernel[], float offset[], int width)
{
	//both h and v are packed...
	int i, j , xw, xwn;
	int halfwidth  = width >>1;
	float * pf = kernel + halfwidth;
	int nhpixel = (halfwidth+1)>>1;	//how many neighbour pixels need to be looked up
	int npixel  = (nhpixel<<1)+1;//
	char buffer[10240];
	float weight[3];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	out<< "void main(uniform samplerRECT tex, float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0 ){\n";
	out<< "FragColor = float4(0, 0, 0, 0); \nfloat4 pc; float2 coord; \n";
	///use multi texture coordinate because nhpixels can be at most 3
	for( i = 0 ; i < npixel ; i++)
	{

		out<<"coord = TexCoord0.xy + float2(float("<<i-nhpixel<<"),0);\n";
		out<<"pc=texRECT(tex, coord);\n";
		if(GlobalUtil::_PreciseBorder)		out<<"if(coord.x < 0) pc = pc.rrbb;\n";

		//for each sub-pixel j  in center, the weight of sub-pixel k 
		xw = (i - nhpixel)*2;
		for( j = 0; j < 3; j++)
		{
			xwn = xw  + j  -1;
			weight[j] = xwn < -halfwidth || xwn > halfwidth? 0 : pf[xwn];
		}
		//if(weight[1]!=0.0)	out<<"FragColor += "<<weight[1]<<"*pc;\n";
		//out<<"FragColor += float4("<<weight[2]<<","<<weight[0]<<","<<weight[2]<<","<<weight[0]<<")*pc.grab;\n";

		if(weight[1] == 0.0)
		{
			out<<"FragColor += float4("<<weight[2]<<","<<weight[0]<<","<<weight[2]<<","<<weight[0]<<")*pc.grab;\n";
		}
		else
		{
			out<<"FragColor += float4("<<weight[1]<<", "<<weight[0]<<", "<<weight[1]<<", "<<weight[0]<<")*pc.rrbb;\n";
			out<<"FragColor += float4("<<weight[2]<<", "<<weight[1]<<", "<<weight[2]<<", "<<weight[1]<<")*pc.ggaa;\n";
		}

	}
	out<<"}\n"<<'\0';
	return new ProgramCG( buffer);
}

ProgramGPU* FilterGLCG::CreateFilterVPK(float kernel[], float offset[], int height)
{

	//both h and v are packed...
	int i, j , yw, ywn;
	int halfh  = height >>1;
	float * pf = kernel + halfh;
	int nhpixel = (halfh+1)>>1;	//how many neighbour pixels need to be looked up
	int npixel  = (nhpixel<<1)+1;//
	char buffer[10240];
	float weight[3];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	out<< "void main(uniform samplerRECT tex, float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0 ){\n";
	out<< "FragColor = float4(0, 0, 0, 0);\nfloat4 pc; float2 coord;\n";
	///use multi texture coordinate because nhpixels can be at most 3

	for( i = 0 ; i < npixel ; i++)
	{

		out<<"coord = TexCoord0.xy + float2(0, float("<<i-nhpixel<<"));\n";
		out<<"pc=texRECT(tex, coord);\n";
		if(GlobalUtil::_PreciseBorder)	out<<"if(coord.y < 0) pc = pc.rgrg;\n";
		//for each sub-pixel j  in center, the weight of sub-pixel k 
		yw = (i - nhpixel)*2;
		for( j = 0; j < 3; j++)
		{
			ywn = yw + j  -1;
			weight[j] = ywn < -halfh || ywn > halfh? 0 : pf[ywn];
		}
		//if(weight[1]!=0.0)	out<<"FragColor += "<<weight[1]<<"*pc;\n";
		//out<<"FragColor += float4("<<weight[2]<<","<<weight[2]<<","<<weight[0]<<","<<weight[0]<<")*pc.barg;\n";
		if(weight[1] == 0.0)
		{
			out<<"FragColor += float4("<<weight[2]<<","<<weight[2]<<","<<weight[0]<<","<<weight[0]<<")*pc.barg;\n";
		}else
		{
			out<<"FragColor += float4("<<weight[1]<<","<<weight[1]<<","<<weight[0]<<","<<weight[0]<<")*pc.rgrg;\n";
			out<<"FragColor += float4("<<weight[2]<<","<<weight[2]<<","<<weight[1]<<","<<weight[1]<<")*pc.baba;\n";
		}
	}
	out<<"}\n"<<'\0';
	return new ProgramCG( buffer);
}


void ShaderBagCG::LoadGenListShader(int ndoglev, int nlev)
{
	ProgramCG * program;

	s_genlist_init_tight = new ProgramCG(
	"void main (\n\
	uniform samplerRECT tex, in float4 TexCoord0 : TEXCOORD0,\n\
	in float4 TexCoord1 : TEXCOORD1, in float4 TexCoord2 : TEXCOORD2, in float4 TexCoord3 : TEXCOORD3,\n\
	out float4 FragColor : COLOR0){\n\
	float4 helper = float4( texRECT(tex, TexCoord0.xy).r,  texRECT(tex, TexCoord1.xy).r,\n\
	texRECT(tex, TexCoord2.xy).r, texRECT(tex, TexCoord3.xy).r);\n\
	FragColor = float4(helper>0.0);\n\
	}");

	s_genlist_init_ex = program = new ProgramCG(
	"void main (uniform float2 bbox, \n\
	uniform samplerRECT tex, \n\
	in float4 TexCoord0 : TEXCOORD0,\n\
	in float4 TexCoord1 : TEXCOORD1, \n\
	in float4 TexCoord2 : TEXCOORD2, \n\
	in float4 TexCoord3 : TEXCOORD3,\n\
	out float4 FragColor : COLOR0){\n\
	float4 helper = float4( \n\
	texRECT(tex, TexCoord0.xy).r, texRECT(tex, TexCoord1.xy).r,\n\
	texRECT(tex, TexCoord2.xy).r, texRECT(tex, TexCoord3.xy).r);\n\
	bool4 helper4 = bool4(TexCoord0.xy < bbox, TexCoord3.xy < bbox); \n\
	bool4 helper2 = helper4.xzxz && helper4.yyww; \n\
	FragColor = float4(helper2 && (helper>0.0 ));\n\
	}");
	_param_genlist_init_bbox = cgGetNamedParameter( *program, "bbox");


	//reduction ...
	s_genlist_histo = new ProgramCG(
	"void main (\n\
	uniform samplerRECT tex, in float2 TexCoord0 : TEXCOORD0,\n\
	in float2 TexCoord1 : TEXCOORD1, in float2 TexCoord2 : TEXCOORD2, in float2 TexCoord3 : TEXCOORD3,\n\
	out float4 FragColor : COLOR0){\n\
	float4 helper; float4 helper2; \n\
	helper = texRECT(tex, TexCoord0); helper2.xy = helper.xy + helper.zw; \n\
	helper = texRECT(tex, TexCoord1); helper2.zw = helper.xy + helper.zw; \n\
	FragColor.rg = helper2.xz + helper2.yw;\n\
	helper = texRECT(tex, TexCoord2); helper2.xy = helper.xy + helper.zw; \n\
	helper = texRECT(tex, TexCoord3); helper2.zw = helper.xy + helper.zw; \n\
	FragColor.ba= helper2.xz+helper2.yw;\n\
	}");


	//read of the first part, which generates tex coordinates 

	s_genlist_start= program =  LoadGenListStepShader(1, 1);
	_param_ftex_width= cgGetNamedParameter(*program, "width");
	_param_genlist_start_tex0 = cgGetNamedParameter(*program, "tex0");
	//stepping
	s_genlist_step = program = LoadGenListStepShader(0, 1);
	_param_genlist_step_tex= cgGetNamedParameter(*program, "tex");
	_param_genlist_step_tex0= cgGetNamedParameter(*program, "tex0");


}

ProgramCG* ShaderBagCG::LoadGenListStepShader(int start, int step)
{
	int i;
	char buffer[10240];
	//char chanels[5] = "rgba";
	ostrstream out(buffer, 10240);
	out<<"void main(out float4 FragColor : COLOR0, \n";

	for(i = 0; i < step; i++) out<<"uniform samplerRECT tex"<<i<<",\n";

	if(start)
	{
		out<<"uniform float width, \nin float2 tpos : TEXCOORD0){\n";
		out<<"float  index = floor(tpos.y) * width + floor(tpos.x) + 0.0001;\n";
		out<<"float2 pos = float2(0.5, 0.5);\n";
	}else
	{
		out<<"uniform samplerRECT tex, in float2 tpos: TEXCOORD0 ){\n";
		out<<"float4 tc = texRECT( tex, tpos);\n";
		out<<"float2 pos = tc.rg; float index = tc.b;\n";
	}
	out<<"float2 sum; 	float4 cc;\n";



	if(step>0)
	{
		out<<"float2 cpos = float2(-0.5, 0.5);\t float2 opos;\n";
		for(i = 0; i < step; i++)
		{
//#define SETP_CODE_2

#ifndef SETP_CODE_2
/*			out<<"cc = texRECT(tex"<<i<<", pos);\n";
			out<<"float sum3[3] = {cc.r, cc.r + cc.g, cc.r + cc.g + cc.b};\n";
			out<<"float3 cmp = float3(index > float3(sum3[0], sum3[1], sum3[2]));\n";
			out<<"opos.y = -0.5 + cmp.y; opos.x = -0.5 + cmp.x + (cmp.z - cmp.y);\n";
			out<<"index -= dot(cmp, cc.rgb);\n";
			out<<"pos = (pos + pos + opos);\n";*/

			out<<"cc = texRECT(tex"<<i<<", pos); sum.x = cc.r + cc.g;\n";
			out<<"if (index < sum.x){ if(index < cc.r) opos = cpos.xx; else {opos = cpos.yx; index -= cc.r;}}\n";
			out<<"else {index -= sum.x; if(index < cc.b) opos = cpos.xy; else{opos = cpos.yy; index -= cc.b;}}";
			out<<"pos = (pos + pos + opos);\n";

/*			out<<"cc = texRECT(tex"<<i<<", pos);\n";
			out<<"if (index <cc.r){ opos = cpos.xx;}\n";
			out<<"else{sum.x = cc.r + cc.g;";
					out<<"if(index < sum.x ) {opos = cpos.yx; index -= cc.r;}\n";
					out<<"else{sum.y = sum.x + cc.b;";
							out<<"if(index < sum.y ) {opos = cpos.xy; index -= sum.x;}\n";
							out<<"else {opos = cpos.yy; index -= sum.y;}}}\n";
			out<<"pos = (pos + pos + opos);\n";*/

#else
			out<<"cc = texRECT(tex"<<i<<", pos);\n";
			out<<"if (index < cc.r) opos = cpos.xx;\n";
			out<<"else if (index < cc.r + cc.g){opos = cpos.yx; index -= cc.r;}\n";
			out<<"else if (index < cc.r + cc.g + cc.b){opos = cpos.xy; index -= (cc.r + cc.g);}\n";
			out<<"else {opos = cpos.yy; index -= (cc.r + cc.g + cc.b);}\n";
			out<<"pos = (pos + pos + opos);\n";
#endif
		}
	}
	out<<"FragColor = float4(pos, index, 1);\n";
	out<<"}\n"<<'\0';
	return new ProgramCG(buffer);
}

void ShaderBagCG::SetGenListInitParam(int w, int h)
{
	float bbox[2] = {w -1.0f, h - 1.0f};
	cgGLSetParameter2fv(_param_genlist_init_bbox, bbox);
}

void ShaderBagCG::SetGenListStartParam(float width, int tex0)
{
	cgGLSetParameter1f(_param_ftex_width, width);

	if(_param_genlist_start_tex0)
	{
		cgGLSetTextureParameter(_param_genlist_start_tex0, tex0);
		cgGLEnableTextureParameter(_param_genlist_start_tex0);
	}
}

void ShaderBagCG::LoadDescriptorShaderF2()
{
	//one shader outpout 128/8 = 16 , each fragout encodes 4
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	void main(uniform samplerRECT tex,		\n\
	uniform	samplerRECT gradTex,			\n\
	uniform float4		dsize,				\n\
	uniform float2		size,				\n\
	in		float2	TexCoord0 : TEXCOORD0,	\n\
	out		float4  FragData0:COLOR0,		\n\
	out		float4	FragData1:COLOR1)		\n\
	{\n\
		float2 dim	= size.xy;	//image size			\n\
		float index = dsize.x*floor(TexCoord0.y * 0.5) + TexCoord0.x;\n\
		float idx = fmod(index, 8.0) + 8.0 * floor(fmod(TexCoord0.y, 2.0));		\n\
		index = floor(index*0.125) + 0.49;  \n\
		float2 coord = floor( float2( fmod(index, dsize.z), index*dsize.w)) + 0.5 ;\n\
		float2 pos = texRECT(tex, coord).xy;		\n\
		if(any(pos.xy <= 1) || any(pos.xy >=dim-1)) discard;	\n\
		float  anglef = texRECT(tex, coord).z;\n\
		float sigma = texRECT(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";

	//rotation
	out<<"\
		float4 cscs, rots;								\n\
		sincos(anglef, cscs.y, cscs.x);					\n\
		cscs.zw = - cscs.xy;							\n\
		rots = cscs /spt;								\n\
		cscs *= spt; \n";

	//here cscs is actually (cos, sin, -cos, -sin) * (factor: 3)*sigma
	//and rots is  (cos, sin, -cos, -sin ) /(factor*sigma)
	//decide which part of the grid  by  coord
	//compute the upper-left corner of the sample region
	//devide the 4x4 sift grid into 8 2x1,..each 2x1 is obtained from a shader thread
	//to use linear interoplation, enlarge 2x1 to 3x2, by adding 0.5 to each side
	out<<"\
		float4 offset, temp; float2 pt;				\n\
		offset.x = fmod(idx, 4) - 3.0;				\n\
		offset.y = floor(idx*0.25) - 2.5;			\n\
		offset.zw = offset.xy + float2(2.0, 2.0);	\n\
		float4 offsetpt = offset + 0.5;				\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;						\n";
	
	//get a horizontal bounding box of the rotated rectangle
	out<<"\
		float2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		float2 pmin = min(min(p1, p2), min(p3, p4));\n\
		float2 pmax = max(max(p1, p2), max(p3, p4));\n\
		float4 sz;	float2 spos;					\n\
		sz.xy = max( pos + pmin , float2(1,1));\n\
		sz.zw = min( pos + pmax , dim -2);		\n\
		sz = floor(sz)+0.5;"; //move sample point to pixel center
	//get voting for two box

	out<<"\n\
		float4 DA, DB;			\n\
		DA = DB  = float4(0, 0, 0, 0);		\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 diff = spos - pt;								\n\
				temp = rots.xywx * diff.xyxy;\n\
				float2 nxy = (temp.xz + temp.yw); \n\
				if(all(nxy > float2(-0.5,-0.5) && nxy < float2(1.5, 1.5)))\n\
				{\n\
					float4 cc = texRECT(gradTex, spos);						\n\
					float mod = cc.b;	float angle = cc.a;					\n\
					float theta = fmod(8+(anglef - angle)*"<<rpi<<", 8); \n\
					diff = nxy + offsetpt.xy;								\n\
					float ww = exp(-0.125*dot(diff, diff));\n\
					float2 weights = 1 - abs(nxy.xy - 0.5);\n\
					float weight = weights.x * weights.y *mod*ww; \n\
					float4 wa, wb; \n\
					float theta1 = floor(theta); \n\
					float weight1 = theta1 + 1.0 - theta; float weight2 = theta - theta1;\n\
					wa = float4(theta1 == float4(0, 1, 2, 3))*weight1 + float4(theta1 == float4(7, 0, 1, 2))*weight2; \n\
					wb = float4(theta1 == float4(4, 5, 6, 7))*weight1 + float4(theta1 == float4(3, 4, 5, 6))*weight2; \n\
					DA += wa * weight ; DB += wb * weight;\n\
				}\n\
			}\n\
		}\n";

	out<<"\
		FragData0 = DA; FragData1 = DB;\n\
	}\n"<<'\0';

	ProgramCG * program; 
	s_descriptor_fp = program =  new ProgramCG(buffer);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");


}

void ShaderBagCG::LoadDescriptorShaderF4()
{
	//one shader outpout 128/8 = 16 , each fragout encodes 4
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	void main(uniform samplerRECT tex,		\n\
	uniform	samplerRECT gradTex,			\n\
	uniform float4		dsize,				\n\
	uniform float2		size,				\n\
	in		float2	TexCoord0 : TEXCOORD0,	\n\
	out		float4  FragData0:COLOR0,		\n\
	out		float4	FragData1:COLOR1,		\n\
	out		float4  FragData2:COLOR2,		\n\
	out		float4  FragData3:COLOR3)		\n\
	{\n\
		float4 D0A, D0B, D1A, D1B;			\n\
		D0A = D0B = D1A = D1B = float4(0, 0, 0, 0);		\n\
		float2 dim	= size.xy;	//image size			\n\
		float index = dsize.x*floor(TexCoord0.y) + TexCoord0.x;\n\
		float idx = fmod(index, 8.0);		\n\
		index = floor(index*0.125) + 0.49;  \n\
		float2 coord = floor( float2( fmod(index, dsize.z), index*dsize.w)) + 0.5 ;\n\
		float2 pos = texRECT(tex, coord).xy;		\n\
		if(any(pos.xy <= 1) || any(pos.xy >=dim-1)) discard;	\n\
		float  anglef = texRECT(tex, coord).z;\n\
		float sigma = texRECT(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";
	//rotation
	out<<"\
		float4 cscs, rots;								\n\
		sincos(anglef, cscs.y, cscs.x);					\n\
		cscs.zw = - cscs.xy;							\n\
		rots = cscs /spt;								\n\
		cscs *= spt; \n";

	//here cscs is actually (cos, sin, -cos, -sin) * (factor: 3)*sigma
	//and rots is  (cos, sin, -cos, -sin ) /(factor*sigma)
	//decide which part of the grid  by  coord
	//compute the upper-left corner of the sample region
	//devide the 4x4 sift grid into 8 2x1,..each 2x1 is obtained from a shader thread
	//to use linear interoplation, enlarge 2x1 to 3x2, by adding 0.5 to each side
	out<<"\
		float4 offset, temp; float2 pt;					\n\
		offset.x = fmod(idx, 2)<1? -2.5:-0.5;		\n\
		offset.y = floor(idx*0.5) - 2.5;			\n\
		offset.zw = offset.xy + float2(3.0, 2.0);	\n\
		float4 offsetpt = offset + 0.5;				\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;						\n";
	
	//get a horizontal bounding box of the rotated rectangle
	out<<"\
		float2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		float2 pmin = min(min(p1, p2), min(p3, p4));\n\
		float2 pmax = max(max(p1, p2), max(p3, p4));\n\
		float4 sz;	float2 spos;					\n\
		sz.xy = max( pos + pmin , float2(1,1));\n\
		sz.zw = min( pos + pmax , dim -2);		\n\
		sz = floor(sz)+0.5;"; //move sample point to pixel center
	//get voting for two box

//#define DESCRIPTOR_DEBUG

#ifdef DESCRIPTOR_DEBUG
//	out<<"\n\nD1B = sz;	D0B = size; D1A = offset;	D0A = float4(pos, pt);\n";
#else
	out<<"\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 diff = spos - pt;								\n\
				temp = rots.xywx * diff.xyxy;\n\
				float2 nxy = (temp.xz + temp.yw); \n\
				if(all(nxy > float2(-0.5,-0.5) && nxy < float2(2.5, 1.5)))\n\
				{\n\
					float4 cc = texRECT(gradTex, spos);						\n\
					float mod = cc.b;	float angle = cc.a;					\n\
					float theta = fmod(8+(anglef - angle)*"<<rpi<<", 8); \n\
					diff = nxy + offsetpt.xy;								\n\
					float ww = exp(-0.125*dot(diff, diff));\n\
					float weight = (1 - abs(nxy.y - 0.5))*mod*ww; \n\
					float4 wa, wb; \n\
					float theta1 = floor(theta); \n\
					float weight1 = theta1 + 1.0 - theta; float weight2 = theta - theta1;\n\
					wa = float4(theta1 == float4(0, 1, 2, 3))*weight1 + float4(theta1 == float4(7, 0, 1, 2))*weight2; \n\
					wb = float4(theta1 == float4(4, 5, 6, 7))*weight1 + float4(theta1 == float4(3, 4, 5, 6))*weight2; \n"
					//the code below is a new implementation dated 9/1/2008
					//it is a little bit faster probably due to less if/else
					<<"\
					weight1 = saturate((1.0- abs(nxy.x-0.5)))*weight ; \n\
					D0A+=wa*weight1;	D0B+=wb*weight1; \n\
					weight2 = saturate((1.0-abs(nxy.x-1.5)))*weight; \n\
					D1A+=wa*weight2;	D1B+=wb*weight2; \n"
/*					<<"\
					if(nxy.x < 0.5)\n\
					{\n\
						weight = (0.5+nxy.x)*weight; \n\
						D0A+=wa*weight;	D0B+=wb*weight; \n\
					}else if(nxy.x < 1.5)\n\
					{\n\
						weight1 = (1.5-nxy.x)*weight; weight2 = (nxy.x - 0.5)*weight; \n\
						D0A+=wa*weight1;	D0B+=wb*weight1; \n\
						D1A+=wa*weight2;	D1B+=wb*weight2; \n\
					}else\n\
					{\n\
						weight = (2.5-nxy.x)*weight; \n\
						D1A+=wa*weight;	D1B+=wb*weight; \n\
					}\n"*/
					<<"\
				}\n\
			}\n\
		}\n";
#endif
	out<<"\
		FragData0 = D0A; FragData1 = D0B;\n\
		FragData2 = D1A; FragData3 = D1B;\n\
	}\n"<<'\0';
	ProgramCG * program; 
	s_descriptor_fp = program =  new ProgramCG(buffer);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");
}


void ShaderBagCG::WriteDescriptorCodeToStream(ostream& out)
{
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	//Descriptor storage
	out<<"\n\
		float4 DA[6][6];	float4 DB[6][6];					\n\
		for (int i = 0; i < 6; i++)								\n\
		{\n\
			for(int j = 0; j < 6; j++)							\n\
			{\n\
					DA[i][j] = DB[i][j] = float4(0, 0, 0, 0);	\n\
			}\n\
		}";

	//rotation
	out<<"\n\
		float4 cscs, rots;								\n\
		sincos(anglef, cscs.y, cscs.x);					\n\
		cscs.zw = - cscs.xy;							\n\
		rots = cscs /spt;								\n\
		cscs *= spt; \n";
	//decide which part of the grid is by y coord
	out<<"\
		float4 offset, temp; float2 pt;				\n\
		offset = float4(-2.5, -2.5, 2.5, 2.5);		\n\
		float4 offsetpt = float4(-2, -2, -2, -2);	\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;						\n";

	//get the coordinate region of the rotated rectangle
	out<<"\
		float2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		float2 pmin = min(min(p1, p2), min(p3, p4));\n\
		float2 pmax = max(max(p1, p2), max(p3, p4));\n\
		float4 sz;	float2 spos;					\n\
		sz.xy = max( pos + pmin , float2(1,1));\n\
		sz.zw = min( pos + pmax , dim -2);		\n\
		sz = floor(sz)+0.5;";

	//get voting 
	out<<"\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 diff = spos - pt;								\n\
				temp = rots.xywx * diff.xyxy;\n\
				float2 nxy = (temp.xz + temp.yw); \n\
				if(all(nxy > float2(-0.5,-0.5) && nxy < float2(4.5, 4.5)))\n\
				{\n\
					float4 cc = texRECT(gradTex, spos);				\n\
					diff = nxy + offsetpt.xy;								\n\
					float mod = cc.b *  exp(-0.125*dot(diff, diff));	\n\
					float angle = cc.a;					\n\
					float theta = fmod(8+ (anglef - angle)*"<<rpi<<", 8); \n\
					float4 wa, wb; \n\
					float theta1 = floor(theta); \n\
					float weight1 = theta1 + 1.0 - theta; float weight2 = theta - theta1;\n\
					wa = float4(theta1 == float4(0, 1, 2, 3))*weight1 + float4(theta1 == float4(7, 0, 1, 2))*weight2; \n\
					wb = float4(theta1 == float4(4, 5, 6, 7))*weight1 + float4(theta1 == float4(3, 4, 5, 6))*weight2; \n\
					wa*= mod; wb*=mod; \n"
				<<"\
					float2 xyi = round(nxy.xy); int2 idx = int2(xyi);\n\
					float4 weightxy = saturate(1.0 - abs(nxy.xxyy - xyi.xxyy - float4(-0.5, 0.5, -0.5, 0.5))); \n\
					float4 weights = weightxy.xyxy * weightxy.zzww; \n\
					DA[idx.y  ][idx.x  ]+= wa * weights.x;	DB[idx.y  ][idx.x  ]+=wb*weights.x;\n\
					DA[idx.y  ][idx.x+1]+= wa * weights.y;	DB[idx.y  ][idx.x+1]+=wb*weights.y;\n\
					DA[idx.y+1][idx.x  ]+= wa * weights.z;	DB[idx.y+1][idx.x  ]+=wb*weights.z;\n\
					DA[idx.y+1][idx.x+1]+= wa * weights.w;	DB[idx.y+1][idx.x+1]+=wb*weights.w;\n\
				}\n\
			}\n\
		}\n";
	out<<"\
		float norm=0;	\n\
		for (int i = 1; i < 5; i++)\n\
		{\n\
			for(int j = 1; j < 5; j++)\n\
			{\n\
				norm+= dot(DA[i][j], DA[i][j]);	\n\
				norm+= dot(DB[i][j], DB[i][j]);	\n\
			}\n\
		}\n\
		norm = rsqrt(norm);			\n\
		for (int i = 0; i < 4; i++)\n\
		{\n\
			for(int j = 0; j < 4; j++)\n\
			{\n\
				DA[i][j] =min(float4(0.2,0.2,0.2, 0.2), DA[i+1][j+1]* norm);\n\
				DB[i][j] =min(float4(0.2,0.2,0.2, 0.2), DB[i+1][j+1]* norm);\n\
			}\n\
		}\n\
		norm = 0; \n\
		for (int i = 0; i < 4; i++)\n\
		{\n\
			for(int j = 0; j < 4; j++)\n\
			{\n\
				norm+= dot(DA[i][j], DA[i][j]);	\n\
				norm+= dot(DB[i][j], DB[i][j]);	\n\
			}\n\
		}\n\
		norm = rsqrt(norm);			\n";
}

void ShaderBagCG::LoadDescriptorShaderFI()
{

	char buffer[18240];
	ostrstream out(buffer, 18240);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	void main(uniform samplerRECT tex,		\n\
	uniform	samplerRECT gradTex,			\n\
	uniform float4		dsize,				\n\
	uniform float2		size,				\n\
	in		float2	TexCoord0 : TEXCOORD0,	\n\
	out		float4  FragData0:COLOR0,		\n\
	out		float4  FragData1:COLOR1,		\n\
	out		float4  FragData2:COLOR2,		\n\
	out		float4  FragData3:COLOR3,		\n\
	out		float4  FragData4:COLOR4,		\n\
	out		float4  FragData5:COLOR5,		\n\
	out		float4  FragData6:COLOR6,		\n\
	out		float4  FragData7:COLOR7)		\n\
	{\n\
		float2 dim	= size.xy;	//image size			\n\
		float2 pos = texRECT(tex, TexCoord0.xy).xy;		\n\
		float  anglef = texRECT(tex, TexCoord0.xy).z;		\n\
		float sigma = texRECT(tex, TexCoord0).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";

	WriteDescriptorCodeToStream(out);
	out<<"\
		norm = (norm * 512.0/255.0);\n";

	//output
	for(int i = 0; i < 8; i++)
	out<<"\
		FragData"<<i<<".r = pack_4ubyte(norm * DA["<<i/2<<"]["<<(2 * (i%2))<<"]);\n\
		FragData"<<i<<".g = pack_4ubyte(norm * DB["<<i/2<<"]["<<(2 * (i%2))<<"]);\n\
		FragData"<<i<<".b = pack_4ubyte(norm * DA["<<i/2<<"]["<<(1+2 * (i%2))<<"]);\n\
		FragData"<<i<<".a = pack_4ubyte(norm * DB["<<i/2<<"]["<<(1+2 * (i%2))<<"]);\n";
	//end of the cg file
	out <<"\
	}\n"<<'\0';
	ProgramCG * program; 
	s_descriptor_fp = program =  new ProgramCG(buffer);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");
}


void ShaderBagCG::LoadDescriptorShaderF8()
{
	//one shader outpout 128/4 = 32 , each fragout encodes 4
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	void main(uniform samplerRECT tex,		\n\
	uniform	samplerRECT gradTex,			\n\
	uniform float4		dsize,				\n\
	uniform float2		size,				\n\
	in		float2	TexCoord0 : TEXCOORD0,	\n\
	out		float4  FragData0:COLOR0,		\n\
	out		float4	FragData1:COLOR1,		\n\
	out		float4  FragData2:COLOR2,		\n\
	out		float4  FragData3:COLOR3,		\n\
	out		float4	FragData4:COLOR4,		\n\
	out		float4  FragData5:COLOR5,		\n\
	out		float4  FragData6:COLOR6,		\n\
	out		float4  FragData7:COLOR7)		\n\
	{\n\
		float4 D00A, D00B, D01A, D01B, D10A, D10B, D11A, D11B;			\n\
		D00A=D00B=D01A=D01B=D10A=D10B=D11A=D11B=float4(0, 0, 0, 0);\n\
		float2 dim	= size.xy;	//image size			\n\
		float index = dsize.x*floor(TexCoord0.y) + TexCoord0.x;\n\
		float idx = fmod(index, 4.0);		\n\
		index = floor(index*0.25) + 0.49;  \n\
		float2 coord = floor( float2( fmod(index, dsize.z), index*dsize.w)) + 0.5 ;\n\
		float2 pos = texRECT(tex, coord).xy;		\n\
		float  anglef = texRECT(tex, coord).z;		\n\
		float sigma = texRECT(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";
	//rotation
	out<<"\n\
		float4 cscs, rots;								\n\
		sincos(anglef, cscs.y, cscs.x);					\n\
		cscs.zw = - cscs.xy;							\n\
		rots = cscs /spt;								\n\
		cscs *= spt; \n";
	//decide which part of the grid is by y coord
	out<<"\
		float4 offset, temp; float2 pt;				\n\
		offset.x = fmod(idx, 2)<1? -2.5:-0.5;		\n\
		offset.y = idx <2?- 2.5 : - 0.5;			\n\
		offset.zw = offset.xy + float2(3.0, 3.0);	\n\
		float4 offsetpt = offset + 0.5;				\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;				\n";
	//get the coordinate region of the rotated rectangle
	out<<"\
		float2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		float2 pmin = min(min(p1, p2), min(p3, p4));\n\
		float2 pmax = max(max(p1, p2), max(p3, p4));\n\
		float4 sz;	float2 spos;					\n\
		sz.xy = max( pos + pmin, float2(1,1));\n\
		sz.zw = min( pos + pmax, dim -2);		\n\
		sz = floor(sz)+0.5;";
	//get voting for two box
	out<<"\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 diff = spos - pt;								\n\
				temp = rots.xywx * diff.xyxy;\n\
				float2 nxy = (temp.xz + temp.yw); \n\
				if(all(nxy >= float2(-0.5,-0.5) && nxy <= float2(2.5, 2.5)))\n\
				{\n\
					float4 cc = texRECT(gradTex, spos);				\n\
					diff = nxy + offsetpt.xy;								\n\
					float mod = cc.b *  exp(-0.125*dot(diff, diff));	\n\
					float angle = cc.a;					\n\
					float theta = fmod(8+ (anglef - angle)*"<<rpi<<", 8); \n\
					float4 wa, wb; \n\
					float theta1 = floor(theta); \n\
					float weight1 = theta1 + 1.0 - theta; float weight2 = theta - theta1;\n\
					wa = float4(theta1 == float4(0, 1, 2, 3))*weight1 + float4(theta1 == float4(7, 0, 1, 2))*weight2; \n\
					wb = float4(theta1 == float4(4, 5, 6, 7))*weight1 + float4(theta1 == float4(3, 4, 5, 6))*weight2; \n\
					wa*= mod; wb*=mod; \n"
				<<"\
					float4 weightxy = saturate(1.0 - abs(nxy.xxyy - float4(0.5,1.5,0.5,1.5))); \n\
					float4 weights = weightxy.xyxy * weightxy.zzww; \n\
					D00A+= wa * weights.x;	D00B+=wb*weights.x;\n\
					D01A+= wa * weights.y;	D01B+=wb*weights.y;\n\
					D10A+= wa * weights.z;	D10B+=wb*weights.z;\n\
					D11A+= wa * weights.w;	D11B+=wb*weights.w;\n\
				}\n\
			}\n\
		}\n";
	out<<"\
		FragData0 = D00A; FragData1 = D00B;\n\
		FragData2 = D01A; FragData3 = D01B;\n\
 		FragData4 = D10A; FragData5 = D10B;\n\
		FragData6 = D11A; FragData7 = D11B;\n\
	}\n"<<'\0';
	ProgramCG * program; 
	s_descriptor_fp = program =  new ProgramCG(buffer);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");
}

//the shader that computes the descriptors
void ShaderBagCG::LoadDescriptorShader()
{

	std::cout<<"[Descriptor Shader Version] :" << GlobalUtil::_DescriptorPPT << endl;
	if(GlobalUtil::_DescriptorPPT == 32)
		LoadDescriptorShaderG();
	else if(GlobalUtil::_DescriptorPPT ==8)
		LoadDescriptorShaderF4();
	else if(GlobalUtil::_DescriptorPPT ==1)
		LoadDescriptorShaderFI();
	else if(GlobalUtil::_DescriptorPPT ==4)
		LoadDescriptorShaderF8();
	else if(GlobalUtil::_DescriptorPPT == 16)
		LoadDescriptorShaderF2();
	
}

void ShaderBagCG::LoadOrientationShader()
{

	char buffer[10240];
	ostrstream out(buffer,10240);


	out<<"\n\
	#define GAUSSIAN_WF "<<GlobalUtil::_OrientationGaussianFactor<<" \n\
	#define SAMPLE_WF ("<<GlobalUtil::_OrientationWindowFactor<< " )\n\
	void main(uniform samplerRECT tex,			\n\
	uniform samplerRECT gradTex,		\n\
			uniform float4 size,				\n\
			in float2 TexCoord0 : TEXCOORD0,	\n\
			out float4 FeatureData : COLOR0	";

	//multi orientation output
	//use one additional texture to store up to four orientations
	//when we use one 32bit float to store two orientations, no extra texture is required

	if(GlobalUtil::_MaxOrientation >1  && GlobalUtil::_OrientationPack2 == 0)
		out<<", out float4 OrientationData : COLOR1";

	if(GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)
	{
		//data for sub-pixel localization
		out<<", uniform samplerRECT texS";
	}

	//use 9 float4 to store histogram of 36 directions
	out<<")		\n\
	{													\n\
		float4 bins[10];								\n\
		int i, j , k ;									\n\
		for (i=0; i<9; i++) bins[i] = float4(0,0,0,0);	\n\
		float4 loc = texRECT(tex, TexCoord0);			\n\
		bool full_process_mode = (size.z > 0);			\n\
		float2 pos = loc.xy;							\n\
		float sigma = full_process_mode? size.z : loc.w; \n";
	if(GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)
	{
		out<<"\
		if(full_process_mode) {\n\
			float4 keyx = texRECT(texS, pos);\n\
			sigma = sigma * pow(size.w, keyx.w); \n\
			pos.xy = pos.xy + keyx.yz; \n\
			#if " << GlobalUtil::_KeepExtremumSign << "\n\
				if(keyx.x<0.6) sigma = - sigma;\n\
			#endif\n\
		}\n";
	}
	out<<"\
		float gsigma = sigma * GAUSSIAN_WF;				\n\
		float2 win = abs(sigma.xx) * (SAMPLE_WF * GAUSSIAN_WF);	\n\
		float2 dim = size.xy;							\n\
		float dist_threshold = win.x*win.x+0.5;			\n\
		float factor = -0.5/(gsigma*gsigma);			\n\
		float4 sz;	float2 spos;						\n\
		//if(any(pos.xy <= 1)) discard;					\n\
		sz.xy = max( pos - win, float2(1,1));			\n\
		sz.zw = min( pos + win, dim-2);				\n\
		sz = floor(sz)+0.5;";
	//loop to get the histogram

	out<<"\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 offset = spos - pos;								\n\
				float sq_dist = dot(offset,offset);						\n\
				if( sq_dist < dist_threshold){							\n\
					float4 cc = texRECT(gradTex, spos);						\n\
					float grad = cc.b;	float theta = cc.a;					\n\
					float idx = floor(degrees(theta)*0.1);		\n\
					if(idx < 0 ) idx += 36;									\n\
					float weight = grad*exp(sq_dist * factor);				\n\
					float vidx = fmod(idx, 4);								\n\
					float4 inc = weight*float4(vidx == float4(0,1,2,3));	";

	if(GlobalUtil::_UseDynamicIndexing && strcmp(cgGetProfileString(ProgramCG::_FProfile), "gp4fp")==0)
//	if(ProgramCG::_FProfile == CG_PROFILE_GPU_FP) this enumerant is not defined in cg1.5
	{
		//gp_fp supports dynamic indexing
		out<<"\n\
					int iidx = int(floor(idx*0.25));	\n\
					bins[iidx]+=inc;					\n\
				}										\n\
			}											\n\
		}";

	}else
	{
		//nvfp40 still does not support dynamic array indexing
		//unrolled binary search...
		out<<"\n\
					if(idx < 16)							\n\
					{										\n\
						if(idx < 8)							\n\
						{									\n\
							if(idx < 4)	{	bins[0]+=inc;}	\n\
							else		{	bins[1]+=inc;}	\n\
						}else								\n\
						{									\n\
							if(idx < 12){	bins[2]+=inc;}	\n\
							else		{	bins[3]+=inc;}	\n\
						}									\n\
					}else if(idx < 32)						\n\
					{										\n\
						if(idx < 24)						\n\
						{									\n\
							if(idx <20)	{	bins[4]+=inc;}	\n\
							else		{	bins[5]+=inc;}	\n\
						}else								\n\
						{									\n\
							if(idx < 28){	bins[6]+=inc;}	\n\
							else		{	bins[7]+=inc;}	\n\
						}									\n\
					}else 						\n\
					{										\n\
						bins[8]+=inc;						\n\
					}										\n\
				}										\n\
			}											\n\
		}";

	}

	WriteOrientationCodeToStream(out);

	ProgramCG * program;
	s_orientation = program = new ProgramCG(buffer);
	_param_orientation_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_orientation_size = cgGetNamedParameter(*program, "size");
	_param_orientation_stex = cgGetNamedParameter(*program, "texS");
}

void ShaderBagCG::WriteOrientationCodeToStream(std::ostream& out)
{
	//smooth histogram and find the largest
/*
	smoothing kernel:	 (1 3 6 7 6 3 1 )/27
	the same as 3 pass of (1 1 1)/3 averaging
	maybe better to use 4 pass on the vectors...
*/


	//the inner loop on different array numbers is always unrolled in fp40

	//bug fixed here:)
	out<<"\n\
		float3x3 mat1 = float3x3(1, 0, 0, 3, 1, 0, 6, 3, 1)/27.0;; //bug fix.. \n\
		float4x4 mat2 = float4x4( 7, 6, 3, 1, 6, 7, 6, 3, 3, 6, 7, 6, 1, 3, 6, 7)/27.0;;\n\
		for (j=0; j<2; j++)								\n\
		{												\n\
			float4 prev  = bins[8];						\n\
			bins[9]		 = bins[0];						\n\
			for (i=0; i<9; i++)							\n\
			{												\n\
				float4 newb	=	mul ( bins[i], mat2);		\n\
				newb.xyz	+=	mul ( prev.yzw, mat1);		\n\
				prev = bins[i];								\n\
				newb.wzy	+=	mul	( bins[i+1].zyx, mat1);	\n\
				bins[i] = newb;							\n\
			}												\n\
		}";

	//find the maximum voting
	out<<"\n\
		float maxh; float2 maxh2; float4 maxh4 = bins[0];				\n\
		for (i=1; i<9; i++) maxh4 = max(maxh4, bins[i]);				\n\
		maxh2 = max(maxh4.xy, maxh4.zw); maxh = max(maxh2.x, maxh2.y);";

	char *testpeak_code;
	char *savepeak_code;



	//save two/three/four orientations with the largest votings?

	//
	if(GlobalUtil::_MaxOrientation>1)
	{
		out<<"\n\
		float4 Orientations = float4(0, 0, 0, 0);				\n\
		float4 weights = float4(0,0,0,0);		";	
		
		testpeak_code = "\n\
			{test = bins[i]>hh;";

		//save the orientations in weight-decreasing order
		if(GlobalUtil::_MaxOrientation ==2)
		{
		savepeak_code = "\n\
			if(weight <=weights.g){}\n\
			else if(weight >weights.r)\n\
			{weights.rg = float2(weight, weights.r); Orientations.rg = float2(th, Orientations.r);}\n\
			else {weights.g = weight; Orientations.g = th;}";
		}else if(GlobalUtil::_MaxOrientation ==3)
		{
		savepeak_code = "\n\
			if(weight <=weights.b){}\n\
			else if(weight >weights.r)\n\
			{weights.rgb = float3(weight, weights.rg); Orientations.rgb = float3(th, Orientations.rg);}\n\
			else if(weight >weights.g)\n\
			{weights.gb = float2(weight, weights.g); Orientations.gb = float2(th, Orientations.g);}\n\
			else {weights.b = weight; Orientations.b = th;}";
		}else
		{
		savepeak_code = "\n\
			if(weight <=weights.a){}\n\
			else if(weight >weights.r)\n\
			{weights = float4(weight, weights.rgb); Orientations = float4(th, Orientations.rgb);}\n\
			else if(weight >weights.g)\n\
			{weights.gba = float3(weight, weights.gb); Orientations.gba = float3(th, Orientations.gb);}\n\
			else if(weight >weights.b)\n\
			{weights.ba = float2(weight, weights.b); Orientations.ba = float2(th, Orientations.b);}\n\
			else {weights.a = weight; Orientations.a = th;}";
		}

	}else
	{
		out<<"\n\
		float Orientation;				";
		testpeak_code ="\n\
			if(npeaks<=0){								\n\
			test = (bins[i] == maxh)	;";
		savepeak_code="\n\
					npeaks++;								\n\
					Orientation = th;";

	}
	//find the peaks
	//the following loop will be unrolled anyway in fp40,
	//taking more than 1000 instrucsions..
	//....
	out<<"\n\
		float hh = maxh * 0.8;	bool4 test;	\n\
		bins[9] = bins[0];								\n\
		int npeaks = 0; k = 0;	float kp=0;				\n\
		float prevb	= bins[8].w;						\n\
		for ( i = 0; i <9 ; i++)						\n\
		{"
		<<testpeak_code<<"									\n\
			if( any ( test.xy || test.zw) )							\n\
			{											\n\
				if(test.r && bins[i].x > prevb && bins[i].x > bins[i].y )	\n\
				{											\n\
				    float	di = -0.5 * (bins[i].y-prevb) / (bins[i].y+prevb-bins[i].x - bins[i].x) ; \n\
					float	th = (k+di+0.5);	float weight = bins[i].x;"
					<<savepeak_code<<"\n\
				}\n\
				else if(test.g && all( bins[i].yy > bins[i].xz) )	\n\
				{											\n\
				    float	di = -0.5 * (bins[i].z-bins[i].x) / (bins[i].z+bins[i].x-bins[i].y- bins[i].y) ; \n\
					float	th = (k+di+1.5);	float weight = bins[i].y;				"
					<<savepeak_code<<"	\n\
				}"
		<<"\n\
				if(test.b && all( bins[i].zz > bins[i].yw) )	\n\
				{											\n\
				    float	di = -0.5 * (bins[i].w-bins[i].y) / (bins[i].w+bins[i].y-bins[i].z- bins[i].z) ; \n\
					float	th = (k+di+2.5);	float weight = bins[i].z;				"
					<<savepeak_code<<"	\n\
				}\n\
				else if(test.a && bins[i].w > bins[i].z && bins[i].w > bins[i+1].x )	\n\
				{											\n\
				    float	di = -0.5 * (bins[i+1].x-bins[i].z) / (bins[i+1].x+bins[i].z-bins[i].w - bins[i].w) ; \n\
					float	th = (k+di+3.5);	float weight = bins[i].w;				"
					<<savepeak_code<<"	\n\
				}\n\
			}}\n\
			k = k + 4;						\n\
			prevb = bins[i].w;\n\
		}";
	//WRITE output
	if(GlobalUtil::_OrientationPack2)
	{
		//pack two orientations in one float
	out<<"\n\
		 if(full_process_mode){\n\
			Orientations.xy = fmod(Orientations.xy / 36.0 + 1.0, 1.0);\n\
			if(weights.x <= 0) Orientations.x = 1.0;\n\
			if(weights.y <= 0) Orientations.y = 1.0;\n\
			float packed_orientation = pack_2ushort(Orientations.xy); \n\
			FeatureData = float4(pos, packed_orientation, sigma);\n\
		}else{\n\
			FeatureData = float4(pos, radians((Orientations.x)*10.0), sigma)\n\
		}\n";		
	}else if(GlobalUtil::_MaxOrientation>1)
	{
	out<<"\n\
		 if(full_process_mode){\n\
			npeaks = dot(float4(1,1,1,1), float4(weights>0));\n\
			npeaks = min(npeaks, float( "<<GlobalUtil::_MaxOrientation<<"));\n\
			OrientationData = radians((Orientations )*10.0);\n\
			FeatureData = float4(pos, npeaks>4?0:npeaks, sigma);\n\
		}else{\n\
			FeatureData = float4(pos, radians((Orientations.x)*10.0), sigma);\n\
		}\n";
	}else
	{
	out<<"\n\
		 FeatureData = float4(pos, radians((Orientation)*10.0), sigma);";
	}
	//end
	out<<"\n\
	}\n"<<'\0';


}

void ShaderBagCG::SetSimpleOrientationInput(int oTex, float sigma, float sigma_step)
{
	cgGLSetTextureParameter(_param_orientation_gtex, oTex);
	cgGLEnableTextureParameter(_param_orientation_gtex);
	cgGLSetParameter1f(_param_orientation_size, sigma);
}

void ShaderBagCG::SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int stex, float step)
{
	///
	cgGLSetTextureParameter(_param_orientation_gtex, gtex);	
	cgGLEnableTextureParameter(_param_orientation_gtex);

	if((GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)&& stex)
	{
		//specify texutre for subpixel subscale localization
		cgGLSetTextureParameter(_param_orientation_stex, stex);
		cgGLEnableTextureParameter(_param_orientation_stex);
	}

	float size[4];
	size[0] = (float)width;
	size[1] = (float)height;
	size[2] = sigma;
	size[3] = step;
	cgGLSetParameter4fv(_param_orientation_size, size);

}

void ShaderBagCG::SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth,  float width, float height, float sigma)
{
	///
	cgGLSetTextureParameter(_param_descriptor_gtex, gtex);	
	cgGLEnableTextureParameter(_param_descriptor_gtex);

	if(GlobalUtil::_DescriptorPPT ==32)
	{
		dwidth = dwidth /32.0f;
	}
	float dsize[4] ={dwidth, 1.0f/dwidth, fwidth, 1.0f/fwidth};
	cgGLSetParameter4fv(_param_descriptor_dsize, dsize);
	float size[2];
	size[0] = width;
	size[1] = height;
	cgGLSetParameter2fv(_param_descriptor_size, size);

}

//use geometry shader to do this??
void ShaderBagCG::LoadDescriptorShaderG()
{
	char buffer[20000];
	ostrstream out(buffer, 20000);
	
	if(GlobalUtil::_verbose)std::cout<<"Use Geometry shaders for Descriptor\n";


	//vertex here is using device coordinate from -1 to 1
	//need to transform
	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	POINT POINT_OUT void main(\n\
	AttribArray<float4> point: TEXCOORD0,		\n\
	uniform samplerRECT tex,				\n\
	uniform	samplerRECT gradTex,			\n\
	uniform float4		dsize,				\n\
	uniform float2		size				)		\n\
	{\n\
		float2 coord; float4 dpos;\n\
		float2 temp1, temp2;\n\
		temp1 = fmod(point[0].xx, dsize.xz);\n\
		temp2 = floor(point[0].xx * dsize.yw) + 0.5;\n\
		coord.x = temp1.y;					coord.y = temp2.y;\n\
		dpos.x = 0.5 + 32*floor(temp1.x);	dpos.y =temp2.x;	\n\
		dpos.zw = float2(0, 1);									\n\
		float2 dim	= size.xy;	//image size			\n\
		float2 pos = texRECT(tex, coord).xy;		\n\
		float  anglef = texRECT(tex, coord).z;		\n\
		float sigma = texRECT(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";

	WriteDescriptorCodeToStream(out);

	//write the result to different location of framebuffer
	out<<"\
		for (int i = 0; i < 4; i++)\n\
		{\n\
			for(int j = 0; j < 4; j++)\n\
			{\n\
				emitVertex(mul(glstate.matrix.mvp,dpos) : POSITION, DA[i][j]*norm : TEXCOORD0);\n\
				dpos.x+=1.0;\n\
				emitVertex(mul(glstate.matrix.mvp,dpos) : POSITION, DB[i][j]*norm : TEXCOORD0);\n\
				dpos.x+=1.0;\n\
			}\n\
		}\n\
	}\n"<<'\0';

	ProgramCG * program; 
	s_descriptor_gp = program =  new ProgramCG(buffer, NULL, ProgramCG::_GProfile);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gradTex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");

	s_descriptor_fp = new ProgramCG(
		"void main(in float4 tex : TEXCOORD0, out float4 FragColor : COLOR0){FragColor = tex;	}");
}





///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////PACKED VERSION?///////////////////////////////////

ShaderBagPKCG::ShaderBagPKCG()
{
	cgSetErrorCallback(ShaderBagCG::ErrorCallback);
	ProgramCG::InitContext();
}

void ShaderBagPKCG::UnloadProgram()
{

	cgGLUnbindProgram(ProgramCG::_FProfile);
	cgGLDisableProfile(ProgramCG::_FProfile);
}

void ShaderBagPKCG::LoadFixedShaders()
{
	ProgramCG * program;
	s_display_gaussian = new ProgramCG(
		"void main(uniform samplerRECT tex, in float4 TexCoord0:TEXCOORD0, out float4 FragData: COLOR0 ){\
		float4 pc = texRECT(tex, TexCoord0.xy);	 bool2 ff = (fract(TexCoord0.xy) < 0.5);\n\
		float v = ff.y?(ff.x? pc.r : pc.g):(ff.x?pc.b:pc.a); FragData = float4(v.xxx, 1.0);}");

	/*
	char *rgb2gray_packing_code =
		"void main(uniform samplerRECT rgbTex, in float4 TexCoord0 : TEXCOORD0, \n\
			in float4 TexCoord1 : TEXCOORD1, in float4 TexCoord2 : TEXCOORD2, \n\
			in float4 TexCoord3 : TEXCOORD3, out float4 FragData : COLOR0){\
			const float3 weight = vec3(0.299, 0.587, 0.114);\n\
			FragData.r = dot(weight, texRECT(rgbTex,TexCoord0.st ).rgb);\
			FragData.g = dot(weight, texRECT(rgbTex,TexCoord1.st ).rgb);\
			FragData.b = dot(weight, texRECT(rgbTex,TexCoord2.st ).rgb);\
			FragData.a = dot(weight, texRECT(rgbTex,TexCoord3.st ).rgb);}";//
	s_gray = new ProgramCG( rgb2gray_packing_code);
	*/

	char *rgb2gray_code =
	"void main(float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float intensity = dot(float3(0.299, 0.587, 0.114), texRECT(tex,TexCoord0.xy ).rgb);\n\
	FragColor= float4(intensity, intensity, intensity, 1.0);}";//

	s_gray = new ProgramCG( rgb2gray_code);

	char * sampling_code = "void main(uniform samplerRECT tex, in float4 TexCoord0 : TEXCOORD0, \n\
			in float4 TexCoord1 : TEXCOORD1, in float4 TexCoord2 : TEXCOORD2, \n\
			in float4 TexCoord3 : TEXCOORD3, out float4 FragData : COLOR0 ){\n\
			FragData= float4(	texRECT(tex,TexCoord0.st ).r,texRECT(tex,TexCoord1.st ).r,\n\
								texRECT(tex,TexCoord2.st ).r,texRECT(tex,TexCoord3.st ).r);}";
	s_sampling = new ProgramCG(sampling_code);


	s_copy_key = new ProgramCG(
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	FragColor.rg= texRECT(tex, TexCoord0.xy).rg; FragColor.ba = float2(0,1);	}");
	
	s_texcoord = new ProgramCG(
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0){FragColor = TexCoord0;	}");
	
	s_display_dog =  new ProgramCG(
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 pc = texRECT(tex, TexCoord0.xy); bool2 ff = (fract(TexCoord0.xy) < 0.5);\n\
	float v = ff.y ?(ff.x ? pc.r : pc.g):(ff.x ? pc.b : pc.a);float g = (0.5+20.0*v);\n\
	FragColor = float4(g, g, g, 1.0);}" );


	s_display_grad = new ProgramCG(
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 pc = texRECT(tex, TexCoord0.xy); bool2 ff = (fract(TexCoord0.xy) < 0.5);\n\
	float v = ff.y ?(ff.x ? pc.r : pc.g):(ff.x ? pc.b : pc.a); FragColor = float4(5.0 *v.xxx, 1.0); }");

	s_display_keys= new ProgramCG(
	//	GlobalUtil::_SubpixelLocalization? 
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy); cc = unpack_4ubyte(cc.r); bool2 ff = (fract(TexCoord0.xy) < 0.5);\n\
	float v = ff.y ?(ff.x ? cc.r : cc.g):(ff.x ? cc.b : cc.a);\n\
	if(v > 0.99) FragColor = float4(1.0, 0, 0,1.0); \n\
	else {if (v > 0.49) FragColor = float4(0.0,1.0,0.0,1.0);	else discard;}}" /*: 
	"void main(in float4 TexCoord0 : TEXCOORD0, out float4 FragColor : COLOR0, uniform samplerRECT tex){\n\
	float4 cc = texRECT(tex, TexCoord0.xy); bool2 ff = (fract(TexCoord0.xy) < 0.5);\n\
	float v = ff.y ?(ff.x ? cc.r : cc.g):(ff.x ? cc.b : cc.a);\n\
	if(v ==1.0) FragColor = float4(1.0, 0, 0,1.0); \n\
	else {if (v ==0.5) FragColor = float4(0.0,1.0,0.0,1.0);	else discard;}}"*/);		


	s_margin_copy = program = new ProgramCG(
	"void main(in float4 texCoord0: TEXCOORD0, out float4 FragColor: COLOR0, \n\
	uniform samplerRECT tex, uniform float4 truncate){\n\
	float4 cc = texRECT(tex, min(texCoord0.xy, truncate.xy)); \n\
	bool2 ob = texCoord0.xy < truncate.xy;\n\
	if(ob.y) { FragColor = (truncate.z ==0 ? cc.rrbb : cc.ggaa); } \n\
	else if(ob.x) {FragColor = (truncate.w <1.5 ? cc.rgrg : cc.baba);} \n\
	else {	float4 weights = float4(float4(0, 1, 2, 3) == truncate.w);\n\
	float v = dot(weights, cc); FragColor = v.xxxx;}}");

	_param_margin_copy_truncate = cgGetNamedParameter(*program, "truncate");


//shader used to write a vertex buffer object
	//which is used to draw the quads of each feature
	s_vertex_list = program = new ProgramCG(
	"void main(in float4 TexCoord0: TEXCOORD0,\n\
	uniform float4 sizes, \n\
	uniform samplerRECT tex, \n\
	out float4 FragColor: COLOR0){\n\
	float fwidth = sizes.y; \n\
	float twidth = sizes.z; \n\
	float rwidth = sizes.w; \n\
	float size = sizes.x; \n\
	float index = 0.1*(fwidth*floor(TexCoord0.y) + TexCoord0.x);\n\
	float px = fmod(index, twidth);\n\
	float2 tpos= floor(float2(px, index*rwidth))+0.5;\n\
	float4 cc = texRECT(tex, tpos );\n\
	FragColor.zw = float2(0.0, 1.0);\n\
	if(any(cc.xy <=0)) {FragColor.xy = cc.xy;}else \n\
	{\n\
		float type = frac(px);\n\
		float2 dxy; float s, c;\n\
		dxy.x = type < 0.1 ? 0 : ((type <0.5 || type > 0.9)? size : -size);\n\
		dxy.y = type < 0.2 ? 0 : ((type < 0.3 || type > 0.7 )? -size :size); \n\
		sincos(cc.b, s, c);\n\
		FragColor.x = cc.x + c*dxy.x-s*dxy.y;\n\
		FragColor.y = cc.y + c*dxy.y+s*dxy.x;}\n\
	}\n\0");
	/*FragColor = float4(tpos, 0.0, 1.0);}\n\0");*/

	_param_genvbo_size = cgGetNamedParameter(*program, "sizes");

	s_zero_pass = new ProgramCG("void main(out float4 FragColor : COLOR0){FragColor = 0;}");

	s_grad_pass = program = new ProgramCG(
	"void main (\
	float4 TexCC : TEXCOORD0, float4 TexLC : TEXCOORD1,\n\
	float4 TexRC : TEXCOORD2, float4 TexCD : TEXCOORD3, float4 TexCU : TEXCOORD4,\n\
	out float4 FragData0 : COLOR0, out float4 FragData1 : COLOR1, \n\
	out float4 FragData2 : COLOR2, uniform samplerRECT tex, uniform samplerRECT texp)\n\
	{\n\
		float4 v1, v2, gg;\n\
		float4 cc = texRECT(tex, TexCC.xy);\n\
		float4 cp = texRECT(texp, TexCC.xy);\n\
		FragData0 = cc - cp; \n\
		float4 cl = texRECT(tex, TexLC.xy);	float4 cr = texRECT(tex, TexRC.xy);\n\
		float4 cd = texRECT(tex, TexCD.xy); float4 cu = texRECT(tex, TexCU.xy);\n\
		float4 dx = (float4(cr.rb, cc.ga) - float4(cc.rb, cl.ga)).zxwy;\n\
		float4 dy = (float4(cu.rg, cc.ba) - float4(cc.rg, cd.ba)).zwxy;\n\
		FragData1 = 0.5 * sqrt(dx*dx + dy * dy);\n\
		FragData2 = FragData1 > 0?  atan2(dy, dx) : float4(0);\n\
	}\n\0");

	_param_grad_pass_texp = cgGetNamedParameter(*program, "texp");



	////
	if(GlobalUtil::_SupportFP40)
	{
		LoadOrientationShader();
		if(GlobalUtil::_DescriptorPPT) LoadDescriptorShader();
	}else
	{
		s_orientation = program =  new ProgramCG(
		"void main(out float4 FragColor : COLOR0, \n\
		uniform samplerRECT fTex, uniform samplerRECT oTex, \n\
		uniform float2 size, \n\
		in float2 tpos : TEXCOORD0){\n\
		float4 cc = texRECT(fTex, tpos);\n\
		float2 co = cc.xy * 0.5; \n\
		float4 oo = texRECT(oTex, co);\n\
		bool2 bo = fract(co) < 0.5; \n\
		float o = bo.y? (bo.x? oo.r : oo.g) : (bo.x? oo.b : oo.a); \n\
		FragColor = float4(cc.rg, o, size.x * pow(size.y, cc.a));}");  
		_param_orientation_gtex= cgGetNamedParameter(*program, "oTex");
		_param_orientation_size= cgGetNamedParameter(*program, "size");

		GlobalUtil::_FullSupported = 0;
		GlobalUtil::_MaxOrientation = 0;
		GlobalUtil::_DescriptorPPT = 0;
		std::cerr<<"Orientation simplified on this hardware"<<endl;
		std::cerr<<"Descriptor ignored on this hardware"<<endl;
	}
}


void ShaderBagPKCG::LoadGenListShader(int ndoglev, int nlev)
{

	//the V2 algorithms are only slightly faster, but way more complicated
	//LoadGenListShaderV2(ndoglev, nlev); return; 
	ProgramCG * program;

	s_genlist_init_tight = new ProgramCG(
	"void main (uniform samplerRECT tex, in float4 TexCoord0 : TEXCOORD0,\n\
	in float4 TexCoord1 : TEXCOORD1, in float4 TexCoord2 : TEXCOORD2, \n\
	in float4 TexCoord3 : TEXCOORD3, out float4 FragColor : COLOR0)\n\
	{\n\
		bool4 helper1 = unpack_4ubyte(texRECT(tex, TexCoord0.xy).r) > 0; \n\
		bool4 helper2 = unpack_4ubyte(texRECT(tex, TexCoord1.xy).r) > 0;\n\
		bool4 helper3 = unpack_4ubyte(texRECT(tex, TexCoord2.xy).r) > 0;\n\
		bool4 helper4 = unpack_4ubyte(texRECT(tex, TexCoord3.xy).r) > 0;\n\
		FragColor.r = any(helper1.xy || helper1.zw);	\n\
		FragColor.g = any(helper2.xy || helper2.zw);	\n\
		FragColor.b = any(helper3.xy || helper3.zw);	\n\
		FragColor.a = any(helper4.xy || helper4.zw);	\n\
	}");

	s_genlist_init_ex = program = new ProgramCG(
	"void main (uniform float4 bbox, uniform samplerRECT tex, \n\
	in float4 TexCoord0 : TEXCOORD0, in float4 TexCoord1 : TEXCOORD1, \n\
	in float4 TexCoord2 : TEXCOORD2, in float4 TexCoord3 : TEXCOORD3,\n\
	out float4 FragColor : COLOR0)\n\
	{\n\
		bool4 helper1 = unpack_4ubyte(texRECT(tex, TexCoord0.xy).r) > 0; \n\
		bool4 helper2 = unpack_4ubyte(texRECT(tex, TexCoord1.xy).r) > 0;\n\
		bool4 helper3 = unpack_4ubyte(texRECT(tex, TexCoord2.xy).r) > 0;\n\
		bool4 helper4 = unpack_4ubyte(texRECT(tex, TexCoord3.xy).r) > 0;\n\
		bool4 bx1 = TexCoord0.xxyy < bbox; \n\
		bool4 bx4 = TexCoord3.xxyy < bbox; \n\
		bool4 bx2 = bool4(bx4.xy, bx1.zw); \n\
		bool4 bx3 = bool4(bx1.xy, bx4.zw);\n\
		helper1 = (bx1.xyxy && bx1.zzww && helper1);\n\
		helper2 = (bx2.xyxy && bx2.zzww && helper2);\n\
		helper3 = (bx3.xyxy && bx3.zzww && helper3);\n\
		helper4 = (bx4.xyxy && bx4.zzww && helper4);\n\
		FragColor.r = any(helper1.xy || helper1.zw);	\n\
		FragColor.g = any(helper2.xy || helper2.zw);	\n\
		FragColor.b = any(helper3.xy || helper3.zw);	\n\
		FragColor.a = any(helper4.xy || helper4.zw);	\n\
	}");
	_param_genlist_init_bbox = cgGetNamedParameter( *program, "bbox");

	s_genlist_end = program = new ProgramCG(
		GlobalUtil::_KeepExtremumSign == 0 ? 
	"\
	void main(	uniform samplerRECT tex, uniform samplerRECT ktex,\n\
				in float4 tpos : TEXCOORD0,	out float4 FragColor : COLOR0)\n\
	{\n\
		float4 tc = texRECT( tex, tpos.xy);\n\
		float2 pos = tc.rg; float index = tc.b;\n	\
		float4 tk = texRECT( ktex, pos); \n\
		float4 key_unpack = unpack_4ubyte(tk.r); \n\
		float4 keys = float4(key_unpack>0); \n\
		float2 opos; \n\
		opos.x = dot(keys, float4(-0.5, 0.5, -0.5, 0.5));\n\
		opos.y = dot(keys, float4(-0.5, -0.5, 0.5, 0.5));\n\
		FragColor = float4(opos + pos + pos + tk.yz, 1.0, tk.w);\n\
	}" : 
	"\
	void main(	uniform samplerRECT tex, uniform samplerRECT ktex,\n\
				in float4 tpos : TEXCOORD0,	out float4 FragColor : COLOR0)\n\
	{\n\
		float4 tc = texRECT( tex, tpos.xy);\n\
		float2 pos = tc.rg; float index = tc.b;\n	\
		float4 tk = texRECT( ktex, pos); \n\
		float4 key_unpack = unpack_4ubyte(tk.r); \n\
		float4 keys = float4(key_unpack>0); \n\
		float2 opos; \n\
		opos.x = dot(keys, float4(-0.5, 0.5, -0.5, 0.5));\n\
		opos.y = dot(keys, float4(-0.5, -0.5, 0.5, 0.5));\n\
		float2 temp = key_unpack.xy + key_unpack.zw; \n\
		float key_sign = (temp.x + temp.y) < 0.6? -1.0 : 1.0; \n\
		FragColor = float4(opos + pos + pos + tk.yz, key_sign, tk.w);\n\
	}"	
	);
	_param_genlist_end_ktex = cgGetNamedParameter(*program, "ktex");

	//reduction ...
	s_genlist_histo = new ProgramCG(
	"void main (uniform samplerRECT tex, in float2 TexCoord0 : TEXCOORD0,\n\
	in float2 TexCoord1 : TEXCOORD1, in float2 TexCoord2 : TEXCOORD2, \n\
	in float2 TexCoord3 : TEXCOORD3, out float4 FragColor : COLOR0)\n\
	{\n\
		float4 helper; float4 helper2; \n\
		helper = texRECT(tex, TexCoord0); helper2.xy = helper.xy + helper.zw; \n\
		helper = texRECT(tex, TexCoord1); helper2.zw = helper.xy + helper.zw; \n\
		FragColor.rg = helper2.xz + helper2.yw;\n\
		helper = texRECT(tex, TexCoord2); helper2.xy = helper.xy + helper.zw; \n\
		helper = texRECT(tex, TexCoord3); helper2.zw = helper.xy + helper.zw; \n\
		FragColor.ba= helper2.xz+helper2.yw;\n\
	}");


	//read of the first part, which generates tex coordinates 

	s_genlist_start= program =  ShaderBagCG::LoadGenListStepShader(1, 1);
	_param_ftex_width= cgGetNamedParameter(*program, "width");
	_param_genlist_start_tex0 = cgGetNamedParameter(*program, "tex0");
	//stepping
	s_genlist_step = program = ShaderBagCG::LoadGenListStepShader(0, 1);
	_param_genlist_step_tex= cgGetNamedParameter(*program, "tex");
	_param_genlist_step_tex0= cgGetNamedParameter(*program, "tex0");


}



void ShaderBagPKCG::LoadGenListShaderV2(int ndoglev, int nlev)
{
	ProgramCG * program;

	s_genlist_init_tight = new ProgramCG(
	"void main (uniform samplerRECT tex, in float4 TexCoord0 : TEXCOORD0,\n\
	in float4 TexCoord1 : TEXCOORD1, in float4 TexCoord2 : TEXCOORD2, \n\
	in float4 TexCoord3 : TEXCOORD3, out float4 FragColor : COLOR0)\n\
	{\n\
		float4 data1 = texRECT(tex, TexCoord0.xy);\n\
		float4 data2 = texRECT(tex, TexCoord1.xy);\n\
		float4 data3 = texRECT(tex, TexCoord2.xy);\n\
		float4 data4 = texRECT(tex, TexCoord3.xy);\n\
		bool4 helper1 = (unpack_4ubyte(data1.r) > 0); \n\
		bool4 helper2 = (unpack_4ubyte(data2.r) > 0);\n\
		bool4 helper3 = (unpack_4ubyte(data3.r) > 0);\n\
		bool4 helper4 = (unpack_4ubyte(data4.r) > 0);\n\
		FragColor.r = any(helper1.xy || helper1.zw);	\n\
		FragColor.g = any(helper2.xy || helper2.zw);	\n\
		FragColor.b = any(helper3.xy || helper3.zw);	\n\
		FragColor.a = any(helper4.xy || helper4.zw);	\n\
		if(dot(FragColor, float4(1,1,1,1)) == 1) \n\
		{\n\
			//use a special method if there is only one in the 16, \n\
			float4 data, helper; float2 pos, opos; \n\
			if(FragColor.r){ \n\
				data = data1; helper = helper1; pos = TexCoord0.xy;\n\
			}else if(FragColor.g){\n\
				data = data2; helper = helper2; pos = TexCoord1.xy;\n\
			}else if(FragColor.b){\n\
				data = data3; helper = helper3; pos = TexCoord2.xy;\n\
			}else{\n\
				data = data4; helper = helper4; pos = TexCoord3.xy;\n\
			}\n\
			opos.x = dot(helper, float4(-0.5, 0.5, -0.5, 0.5));\n\
			opos.y = dot(helper, float4(-0.5, -0.5, 0.5, 0.5));\n\
			FragColor = float4( pos + pos + opos + data.yz, -1, data.w); \n\
		}\n\
	}");

	s_genlist_init_ex = program = new ProgramCG(
	"void main (uniform float4 bbox, uniform samplerRECT tex, \n\
	in float4 TexCoord0 : TEXCOORD0, in float4 TexCoord1 : TEXCOORD1, \n\
	in float4 TexCoord2 : TEXCOORD2, in float4 TexCoord3 : TEXCOORD3,\n\
	out float4 FragColor : COLOR0)\n\
	{\n\
		float4 data1 = texRECT(tex, TexCoord0.xy);\n\
		float4 data2 = texRECT(tex, TexCoord1.xy);\n\
		float4 data3 = texRECT(tex, TexCoord2.xy);\n\
		float4 data4 = texRECT(tex, TexCoord3.xy);\n\
		bool4 helper1 = unpack_4ubyte(data1.r) > 0; \n\
		bool4 helper2 = unpack_4ubyte(data2.r) > 0;\n\
		bool4 helper3 = unpack_4ubyte(data3.r) > 0;\n\
		bool4 helper4 = unpack_4ubyte(data4.r) > 0;\n\
		bool4 bx1 = TexCoord0.xxyy < bbox; \n\
		bool4 bx4 = TexCoord3.xxyy < bbox; \n\
		bool4 bx2 = bool4(bx4.xy, bx1.zw); \n\
		bool4 bx3 = bool4(bx1.xy, bx4.zw);\n\
		helper1 = bx1.xyxy && bx1.zzww && helper1; \n\
		helper2 = bx2.xyxy && bx2.zzww && helper2; \n\
		helper3 = bx3.xyxy && bx3.zzww && helper3; \n\
		helper4 = bx4.xyxy && bx4.zzww && helper4; \n\
		FragColor.r = any(helper1.xy || helper1.zw);	\n\
		FragColor.g = any(helper2.xy || helper2.zw);	\n\
		FragColor.b = any(helper3.xy || helper3.zw);	\n\
		FragColor.a = any(helper4.xy || helper4.zw);	\n\
		if(dot(FragColor, float4(1,1,1,1)) == 1) \n\
		{\n\
			//use a special method if there is only one in the 16, \n\
			float4 data, helper; bool4 bhelper; float2 pos, opos; \n\
			if(FragColor.r){ \n\
				data = data1; bhelper = helper1; pos = TexCoord0.xy;\n\
			}else if(FragColor.g){\n\
				data = data2; bhelper = helper2; pos = TexCoord1.xy;\n\
			}else if(FragColor.b){\n\
				data = data3; bhelper = helper3; pos = TexCoord2.xy;\n\
			}else{\n\
				data = data4; bhelper = helper4; pos = TexCoord3.xy;\n\
			}\n\
			helper = float4(bhelper); \n\
			opos.x = dot(helper, float4(-0.5, 0.5, -0.5, 0.5));\n\
			opos.y = dot(helper, float4(-0.5, -0.5, 0.5, 0.5));\n\
			FragColor = float4(pos + pos + opos + data.yz, -1, data.w); \n\
		}\n\
	}");
	_param_genlist_init_bbox = cgGetNamedParameter( *program, "bbox");

	s_genlist_end = program = new ProgramCG(
	"\
	void main(	uniform samplerRECT tex, uniform samplerRECT ktex,\n\
				in float4 tpos : TEXCOORD0,	out float4 FragColor : COLOR0)\n\
	{\n\
		float4 tc = texRECT( tex, tpos.xy);\n\
		float2 pos = tc.rg; float index = tc.b;\n	\
		if(index == -1)\n\
		{\n\
			FragColor = float4(tc.xy, 0, tc.w);\n\
		}else\n\
		{\n\
			float4 tk = texRECT( ktex, pos); \n\
			float4 keys = float4(unpack_4ubyte(tk.r)>0); \n\
			float2 opos; \n\
			opos.x = dot(keys, float4(-0.5, 0.5, -0.5, 0.5));\n\
			opos.y = dot(keys, float4(-0.5, -0.5, 0.5, 0.5));\n\
			FragColor = float4(opos + pos + pos + tk.yz, 0, tk.w);\n\
		}\n\
	}");
	_param_genlist_end_ktex = cgGetNamedParameter(*program, "ktex");

	//reduction ...
	s_genlist_histo = new ProgramCG(
	"void main (uniform samplerRECT tex, in float2 TexCoord0 : TEXCOORD0,\n\
	in float2 TexCoord1 : TEXCOORD1, in float2 TexCoord2 : TEXCOORD2, \n\
	in float2 TexCoord3 : TEXCOORD3, out float4 FragColor : COLOR0)\n\
	{\n\
		float4 helper[4]; float4 helper2; \n\
		helper[0] = texRECT(tex, TexCoord0); helper2.xy = helper[0].xy + helper[0].zw; \n\
		helper[1] = texRECT(tex, TexCoord1); helper2.zw = helper[1].xy + helper[1].zw; \n\
		FragColor.rg = helper2.xz + helper2.yw;\n\
		helper[2] = texRECT(tex, TexCoord2); helper2.xy = helper[2].xy + helper[2].zw; \n\
		helper[3] = texRECT(tex, TexCoord3); helper2.zw = helper[3].xy + helper[3].zw; \n\
		FragColor.ba= helper2.xz+helper2.yw;\n\
		bool4 keyt = float4(helper[0].z, helper[1].z, helper[2].z, helper[3].z) == -1.0; \n\
		float keyc = dot(float4(keyt), float4(1,1,1,1)); \n\
		if(keyc == 1.0 && dot(FragColor, float4(1,1,1,1)) == -1.0) \n\
		{\n\
			if(keyt.x) FragColor = helper[0];\n\
			else if(keyt.y) FragColor = helper[1]; \n\
			else if(keyt.z) FragColor = helper[2]; \n\
			else FragColor = helper[3]; \n\
		}else\n\
		{\n\
			FragColor = keyt? float4(1,1,1,1) : FragColor;\n\
		}\n\
	}");

	//read of the first part, which generates tex coordinates 

	s_genlist_start= program =  ShaderBagCG::LoadGenListStepShaderV2(1, 1);
	_param_ftex_width= cgGetNamedParameter(*program, "width");
	_param_genlist_start_tex0 = cgGetNamedParameter(*program, "tex0");
	//stepping
	s_genlist_step = program = ShaderBagCG::LoadGenListStepShaderV2(0, 1);
	_param_genlist_step_tex= cgGetNamedParameter(*program, "tex");
	_param_genlist_step_tex0= cgGetNamedParameter(*program, "tex0");


}



ProgramCG* ShaderBagCG::LoadGenListStepShaderV2(int start, int step)
{
	int i;
	char buffer[10240];
	//char chanels[5] = "rgba";
	ostrstream out(buffer, 10240);
	out<<"void main(out float4 FragColor : COLOR0, \n";

	for(i = 0; i < step; i++) out<<"uniform samplerRECT tex"<<i<<",\n";

	if(start)
	{
		out<<"uniform float width, \nin float2 tpos : TEXCOORD0){\n";
		out<<"float  index = floor(tpos.y) * width + floor(tpos.x);\n";
		out<<"float2 pos = float2(0.5, 0.5);\n";
	}else
	{
		out<<"uniform samplerRECT tex, in float2 tpos: TEXCOORD0 ){\n";
		out<<"float4 tc = texRECT( tex, tpos);\n";
		out<<"float2 pos = tc.rg; float index = tc.b;\n";
		out<<"if(index==-1) {FragColor = tc; return;}\n";
	}
	out<<"float2 sum; 	float4 cc;\n";



	if(step>0)
	{
		out<<"float2 cpos = float2(-0.5, 0.5);\t float2 opos;\n";
		for(i = 0; i < step; i++)
		{

			out<<"cc = texRECT(tex"<<i<<", pos);\n";
			out<<"if(cc.z == -1){FragColor = cc; return;}";
			out<<"sum.x = cc.r + cc.g;if (index < sum.x){ if(index < cc.r) opos = cpos.xx; else {opos = cpos.yx; index -= cc.r;}}\n";
			out<<"else {index -= sum.x; if(index < cc.b) opos = cpos.xy; else{opos = cpos.yy; index -= cc.b;}}";
			out<<"pos = (pos + pos + opos);\n";
		}
	}
	out<<"FragColor = float4(pos, index, 1);\n";
	out<<"}\n"<<'\0';
	return new ProgramCG(buffer);
}


void ShaderBagPKCG:: LoadKeypointShader(float threshold, float edge_threshold)
{
	//
	ProgramCG * program;
	char buffer[10240];
	float threshold0 = threshold* (GlobalUtil::_SubpixelLocalization?0.8f:1.0f);
	float threshold1 = threshold;
	float threshold2 = (edge_threshold+1)*(edge_threshold+1)/edge_threshold;
	ostrstream out(buffer, 10240);
	out<<setprecision(8);
	//tex(X)(Y)
	//X: (CLR) (CENTER 0, LEFT -1, RIGHT +1)  
	//Y: (CDU) (CENTER 0, DOWN -1, UP    +1) 

	out<<"\
	void main (\
	float4 TexCC : TEXCOORD0, float4 TexLC : TEXCOORD1,\n\
	float4 TexRC : TEXCOORD2, float4 TexCD : TEXCOORD3, \n\
	float4 TexCU : TEXCOORD4, float4 TexLD : TEXCOORD5, \n\
	float4 TexLU : TEXCOORD6, float4 TexRD : TEXCOORD7,\n\
	out float4 FragData0 : COLOR0, uniform samplerRECT tex, \n\
	uniform samplerRECT texU, uniform samplerRECT texD)\n\
	{\n\
		float2 TexRU = float2(TexRC.x, TexCU.y); \n\
		float4 ccc = texRECT(tex, TexCC.xy);\n\
		float4 clc = texRECT(tex, TexLC.xy);\n\
		float4 crc = texRECT(tex, TexRC.xy);\n\
		float4 ccd = texRECT(tex, TexCD.xy);\n\
		float4 ccu = texRECT(tex, TexCU.xy);\n\
		float4 cld = texRECT(tex, TexLD.xy);\n\
		float4 clu = texRECT(tex, TexLU.xy);\n\
		float4 crd = texRECT(tex, TexRD.xy);\n\
		float4 cru = texRECT(tex, TexRU.xy);\n\
		float  cc[4] = {ccc.r, ccc.g, ccc.b, ccc.a};\n\
		float4  v1[4], v2[4];\n\
		v1[0] = float4(clc.g, ccc.g, ccd.b, ccc.b);\n\
		v1[1] = float4(ccc.r, crc.r, ccd.a, ccc.a);\n\
		v1[2] = float4(clc.a, ccc.a, ccc.r, ccu.r);\n\
		v1[3] = float4(ccc.b, crc.b, ccc.g, ccu.g);\n\
		v2[0] = float4(cld.a, clc.a, ccd.a, ccc.a);\n\
		v2[1] = float4(ccd.b, ccc.b, crd.b, crc.b);\n\
		v2[2] = float4(clc.g, clu.g, ccc.g, ccu.g);\n\
		v2[3] = float4(ccc.r, ccu.r, crc.r, cru.r);\n"

	//test against 8 neighbours
	//use variable to identify type of extremum
	//1.0 for local maximum and 0.5 for minimum
	<<"\
		float4 key ={0, 0, 0, 0}; \n\
		for(int i = 0; i < 4; i++)\n\
		{\n\
			bool4 test1 = cc[i] > max(v1[i], v2[i]), test2 = cc[i] < min(v1[i], v2[i]);\n\
			key[i] = cc[i] > "	<<threshold0 <<" && all(test1.xy&&test1.zw)?1.0: 0.0;\n\
			key[i] = cc[i] < "	<<-threshold0 <<" && all(test2.xy&&test2.zw)?0.5: key[i];\n\
		}\n\
		if(TexCC.x < 1.0) {key.rb = 0;}\n\
		if(TexCC.y < 1.0) {key.rg = 0;}\n\
		if(all(key == 0.0)) return; \n";

	//do edge supression first.. 
	//vector v1 is < (-1, 0), (1, 0), (0,-1), (0, 1)>
	//vector v2 is < (-1,-1), (-1,1), (1,-1), (1, 1)>

	out<<"\
		float fxx[4], fyy[4], fxy[4], fx[4], fy[4];\n\
		for(int i = 0; i < 4; i++) \n\
		{\n\
			if(key[i]>0)\n\
			{\n\
				float4 D2 = v1[i].xyzw - cc[i];\n\
				float2 D4 = v2[i].xw - v2[i].yz;\n\
				float2 D5 = 0.5*(v1[i].yw-v1[i].xz); \n\
				fx[i] = D5.x;\n\
				fy[i] = D5.y ;\n\
				fxx[i] = D2.x + D2.y;\n\
				fyy[i] = D2.z + D2.w;\n\
				fxy[i] = 0.25*(D4.x + D4.y);\n\
				float fxx_plus_fyy = fxx[i] + fyy[i];\n\
				float score_up = fxx_plus_fyy*fxx_plus_fyy; \n\
				float score_down = (fxx[i]*fyy[i] - fxy[i]*fxy[i]);\n\
				if( score_down <= 0 || score_up > "<<threshold2<<" * score_down)key[i] = 0;\n\
			}\n\
		}\n\
		if(all(key == 0.0)) return; \n\n";

	////////////////////////////////////////////////
	//read 9 pixels of upper/lower level
	out<<"\
		float4  v4[4], v5[4], v6[4];\n\
		ccc = texRECT(texU, TexCC.xy);\n\
		clc = texRECT(texU, TexLC.xy);\n\
		crc = texRECT(texU, TexRC.xy);\n\
		ccd = texRECT(texU, TexCD.xy);\n\
		ccu = texRECT(texU, TexCU.xy);\n\
		cld = texRECT(texU, TexLD.xy);\n\
		clu = texRECT(texU, TexLU.xy);\n\
		crd = texRECT(texU, TexRD.xy);\n\
		cru = texRECT(texU, TexRU.xy);\n\
		float  cu[4] = {ccc.r, ccc.g, ccc.b, ccc.a};\n\
		v4[0] = float4(clc.g, ccc.g, ccd.b, ccc.b);\n\
		v4[1] = float4(ccc.r, crc.r, ccd.a, ccc.a);\n\
		v4[2] = float4(clc.a, ccc.a, ccc.r, ccu.r);\n\
		v4[3] = float4(ccc.b, crc.b, ccc.g, ccu.g);\n\
		v6[0] = float4(cld.a, clc.a, ccd.a, ccc.a);\n\
		v6[1] = float4(ccd.b, ccc.b, crd.b, crc.b);\n\
		v6[2] = float4(clc.g, clu.g, ccc.g, ccu.g);\n\
		v6[3] = float4(ccc.r, ccu.r, crc.r, cru.r);\n"
	<<"\
		for(int i = 0; i < 4; i++)\n\
		{\n\
			if(key[i] == 1.0)\n\
			{\n\
				bool4 test = cc[i]<v4[i] || cc[i]<v6[i]; \n\
				if(cc[i] < cu[i] || any(test.xy||test.zw))key[i] = 0.0; \n\
			}else if(key[i] == 0.5)\n\
			{\n\
				bool4 test = cc[i]>v4[i] || cc[i]>v6[i]; \n\
				if(cc[i] > cu[i] || any(test.xy||test.zw))key[i] = 0.0; \n\
			}\n\
		}\n\
		if(all(key == 0.0)) return; "
	<<"\
		ccc = texRECT(texD, TexCC.xy);\n\
		clc = texRECT(texD, TexLC.xy);\n\
		crc = texRECT(texD, TexRC.xy);\n\
		ccd = texRECT(texD, TexCD.xy);\n\
		ccu = texRECT(texD, TexCU.xy);\n\
		cld = texRECT(texD, TexLD.xy);\n\
		clu = texRECT(texD, TexLU.xy);\n\
		crd = texRECT(texD, TexRD.xy);\n\
		cru = texRECT(texD, TexRU.xy);\n\
		float  cd[4] = {ccc.r, ccc.g, ccc.b, ccc.a};\n\
		v5[0] = float4(clc.g, ccc.g, ccd.b, ccc.b);\n\
		v5[1] = float4(ccc.r, crc.r, ccd.a, ccc.a);\n\
		v5[2] = float4(clc.a, ccc.a, ccc.r, ccu.r);\n\
		v5[3] = float4(ccc.b, crc.b, ccc.g, ccu.g);\n\
		v6[0] = float4(cld.a, clc.a, ccd.a, ccc.a);\n\
		v6[1] = float4(ccd.b, ccc.b, crd.b, crc.b);\n\
		v6[2] = float4(clc.g, clu.g, ccc.g, ccu.g);\n\
		v6[3] = float4(ccc.r, ccu.r, crc.r, cru.r);\n"
	<<"\
		for(int i = 0; i < 4; i++)\n\
		{\n\
			if(key[i] == 1.0)\n\
			{\n\
				bool4 test = cc[i]<v5[i] || cc[i]<v6[i];\n\
				if(cc[i] < cd[i] || any(test.xy||test.zw))key[i] = 0.0; \n\
			}else if(key[i] == 0.5)\n\
			{\n\
				bool4 test = cc[i]>v5[i] || cc[i]>v6[i];\n\
				if(cc[i] > cd[i] || any(test.xy||test.zw))key[i] = 0.0; \n\
			}\n\
		}\n\
		float keysum = dot(key, float4(1, 1, 1, 1)) ;\n\
		//assume there is only one keypoint in the four. \n\
		if(keysum==0 || keysum > 1) return;	\n";

	//////////////////////////////////////////////////////////////////////
	if(GlobalUtil::_SubpixelLocalization)

	out <<"\
		float3 offset = float3(0, 0, 0); \n\
		/*The unrolled follwing loop is faster than a dynamic indexing version.*/\n\
		for(int idx = 1; idx < 4; idx++)\n\
		{\n\
			if(key[idx] > 0) \n\
			{\n\
				cu[0] = cu[idx];	cd[0] = cd[idx];	cc[0] = cc[idx];	\n\
				v4[0] = v4[idx];	v5[0] = v5[idx];						\n\
				fxy[0] = fxy[idx];	fxx[0] = fxx[idx];	fyy[0] = fyy[idx];	\n\
				fx[0] = fx[idx];	fy[0] = fy[idx];						\n\
			}\n\
		}\n"
	<<
		"\
		float fs = 0.5*( cu[0] - cd[0] );				\n\
		float fss = cu[0] + cd[0] - cc[0] - cc[0];\n\
		float fxs = 0.25 * (v4[0].y + v5[0].x - v4[0].x - v5[0].y);\n\
		float fys = 0.25 * (v4[0].w + v5[0].z - v4[0].z - v5[0].w);\n\
		float4 A0, A1, A2 ;			\n\
		A0 = float4(fxx[0], fxy[0], fxs, -fx[0]);	\n\
		A1 = float4(fxy[0], fyy[0], fys, -fy[0]);	\n\
		A2 = float4(fxs, fys, fss, -fs);	\n\
		float3 x3 = abs(float3(fxx[0], fxy[0], fxs));		\n\
		float maxa = max(max(x3.x, x3.y), x3.z);	\n\
		if(maxa >= 1e-10 ) \n\
		{												\n\
			if(x3.y ==maxa )							\n\
			{											\n\
				float4 TEMP = A1; A1 = A0; A0 = TEMP;	\n\
			}else if( x3.z == maxa )					\n\
			{											\n\
				float4 TEMP = A2; A2 = A0; A0 = TEMP;	\n\
			}											\n\
			A0 /= A0.x;									\n\
			A1 -= A1.x * A0;							\n\
			A2 -= A2.x * A0;							\n\
			float2 x2 = abs(float2(A1.y, A2.y));		\n\
			if( x2.y > x2.x )							\n\
			{											\n\
				float3 TEMP = A2.yzw;					\n\
				A2.yzw = A1.yzw;						\n\
				A1.yzw = TEMP;							\n\
				x2.x = x2.y;							\n\
			}											\n\
			if(x2.x >= 1e-10) {								\n\
				A1.yzw /= A1.y;								\n\
				A2.yzw -= A2.y * A1.yzw;					\n\
				if(abs(A2.z) >= 1e-10) {\n\
					offset.z = A2.w /A2.z;				    \n\
					offset.y = A1.w - offset.z*A1.z;			    \n\
					offset.x = A0.w - offset.z*A0.z - offset.y*A0.y;	\n\
					bool test = (abs(cc[0] + 0.5*dot(float3(fx[0], fy[0], fs), offset ))>"<<threshold1<<") ;\n\
					if(!test || any( abs(offset) >= 1.0)) return;\n\
				}\n\
			}\n\
		}\n"
	<<"\n\
		float keyv = pack_4ubyte(key);\n\
		FragData0 = float4(keyv,  offset);\n\
	}\n"	<<'\0';

	else out << "\n\
		float keyv = pack_4ubyte(key);\n\
		FragData0 =   float4(keyv, 0, 0, 0);\n\
	}\n"	<<'\0';

	//	float kid = dot(key>0? float4(1,1,1,1) : float4(0,0,0,0), float4(1,1,1,1);
	//	float keyv = pack_4ubyte(key);

	s_keypoint = program = new ProgramCG(buffer);
	//parameter
	_param_dog_texu = cgGetNamedParameter(*program, "texU");
	_param_dog_texd = cgGetNamedParameter(*program, "texD");
}

void ShaderBagPKCG::LoadOrientationShader()
{
	char buffer[10240];
	ostrstream out(buffer,10240);

	out<<"\n\
	#define GAUSSIAN_WF "<<GlobalUtil::_OrientationGaussianFactor<<" \n\
	#define SAMPLE_WF ("<<GlobalUtil::_OrientationWindowFactor<< " )\n\
	void main(uniform samplerRECT tex,	uniform samplerRECT gtex,		\n\
			uniform samplerRECT otex, 	uniform float4 size, in float2 TexCoord0 : TEXCOORD0,	\n\
			out float4 FeatureData : COLOR0	";

	//multi orientation output
	//use one additional texture to store up to four orientations
	//when we use one 32bit float to store two orientations, no extra texture is required

	if(GlobalUtil::_MaxOrientation >1  && GlobalUtil::_OrientationPack2 == 0)
		out<<", out float4 OrientationData : COLOR1";


	//use 9 float4 to store histogram of 36 directions
	out<<")		\n\
	{													\n\
		float4 bins[10];								\n\
		int i, j , k ;									\n\
		for (i=0; i<9; i++) bins[i] = float4(0,0,0,0);	\n\
		float4 sift = texRECT(tex, TexCoord0);		\n\
		float2 pos = sift.xy; \n\
		bool full_process_mode = (size.z > 0);		\n\
		float sigma = full_process_mode? (size.z * pow(size.w, sift.w) * sift.z) : (sift.w); \n\
		float gsigma = sigma * GAUSSIAN_WF;				\n\
		float2 win = abs(sigma.xx) * (SAMPLE_WF * GAUSSIAN_WF);	\n\
		float2 dim = size.xy;							\n\
		float dist_threshold = win.x*win.x+0.5;			\n\
		float factor = -0.5/(gsigma*gsigma);			\n\
		float4 sz;	float2 spos;						\n\
		//if(any(pos.xy <= 1)) discard;					\n\
		sz.xy = max( pos - win, float2(2,2));			\n\
		sz.zw = min( pos + win, dim-3);				\n\
		sz = floor(sz*0.5) + 0.5; ";
	//loop to get the histogram

	out<<"\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				float2 offset = 2* spos - pos - 0.5;					\n\
				float4 off = float4(offset, offset + 1);				\n\
				float4 distsq = off.xzxz * off.xzxz + off.yyww * off.yyww;		\n\
				bool4 inside = distsq < dist_threshold;					\n\
				if(any(inside.xy||inside.zw))										\n\
				{														\n\
					float4 gg = texRECT(gtex, spos);						\n\
					float4 oo = texRECT(otex, spos);						\n\
					float4 weight = gg * exp(distsq * factor);				\n\
					float4 idxv  = floor(degrees(oo)*0.1); 				\n\
					idxv = idxv<0? idxv + 36: idxv;	;						\n\
					float4 vidx = fmod(idxv, 4);							\n";


	//
	if(GlobalUtil::_UseDynamicIndexing && strcmp(cgGetProfileString(ProgramCG::_FProfile), "gp4fp")==0)
//	if(ProgramCG::_FProfile == CG_PROFILE_GPU_FP) this enumerant is not defined in cg1.5
	{
		//gp4fp supports dynamic indexing, but it might be slow on some GPUs
		out<<"\n\
					for(i = 0 ; i < 4; i++)\n\
					{\n\
						if(inside[i])\n\
						{\n\
							float idx = idxv[i];								\n\
							float4 inc = weight[i] * float4(vidx[i] == float4(0,1,2,3));	\n\
							int iidx = int(floor(idx*0.25));	\n\
							bins[iidx]+=inc;					\n\
						}										\n\
					}											\n\
				}												\n\
			}													\n\
		}";

	}else
	{
		//nvfp40 still does not support dynamic array indexing
		//unrolled binary search
		//it seems to be faster than the dyanmic indexing version on some GPUs
		out<<"\n\
					for(i = 0 ; i < 4; i++)\n\
					{\n\
						if(inside[i])\n\
						{\n\
							float idx = idxv[i];											\n\
							float4 inc = weight[i] * float4(vidx[i] == float4(0,1,2,3));	\n\
							if(idx < 16)							\n\
							{										\n\
								if(idx < 8)							\n\
								{									\n\
									if(idx < 4)	{	bins[0]+=inc;}	\n\
									else		{	bins[1]+=inc;}	\n\
								}else								\n\
								{									\n\
									if(idx < 12){	bins[2]+=inc;}	\n\
									else		{	bins[3]+=inc;}	\n\
								}									\n\
							}else if(idx < 32)						\n\
							{										\n\
								if(idx < 24)						\n\
								{									\n\
									if(idx <20)	{	bins[4]+=inc;}	\n\
									else		{	bins[5]+=inc;}	\n\
								}else								\n\
								{									\n\
									if(idx < 28){	bins[6]+=inc;}	\n\
									else		{	bins[7]+=inc;}	\n\
								}									\n\
							}else 						\n\
							{										\n\
								bins[8]+=inc;						\n\
							}										\n\
						}											\n\
					}												\n\
				}										\n\
			}											\n\
		}";

	}

	//reuse the code from the unpacked version..
	ShaderBagCG::WriteOrientationCodeToStream(out);

	ProgramCG * program;
	s_orientation = program = new ProgramCG(buffer);
	_param_orientation_gtex = cgGetNamedParameter(*program, "gtex");
	_param_orientation_otex = cgGetNamedParameter(*program, "otex");
	_param_orientation_size = cgGetNamedParameter(*program, "size");


}

void ShaderBagPKCG::LoadDescriptorShader()
{
	GlobalUtil::_DescriptorPPT = 16;
	GlobalUtil::_DescriptorVPC = 1;
	LoadDescriptorShaderF2();
}

void ShaderBagPKCG::LoadDescriptorShaderF2()
{
	//one shader outpout 128/8 = 16 , each fragout encodes 4
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	void main(uniform samplerRECT tex,		\n\
	uniform	samplerRECT gtex,				\n\
	uniform samplerRECT otex,				\n\
	uniform float4		dsize,				\n\
	uniform float2		size,				\n\
	in		float2	TexCoord0 : TEXCOORD0,	\n\
	out		float4  FragData0:COLOR0,		\n\
	out		float4	FragData1:COLOR1)		\n\
	{\n\
		float2 dim	= size.xy;	//image size			\n\
		float index = dsize.x*floor(TexCoord0.y * 0.5) + TexCoord0.x;\n\
		float idx = fmod(index, 8.0) + 8.0 * floor(fmod(TexCoord0.y, 2.0));		\n\
		index = floor(index*0.125)+ 0.49;  \n\
		float2 coord = floor( float2( fmod(index, dsize.z), index*dsize.w)) + 0.5 ;\n\
		float2 pos = texRECT(tex, coord).xy;		\n\
		if(any(pos.xy <= 1) || any(pos.xy >=dim-1)) discard;	\n\
		float anglef = texRECT(tex, coord).z;\n\
		float sigma = texRECT(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";
	//rotation
	out<<"\
		float4 cscs, rots;								\n\
		sincos(anglef, cscs.y, cscs.x);					\n\
		cscs.zw = - cscs.xy;							\n\
		rots = cscs /spt;								\n\
		cscs *= spt; \n";

	//here cscs is actually (cos, sin, -cos, -sin) * (factor: 3)*sigma
	//and rots is  (cos, sin, -cos, -sin ) /(factor*sigma)
	//decide which part of the grid  by  coord
	//compute the upper-left corner of the sample region
	//devide the 4x4 sift grid into 16 1x1,..each 1x1 is obtained from a shader thread
	//to use linear interoplation, enlarge 1x1 to 2x2, by adding 0.5 to each side
	out<<"\
		float4 offset, temp; float2 pt;				\n\
		/*the fraction part of idx is .5*/			\n\
		offset.x = fmod(idx, 4) - 3.0;				\n\
		offset.y = floor(idx*0.25) - 2.5;			\n\
		offset.zw = offset.xy + float2(2.0, 2.0);	\n\
		float4 offsetpt = offset + 0.5;				\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;				\n";
	
	//get a horizontal bounding box of the rotated rectangle
	out<<"\
		float2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		float2 pmin = min(min(p1, p2), min(p3, p4));\n\
		float2 pmax = max(max(p1, p2), max(p3, p4));\n\
		float4 sz;	float2 spos;					\n\
		sz.xy = max( pos + pmin , float2(2,2));\n\
		sz.zw = min( pos + pmax , dim - 3);		\n\
		sz = floor(sz * 0.5)+0.5;"; //move sample point to pixel center
	//get voting for two box

	out<<"\n\
		float4 DA, DB;						\n\
		DA = DB  = float4(0, 0, 0, 0);		\n\
		float3 nox, noy; float2 tpt;				\n\
		tpt = float2(1, 0);	temp = rots.xywx * tpt.xyxy;				\n\
		nox.x = temp.x + temp.y; noy.x 	=temp.z + temp.w;				\n\
		tpt = float2(0, 1);	temp = rots.xywx * tpt.xyxy;				\n\
		nox.y = temp.x + temp.y; noy.y 	=temp.z + temp.w;				\n\
		tpt = float2(1, 1);	temp = rots.xywx * tpt.xyxy;				\n\
		nox.z = temp.x + temp.y; noy.z 	=temp.z + temp.w;				\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				tpt = spos + spos - pt - 0.5;							\n\
				temp = rots.xywx * tpt.xyxy;							\n\
				float4 nx, ny;											\n\
				nx.x = temp.x + temp.y; ny.x = temp.z + temp.w;			\n\
				nx.yzw = nx.x + nox;	ny.yzw = ny.x + noy;			\n\
				bool4 inside = (nx > -0.5) && (ny > -0.5) && (nx < 1.5) && (ny < 1.5);	\n\
				if(any(inside.xy || inside.zw))\n\
				{\n\
					float4 gg = texRECT(gtex, spos);\n\
					float4 oo = texRECT(otex, spos);\n\
					float4 theta = fmod(8+(anglef - oo)*"<<rpi<<", 8);			\n\
					float4 theta1 = floor(theta);								\n\
					float4 weight1 = theta1 + 1.0 - theta;						\n\
					float4 weight2 = theta - theta1;							\n\
					float4 diffx = nx + offsetpt.x, diffy = ny + offsetpt.y;	\n\
					float4 ww = exp(-0.125 * (diffx * diffx + diffy * diffy ));	\n\
					float4 weight = (1 - abs(nx - 0.5)) * (1 - abs(ny - 0.5)) * gg * ww; \n\
					for(int i = 0;i < 4; i++)\n\
					{\n\
						if(inside[i])\n\
						{\n\
							float4 wa = float4(theta1[i] == float4(0, 1, 2, 3))*weight1[i] \n\
									  + float4(theta1[i] == float4(7, 0, 1, 2))*weight2[i]; \n\
							float4 wb = float4(theta1[i] == float4(4, 5, 6, 7))*weight1[i] \n\
							          + float4(theta1[i] == float4(3, 4, 5, 6))*weight2[i]; \n\
							DA += wa * weight[i] ; DB += wb * weight[i];\n\
						}\n\
					}\n\
				}\n\
			}\n\
		}\n";
	out<<"\
		 FragData0 = DA; FragData1 = DB;\n\
	}\n"<<'\0';
	ProgramCG * program; 
	s_descriptor_fp = program =  new ProgramCG(buffer);
	_param_descriptor_gtex = cgGetNamedParameter(*program, "gtex");
	_param_descriptor_otex = cgGetNamedParameter(*program, "otex");
	_param_descriptor_size = cgGetNamedParameter(*program, "size");
	_param_descriptor_dsize = cgGetNamedParameter(*program, "dsize");

}

void ShaderBagPKCG::SetMarginCopyParam(int xmax, int ymax)
{
	float truncate[4];
	truncate[0] = (xmax - 0.5f) * 0.5f; //((xmax + 1)  >> 1) - 0.5f;
	truncate[1] = (ymax - 0.5f) * 0.5f; //((ymax + 1)  >> 1) - 0.5f;
	truncate[2] = (xmax %2 == 1)? 0.0f: 1.0f;
	truncate[3] = truncate[2] +  (((ymax % 2) == 1)? 0.0f : 2.0f);
	cgGLSetParameter4fv(_param_margin_copy_truncate, truncate);
}

void ShaderBagPKCG::SetGradPassParam(int texP)
{
	cgGLSetTextureParameter(_param_grad_pass_texp, texP);
	cgGLEnableTextureParameter(_param_grad_pass_texp);
}

void ShaderBagPKCG::SetGenListEndParam(int ktex)
{
	cgGLSetTextureParameter(_param_genlist_end_ktex, ktex);
	cgGLEnableTextureParameter(_param_genlist_end_ktex);
}

void ShaderBagPKCG::SetDogTexParam(int texU, int texD)
{
	cgGLSetTextureParameter(_param_dog_texu, texU);
	cgGLEnableTextureParameter(_param_dog_texu);
	cgGLSetTextureParameter(_param_dog_texd, texD);
	cgGLEnableTextureParameter(_param_dog_texd);
}

void ShaderBagPKCG::SetGenListInitParam(int w, int h)
{
	float bbox[4] = {(w -1.0f) * 0.5f +0.25f, (w-1.0f) * 0.5f - 0.25f,  (h - 1.0f) * 0.5f + 0.25f, (h-1.0f) * 0.5f - 0.25f};
	cgGLSetParameter4fv(_param_genlist_init_bbox, bbox);
}


void ShaderBagPKCG::SetGenListStartParam(float width, int tex0)
{
	cgGLSetParameter1f(_param_ftex_width, width);

	if(_param_genlist_start_tex0)
	{
		cgGLSetTextureParameter(_param_genlist_start_tex0, tex0);
		cgGLEnableTextureParameter(_param_genlist_start_tex0);
	}
}



void ShaderBagPKCG::SetGenListStepParam(int tex, int tex0)
{
	cgGLSetTextureParameter(_param_genlist_step_tex, tex);
	cgGLEnableTextureParameter(_param_genlist_step_tex);
	cgGLSetTextureParameter(_param_genlist_step_tex0, tex0);
	cgGLEnableTextureParameter(_param_genlist_step_tex0);
}

void ShaderBagPKCG::SetGenVBOParam(float width, float fwidth, float size)
{
	float sizes[4] = {size*3.0f, fwidth, width, 1.0f/width};
	cgGLSetParameter4fv(_param_genvbo_size, sizes);
}

void ShaderBagPKCG::SetSimpleOrientationInput(int oTex, float sigma, float sigma_step)
{
	cgGLSetTextureParameter(_param_orientation_gtex, oTex);
	cgGLEnableTextureParameter(_param_orientation_gtex);
	cgGLSetParameter2f(_param_orientation_size, sigma, sigma_step);
}


void ShaderBagPKCG::SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int otex, float step)
{
	///
	cgGLSetTextureParameter(_param_orientation_gtex, gtex);	
	cgGLEnableTextureParameter(_param_orientation_gtex);
	cgGLSetTextureParameter(_param_orientation_otex, otex);	
	cgGLEnableTextureParameter(_param_orientation_otex);

	float size[4];
	size[0] = (float)width;
	size[1] = (float)height;
	size[2] = sigma;
	size[3] = step;
	cgGLSetParameter4fv(_param_orientation_size, size);

}

void ShaderBagPKCG::SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth,  float width, float height, float sigma)
{
	///
	cgGLSetTextureParameter(_param_descriptor_gtex, gtex);	
	cgGLEnableTextureParameter(_param_descriptor_gtex);
	cgGLSetTextureParameter(_param_descriptor_otex, otex);	
	cgGLEnableTextureParameter(_param_descriptor_otex);

	if(GlobalUtil::_DescriptorPPT ==32)
	{
		dwidth = dwidth /32.0f;
	}
	float dsize[4] ={dwidth, 1.0f/dwidth, fwidth, 1.0f/fwidth};
	cgGLSetParameter4fv(_param_descriptor_dsize, dsize);
	float size[2];
	size[0] = width;
	size[1] = height;
	cgGLSetParameter2fv(_param_descriptor_size, size);

}

