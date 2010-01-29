////////////////////////////////////////////////////////////////////////////
//	File:		ProgramGLSL.cpp
//	Author:		Changchang Wu
//	Description : GLSL related classes
//		class ProgramGLSL		A simple wrapper of GLSL programs
//		class ShaderBagGLSL		GLSL shaders for SIFT
//		class FilterGLSL		GLSL gaussian filters for SIFT
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
#include <iomanip>
#include <iostream>
#include <strstream>
#include <vector>
#include <algorithm>
using namespace std;
#include "GlobalUtil.h"
#include "ProgramGLSL.h"
#include "GLTexImage.h"
#include "ShaderMan.h"
#include "FrameBufferObject.h"
#include "math.h"


ProgramGLSL::ShaderObject::ShaderObject(int shadertype, const char * source, int filesource)
{


	_type = shadertype; 
	_compiled = 0;


	_shaderID = glCreateShader(shadertype);
	if(_shaderID == 0) return;
	
	if(source)
	{

		GLint				code_length;
		if(filesource ==0)
		{
			const char* code  = source;
			code_length = strlen(code);
			glShaderSource(_shaderID, 1, (const char **) &code, &code_length);
		}else
		{
			char * code;
			if((code_length= ReadShaderFile(source, code)) ==0) return;
			glShaderSource(_shaderID, 1, (const char **) &code, &code_length);
			delete code;
		}

		glCompileShader(_shaderID);


		CheckCompileLog();
	}




}

int ProgramGLSL::ShaderObject::ReadShaderFile(const char *sourcefile,  char*& code )
{
	code = NULL;
	FILE * file;
	int    len=0;

	if(sourcefile == NULL) return 0;

	file = fopen(sourcefile,"rt");
	if(file == NULL) return 0;

	
	fseek(file, 0, SEEK_END);
	len = ftell(file);
	rewind(file);
	if(len >1)
	{
		code = new  char[len+1];
		fread(code, sizeof( char), len, file);
		code[len] = 0;
	}else
	{
		len = 0;
	}

	fclose(file);

	return len;
	
}

void ProgramGLSL::ShaderObject::CheckCompileLog()
{

	GLint status;
	glGetShaderiv(_shaderID, GL_COMPILE_STATUS, &status);
	_compiled = (status ==GL_TRUE);

}

ProgramGLSL::ShaderObject::~ShaderObject()
{
	if(_shaderID)	glDeleteShader(_shaderID);

}

int ProgramGLSL::ShaderObject::IsValidFragmentShader()
{
	return _type == GL_FRAGMENT_SHADER && _shaderID && _compiled;
}

int  ProgramGLSL::ShaderObject::IsValidVertexShader()
{
	return _type == GL_VERTEX_SHADER && _shaderID && _compiled;
}


void ProgramGLSL::ShaderObject::PrintCompileLog(ostream&os)
{
	GLint len = 0;	

	glGetShaderiv(_shaderID, GL_INFO_LOG_LENGTH , &len);
	if(len <=1) return;
	
	char * compileLog = new char[len+1];
	if(compileLog == NULL) return;

	glGetShaderInfoLog(_shaderID, len, &len, compileLog);
	

	os<<"Compile Log\n"<<compileLog<<"\n";

	delete compileLog;
}


ProgramGLSL::ProgramGLSL()
{
	_used = 0;
	_linked = 0;
	_programID = glCreateProgram();
}
ProgramGLSL::~ProgramGLSL()
{
	if(_programID)glDeleteProgram(_programID);
}
void ProgramGLSL::AttachShaderObject(ShaderObject &shader)
{
	if(_programID  && shader.IsValidShaderObject()) 
		glAttachShader(_programID, shader.GetShaderID());
}
void ProgramGLSL::DetachShaderObject(ShaderObject &shader)
{
	if(_programID  && shader.IsValidShaderObject()) 
		glDetachShader(_programID, shader.GetShaderID());
}
int ProgramGLSL::LinkProgram()
{
	_linked = 0;

	if(_programID==0) return 0;

	glLinkProgram(_programID);

	CheckLinkLog();

//	GlobalUtil::StartTimer("100 link test");
//	for(int i = 0; i<100; i++) glLinkProgram(_programID);
//	GlobalUtil::StopTimer();

	return _linked;
}

void ProgramGLSL::CheckLinkLog()
{
	GLint status;
	glGetProgramiv(_programID, GL_LINK_STATUS, &status);

	_linked = (status == GL_TRUE);

}


int ProgramGLSL::ValidateProgram()
{
	if(_programID && _linked)
	{
///		GLint status;
//		glValidateProgram(_programID);
//		glGetProgramiv(_programID, GL_VALIDATE_STATUS, &status);
//		return status == GL_TRUE;
		return 1;
	}
	else
		return 0;
}

void ProgramGLSL::PrintLinkLog(std::ostream &os)
{
	GLint len = 0;	

	glGetProgramiv(_programID, GL_INFO_LOG_LENGTH , &len);
	if(len <=1) return;
	
	char* linkLog = new char[len+1];
	if(linkLog == NULL) return;

	glGetProgramInfoLog(_programID, len, &len, linkLog);
	
	linkLog[len] = 0;

	if(strstr(linkLog, "Link successful")==NULL)
		os<<linkLog + (linkLog[0] == ' '? 1:0)<<"\n";

	delete linkLog;
}

int ProgramGLSL::UseProgram()
{
	if(ValidateProgram())
	{
		/*{
			//I got weried problem on my ATI
			//I have to re link the GLSL program, 
			//otherwise it will get wrong result.
			if(_used)
			{
				glLinkProgram(_programID);

			}
			else
				_used = 1;
			
		}*/
		//The weried problems happens when viewport size changes
		//but the following code does not either..
		glUseProgram(_programID);

		return true;
	}
	else
	{
		return false;
	}
}

ProgramGLSL::ProgramGLSL(const char *frag_source)
{
	_used = 0;
	_linked = 0;
	_programID = glCreateProgram();
	ShaderObject shader(GL_FRAGMENT_SHADER, frag_source);
	AttachShaderObject(shader);
	LinkProgram();

	if(!_linked)
	{
		shader.PrintCompileLog(std::cout);
		PrintLinkLog(std::cout);
	}
	if(!_linked)std::cout<<frag_source;

}

/*
ProgramGLSL::ProgramGLSL(char*frag_source, char * vert_source)
{
	_used = 0;
	_linked = 0;
	_programID = glCreateProgram();
	ShaderObject shader(GL_FRAGMENT_SHADER, frag_source);
	ShaderObject vertex_shader(GL_VERTEX_SHADER, vert_source);
	AttachShaderObject(shader);
	AttachShaderObject(vertex_shader);
	LinkProgram();
	if(!_linked)
	{
		shader.PrintCompileLog(std::cout);
		vertex_shader.PrintCompileLog(std::cout);
		PrintLinkLog(std::cout);
		std::cout<<vert_source;
		std::cout<<frag_source;
	}

}
*/


void ProgramGLSL::SetTexParameter(int texUnit)
{
//	GLint loc = glGetUniformLocation(_programID, "tex");
//	glUniform1i(loc, texUnit);
}

void ProgramGLSL::ReLink()
{
	glLinkProgram(_programID);
}

int ProgramGLSL::IsNative()
{
	GLint errorPos, isNative;
	glGetIntegerv(GL_PROGRAM_ERROR_POSITION_ARB, &errorPos);
	glGetProgramivARB(GL_FRAGMENT_PROGRAM_ARB, GL_PROGRAM_UNDER_NATIVE_LIMITS_ARB, &isNative);
	return ((errorPos == -1) && (isNative == 1));
}


ProgramGPU* FilterGLSL::CreateFilterH(float kernel[], float offset[], int width)
{

	char buffer[10240];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	out<<  "uniform sampler2DRect tex;";
	out<< "\nvoid main(void){ float intensity = 0.0 ;  vec2 pos;\n";


	for(int i = 0; i< width; i++)
	{
		if(offset[i]==0.0)
		{

			out<<"float or = texture2DRect(tex, gl_TexCoord[0].st).r;\n";
			out<<"intensity+= or * "<<kernel[i]<<";\n";
		}else
		{
			out<<"pos = gl_TexCoord[0].st + vec2(float("<<offset[i] <<") , 0);\n";
			out<<"intensity+= "<<kernel[i]<<"*texture2DRect(tex, pos).r;\n";
		}
	}

	//copy original data to red channel
	out<<"gl_FragColor.r = or;\n"; 
	out<<"gl_FragColor.b  = intensity;}\n"<<'\0';

	return new ProgramGLSL( buffer);
}


ProgramGPU* FilterGLSL::CreateFilterV(float kernel[], float offset[], int height)
{

	char buffer[10240];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	out<<  "uniform sampler2DRect tex;";
	out<< "\nvoid main(void){ float intensity = 0.0;vec2 pos; \n";

	for(int i = 0; i< height; i++)
	{

		if(offset[i]==0.0)
		{
			out<<"vec2 orb = texture2DRect(tex, gl_TexCoord[0].st).rb;\n";
			out<<"intensity+= orb.y * "<<kernel[i]<<";\n";

		}else
		{
			out<<"pos = gl_TexCoord[0].st + vec2(0, float("<<offset[i] <<") );\n";
			out<<"intensity+= texture2DRect(tex, pos).b * "<<kernel[i]<<";\n";
		}
		
	}

	out<<"gl_FragColor.b = orb.y;\n";
	out<<"gl_FragColor.g = intensity - orb.x;\n"; // difference of gaussian..
	out<<"gl_FragColor.r = intensity;}\n"<<'\0';
	
//	std::cout<<buffer<<endl;
	return new ProgramGLSL( buffer);
}



ProgramGPU* FilterGLSL::CreateFilterHPK(float kernel[], float offset[], int width)
{
	//both h and v are packed...
	int i, j , ip, di, k,  xw, xwn;

	int halfwidth  = width >>1;
	float * pf = kernel + halfwidth;
	int nhpixel = (halfwidth+1)>>1;	//how many neighbour pixels need to be looked up
	int npixel  = (nhpixel<<1)+1;//
	char buffer[10240];
	float weight[2][2];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	char * texLookUp   = "texture2DRect" ;
	out<<  "uniform sampler2DRect tex;";
	out<< "\nvoid main(void){ gl_FragColor = vec4(0, 0, 0, 0);\n";
	///use multi texture coordinate because nhpixels can be at most 3
	out<<"vec4 pc; \n";
	for( i = 0 ; i < npixel ; i++)
	{
		if(i - nhpixel >=-3 && i -nhpixel <=4)
		{
			di = i - nhpixel;
			ip =  di >0 ? ( 2*di  -1) : ( - 2*di); 
			out<<"pc="<<texLookUp<<"(tex, gl_TexCoord["<<ip<<"].xy);\n";
		}
		else 
		{
			out<<"pc="<<texLookUp<<"(tex, gl_TexCoord[0].xy + vec2(float("<<i-nhpixel<<"),0));\n";
		}
		//for each sub-pixel j  in center, the weight of sub-pixel k 
		xw = (i - nhpixel)*2;
		for( j = 0; j < 2; j++)
		{
			for (k = 0; k < 2 ; k++)
			{
				xwn = xw - j + k;
				if( xwn < -halfwidth || xwn > halfwidth)
				{
					weight[j][k] = 0;
				}else
				{
					weight[j][k] = pf[xwn];
				}
			}
		}
		if(weight[0][0]!=0.0)	out<<"gl_FragColor += "<<weight[0][0]<<"*pc;\n";
		out<<"gl_FragColor += vec4("<<weight[0][1]<<","<<weight[1][0]<<","<<weight[0][1]<<","<<weight[1][0]<<")*pc.grab;\n";
	}
	out<<"}\n"<<'\0';
//	std::cout<<buffer<<endl;
	
	return new ProgramGLSL( buffer);


}


ProgramGPU* FilterGLSL::CreateFilterVPK(float kernel[], float offset[], int height)
{

	//both h and v are packed...
	int i, j , ip, di, k,  yw, ywn;

	int halfh  = height >>1;
	float * pf = kernel + halfh;
	int nhpixel = (halfh+1)>>1;	//how many neighbour pixels need to be looked up
	int npixel  = (nhpixel<<1)+1;//
	char buffer[10240];
	float weight[2][2];
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	char * texLookUp   = "texture2DRect" ;
	out<<  "uniform sampler2DRect tex;";
	out<< "\nvoid main(void){ gl_FragColor = vec4(0, 0, 0, 0);\n";
	///use multi texture coordinate because nhpixels can be at most 3
	out<<"vec4 pc;\n";
	for( i = 0 ; i < npixel ; i++)
	{
		if(i - nhpixel >=-3 && i -nhpixel <=4)
		{
			di = i - nhpixel;
			ip =  di >0 ? ( 2*di  -1) : ( - 2*di); 
			out<<"pc="<<texLookUp<<"(tex, gl_TexCoord["<<ip<<"].xy);\n";
		}
		else 
		{
			out<<"pc="<<texLookUp<<"(tex, gl_TexCoord[0].xy + vec2(0, float("<<i-nhpixel<<")));\n";
		}
		//for each sub-pixel j  in center, the weight of sub-pixel k 
		yw = (i - nhpixel)*2;
		for( j = 0; j < 2; j++)
		{
			for (k = 0; k < 2 ; k++)
			{
				ywn = yw - j + k;
				if( ywn < -halfh || ywn > halfh)
				{
					weight[j][k] = 0;
				}else
				{
					weight[j][k] = pf[ywn];
				}
			}
		}
		if(weight[0][0]!=0.0)	out<<"gl_FragColor += "<<weight[0][0]<<"*pc;\n";
		out<<"gl_FragColor += vec4("<<weight[0][1]<<","<<weight[1][0]<<","<<weight[0][1]<<","<<weight[1][0]<<")*pc.brag;\n";
	}
	out<<"}\n"<<'\0';
//	std::cout<<buffer<<endl;
	
	return new ProgramGLSL( buffer);
}




ProgramGPU* FilterGLSL::CreateTex2DFilterH(float kernel[], float offset[], int width)
{

	char buffer[10240];
	int half_size = width/2;
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	out<< "uniform sampler2D tex;";
	out<< "uniform float offset["<<half_size<<"];\n";
	out<< "void main(void){ float intensity = 0.0 ;  vec2 pos;\n";
	for(int i = 0; i< width; i++)
	{
		if(i == half_size)
		{
			out<<"float or = texture2D(tex, gl_TexCoord[0].st).r;\n";
			out<<"intensity+= or * "<<kernel[i]<<";\n";

		}else
		{
			if( i > half_size)
				out<<"pos = gl_TexCoord[0].st + vec2(offset["<<(i-half_size-1)<<"], 0);\n";
			else
				out<<"pos = gl_TexCoord[0].st - vec2(offset["<<(half_size-i -1)<<"], 0);\n";
			out<<"intensity+= "<<kernel[i]<<"*texture2D(tex, pos).r;\n";
		}
	}
	//copy original data to red channel
	out<<"gl_FragColor.r = or;\n"; 
	out<<"gl_FragColor.b  = intensity;}\n"<<'\0';

	return new ProgramGLSL( buffer);
}

ProgramGPU* FilterGLSL::CreateTex2DFilterV(float kernel[], float offset[],  int height)
{
	char buffer[10240];
	int half_size = height/2;
	ostrstream out(buffer, 10240);
	out<<setprecision(8);

	char * texLookUp   = "texture2D";
	out<< "uniform sampler2D tex;";
	out<< "uniform float offset["<<half_size<<"];\n";
	out<< "void main(void){ float intensity = 0.0 ;  vec2 pos;\n";
	for(int i = 0; i< height; i++)
	{
		if(i == half_size)
		{
			out<<"vec2 orb = "<<texLookUp<<"(tex, gl_TexCoord[0].st).rb;\n";
			out<<"intensity+= orb.y * "<<kernel[i]<<";\n";

		}else
		{
			if( i > half_size)
				out<<"pos = gl_TexCoord[0].st + vec2(0, offset["<<(i-half_size-1)<<"]);\n";
			else
				out<<"pos = gl_TexCoord[0].st - vec2(0, offset["<<(half_size-i -1)<<"]);\n";
			out<<"intensity+= "<<texLookUp<<"(tex, pos).b * "<<kernel[i]<<";\n";
		}	
	}
	out<<"gl_FragColor.b = orb.y;\n";
	out<<"gl_FragColor.g = intensity - orb.x;\n"; // difference of gaussian..
	out<<"gl_FragColor.r = intensity;}\n"<<'\0';
	
	return new ProgramGLSL( buffer);
}





void FilterGLSL::UnloadProgram()
{
	glUseProgram(0);
}

void ProgramGLSL::SetTexParameter(unsigned int texID)
{

}

void ShaderBagGLSL::LoadFixedShaders()
{

/*
	char vertex_shader_code[]="uniform sampler2DRect tex; void main(void){ gl_Position = ftransform(); \
		gl_Color = textue2DRect(tex, gl_TexCoord[0].st); \
		gl_TexCoord[0].st = gl_Vertex.xy;}";
*/

	char * display_r_code=
		"uniform sampler2DRect tex; void main(void){float r = texture2DRect(tex, gl_TexCoord[0].st).r;\
		gl_FragColor = vec4(r, r, r, 1);}";


	s_display_gaussian =  new ProgramGLSL( display_r_code);




	char *rgb2gray_code =
		"uniform sampler2DRect rgbTex; void main(void){\
		 float intensity = dot(vec3(0.299, 0.587, 0.114), texture2DRect(rgbTex,gl_TexCoord[0].st ).rgb);\
		 gl_FragColor = vec4(intensity, intensity, intensity, 1.0);}";//

	s_gray = new ProgramGLSL( rgb2gray_code);

	//
	s_debug = new ProgramGLSL( "void main(void){gl_FragColor.rg =  gl_TexCoord[0].st;}");
/*	s_debug = new ProgramGLSL(
		"uniform sampler2DRect tex; void main(void){\n\
		float c = texture2DRect(tex, gl_TexCoord[0].st).r;\n\
		float it[3]; int idx = int(c);\n\
		it[idx] = 1.0; gl_FragColor.r = float(it[2]); }\n");*/

	char* copy_rg_code=
		"uniform sampler2DRect tex; void main(void){gl_FragColor.rg= texture2DRect(tex, gl_TexCoord[0].st).rg;}";//gl_FragColor.r = texture2DRect(tex, gl_TexCoord[0].st).r;


	s_sampling = new ProgramGLSL(copy_rg_code);


	s_copy_key = new ProgramGLSL(
		"uniform sampler2DRect tex; void main(){\n\
	gl_FragColor.rg= texture2DRect(tex, gl_TexCoord[0].st).rg; gl_FragColor.ba = vec2(0.0,1.0);	}");
	


	s_texcoord = new ProgramGLSL("void main(){gl_FragColor= gl_TexCoord[0];}");
	

	s_display_dog =  new ProgramGLSL(
	"uniform sampler2DRect tex; void main(void){float g = 0.5+(20.0*texture2DRect(tex, gl_TexCoord[0].st).g);\
		gl_FragColor = vec4(g, g, g, 0.0);}" );

	s_display_grad = new ProgramGLSL(
		"uniform sampler2DRect tex; void main(void){\n\
		vec4 cc = texture2DRect(tex, gl_TexCoord[0].st);gl_FragColor = vec4(5.0* cc.bbb, 1.0);}");

	s_display_keys= new ProgramGLSL(
		"uniform sampler2DRect tex; void main(void){\n\
		vec4 cc = texture2DRect(tex, gl_TexCoord[0].st);\n\
		if(cc.r ==0.0) discard; gl_FragColor =  (cc.r==1.0? vec4(1.0, 0.0, 0,1.0):vec4(0.0,1.0,0.0,1.0));}");	

	ProgramGLSL * program;
	s_vertex_list = program = new ProgramGLSL(
	"uniform vec4 sizes; uniform sampler2DRect tex;\n\
	void main(void){\n\
	float fwidth = sizes.y; float twidth = sizes.z; float rwidth = sizes.w; \n\
	float size = sizes.x; \n\
	float index = 0.1*(fwidth*floor(gl_TexCoord[0].y) + gl_TexCoord[0].x);\n\
	float px = mod(index, twidth);\n\
	vec2 tpos= floor(vec2(px, index*rwidth))+0.5;\n\
	vec4 cc = texture2DRect(tex, tpos );\n\
	gl_FragColor.zw = vec2(0.0, 1.0);\n\
	if(cc.x<=0 || cc.y <=0) {gl_FragColor.xy = cc.xy; }\n\
	else {float type = fract(px);\n\
	vec2 dxy; \n\
	dxy.x = type < 0.1 ? 0.0 : ((type <0.5 || type > 0.9)? size : -size);\n\
	dxy.y = type < 0.2 ? 0.0 : ((type < 0.3 || type > 0.7 )? -size :size); \n\
	float s = sin(cc.b); float c = cos(cc.b); \n\
	gl_FragColor.x = cc.x + c*dxy.x-s*dxy.y;\n\
	gl_FragColor.y = cc.y + c*dxy.y+s*dxy.x;}\n}\n");


	_param_genvbo_size = glGetUniformLocation(*program, "sizes");

	//
	s_grad_pass = new ProgramGLSL(
	"uniform sampler2DRect tex; void main ()\n\
	{\n\
		vec4 v1, v2, gg;\n\
		vec4 cc  = texture2DRect(tex, gl_TexCoord[0].xy);\n\
		gg.x = texture2DRect(tex, gl_TexCoord[1].xy).r;\n\
		gg.y = texture2DRect(tex, gl_TexCoord[2].xy).r;\n\
		gg.z = texture2DRect(tex, gl_TexCoord[3].xy).r;\n\
		gg.w = texture2DRect(tex, gl_TexCoord[4].xy).r;\n\
		vec2 dxdy = (gg.yw - gg.xz); \n\
		float grad = 0.5*length(dxdy);\n\
		float theta = grad==0? 0: atan2(dxdy.y, dxdy.x);\n\
		gl_FragData[0] = vec4(cc.rg, grad, theta);\n\
	}\n\0");


	s_margin_copy = program = new ProgramGLSL(
	"uniform sampler2DRect tex; uniform vec2 truncate;\n\
	void main(){ gl_FragColor = texture2DRect(tex, min(gl_TexCoord[0].xy, truncate)); }");

	_param_margin_copy_truncate = glGetUniformLocation(*program, "truncate");


	LoadOrientationShader();

	if(s_orientation == NULL)
	{
		//Load a simplified version if the right version is not supported
		s_orientation = program =  new ProgramGLSL(
		"uniform sampler2DRect fTex; uniform sampler2DRect oTex;\n\
		uniform float size; void main(){\n\
		vec4 cc = texture2DRect(fTex, gl_TexCoord[0].st);\n\
		vec4 oo = texture2DRect(oTex, cc.rg);\n\
		gl_FragColor.rg = cc.rg;\n\
		gl_FragColor.b = oo.a;\n\
		gl_FragColor.a = size;}");  

		_param_orientation_gtex = glGetUniformLocation(*program, "oTex");
		_param_orientation_size = glGetUniformLocation(*program, "size");

		GlobalUtil::_MaxOrientation = 0;
		GlobalUtil::_FullSupported = 0;
		std::cerr<<"Orientation simplified on this hardware"<<endl;
	}


	if(GlobalUtil::_DescriptorPPT) LoadDescriptorShader();
	if(s_descriptor_fp == NULL) 
	{
		GlobalUtil::_DescriptorPPT = GlobalUtil::_FullSupported = 0; 
		std::cerr<<"Descriptor ignored on this hardware"<<endl;
	}

	s_zero_pass = new ProgramGLSL("void main(){gl_FragColor = 0;}");
}

void ShaderBagGLSL::LoadKeypointShader(float threshold, float edge_threshold)
{


	char buffer[10240];
	float threshold1 = threshold;
	float threshold2 = (edge_threshold+1)*(edge_threshold+1)/edge_threshold;
	ostrstream out(buffer, 10240);

	//tex(X)(Y)
	//X: (CLR) (CENTER 0, LEFT -1, RIGHT +1)  
	//Y: (CDU) (CENTER 0, DOWN -1, UP    +1) 

	out<<"\
	uniform sampler2DRect tex, texU, texD; void main ()\n\
	{\n\
		vec4 v1, v2, gg;\n\
		vec2 TexRU = vec2(gl_TexCoord[2].x, gl_TexCoord[4].y); \n\
		vec4 cc  = texture2DRect(tex, gl_TexCoord[0].xy);\n\
		v1.x = texture2DRect(tex, gl_TexCoord[1].xy).g;\n\
		gg.x = texture2DRect(tex, gl_TexCoord[1].xy).r;\n\
		v1.y = texture2DRect(tex, gl_TexCoord[2].xy).g;\n\
		gg.y = texture2DRect(tex, gl_TexCoord[2].xy).r;\n\
		v1.z = texture2DRect(tex, gl_TexCoord[3].xy).g;\n\
		gg.z = texture2DRect(tex, gl_TexCoord[3].xy).r;\n\
		v1.w = texture2DRect(tex, gl_TexCoord[4].xy).g;\n\
		gg.w = texture2DRect(tex, gl_TexCoord[4].xy).r;\n\
		v2.x = texture2DRect(tex, gl_TexCoord[5].xy).g;\n\
		v2.y = texture2DRect(tex, gl_TexCoord[6].xy).g;\n\
		v2.z = texture2DRect(tex, gl_TexCoord[7].xy).g;\n\
		v2.w = texture2DRect(tex, TexRU.xy).g;\n\
		vec2 dxdy = (gg.yw - gg.xz); \n\
		float grad = 0.5*length(dxdy);\n\
		float theta = grad==0? 0: atan2(dxdy.y, dxdy.x);\n\
		gl_FragData[0] = vec4(cc.rg, grad, theta);\n"

	//test against 8 neighbours
	//use variable to identify type of extremum
	//1.0 for local maximum and 0.5 for minimum
	<<"\
		float dog = 0.0; \n\
		gl_FragData[1] = vec4(0, 0, 0, 0); \n\
		dog = cc.g > "<<threshold1* (GlobalUtil::_SubpixelLocalization?0.8:1.0) <<" && all(greaterThan(cc.gggg, max(v1, v2)))?1.0: 0.0;\n\
		dog = cc.g < "<<-threshold1*(GlobalUtil::_SubpixelLocalization?0.8:1.0)<<" && all(lessThan(cc.gggg, min(v1, v2)))?0.5: dog;\n\
		if(dog == 0.0) return;\n"

	//do edge supression first.. 
	//vector v1 is < (-1, 0), (1, 0), (0,-1), (0, 1)>
	//vector v2 is < (-1,-1), (-1,1), (1,-1), (1, 1)>

	<<"\
		float fxx, fyy, fxy; \n\
		vec4 D2 = v1.xyzw - cc.gggg;\n\
		vec2 D4 = v2.xw - v2.yz;\n\
		fxx = D2.x + D2.y;\n\
		fyy = D2.z + D2.w;\n\
		fxy = 0.25*(D4.x + D4.y);\n\
		float fxx_plus_fyy = fxx + fyy;\n\
		float score_up = fxx_plus_fyy*fxx_plus_fyy; \n\
		float score_down = (fxx*fyy - fxy*fxy);\n\
		if( score_down <= 0 || score_up > "<<threshold2<<" * score_down)return;\n"
	//...
	<<" \
		vec2 D5 = 0.5*(v1.yw-v1.xz); \n\
		float fx = D5.x, fy = D5.y ; \n\
		float fs, fss , fxs, fys ; \n\
		vec2 v3; vec4 v4, v5, v6;\n"
	//read 9 pixels of upper level
	<<"\
		v3.x = texture2DRect(texU, gl_TexCoord[0].xy).g;\n\
		v4.x = texture2DRect(texU, gl_TexCoord[1].xy).g;\n\
		v4.y = texture2DRect(texU, gl_TexCoord[2].xy).g;\n\
		v4.z = texture2DRect(texU, gl_TexCoord[3].xy).g;\n\
		v4.w = texture2DRect(texU, gl_TexCoord[4].xy).g;\n\
		v6.x = texture2DRect(texU, gl_TexCoord[5].xy).g;\n\
		v6.y = texture2DRect(texU, gl_TexCoord[6].xy).g;\n\
		v6.z = texture2DRect(texU, gl_TexCoord[7].xy).g;\n\
		v6.w = texture2DRect(texU, TexRU.xy).g;\n"
	//compare with 9 pixels of upper level
	//read and compare with 9 pixels of lower level
	//the maximum case
	<<"\
		if(dog == 1.0)\n\
		{\n\
			if(cc.g < v3.x || any(lessThan(cc.gggg, v4)) ||any(lessThan(cc.gggg, v6)))return; \n\
			v3.y = texture2DRect(texD, gl_TexCoord[0].xy).g;\n\
			v5.x = texture2DRect(texD, gl_TexCoord[1].xy).g;\n\
			v5.y = texture2DRect(texD, gl_TexCoord[2].xy).g;\n\
			v5.z = texture2DRect(texD, gl_TexCoord[3].xy).g;\n\
			v5.w = texture2DRect(texD, gl_TexCoord[4].xy).g;\n\
			v6.x = texture2DRect(texD, gl_TexCoord[5].xy).g;\n\
			v6.y = texture2DRect(texD, gl_TexCoord[6].xy).g;\n\
			v6.z = texture2DRect(texD, gl_TexCoord[7].xy).g;\n\
			v6.w = texture2DRect(texD, TexRU.xy).g;\n\
			if(cc.g < v3.y || any(lessThan(cc.gggg, v5)) ||any(lessThan(cc.gggg, v6)))return; \n\
		}\n"
	//the minimum case
	<<"\
		else{\n\
		if(cc.g > v3.x || any(greaterThan(cc.gggg, v4)) ||any(greaterThan(cc.gggg, v6)))return; \n\
			v3.y = texture2DRect(texD, gl_TexCoord[0].xy).g;\n\
			v5.x = texture2DRect(texD, gl_TexCoord[1].xy).g;\n\
			v5.y = texture2DRect(texD, gl_TexCoord[2].xy).g;\n\
			v5.z = texture2DRect(texD, gl_TexCoord[3].xy).g;\n\
			v5.w = texture2DRect(texD, gl_TexCoord[4].xy).g;\n\
			v6.x = texture2DRect(texD, gl_TexCoord[5].xy).g;\n\
			v6.y = texture2DRect(texD, gl_TexCoord[6].xy).g;\n\
			v6.z = texture2DRect(texD, gl_TexCoord[7].xy).g;\n\
			v6.w = texture2DRect(texD, TexRU.xy).g;\n\
			if(cc.g > v3.y || any(greaterThan(cc.gggg, v5)) ||any(greaterThan(cc.gggg, v6)))return; \n\
		}\n";

	if(GlobalUtil::_SubpixelLocalization)

	// sub-pixel localization FragData1 = vec4(dog, 0, 0, 0); return;
	out <<" \
		fs = 0.5*( v3.x - v3.y );  \n\
		fss = v3.x + v3.y - cc.g - cc.g;\n\
		fxs = 0.25 * ( v4.y + v5.x - v4.x - v5.y);\n\
		fys = 0.25 * ( v4.w + v5.z - v4.z - v5.w);\n"
	
	// 
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
		gl_FragData[1] = vec4(dog, 0, 0, 0);	\n\
		vec4 A0, A1, A2 ;			\n\
		A0 = vec4(fxx, fxy, fxs, -fx);	\n\
		A1 = vec4(fxy, fyy, fys, -fy);	\n\
		A2 = vec4(fxs, fys, fss, -fs);	\n\
		vec3 x3 = abs(vec3(fxx, fxy, fxs));		\n\
		float maxa = max(max(x3.x, x3.y), x3.z);	\n\
		if(maxa < 1e-10 ) return;					\n\
		if(x3.y ==maxa )							\n\
		{											\n\
			vec4 TEMP = A1; A1 = A0; A0 = TEMP;	\n\
		}else if( x3.z == maxa )					\n\
		{											\n\
			vec4 TEMP = A2; A2 = A0; A0 = TEMP;	\n\
		}											\n\
		A0 /= A0.x;									\n\
		A1 -= A1.x * A0;							\n\
		A2 -= A2.x * A0;							\n\
		vec2 x2 = abs(vec2(A1.y, A2.y));		\n\
		if( x2.y > x2.x )							\n\
		{											\n\
			vec3 TEMP = A2.yzw;					\n\
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
	    vec3 dxys;							\n\
		dxys.z = A2.w /A2.z;				    \n\
		dxys.y = A1.w - dxys.z*A1.z;			    \n\
		dxys.x = A0.w - dxys.z*A0.z - dxys.y*A0.y;	\n"

	//one more threshold which I forgot in versions prior to 286
	<<"\
		bool bugfix_test = (abs(cc.g + 0.5*dot(vec3(fx, fy, fs), dxys ))>"<<threshold1<<") ;\n"
    //keep the point when the offset is less than 1
	<<"\
	  gl_FragData[1] = bugfix_test && all( lessThan(abs(dxys), 1.0))? vec4( dog, dxys) : vec4(0, 0, 0, 0); \n\
	}\n"	<<'\0';

	else		out<<"\
		gl_FragData[1] =  vec4( dog, 0, 0, 0) ;	}\n"	<<'\0';

	ProgramGLSL * program; 
	s_keypoint = program = new ProgramGLSL(buffer);
	//parameter
	_param_dog_texu = glGetUniformLocation(*program, "texU");
	_param_dog_texd = glGetUniformLocation(*program, "texD");


}


void ShaderBagGLSL::SetDogTexParam(int texU, int texD)
{
	glUniform1i(_param_dog_texu, 1);
	glUniform1i(_param_dog_texd, 2);
}

void ShaderBagGLSL::SetGenListStepParam(int tex, int tex0)
{
	glUniform1i(_param_genlist_step_tex0, 1);	
}
void ShaderBagGLSL::SetGenVBOParam( float width, float fwidth,  float size)
{
	float sizes[4] = {size*3.0f, fwidth, width, 1.0f/width};
	glUniform4fv(_param_genvbo_size, 1, sizes);

}


/*
void ShaderBagPKSL::LoadFixedShaders()
{
	char * display_pk_code = 
	"uniform sampler2DRect tex; void main(void){\
		 vec4 pc = texture2DRect(tex,gl_TexCoord[0].st );\n\
		 float ff = fract(gl_TexCoord[0].s);\n\
		 gl_FragColor.rgb = ff<0.5? (ff<0.25?pc.rrr:pc.ggg):(ff<0.75?pc.bbb:pc.aaa);\n\
		 gl_FragColor.a = 1.0;}";
	s_display_gaussian = new ProgramGLSL(display_pk_code);


	char *rgb2gray_packing_code =
		"uniform sampler2DRect rgbTex; const vec3 weight = vec3(0.299, 0.587, 0.114); void main(void){\
		 gl_FragColor.r = dot(weight, texture2DRect(rgbTex,gl_TexCoord[0].st ).rgb);\
		 gl_FragColor.g = dot(weight, texture2DRect(rgbTex,gl_TexCoord[1].st ).rgb);\
		 gl_FragColor.b = dot(weight, texture2DRect(rgbTex,gl_TexCoord[2].st ).rgb);\
		 gl_FragColor.a = dot(weight, texture2DRect(rgbTex,gl_TexCoord[3].st ).rgb);}";//

	s_gray = new ProgramGLSL( rgb2gray_packing_code);

	char * downsample_code = "uniform sampler2DRect tex; void main(void){\
	 gl_FragColor = vec4( texture2DRect(tex,gl_TexCoord[0].st ).r,\
		texture2DRect(tex,gl_TexCoord[1].st ).r,\
		texture2DRect(tex,gl_TexCoord[2].st ).r, \
		texture2DRect(tex,gl_TexCoord[2].st ).r);}";
	s_downsampling = new ProgramGLSL(downsample_code);

}
*/

void ShaderBagGLSL::UnloadProgram()
{
	glUseProgram(0);
} 



void ShaderBagGLSL::LoadGenListShader(int ndoglev, int nlev)
{
	ProgramGLSL * program;

	s_genlist_init_tight = new ProgramGLSL(
	"uniform sampler2DRect tex; void main (void){\n\
	vec4 helper = vec4( texture2DRect(tex, gl_TexCoord[0].xy).r,  texture2DRect(tex, gl_TexCoord[1].xy).r,\n\
	texture2DRect(tex, gl_TexCoord[2].xy).r, texture2DRect(tex, gl_TexCoord[3].xy).r);\n\
	gl_FragColor = vec4(greaterThan(helper, vec4(0.0,0.0,0.0,0.0)));\n\
	}");

	
	s_genlist_init_ex = program = new ProgramGLSL(
	"uniform sampler2DRect tex;uniform vec2 bbox;\n\
	void main (void ){\n\
	vec4 helper = vec4( texture2DRect(tex, gl_TexCoord[0].xy).r,  texture2DRect(tex, gl_TexCoord[1].xy).r,\n\
	texture2DRect(tex, gl_TexCoord[2].xy).r, texture2DRect(tex, gl_TexCoord[3].xy).r);\n\
	bvec4 helper2 = bvec4( \n\
	all(lessThan(gl_TexCoord[0].xy , bbox)) && helper.x >0,\n\
	all(lessThan(gl_TexCoord[1].xy , bbox)) && helper.y >0,\n\
	all(lessThan(gl_TexCoord[2].xy , bbox)) && helper.z >0,\n\
	all(lessThan(gl_TexCoord[3].xy , bbox)) && helper.w >0);\n\
	gl_FragColor = vec4(helper2);\n\
	}");
	_param_genlist_init_bbox = glGetUniformLocation( *program, "bbox");


	//reduction ...
	s_genlist_histo = new ProgramGLSL(
	"uniform sampler2DRect tex; void main (void){\n\
	vec4 helper; vec4 helper2; \n\
	helper = texture2DRect(tex, gl_TexCoord[0].xy); helper2.xy = helper.xy + helper.zw; \n\
	helper = texture2DRect(tex, gl_TexCoord[1].xy); helper2.zw = helper.xy + helper.zw; \n\
	gl_FragColor.rg = helper2.xz + helper2.yw;\n\
	helper = texture2DRect(tex, gl_TexCoord[2].xy); helper2.xy = helper.xy + helper.zw; \n\
	helper = texture2DRect(tex, gl_TexCoord[3].xy); helper2.zw = helper.xy + helper.zw; \n\
	gl_FragColor.ba= helper2.xz+helper2.yw;\n\
	}");


	//read of the first part, which generates tex coordinates 
	s_genlist_start= program =  LoadGenListStepShader(1, 1);
	_param_ftex_width= glGetUniformLocation(*program, "width");
	_param_genlist_start_tex0 = glGetUniformLocation(*program, "tex0");
	//stepping
	s_genlist_step = program = LoadGenListStepShader(0, 1);
	_param_genlist_step_tex= glGetUniformLocation(*program, "tex");
	_param_genlist_step_tex0= glGetUniformLocation(*program, "tex0");

}

void ShaderBagGLSL::SetMarginCopyParam(int xmax, int ymax)
{
	float truncate[2] = {xmax - 0.5f , ymax - 0.5f};
	glUniform2fv(_param_margin_copy_truncate, 1, truncate);
}

void ShaderBagGLSL::SetGenListInitParam(int w, int h)
{
	float bbox[2] = {w - 1.0f, h - 1.0f};
	glUniform2fv(_param_genlist_init_bbox, 1, bbox);
}
void ShaderBagGLSL::SetGenListStartParam(float width, int tex0)
{
	glUniform1f(_param_ftex_width, width);
}


ProgramGLSL* ShaderBagGLSL::LoadGenListStepShader(int start, int step)
{
	int i;
	char buffer[10240];
	// char chanels[5] = "rgba";
	ostrstream out(buffer, 10240);

	for(i = 0; i < step; i++) out<<"uniform sampler2DRect tex"<<i<<";\n";
	if(start)
	{
		out<<"uniform float width;\n";
		out<<"void main(void){\n";
		out<<"float  index = floor(gl_TexCoord[0].y) * width + floor(gl_TexCoord[0].x);\n";
		out<<"vec2 pos = vec2(0.5, 0.5);\n";
	}else
	{
		out<<"uniform sampler2DRect tex;\n";
		out<<"void main(void){\n";
		out<<"vec4 tc = texture2DRect( tex, gl_TexCoord[0].xy);\n";
		out<<"vec2 pos = tc.rg; float index = tc.b;\n";
	}
	out<<"vec2 sum; 	vec4 cc;\n";


	if(step>0)
	{
		out<<"vec2 cpos = vec2(-0.5, 0.5);\t vec2 opos;\n";
		for(i = 0; i < step; i++)
		{

			out<<"cc = texture2DRect(tex"<<i<<", pos);\n";
			out<<"sum.x = cc.r + cc.g; sum.y = sum.x + cc.b;  \n";
			out<<"if (index <cc.r){ opos = cpos.xx;}\n";
			out<<"else if(index < sum.x ) {opos = cpos.yx; index -= cc.r;}\n";
			out<<"else if(index < sum.y ) {opos = cpos.xy; index -= sum.x;}\n";
			out<<"else {opos = cpos.yy; index -= sum.y;}\n";
			out<<"pos = (pos + pos + opos);\n";
		}
	}
	out<<"gl_FragColor = vec4(pos, index, 1.0);\n";
	out<<"}\n"<<'\0';
	return new ProgramGLSL(buffer);
}


void ShaderBagGLSL::LoadOrientationShader()
{
	char buffer[10240];
	ostrstream out(buffer,10240);

	out<<"\n\
	#define GAUSSIAN_WF "<<GlobalUtil::_OrientationGaussianFactor<<" \n\
	#define SAMPLE_WF ("<<GlobalUtil::_OrientationWindowFactor<< " )\n\
	uniform sampler2DRect tex;					\n\
	uniform sampler2DRect gradTex;				\n\
	uniform vec4 size;						\n"
	<< (GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign? 
	"	uniform samplerRECT texS;	\n" : " ")
	<<"\
	void main()		\n\
	{													\n\
		vec4 bins[10];								\n\
		int i, j , k ;									\n\
		for (i=0; i<9; i++) bins[i] = vec4(0,0,0,0);	\n\
		vec4 loc = texture2DRect(tex, gl_TexCoord[0]);	\n\
		vec2 pos = loc.xy;		\n\
		bool full_process_mode = (size.z > 0);			\n\
		float sigma = full_process_mode? size.z : loc.w; \n";
	if(GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)
	{
		out<<"\
		if(full_process_mode){\n\
			vec4 offset = texture2DRect(texS, pos);\n\
			pos.xy = pos.xy + offset.yz; \n\
			sigma = sigma * pow(size.w, offset.w);\n\
			#if "<< GlobalUtil::_KeepExtremumSign << "\n\
				if(offset.x < 0.6) sigma = -sigma; \n\
			#endif\n\
		}\n";
	}
	out<<"\
		float gsigma = sigma * GAUSSIAN_WF;				\n\
		vec2 win = abs(sigma.xx) * (SAMPLE_WF * GAUSSIAN_WF);	\n\
		vec2 dim = size.xy;							\n\
		float dist_threshold = win.x*win.x+0.5;			\n\
		float factor = -0.5/(gsigma*gsigma);			\n\
		vec4 sz;	vec2 spos;						\n\
		//if(any(pos.xy <= 1)) discard;					\n\
		sz.xy = max( pos - win, vec2(1,1));			\n\
		sz.zw = min( pos + win, dim-2);				\n\
		sz = floor(sz)+0.5;";
	//loop to get the histogram

	out<<"\n\
	    vec4 debug = 0; \n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				vec2 offset = spos - pos;								\n\
				float sq_dist = dot(offset,offset);						\n\
				if( sq_dist < dist_threshold){							\n\
					vec4 cc = texture2DRect(gradTex, spos);						\n\
					float grad = cc.b;	float theta = cc.a;					\n\
					float idx = floor(degrees(theta)*0.1);				\n\
					if(idx < 0 ) idx += 36;									\n\
					float weight = grad*exp(sq_dist * factor);				\n\
					float vidx = fmod(idx, 4.0) ;							\n\
					vec4 inc = weight*vec4(equal(vidx, vec4(0,1,2,3)));";

	if(GlobalUtil::_UseDynamicIndexing)
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

	ProgramGLSL * program = new ProgramGLSL(buffer);
	if(program->IsNative())
	{
		s_orientation = program ;
		_param_orientation_gtex = glGetUniformLocation(*program, "gradTex");
		_param_orientation_size = glGetUniformLocation(*program, "size");
		_param_orientation_stex = glGetUniformLocation(*program, "texS");
	}else
	{
		delete program;
	}
}


void ShaderBagGLSL::WriteOrientationCodeToStream(std::ostream& out)
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
		//mat3 m1 = mat3(1, 0, 0, 3, 1, 0, 6, 3, 1)/27.0;  \n\
		mat3 m1 = mat3(1, 3, 6, 0, 1, 3,0, 0, 1)/27.0;  \n\
		mat4 m2 = mat4(7, 6, 3, 1, 6, 7, 6, 3, 3, 6, 7, 6, 1, 3, 6, 7)/27.0;\n\
		for (j=0; j<2; j++)								\n\
		{												\n\
			vec4 prev  = bins[8];						\n\
			bins[9]		 = bins[0];						\n\
			for (i=0; i<9; i++)							\n\
			{												\n\
				vec4 newb	=	(bins[i]* m2);		\n\
				newb.xyz	+=	( prev.yzw * m1);		\n\
				prev = bins[i];								\n\
				newb.wzy	+=	( bins[i+1].zyx *m1);	\n\
				bins[i] = newb;							\n\
			}												\n\
		}";
	//find the maximum voting
	out<<"\n\
		float maxh; vec2 maxh2; vec4 maxh4 = bins[0];				\n\
		for (i=1; i<9; i++) maxh4 = max(maxh4, bins[i]);				\n\
		maxh2 = max(maxh4.xy, maxh4.zw); maxh = max(maxh2.x, maxh2.y);";

	char *testpeak_code;
	char *savepeak_code;



	//save two/three/four orientations with the largest votings?

	//
	if(GlobalUtil::_MaxOrientation>1)
	{
		out<<"\n\
		vec4 Orientations = vec4(0, 0, 0, 0);				\n\
		vec4 weights = vec4(0,0,0,0);		";	
		
		testpeak_code = "\n\
			{test = greaterThan(bins[i], hh.xxxx);";

		//save the orientations in weight-decreasing order
		if(GlobalUtil::_MaxOrientation ==2)
		{
		savepeak_code = "\n\
			if(weight <=weights.g){}\n\
			else if(weight >weights.r)\n\
			{weights.rg = vec2(weight, weights.r); Orientations.rg = vec2(th, Orientations.r);}\n\
			else {weights.g = weight; Orientations.g = th;}";
		}else if(GlobalUtil::_MaxOrientation ==3)
		{
		savepeak_code = "\n\
			if(weight <=weights.b){}\n\
			else if(weight >weights.r)\n\
			{weights.rgb = vec3(weight, weights.rg); Orientations.rgb = vec3(th, Orientations.rg);}\n\
			else if(weight >weights.g)\n\
			{weights.gb = vec2(weight, weights.g); Orientations.gb = vec2(th, Orientations.g);}\n\
			else {weights.b = weight; Orientations.b = th;}";
		}else
		{
		savepeak_code = "\n\
			if(weight <=weights.a){}\n\
			else if(weight >weights.r)\n\
			{weights = vec4(weight, weights.rgb); Orientations = vec4(th, Orientations.rgb);}\n\
			else if(weight >weights.g)\n\
			{weights.gba = vec3(weight, weights.gb); Orientations.gba = vec3(th, Orientations.gb);}\n\
			else if(weight >weights.b)\n\
			{weights.ba = vec2(weight, weights.b); Orientations.ba = vec2(th, Orientations.b);}\n\
			else {weights.a = weight; Orientations.a = th;}";
		}

	}else
	{
		out<<"\n\
		float Orientation;				";
		testpeak_code ="\n\
			if(npeaks<=0){								\n\
			test = equal(bins[i], maxh)	;";
		savepeak_code="\n\
					npeaks++;								\n\
					Orientation = th;";

	}
	//find the peaks
	//the following loop will be unrolled anyway in fp40,
	//taking more than 1000 instrucsions..
	//....
	out<<"\n\
		float hh = maxh * 0.8;	bvec4 test;	\n\
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
				else if(test.g && all( greaterThan(bins[i].yy , bins[i].xz)) )	\n\
				{											\n\
				    float	di = -0.5 * (bins[i].z-bins[i].x) / (bins[i].z+bins[i].x-bins[i].y- bins[i].y) ; \n\
					float	th = (k+di+1.5);	float weight = bins[i].y;				"
					<<savepeak_code<<"	\n\
				}"
		<<"\n\
				if(test.b && all( greaterThan( bins[i].zz , bins[i].yw)) )	\n\
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
			k = k + 4;	\n\
			prevb = bins[i].w;\n\
		}";
	//WRITE output
	if(GlobalUtil::_MaxOrientation>1)
	{
	out<<"\n\
		 if(full_process_mode){\n\
			npeaks = dot(vec4(1,1,1,1), vec4(greaterThan(weights, hh)));\n\
			npeaks = min(npeaks,  "<<GlobalUtil::_MaxOrientation<<");\n\
			gl_FragData[1] = radians((Orientations )*10.0);\n\
			gl_FragData[0] = vec4(pos, npeaks, sigma);\n\
		}else{\n\
			gl_FragData[0] = vec4(pos, radians((Orientations.x)*10.0), sigma);\n\
		}\n";
	}else
	{
	out<<"\n\
		 gl_FragData[0] = vec4(pos, radians((Orientation)*10.0), sigma);";
	}
	//end
	out<<"\n\
	}\n"<<'\0';


}

void ShaderBagGLSL::SetSimpleOrientationInput(int oTex, float sigma, float sigma_step)
{
	glUniform1i(_param_orientation_gtex, 1);
	glUniform1f(_param_orientation_size, sigma);
}




void ShaderBagGLSL::SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int stex, float step)
{
	///
	glUniform1i(_param_orientation_gtex, 1);	

	if((GlobalUtil::_SubpixelLocalization || GlobalUtil::_KeepExtremumSign)&& stex)
	{
		//specify texutre for subpixel subscale localization
		glUniform1i(_param_orientation_stex, 2);
	}

	float size[4];
	size[0] = (float)width;
	size[1] = (float)height;
	size[2] = sigma;
	size[3] = step;
	glUniform4fv(_param_orientation_size, 1, size);

}


void ShaderBagGLSL::LoadDescriptorShaderF2()
{
	//one shader outpout 128/8 = 16 , each fragout encodes 4
	const double twopi = 2.0*3.14159265358979323846;
	const double rpi  = 8.0/twopi;
	char buffer[10240];
	ostrstream out(buffer, 10240);

	out<<setprecision(8);

	out<<"\n\
	#define WF "<<GlobalUtil::_DescriptorWindowFactor <<"\n\
	uniform sampler2DRect tex;				\n\
	uniform sampler2DRect gradTex;			\n\
	uniform vec4 dsize;						\n\
	uniform vec2 size;						\n\
	void main()		\n\
	{\n\
		vec2 dim	= size.xy;	//image size			\n\
		float index = dsize.x*floor(gl_TexCoord[0].y * 0.5) + gl_TexCoord[0].x;\n\
		float idx = fmod(index, 8.0) + 8.0 * floor(fmod(gl_TexCoord[0].y, 2.0));		\n\
		index = floor(index*0.125) + 0.49;  \n\
		vec2 coord = floor( vec2( fmod(index, dsize.z), index*dsize.w)) + 0.5 ;\n\
		vec2 pos = texRECT(tex, coord).xy;		\n\
		if(any(lessThanEqual(pos.xy,  1.0)) || any(greaterThanEqual(pos.xy, dim-1.0))) discard;	\n\
		float  anglef = texture2DRect(tex, coord).z;\n\
		float sigma = texture2DRect(tex, coord).w; \n\
		float spt  = abs(sigma * WF);	//default to be 3*sigma	\n";

	//rotation
	out<<"\
		vec4 cscs, rots;								\n\
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
		vec4 offset, temp; vec2 pt;				\n\
		offset.x = fmod(idx, 4.0) - 3.0;				\n\
		offset.y = floor(idx*0.25) - 2.5;			\n\
		offset.zw = offset.xy + vec2(2.0, 2.0);	\n\
		vec4 offsetpt = offset + 0.5;				\n\
		temp = cscs.xwyx*offsetpt.xyxy;				\n\
		pt = pos + temp.xz + temp.yw;						\n";
	
	//get a horizontal bounding box of the rotated rectangle
	out<<"\
		vec2 p1, p2, p3, p4;			\n\
		temp = cscs.xwyx*offset.xyxy;				\n\
		p1 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zyzy;				\n\
		p2 = temp.xz + temp.yw;						\n\
		temp = cscs.xwyx*offset.zwzw;				\n\
		p3 =  temp.xz + temp.yw;					\n\
		temp = cscs.xwyx*offset.xwxw;				\n\
		p4 =  temp.xz + temp.yw;					\n\
		vec2 pmin = min(min(p1, p2), min(p3, p4));\n\
		vec2 pmax = max(max(p1, p2), max(p3, p4));\n\
		vec4 sz;	vec2 spos;					\n\
		sz.xy = max( pos + pmin , vec2(1,1));\n\
		sz.zw = min( pos + pmax , dim -2);		\n\
		sz = floor(sz)+0.5;"; //move sample point to pixel center
	//get voting for two box

	out<<"\n\
		vec4 DA, DB;			\n\
		DA = DB  = vec4(0, 0, 0, 0);		\n\
		for(spos.y = sz.y; spos.y <= sz.w;	spos.y+=1.0)				\n\
		{																\n\
			for(spos.x = sz.x; spos.x <= sz.z;	spos.x+=1.0)			\n\
			{															\n\
				vec2 diff = spos - pt;								\n\
				temp = rots.xywx * diff.xyxy;\n\
				vec2 nxy = (temp.xz + temp.yw); \n\
				if(all(greaterThan(nxy , vec2(-0.5,-0.5)) && lessThan(nxy, vec2(1.5, 1.5))))\n\
				{\n\
					vec4 cc = texture2DRect(gradTex, spos);						\n\
					float mod = cc.b;	float angle = cc.a;					\n\
					float theta = fmod(8+(anglef - angle)*"<<rpi<<", 8.0); \n\
					diff = nxy + offsetpt.xy;								\n\
					float ww = exp(-0.125*dot(diff, diff));\n\
					vec2 weights = 1 - abs(nxy.xy - 0.5);\n\
					float weight = weights.x * weights.y *mod*ww; \n\
					vec4 wa, wb; \n\
					float theta1 = floor(theta); \n\
					float weight1 = theta1 + 1.0 - theta; float weight2 = theta - theta1;\n\
					wa = vec4(equal(theta1,  vec4(0, 1, 2, 3)))*weight1 + vec4(equal(theta1 ,   vec4(7, 0, 1, 2)))*weight2; \n\
					wb = vec4(equal(theta1 ,   vec4(4, 5, 6, 7)))*weight1 + vec4(equal(theta1 ,   vec4(3, 4, 5, 6)))*weight2; \n\
					DA += wa * weight ; DB += wb * weight;\n\
				}\n\
			}\n\
		}\n";

	out<<"\
		 gl_FragData[0] = DA; gl_FragData[1] = DB;\n\
	}\n"<<'\0';

	ProgramGLSL * program =  new ProgramGLSL(buffer); 

	if(program->IsNative())
	{
		s_descriptor_fp = program ;
		_param_descriptor_gtex = glGetUniformLocation(*program, "gradTex");
		_param_descriptor_size = glGetUniformLocation(*program, "size");
		_param_descriptor_dsize = glGetUniformLocation(*program, "dsize");
	}else
	{
		delete program;
	}


}

void ShaderBagGLSL::LoadDescriptorShader()
{
	GlobalUtil::_DescriptorPPT = 16;
	GlobalUtil::_DescriptorVPC = 1;
	LoadDescriptorShaderF2();
}


void ShaderBagGLSL::SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth,  float width, float height, float sigma)
{
	///
	glUniform1i(_param_descriptor_gtex, 1);	

	float dsize[4] ={dwidth, 1.0f/dwidth, fwidth, 1.0f/fwidth};
	glUniform4fv(_param_descriptor_dsize, 1, dsize);
	float size[2];
	size[0] = width;
	size[1] = height;
	glUniform2fv(_param_descriptor_size, 1, size);

}

