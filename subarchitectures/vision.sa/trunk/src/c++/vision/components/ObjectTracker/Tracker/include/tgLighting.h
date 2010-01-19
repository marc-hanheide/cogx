//HEADER:
//			Title:			class tgLighting
//			File:				tgLighting.h
//
//			Function:		settings for OpenGL lighting
//
//			Author:			Thomas Mörwald
//			Date:				20.11.2009
// ----------------------------------------------------------------------------


#ifndef TG_LIGHTING
#define TG_LIGHTING

#include <GL/gl.h>
#include "mathlib.h"

struct tgMaterial{
	vec4 ambient;
	vec4 diffuse;
	vec4 specular;
	float shininess;
};

struct tgLight{
	vec4 ambient;
	vec4 diffuse;
	vec4 specular;
	vec4 position;
};

class tgLighting
{
private:
	vec3 lightPos;
	vec3 lightDir;

public:
	tgLighting();
	void ApplyLight(tgLight light, int index=0);
	void ApplyMaterial(tgMaterial mat);
	void Activate();
	void Deactivate();
	
	void SetLightPos(vec3 v){ lightPos = v; }
	void SetLightDir(vec3 v){ lightDir = v; }
	
	void SetLightPos(float x, float y, float z){ lightPos = vec3(x,y,z); }
	void SetLightDir(float x, float y, float z){ lightDir = vec3(x,y,z); }
};

#endif