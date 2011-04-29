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

#include "headers.h"
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

/** @brief Handling OpenGL lighting */
class tgLighting
{
private:

public:
	tgLighting();
	void ApplyLight(tgLight light, int index=0);
	void ApplyMaterial(tgMaterial mat);
	void Activate();
	void Deactivate();
};

#endif