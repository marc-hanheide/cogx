

#ifndef LIGHTING_H
#define LIGHTING_H

#include <GL/gl.h>
#include "Resources.h"
#include "mathlib.h"

class Lighting
{
private:
	vec4 lightDir;
	Shader* m_shadeDifference;

public:
	Lighting();
	void Activate();
	void Deactivate();
	
	void getLightDirection(Texture* tex_image, Particle* pose, Model* model, Camera* camera);

};

#endif