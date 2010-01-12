
#include "tgLighting.h"

using namespace TomGine;

void tgLighting::ApplyLight(tgLight light, int index){
	glLightfv(GL_LIGHT0+index, GL_AMBIENT, light.ambient);
	glLightfv(GL_LIGHT0+index, GL_DIFFUSE, light.diffuse);
	glLightfv(GL_LIGHT0+index, GL_SPECULAR, light.specular);
	glLightfv(GL_LIGHT0+index, GL_POSITION, light.position);
	glEnable(GL_LIGHT0+index);
}

void tgLighting::Activate(){
	glEnable(GL_LIGHTING);
}

void tgLighting::Deactivate(){
	glDisable(GL_LIGHTING);
}


