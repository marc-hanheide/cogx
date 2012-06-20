
#include "tgLighting.h"

tgLighting::tgLighting(){
}

void tgLighting::ApplyLight(tgLight light, int index){
	glLightfv(GL_LIGHT0+index, GL_AMBIENT, light.ambient);
	glLightfv(GL_LIGHT0+index, GL_DIFFUSE, light.diffuse);
	glLightfv(GL_LIGHT0+index, GL_SPECULAR, light.specular);
	glLightfv(GL_LIGHT0+index, GL_POSITION, light.position);
	glEnable(GL_LIGHT0+index);
}

void tgLighting::ApplyMaterial(tgMaterial mat){
	glMaterialfv(GL_FRONT,GL_AMBIENT,mat.ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,mat.diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,mat.specular);
	glMaterialfv(GL_FRONT,GL_SHININESS,&mat.shininess);
}

void tgLighting::Activate(){
	glEnable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);	
}

void tgLighting::Deactivate(){
	glDisable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);
}

