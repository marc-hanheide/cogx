
#include "Lighting.h"

Lighting::Lighting(){

	lightDir = vec4(0.0,0.2,1.0,0);

	int id;
	// Shader
	if((id = g_Resources->AddShader("difference", "difference.vert", "difference.frag")) == -1)
		exit(1);
	m_shadeDifference = g_Resources->GetShader(id);
}

void Lighting::Activate(){

	float Ambient0[4]={0.3f,0.3f,0.3f,0.0f};
	//float Diffuse0[4]={1.0f,1.0f,1.0f,1.0f};
	//float Specular0[4]={1.0f,1.0f,1.0f,1.0f};
	glLightfv(GL_LIGHT0,GL_AMBIENT,Ambient0);
	//glLightfv(GL_LIGHT0,GL_DIFFUSE, Diffuse0);
	//glLightfv(GL_LIGHT0,GL_SPECULAR, Specular0);
	glLightfv(GL_LIGHT0,GL_POSITION, lightDir);
	glEnable(GL_LIGHT0);
	
	float MatAmbient[4]={1.0,1.0,1.0,1.0};
	float MatDiffuse[4]={1.0,1.0,1.0,1.0};
	glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT,GL_AMBIENT,MatAmbient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,MatDiffuse);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);	
}

void Lighting::Deactivate(){
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glDisable(GL_COLOR_MATERIAL);
}

void Lighting::getLightDirection(Texture* tex_image, Pose* pose, Model* model, Camera* camera){
	int i=0,j=0;
	float err=0.0, max=0.0, min=0.0;
	int w = tex_image->getWidth();
	int h = tex_image->getHeight();
	glDisable(GL_DEPTH_TEST);
	glClearColor(0,0,0,0);
	float* image = (float*)malloc( sizeof(float) * w * h );
	
	// collect necessary resources
	Texture* tex_model = model->getOriginalTexture();
	
	// Evaluate modelview and projection matrix
	camera->Activate();
	pose->activateGL();
	mat3 rot;
	vec3 pos;
	pose->getPose(rot, pos);
	mat4 modelview, projection, modelviewprojection;
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
	modelviewprojection = projection * modelview;	
	
	// Setup shader
	m_shadeDifference->bind();
	m_shadeDifference->setUniform("modelviewprojection", modelviewprojection, GL_FALSE); // send matrix to shader
	m_shadeDifference->setUniform("tex_frame", 1);
	m_shadeDifference->setUniform("tex_model", 0);
	m_shadeDifference->setUniform("fTol", 1.0f);
		
	vector<Model::Face> facelist = model->getFacelist();
	vector<Model::Vertex> vertexlist = model->getVertexlist();
	vector<vec3> vError;
	
	tex_image->bind(1);
	model->restoreTexture();
	for(i=0; i<facelist.size(); i++){
		glClear(GL_COLOR_BUFFER_BIT);
		model->drawFace(i);
		glReadPixels(0, 0, w, h, GL_RED, GL_FLOAT, image);
		err = 0.0; max=0.0;
		for(j=0; j<(w*h); j++){
			if(image[j] > 0.01f){
				err += ((image[j] * 2.0) - 1.0);
				if(max<((image[j] * 2.0) - 1.0))
					max=((image[j] * 2.0) - 1.0);
				if(min>((image[j] * 2.0) - 1.0))
					min=((image[j] * 2.0) - 1.0);
			}
		}
		
		// Test if face is visibile
		if(err > 0.01f || err < -0.01f){
			vec3 normal = vertexlist[facelist[i].v[0]].normal;
			//printf("normal: %f %f %f err: %f max: %f min: %f\n", normal.x, normal.y, normal.z, err, max, min);
			normal = rot * normal;
			vError.push_back(normal * err);
			
		}
			
	}
	
	for(i=0; i<vError.size(); i++){
		lightDir = lightDir + vError[i];
	}
	lightDir.normalize();
	printf("Light Direction: %f %f %f\n", lightDir.x, lightDir.y, lightDir.z);
	
	model->drawFaces();
	
	m_shadeDifference->unbind();
	pose->deactivateGL();
	
	
	free(image);
	//printf("Lighting Direction calculated\n");
}


