
#include "Distribution.h"

using namespace Tracking;

// *** private ***
void Distribution::updateQueries(){
	
	if(queryMatches.empty()){
		queryMatches.assign(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryMatches[0]);
	}
		
	if(queryEdges.empty()){
		queryEdges.assign(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryEdges[0]);
	}
	
	if(	m_particlelist.size() != queryMatches.size() ){
	 glDeleteQueriesARB(queryMatches.size(), &queryMatches[0]);
	 queryMatches.resize(m_particlelist.size(), 0);
	 glGenQueriesARB(m_particlelist.size(), &queryMatches[0]);
	}
	 
	if( m_particlelist.size() != queryEdges.size() ){
		glDeleteQueriesARB(queryEdges.size(), &queryEdges[0]);
		queryEdges.resize(m_particlelist.size(), 0);
		glGenQueriesARB(m_particlelist.size(), &queryEdges[0]);
	}
}

void Distribution::calcMean(){
	m_meanParticle = Particle(0.0);
	vec3 maxAxis = vec3(0.0, 0.0, 0.0);
	double maxAngle = 0.0;
	vec3 axis; double angle;
	w_sum = 0.0;
	int id;
	
	int num_particles = m_particlelist.size()*0.8;
	
	// Normalise particles
	for(id=0; id<num_particles; id++)
		w_sum += m_particlelist[id].w;

	for(id=0; id<num_particles && w_sum > 0.0; id++)
		m_particlelist[id].w = m_particlelist[id].w / w_sum;
	
	// Weighted sum over all particles
	for(id=0; id<num_particles; id++){
// 		m_meanParticle.r  += m_particlelist[id].r * m_particlelist[id].w;
		m_meanParticle.t 	+= m_particlelist[id].t * m_particlelist[id].w;
		m_meanParticle.tp += m_particlelist[id].tp * m_particlelist[id].w;
		m_meanParticle.rp += m_particlelist[id].rp * m_particlelist[id].w;
		c_mean 	+= m_particlelist[id].c;
		
		m_particlelist[id].q.getAxisAngle(axis, angle);
		maxAxis += axis * m_particlelist[id].w;
		maxAngle += angle * m_particlelist[id].w;
	}
	m_meanParticle.q.fromAxis(maxAxis, maxAngle);
	
	if(!m_particlelist.empty())
		c_mean = c_mean / num_particles;
	
	m_meanParticle.c = c_mean;
	m_meanParticle.w = w_max;
}

// *** public ***
Distribution::Distribution(){
	v_max = 1.0;
	w_sum = 0.0;
}

Distribution::~Distribution(){
	if(!queryMatches.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryMatches[0]);
	if(!queryEdges.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryEdges[0]);
}

double Distribution::getVariance(){
	int id=0;
	double mean = 0.0;
	double var = 0.0;
	
	// Evaluate mean
	for(id=0; id<m_particlelist.size(); id++)
		mean += m_particlelist[id].c;
	mean = mean / m_particlelist.size();
	
	// Evaluate standard variation
	for(id=0; id<m_particlelist.size(); id++)
		var += pow(m_particlelist[id].c - mean, 2);
	
	return var;
}

void Distribution::normalizeW(){
	int id;
	double dw_sum = 0.0;
	if(w_sum>0.0){
		dw_sum = 1.0/w_sum;
		for(id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = m_particlelist[id].w * dw_sum;
			if(m_particlelist[id].w > w_max)
				w_max = m_particlelist[id].w;
		}
	}else{
		dw_sum = 1.0/m_particlelist.size();
		for(id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = dw_sum;
		}
		w_max = dw_sum;
	}
}

// Measurement
bool sortfunction(Particle p1, Particle p2){ return (p1.w>p2.w); }

void Distribution::drawParticlesEdges(TrackerModel& model, Shader* shadeCompare, bool showparticles){
	model.setTexture(0);
	glEnable(GL_DEPTH_TEST);
	
	for(int i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].activate();
		
		glColorMask(0,0,0,0); glDepthMask(1);
		glClear(GL_DEPTH_BUFFER_BIT);
		model.drawFaces();
		
		glDepthMask(0);
		if(showparticles)
			glColorMask(1,1,1,1);
		
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
		model.drawEdges();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
				
		glColorMask(0,0,0,0);
		shadeCompare->bind();
		shadeCompare->setUniform("analyze", false);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[i]);
		model.drawEdges();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		shadeCompare->unbind();
		
		m_particlelist[i].deactivate();
	}
	// Reset render masks
	glColorMask(1,1,1,1); glDepthMask(1);
}

void Distribution::drawParticlesTextured(TrackerModel& model, Shader* shadeCompare, bool showparticles){
	m_particlelist.size();
	
	// Set perspective mode and smaller viewport
	glColorMask(0,0,0,0); glDepthMask(0);
	glEnable(GL_DEPTH_TEST);
		
	// Draw particles and count pixels
	shadeCompare->bind();
	shadeCompare->setUniform("analyze", false);
	for(int i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].activate();
		
		// Draw all model edge pixels
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
		shadeCompare->setUniform("compare", false);
		if(showparticles)
			glColorMask(1,1,1,1);
		model.drawTexturedFaces();
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[i]);
		shadeCompare->setUniform("compare", true);
		shadeCompare->setUniform("textured", true);
		model.drawTexturedFaces();
		shadeCompare->setUniform("textured", false);
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		m_particlelist[i].deactivate();
	}
	shadeCompare->unbind();
	glColorMask(1,1,1,1); glDepthMask(1);	
}

void Distribution::calcLikelihood(int convergence){
	int v, d, id;
	int v_max_tmp = 0;
	w_sum = 0.0;
	c_max = 0.0;
	w_max = 0.0;
	
	for(id=0; id<m_particlelist.size(); id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[id], GL_QUERY_RESULT_ARB, &d);
		
		// Get maximum edge pixels (view dependent)
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// avoids  (d/v_max) > 1.0
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		if(v != 0){
			// TODO evaluate weights and convergence factor (w.r.t. robustness / accuracy / anti-locking)
			m_particlelist[id].c = 0.5 * double(d)/double(v) + 0.5 * double(d)/double(v_max);
			m_particlelist[id].w = pow(m_particlelist[id].c, convergence*(1.0-m_particlelist[id].c));
		}else{
			m_particlelist[id].c = 0.0;
			m_particlelist[id].w = 0.0;
		}
		
		if(m_particlelist[id].c > c_max)
			c_max = m_particlelist[id].c;
		
		// sum of likelihood over all particles
		w_sum += m_particlelist[id].w;
	}	
	
	// Update v_max (in case of decreasing v_max_tmp)
	v_max = v_max_tmp;
	
	// normalize weights
	normalizeW();
	
	if(c_max<=0.0){
		for(id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].c = 0.01;
		}
		c_max=0.01;
	}

	// sort particles by likelihood and average most likely particles
	std::sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);
	
	// calculate mean of distribution
	calcMean();
}

void Distribution::updateLikelihood(TrackerModel& model, Shader* shadeCompare, bool textured, int convergence, bool showparticles){
	
	// no particles to update
	if(m_particlelist.size() <= 0){
		return;
	}
		
	updateQueries();
	
	if(textured)
		drawParticlesTextured(model, shadeCompare, showparticles);
	else
		drawParticlesEdges(model, shadeCompare, showparticles);
	
	calcLikelihood(convergence);
}



