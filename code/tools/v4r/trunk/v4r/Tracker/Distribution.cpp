
#include "Distribution.h"
#include <v4r/TomGine/tgError.h>

using namespace Tracking;
using namespace TomGine;

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

Particle Distribution::calcMean(const ParticleList &pl)
{
	if(pl.empty()){
		printf("[Distribution::calcWeightedMean] Warning ParticleList empty.\n");
		return Particle();
	}
	Particle mean;
	vec3 maxAxis = vec3(0.0f, 0.0f, 0.0f);
	float maxAngle = 0.0f;
	vec3 axis;
	float angle;

	int num_particles = pl.size();
	float dp = 1.0/num_particles;

	// Weighted sum over all particles
	for(int id=0; id<num_particles; id++){
		mean.t += pl[id].t * dp;
		mean.c 	+= pl[id].c;
		mean.cc += pl[id].cc;
		mean.ec += pl[id].ec;

		pl[id].q.getAxisAngle(axis, angle);
		maxAxis += axis * dp;
		maxAngle += angle * dp;
	}

	mean.c *= dp;
	mean.cc *= dp;
	mean.ec *= dp;
	mean.w *= dp;

	return mean;
}

Particle Distribution::calcWeightedMean(ParticleList &pl)
{
	if(pl.empty()){
		printf("[Distribution::calcWeightedMean] Warning ParticleList empty.\n");
		return Particle();
	}
	Particle mean;
	vec3 maxAxis = vec3(0.0f, 0.0f, 0.0f);
	float maxAngle = 0.0f;
	vec3 axis;
	float angle;
	
	normalizeWeights(pl);

	int num_particles = pl.size();
	
	// Weighted sum over all particles
	for(int id=0; id<num_particles; id++){
		mean.t += pl[id].t * pl[id].w;
		mean.cc += pl[id].cc;
		mean.ec += pl[id].ec;
		mean.c 	+= pl[id].c;
		mean.w += pl[id].w;

		pl[id].q.getAxisAngle(axis, angle);
		maxAxis += axis * pl[id].w;
		maxAngle += angle * pl[id].w;
	}
	mean.q.fromAxis(maxAxis, maxAngle);

	float dp = 1.0 / num_particles;
	mean.c *= dp;
	mean.cc *= dp;
	mean.ec *= dp;
	mean.w *= dp;
	
	return mean;
}

float Distribution::calcMedianC(const ParticleList &pl){
	float median;
	unsigned n = pl.size();

	if(n<2)
		return 1.0f;

	if(n%2==0){
		unsigned i1 = (n>>1);
		unsigned i2 = i1+1;
		median = 0.5f * (pl[i1].c + pl[i2].c);
	}else{
		unsigned i=((n+1)>>1);
		median = pl[i].c;
	}

	return median;
}

// *** public ***
Distribution::~Distribution(){
	if(!queryMatches.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryMatches[0]);
	if(!queryEdges.empty())
		glDeleteQueriesARB(m_particlelist.size(), &queryEdges[0]);
}

double Distribution::getVarianceC(){
	double mean = 0.0;
	double var = 0.0;
	double stddev = 0.0;
	
	// Evaluate mean
	for(unsigned id=0; id<m_particlelist.size(); id++)
		mean += m_particlelist[id].c;
	mean = mean / m_particlelist.size();
	
	// Evaluate variance
	for(unsigned id=0; id<m_particlelist.size(); id++)
		var += fabs(m_particlelist[id].c - mean);
	var = var / m_particlelist.size();
	
	return var;
}

double Distribution::getPositionVariance(){
	double var = 0.0;
	
	// Evaluate mean
	Particle mean = getMean();
	
	// Evaluate variance
	for(unsigned id=0; id<m_particlelist.size(); id++){
		vec3 d = m_particlelist[id].t - mean.t;
		var += sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
	}
	var = var / m_particlelist.size();
	
	return var;
}

double Distribution::getOrientationVariance(){
	double var_ax =0.0;
	double var_ang = 0.0;

	tgPose mean = getMean();
	vec3 ax0, ax;
	float ang0, ang;

	mean.q.getAxisAngle(ax0, ang0);
	ax0.normalize();

	for(unsigned id=0; id<m_particlelist.size(); id++){
		m_particlelist[id].q.getAxisAngle(ax,ang);
		ax.normalize();
		double dev = abs(ax0 * ax);
		var_ax += (1.0-dev);
		var_ang += abs(ang-ang0);
	}
	var_ax = var_ax/m_particlelist.size();
	var_ang = var_ang/m_particlelist.size();

	return 0.5*(var_ax+var_ang);
}

void Distribution::normalizeWeights(ParticleList &pl){
	float sum = 0.0;
	float dsum;

	for(unsigned i=0; i<pl.size(); i++)
		sum += pl[i].w;

	dsum = 1.0/sum;

	for(unsigned i=0; i<pl.size(); i++)
		pl[i].w *= dsum;
}

// Measurement
bool sortfunction(Particle p1, Particle p2){ return (p1.w > p2.w); }

void Distribution::drawParticlesEdges(TrackerModel& model, TomGine::tgShader* shadeCompare, bool showparticles){
	model.setTexture(0);
	glEnable(GL_DEPTH_TEST);
	
	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].Activate();
		
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
		
		m_particlelist[i].Deactivate();
	}
	// Reset render masks
	glColorMask(1,1,1,1); glDepthMask(1);
}

void Distribution::drawParticlesTextured(TrackerModel& model, TomGine::tgShader* shader, bool showparticles){
	m_particlelist.size();
	
	// Set perspective mode and smaller viewport
	glColorMask(0,0,0,0); glDepthMask(0);
	glEnable(GL_DEPTH_TEST);
		
	// Draw particles and count pixels
	shader->bind();
	shader->setUniform("analyze", false);
	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].Activate();
		
		// Draw all model edge pixels
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
		shader->setUniform("compare", false);
		if(showparticles)
			glColorMask(1,1,1,1);
		model.drawTexturedFaces();
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[i]);
		shader->setUniform("compare", true);
		shader->setUniform("textured", true);
		model.drawTexturedFaces();
		shader->setUniform("textured", false);
		model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		m_particlelist[i].Deactivate();
	}
	shader->unbind();
	glColorMask(1,1,1,1); glDepthMask(1);	
}

float Distribution::confidenceFunction(const float &m, const float &e, const float& e_max)
{
	float c = 0.0f;
	
	if( e!=0 && e_max!=0 ){
		// TODO evaluate weights and convergence factor (w.r.t. robustness / accuracy / anti-locking)
		c = (0.5f * m/e + 0.5f * m/e_max);
// 		c = ( 0.5f * float(d)/float(v) + 0.5f * float(d)/float(v_max));
// 		c = (1.0f * float(d)/float(v) + 0.0f * float(d)/float(v_max));
// 		c = (float(d)/float(v));
	}else{
		c = 0.0;
	}
	
	return c;
}

void Distribution::calcEdgeConfidence(int convergence){
	int v, d;
	int v_max = 0;
	
	for(unsigned i=0; i<m_particlelist.size(); i++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[i], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[i], GL_QUERY_RESULT_ARB, &d);
		
		// Get maximum edge pixels (view dependent)
		if(v>v_max)
			v_max = v;
		
		m_particlelist[i].et = v;
		m_particlelist[i].em = d;
	}

	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].ec = confidenceFunction(m_particlelist[i].em, m_particlelist[i].et, v_max);
//		m_particlelist[i].w = pow(m_particlelist[i].c, float(convergence)*(1.0f-m_particlelist[i].c));
	}
}

void Distribution::calcColorConfidence(int convergence){
	int v, d;
	int v_max = 0;

	for(unsigned i=0; i<m_particlelist.size(); i++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[i], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[i], GL_QUERY_RESULT_ARB, &d);

		// Get maximum edge pixels (view dependent)
		if(v>v_max)
			v_max = v;

		m_particlelist[i].ct = v;
		m_particlelist[i].cm = d;
	}

	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].cc = confidenceFunction(m_particlelist[i].cm, m_particlelist[i].ct, v_max);
//		m_particlelist[i].w = pow(m_particlelist[i].c, float(convergence)*(1.0f-m_particlelist[i].c));

//		if(m_particlelist[i].ec > m_maxParticle.ec)
//			m_maxParticle = m_particlelist[i];
	}

//	if(m_maxParticle.c<=0.0){
//		for(unsigned id=0; id<m_particlelist.size(); id++){
//			m_particlelist[id].c = 0.01f;
//		}
//		m_maxParticle.c=0.01f;
//	}
}

void Distribution::updateLikelihood(TrackerModel& model,
		TomGine::tgShader* shEdge, TomGine::tgShader* shColor, Method method,
		bool textured, bool fulltextured, int convergence, bool showparticles)
{
	// no particles to update
	if(m_particlelist.empty()){
		return;
	}
		
	updateQueries();

	// Evaluate edge confidence
	if(method==EDGE || method==EDGECOLOR){
		if(textured){
			drawParticlesTextured(model, shEdge, showparticles);
		}else{
			drawParticlesEdges(model, shEdge, showparticles);
		}
		calcEdgeConfidence(convergence);
	}

	// Evaluate color confidence
	if(method==COLOR || method==EDGECOLOR){
		if(fulltextured){
			drawParticlesTextured(model, shColor, showparticles);
			calcColorConfidence(convergence);
		}else{
			for(unsigned i=0; i<m_particlelist.size(); i++)
				m_particlelist[i].cc = 1.0;
		}
	}

	// Evaluate total confidence and weights
	m_maxParticle.c = 0.0;
	w_sum = 0;
	for(unsigned i=0; i<m_particlelist.size(); i++){

		if(method==EDGE)
			m_particlelist[i].c = m_particlelist[i].ec;
		if(method==COLOR)
			m_particlelist[i].c = m_particlelist[i].cc;
		if(method==EDGECOLOR)
			m_particlelist[i].c = m_particlelist[i].ec * m_particlelist[i].cc;
//			m_particlelist[i].c = 0.6*m_particlelist[i].ec + 0.4*m_particlelist[i].cc;
//			m_particlelist[i].c = m_particlelist[i].ec;
//			m_particlelist[i].c = m_particlelist[i].cc;

		m_particlelist[i].w = pow(m_particlelist[i].c, float(convergence)*(1.0f-m_particlelist[i].c));

		if(m_particlelist[i].c > m_maxParticle.c)
			m_maxParticle = m_particlelist[i];

		w_sum += m_particlelist[i].w;
	}

	// sort particles by likelihood
	std::stable_sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);

	// calculate mean of distribution
	m_meanParticle = calcWeightedMean(m_particlelist);

	// calculate median with respect to confidence c
	c_median = calcMedianC(m_particlelist);
}

//void Distribution::updateLikelihood(TrackerModel& model, TomGine::tgShader* shadeCompare, TomGine::tgCamera* cam_persp, Texture* tex_edge, int res)
//{
//	if(m_particlelist.empty()){
//		return;
//	}
//
//	updateQueries();
//
//	ImageProcessor *ip = g_Resources->GetImageProcessor();
//
//	TomGine::tgPose p = getMean();
//	int minX, maxX, minY, maxY;
//	int fbo_res = ip->avgGetResolution();
//	int segs = fbo_res / res;
//
//	model.getBoundingBox2D( ip->getWidth(), ip->getHeight(), p, cam_persp, minX, maxX, minY, maxY );
//
//	int w2 = ip->getWidth() >> 1;
//	int h2 = ip->getHeight() >> 1;
//
//	int width = 1.2 * (maxX-minX); // TODO hardcoded error tolerance (variance)
//	int height = 1.2 * (maxY-minY); // TODO hardcoded error tolerance (variance)
//
//	int minXsq = minX - ((width-(maxX-minX))>>1);
//	int minYsq = minY - ((height-(maxY-minY))>>1);
//	if(minXsq < 0) minXsq = 0;
//	if(minYsq < 0) minYsq = 0;
//
//	ip->setCamOrtho();
//	glColor3f(1,1,1);
//	glBegin(GL_LINE_LOOP);
//		glTexCoord2f(0,0); glVertex3f(minX-w2,minY-h2,0);
//		glTexCoord2f(1,0); glVertex3f(maxX-w2,minY-h2,0);
//		glTexCoord2f(1,1); glVertex3f(maxX-w2,maxY-h2,0);
//		glTexCoord2f(0,1); glVertex3f(minX-w2,maxY-h2,0);
//	glEnd();
//
//	cam_persp->Activate();
//
//	p.Activate();
//	glColor3f(1,0,0);
//	model.drawEdges();
//	p.Deactivate();
//
//	float fResW = float(res)/width;
//	float fResH = float(res)/height;
//
//	unsigned steps = (unsigned)ceil(float(m_particlelist.size())/(segs*segs));
//// 	printf("%d %d %d\n", m_particlelist.size(), segs, steps);
//
//	float* avgs = (float*)malloc(sizeof(float)*segs*segs);
//	int* pxs = (int*)malloc(sizeof(int)*segs*segs);
//	for(unsigned i=0; i<segs*segs; i++){
//		avgs[i] = -1;
//		pxs[i] = 0;
//	}
//	unsigned i=0;
//	shadeCompare->bind();
//	shadeCompare->setUniform("analyze", false);
//	for(unsigned s=0; s<steps; s++){
//		unsigned j=i;
//
//// 		ip->avgActivate();
//		tex_edge->bind(0);
//		for(unsigned r=0; r<segs; r++){
//			for(unsigned c=0; c<segs; c++){
//				if(i<m_particlelist.size()){
//					glViewport(res*c-minXsq*fResW,res*r-minYsq*fResH, ip->getWidth()*fResW, ip->getHeight()*fResH);
//
//					m_particlelist[i].Activate();
//					glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[i]);
//
//					shadeCompare->setUniform("compare", true);
//					shadeCompare->setUniform("textured", true);
//					model.drawTexturedFaces();
//					shadeCompare->setUniform("textured", false);
//					model.drawUntexturedFaces();
//
//					glEndQueryARB(GL_SAMPLES_PASSED_ARB);
//					m_particlelist[i].Deactivate();
//				}
//				i++;
//			}
//		}
//		ip->avgGet(avgs, 2);
//		ip->avgDeactivate();
//
//		unsigned k=j;
//		float c;
//		float w;
//// 		w_sum = 0.0;
//// 		c_max = 0.0;
//// 		w_max = 0.0;
//// 		c_min = 1.0;
//		for(unsigned i=0; i<segs*segs; i++){
//			if(k<m_particlelist.size()){
//				glGetQueryObjectivARB(queryEdges[k], GL_QUERY_RESULT_ARB, &pxs[i]);
//				TomGine::tgCheckError("Distribustion::updateLikelihood glGetQueryObjectivARB: ");
//
//
//				c = avgs[i]*float(res*res)/pxs[i];
//				w = pow(c,3.0*(1.0f-c));
//
//// 				w_sum += w;
////
//// 				if(w > w_max)
//// 					w_max = w;
////
//// 				if(c > c_max)
//// 					c_max = c;
////
//// 				if(c < c_min)
//// 					c_min = c;
//
//				if(i%segs==0) printf("\n");
//				printf("%f %f %f %d | ", c, m_particlelist[k].c, w, pxs[i]);
//// 				m_particlelist[k].c = c;
//// 				m_particlelist[k].w = w;
//			}
//			k++;
//		}
//		printf("\n");
//// 		printf("\n %f %f %f %f\n", w_sum, w_max, c_max, c_min);
//
//	}
//	shadeCompare->unbind();
//	glColorMask(1,1,1,1); glDepthMask(1);
//
//// 	// normalize weights
//// 	normalizeW();
////
//// 	// sort particles by likelihood and average most likely particles
//// 	std::sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);
////
//// 	// calculate mean of distribution
//// 	calcMean();
//
//	cam_persp->Activate();
//	glViewport(0,0,ip->getWidth(), ip->getHeight());
//}

void Distribution::drawCoordinates(float linelength, float linewidth){

	for(unsigned i=0; i<m_particlelist.size(); i++){
		m_particlelist[i].DrawCoordinates(linelength, linewidth);
	}
}


