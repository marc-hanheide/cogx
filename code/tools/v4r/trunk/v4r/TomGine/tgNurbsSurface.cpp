
#include "tgNurbsSurface.h"
#include <string>

using namespace TomGine;

tgNurbsSurface::tgNurbsSurface(){
	std::string nurbs_vert = std::string(V4R_DIR) + "/v4r/TomGine/shader/nurbssurface.vert";
	std::string cox_head = std::string(V4R_DIR) + "/v4r/TomGine/shader/coxdeboor.c";
	std::string color_frag = std::string(V4R_DIR) + "/v4r/TomGine/shader/color.frag";
	shNurbsSurface = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());
}

tgNurbsSurface::tgNurbsSurface(	const tgNurbsSurfacePatch &data )
{
	// setup NURBS shader
	std::string nurbs_vert = std::string(V4R_DIR) + "/v4r/TomGine/shader/nurbssurface.vert";
	std::string cox_head = std::string(V4R_DIR) + "/v4r/TomGine/shader/coxdeboor.c";
	std::string color_frag = std::string(V4R_DIR) + "/v4r/TomGine/shader/color.frag";
	shNurbsSurface = new tgShader(nurbs_vert.c_str(), color_frag.c_str(), cox_head.c_str());

    Set(data);

}

tgNurbsSurface::~tgNurbsSurface(){
	delete shNurbsSurface;
	gluDeleteNurbsRenderer(m_glunurb);
}

void tgNurbsSurface::Set( const tgNurbsSurfacePatch &data )
{
	nurbsData = data;

	texKnotsU.Load(&nurbsData.knotsU[0], nurbsData.knotsU.size(), GL_R32F, GL_RED, GL_FLOAT);
    texKnotsV.Load(&nurbsData.knotsV[0], nurbsData.knotsV.size(), GL_R32F, GL_RED, GL_FLOAT);
    texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, GL_RGBA32F, GL_RGBA, GL_FLOAT);

    // setup NURBS shader
    shNurbsSurface->bind();
    shNurbsSurface->setUniform("orderU", (int)nurbsData.orderU);
    shNurbsSurface->setUniform("orderV", (int)nurbsData.orderV);
    shNurbsSurface->setUniform("nknotsU", (int)nurbsData.knotsU.size());
    shNurbsSurface->setUniform("nknotsV", (int)nurbsData.knotsV.size());
    shNurbsSurface->setUniform("knotsU", 0);
    shNurbsSurface->setUniform("knotsV", 1);
    shNurbsSurface->setUniform("cps", 2);
    shNurbsSurface->unbind();

    m_glunurb = gluNewNurbsRenderer();
	gluNurbsProperty(m_glunurb, GLU_SAMPLING_TOLERANCE, 25.0);
	gluNurbsProperty(m_glunurb, GLU_DISPLAY_MODE, GLU_FILL);
//	gluNurbsCallback(m_glunurb, GLU_ERROR, (GLvoid(*)()) nurbsError);

    Remesh(nurbsData.resU, nurbsData.resV);
}

void tgNurbsSurface::Remesh(unsigned resU, unsigned resV){
	float x0 = nurbsData.knotsU[0];
	float y0 = nurbsData.knotsV[0];
	float w = nurbsData.knotsU[nurbsData.knotsU.size()-1] - x0;
	float h = nurbsData.knotsV[nurbsData.knotsV.size()-1] - y0;

	m_model.Clear();
	tgShapeCreator::CreatePlaneXY(m_model, x0, y0, 0.0, w, h, resU, resV);
}

void tgNurbsSurface::SetCP(unsigned i, unsigned j, const vec4 &cpv){
	unsigned idx = j*nurbsData.ncpsU + i;
	SetCP(idx, cpv);
}
void tgNurbsSurface::SetCP(unsigned i, const vec4 &cpv){
	if(i>=nurbsData.cps.size() || i<0){
		printf("[tgNurbsSurface::UpdateCV] Warning: index out of bounds.\n");
		return;
	}
	nurbsData.cps[i] = cpv;
	texCP.Load(&nurbsData.cps[0].x, nurbsData.ncpsU, nurbsData.ncpsV, GL_RGBA32F, GL_RGBA, GL_FLOAT);
}

vec4 tgNurbsSurface::GetCP(unsigned i, unsigned j){
	unsigned idx = j*nurbsData.ncpsU + i;
	if(idx>=nurbsData.cps.size() || idx<0){
		printf("[tgNurbsSurface::GetCV] Warning: index out of bounds.\n");
		return vec4(0.0,0.0,0.0,1.0);
	}
	return nurbsData.cps[idx];
}

vec4 tgNurbsSurface::GetCP(unsigned i){
	if(i>=nurbsData.cps.size() || i<0){
		printf("[tgNurbsSurface::GetCV] Warning: index out of bounds.\n");
		return vec4(0.0,0.0,0.0,1.0);
	}
	return nurbsData.cps[i];
}


void tgNurbsSurface::DrawVertices()
{
	// draw B-Spline surface
	shNurbsSurface->bind();

		texKnotsU.Bind(0);
		texKnotsV.Bind(1);
		texCP.Bind(2);

		m_model.DrawVertices();

	shNurbsSurface->unbind();
	glDisable(GL_TEXTURE_1D);
	glDisable(GL_TEXTURE_2D);
}

void tgNurbsSurface::DrawFaces()
{
	// draw B-Spline surface
	shNurbsSurface->bind();

		texKnotsU.Bind(0);
		texKnotsV.Bind(1);
		texCP.Bind(2);

		m_model.DrawFaces();

	shNurbsSurface->unbind();
	glDisable(GL_TEXTURE_1D);
	glDisable(GL_TEXTURE_2D);

//	glEnable(GL_LIGHTING);
//	gluBeginSurface(m_glunurb);
//	gluNurbsSurface(m_glunurb,
//				nurbsData.knotsU.size(), &nurbsData.knotsU[0],
//				nurbsData.knotsV.size(), &nurbsData.knotsV[0],
//				nurbsData.ncpsU * 4, 4,
//				&nurbsData.cps[0].x, nurbsData.orderU+1, nurbsData.orderV+1, GL_MAP2_VERTEX_4);
//
//	gluEndSurface(m_glunurb);
}

void tgNurbsSurface::DrawCPs(){
	glDisable(GL_LIGHTING);

	glBegin(GL_POINTS);
		for(unsigned i=0; i<nurbsData.cps.size(); i++)
			glVertex3f(nurbsData.cps[i].x, nurbsData.cps[i].y, nurbsData.cps[i].z);
	glEnd();

	glEnable(GL_LIGHTING);
}
