/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 */
#include "CTomGineModel.hpp"


#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CTomGineModel::renderGL(new CTomGineModel_RenderGL());

CTomGineModel::CTomGineModel()
{
   m_pModel = NULL;
}

CTomGineModel::~CTomGineModel()
{
}

bool CTomGineModel::is3D()
{
   return true;
}

CRenderer* CTomGineModel::getRenderer(ERenderContext context)
{
   switch(context) {
      case ContextGL: return renderGL.get();
   }
   return NULL;
}

// OpenGL context is selected by Qt, so pContext is NULL.
void CTomGineModel_RenderGL::draw(CDisplayObject *pObject, void *pContext)
{
   DTRACE("CTomGineModel_RenderGL::draw");
   if (pObject == NULL) return;
   CTomGineModel *pModel = (CTomGineModel*) pObject;
   if (! pModel->m_pModel) return;
   DMESSAGE("Model present.");

   // TODO: Conditional Drawing!
   if (0) {
      glEnable(GL_BLEND);
      glEnable(GL_CULL_FACE);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   }

   //GLfloat faceColor[4] = {0.5, 0.9, 0.5, 1.0};
   //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, faceColor);
	//glColor4f(0.5, 0.9, 0.9, 0.9);

   pModel->m_pModel->DrawFaces(false);
   // pModel->m_pModel->DrawQuadstrips();
   pModel->m_pModel->DrawNormals(0.01);

   if (0) {
      glDisable(GL_CULL_FACE);
      glDisable(GL_BLEND);
   }
}

}} // namespace

