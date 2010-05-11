/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 */
#include "CTomGineModel.hpp"

#include "xtgSerialize.h"
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/array.hpp>

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CTomGineModel::renderGL(new CTomGineModel_RenderGL());

CTomGineModel::CTomGineModel()
{
}

CTomGineModel::~CTomGineModel()
{
   TomGine::tgRenderModel* pModel;
   FOR_EACH_V(pModel, m_Models) {
      if (pModel) delete pModel;
   }
   m_Models.erase(m_Models.begin(), m_Models.end());
}

void CTomGineModel::deserialize(const std::string& partId, const std::vector<unsigned char>& data)
{
   namespace bio = boost::iostreams;
   namespace barch = boost::archive;

   TomGine::tgRenderModel* pModel = NULL;
   if (m_Models.find(partId)->second != NULL) {
      pModel = m_Models[partId];
   }

   if (pModel == NULL) {
      pModel = new TomGine::tgRenderModel();
      m_Models[partId] = pModel;
   }

   try {
      bio::stream_buffer<bio::array_source> buf((char*)&data[0], data.size());
      std::istream iss(&buf);
      barch::text_iarchive ia(iss);
      ia >> *pModel;
   }
   catch (barch::archive_exception &e) {
      printf("Error in tgRenderModel stream: %s\n", e.what());
      m_Models.erase(m_Models.find(partId));
      if (pModel) delete pModel;
   }
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
   if (pModel->m_Models.size() < 1) return;
   DMESSAGE("Models present.");

   TomGine::tgRenderModel* pPart;
   FOR_EACH_V(pPart, pModel->m_Models) {
      if (!pPart) continue;

      // TODO: Conditional GL settings - per object
      //if (0) {
      //   glEnable(GL_BLEND);
      //   glEnable(GL_CULL_FACE);
      //   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      //}

      // TODO: Conditional drawing of object parts (faces, normals, ...)
      pPart->DrawFaces(true);
      // pPart->DrawQuadstrips();
      pPart->DrawNormals(0.01);

      //if (0) {
      //   glDisable(GL_CULL_FACE);
      //   glDisable(GL_BLEND);
      //}
   }

}

}} // namespace

