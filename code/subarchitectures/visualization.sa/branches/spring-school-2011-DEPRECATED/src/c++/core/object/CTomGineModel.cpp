/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
   //if (m_Models.find(partId)->second != NULL) {
   typeof(m_Models.begin()) itExtng = m_Models.find(partId);
   if (itExtng != m_Models.end()) {
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

ERenderContext CTomGineModel::getPreferredContext()
{
   return ContextGL;
}

void CTomGineModel::removePart(const std::string&partId)
{
   typeof(m_Models.begin()) it = m_Models.find(partId);
   //if (it->second != NULL) {
   if (it != m_Models.end()) {
      TomGine::tgRenderModel* pModel = m_Models[partId];
      m_Models.erase(it);
      delete pModel;
   }
}

CRenderer* CTomGineModel::getRenderer(ERenderContext context)
{
   switch(context) {
      case ContextGL: return renderGL.get();
   }
   return NULL;
}

// OpenGL context is selected by Qt, so pContext is NULL.
void CTomGineModel_RenderGL::draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext)
{
   DTRACE("CTomGineModel_RenderGL::draw");
   if (pObject == NULL) return;
   CTomGineModel *pModel = dynamic_cast<CTomGineModel*>(pObject);
   if (pModel == NULL) return;
   if (pModel->m_Models.size() < 1) return;
   DMESSAGE("Models present.");

   TomGine::tgRenderModel* pPart;
   FOR_EACH_V(pPart, pModel->m_Models) {
      if (!pPart) continue;

      glPushMatrix();
      // TODO: Conditional drawing of object parts (faces, normals, ...)
      pPart->DrawFaces(true);
      // pPart->DrawQuadstrips();
      pPart->DrawNormals(0.01);
      glPopMatrix();
   }
}

void CTomGineModel::setPose3D(const std::string& partId, const std::vector<double>& position,
      const std::vector<double>& rotation)
{
   assert(position.size() == 3);
   assert(rotation.size() == 4);

   TomGine::tgRenderModel* pModel = NULL;
   //if (m_Models.find(partId)->second != NULL) {
   typeof(m_Models.begin()) itExtng = m_Models.find(partId);
   if (itExtng != m_Models.end()) {
      pModel = m_Models[partId];
   }
   if (! pModel) return;

   pModel->m_pose.pos.x = position[0];
   pModel->m_pose.pos.y = position[1];
   pModel->m_pose.pos.z = position[2];
   pModel->m_pose.q.x = rotation[0];
   pModel->m_pose.q.y = rotation[1];
   pModel->m_pose.q.z = rotation[2];
   pModel->m_pose.q.w = rotation[3];
}

}} // namespace

