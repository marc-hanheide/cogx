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
#ifndef CTOMGINEMODEL_PQ3LXGLL
#define CTOMGINEMODEL_PQ3LXGLL

#include "../Model.hpp"
#include "tgRenderModel.h"

namespace cogx { namespace display {

class CTomGineModel: public CDisplayObject
{
   friend class CTomGineModel_RenderGL;
   static std::auto_ptr<CRenderer> renderGL;

public:
   std::map<std::string, TomGine::tgRenderModel*> m_Models;

public:
   CTomGineModel();
   ~CTomGineModel();
   void deserialize(const std::string& partId, const std::vector<unsigned char>& data);
   void removePart(const std::string& partId);
   virtual ERenderContext getPreferredContext(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   virtual void setPose3D(const std::string& partId, const std::vector<double>& position,
         const std::vector<double>& rotation); /*override*/
   virtual bool removePart(const std::string& partId, CPtrVector<CDisplayObjectPart>& parts) /*override*/
   {
      return false;
   }
};

class CTomGineModel_RenderGL: public CRenderer
{
public:
   virtual void draw(CDisplayView *pView, CDisplayObject *pObject, void *pContext); /*override*/
};


}} // namespace
#endif /* end of include guard: CTOMGINEMODEL_PQ3LXGLL */
