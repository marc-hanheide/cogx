/*
 * Author: Marko Mahnič
 * Created: 2010-05-10
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
   TomGine::tgRenderModel* m_pModel;

public:
   CTomGineModel();
   ~CTomGineModel();
   virtual bool is3D(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
};

class CTomGineModel_RenderGL: public CRenderer
{
public:
   virtual void draw(CDisplayObject *pObject, void *pContext); /*override*/
};


}} // namespace
#endif /* end of include guard: CTOMGINEMODEL_PQ3LXGLL */
