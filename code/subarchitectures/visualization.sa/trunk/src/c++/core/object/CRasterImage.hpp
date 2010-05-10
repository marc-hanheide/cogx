/*
 * Author: Marko Mahnič
 * Created: 2010-03-31
 */
#ifndef CRASTERIMAGE_4RYC90L
#define CRASTERIMAGE_4RYC90L

#include "../Model.hpp"
#include <QImage>

namespace cogx { namespace display {

class CRasterImage: public CDisplayObject
{
   friend class CRasterImage_Render2D;
   static std::auto_ptr<CRenderer> render2D;

public:
   QImage* m_pImage;

public:
   CRasterImage();
   ~CRasterImage();
   bool isBitmap(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
};

class CRasterImage_Render2D: public CRenderer
{
public:
   virtual void draw(CDisplayObject *pObject, void *pContext); /*override*/
};

}} // namespace
#endif /* end of include guard: CRASTERIMAGE_4RYC90L */
