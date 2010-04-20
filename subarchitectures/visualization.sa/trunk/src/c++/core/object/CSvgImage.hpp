/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */
#ifndef CSVGIMAGE_IRJTSHTI
#define CSVGIMAGE_IRJTSHTI

#include "../Model.hpp"

namespace cogx { namespace display {

class CSvgImage: public CDisplayObject
{
   friend class CSvgImage_Render2D;
   static std::auto_ptr<CRenderer> render2D;
public:
   // TODO: a SVG image may have multiple parts: data + transformation
   std::string data;
   std::vector<double> trmatrix;

public:
   CSvgImage();
   ~CSvgImage();
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   void setPart(const std::string& partId, const std::string& xmlData);
   void setPartTransform(const std::string& partId, const std::vector<double> &matrix);
};

class CSvgImage_Render2D: public CRenderer
{
public:
   virtual void draw(CDisplayObject *pObject, void *pContext); /*override*/
};


}} // namespace
#endif /* end of include guard: CSVGIMAGE_IRJTSHTI */
