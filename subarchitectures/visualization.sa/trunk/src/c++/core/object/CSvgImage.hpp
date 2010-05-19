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
   // A SVG image may have multiple parts: data + transformation
   struct SPart
   {
      std::string id;
      std::string data;
      std::vector<double> trmatrix;
      SPart(const std::string& partId) {
         id = partId;
      }
      void setIdentity() {
         trmatrix.resize(0);
      }
   };
   std::vector<SPart*> m_Parts;

public:
   CSvgImage();
   ~CSvgImage();
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   void setPart(const std::string& partId, const std::string& xmlData);
   void setTransform2D(const std::string& partId, const std::vector<double> &matrix); /*override*/

private:
   SPart* findPart(const std::string& partId);
};

class CSvgImage_Render2D: public CRenderer
{
public:
   virtual void draw(CDisplayObject *pObject, void *pContext); /*override*/
};


}} // namespace
#endif /* end of include guard: CSVGIMAGE_IRJTSHTI */
