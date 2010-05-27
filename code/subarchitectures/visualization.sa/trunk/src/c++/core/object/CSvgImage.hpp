/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */
#ifndef CSVGIMAGE_IRJTSHTI
#define CSVGIMAGE_IRJTSHTI

#include "../Model.hpp"
#include <QSvgRenderer>

namespace cogx { namespace display {

class CSvgImage: public CDisplayObject
{
private:
   friend class CSvgImage_Render2D;
   static std::auto_ptr<CRenderer> render2D;
// public:
   // A SVG image may have multiple parts: data + transformation
   class SPart
   {
      QSvgRenderer* _psvgdoc;
   public:
      std::string id;
      std::string data;
      std::vector<double> trmatrix;
      SPart(const std::string& partId) {
         id = partId;
         _psvgdoc = NULL;
      }
      ~SPart() {
         if (_psvgdoc) delete _psvgdoc;
         _psvgdoc = NULL;
      }
      void setIdentity() {
         trmatrix.resize(0);
      }
      QSvgRenderer& getSvgDoc() {
         if (! _psvgdoc) {
            _psvgdoc = new QSvgRenderer();
            _psvgdoc->load(QByteArray::fromRawData(data.c_str(), data.length()));
         }
         return *_psvgdoc;
      }
      void setData(const std::string& xmlData) {
         if (_psvgdoc) delete _psvgdoc;
         _psvgdoc = NULL;
         data = xmlData;
      }
   };
   std::vector<SPart*> m_Parts;

public:
   CSvgImage();
   ~CSvgImage();
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   void setPart(const std::string& partId, const std::string& xmlData);
   virtual void setTransform2D(const std::string& partId, const std::vector<double> &transform); /*override*/

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
