/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */
#include "CSvgImage.hpp"
#include <QtSvg>
#include <QSvgRenderer>

using namespace std;

namespace cogx { namespace display {

std::auto_ptr<CRenderer> CSvgImage::render2D(new CSvgImage_Render2D());

CSvgImage::CSvgImage()
{
}

CSvgImage::~CSvgImage()
{
}

void CSvgImage::setPart(const std::string& partId, const std::string& xmlData)
{
}

void CSvgImage::setPartTransform(const std::string& partId, const std::vector<double> &matrix)
{
   if (matrix.size() != 9) return;
   trmatrix = matrix;
}

CRenderer* CSvgImage::getRenderer(ERenderContext context)
{
   switch(context) {
      case Context2D: return render2D.get();
   }
   return NULL;
}

void CSvgImage_Render2D::draw(CDisplayObject *pObject, void *pContext)
{
   if (pObject == NULL || pContext == NULL) return;
   CSvgImage *pImage = (CSvgImage*) pObject;
   QPainter *pPainter = (QPainter*) pContext;

   QSvgRenderer doc(QByteArray::fromRawData(pImage->data.c_str(), pImage->data.length()), NULL);
   QSize size = doc.defaultSize();

   pPainter->save();
   // TODO: render each part with its own transformation
   // (for test we just transform a copy the only object)
   try {
      doc.render(pPainter, QRectF(0, 0, size.width(), size.height()));
      std::vector<double>& trmatrix = pImage->trmatrix;
      if (trmatrix.size() == 9) {
         QTransform trans;

         //trans.translate(100, 100);
         //trans.rotate(30);
         //trans.scale(0.2, 0.2);
         //printf("---------\n");
         //printf("%f\n", trans.m11());
         //printf("%f\n", trans.m12());
         //printf("%f\n", trans.m13());
         //printf("%f\n", trans.m21());
         //printf("%f\n", trans.m22());
         //printf("%f\n", trans.m23());
         //printf("%f\n", trans.m31());
         //printf("%f\n", trans.m32());
         //printf("%f\n", trans.m33());

         trans.setMatrix(
              trmatrix[0], trmatrix[1], trmatrix[2],
              trmatrix[3], trmatrix[4], trmatrix[5],
              trmatrix[6], trmatrix[7], trmatrix[8]);
         pPainter->setWorldTransform(trans, true);
         doc.render(pPainter, QRectF(0, 0, size.width(), size.height()));
      }
   }
   catch (...) {
   }
   pPainter->restore();
}

}} // namespace
