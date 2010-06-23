/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"
#include "sifts/ExtractorSiftGpu.h"
#include "sifts/MatcherCudaSift.h"

#include "VideoUtils.h"

#include <opencv/cv.h> // test
#include <opencv/highgui.h> // test

#include <fstream>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CObjectRecognizer();
   }
}

#include "convenience.hpp"

using namespace std;
namespace orice = ObjectRecognizerIce;

namespace cogx { namespace vision {

CObjectRecognizer::CObjectRecognizer()
{
   m_pSiftExtractor = NULL;
   m_pSiftMatcher = NULL;
}

CObjectRecognizer::~CObjectRecognizer()
{
   if (m_pSiftMatcher) delete m_pSiftMatcher;
   if (m_pSiftExtractor) delete m_pSiftExtractor;
}

void CObjectRecognizer::startIceServer()
{
   orice::ObjectRecognizerInterfacePtr servant = new ObjectRecognizerI(this);
   registerIceServer<orice::ObjectRecognizerInterface, ObjectRecognizerI>(servant);
}

void CObjectRecognizer::configure(const map<string,string> & _config)
      throw(runtime_error)
{
   debug("CObjectRecognizer Server: configuring");
   CASTComponent::configure(_config);

   m_pyRecognizer.configureRecognizer(_config);

   debug("CObjectRecognizer Server: starting");
   m_pyRecognizer.initModule();
   startIceServer();

   m_pSiftExtractor = new CSiftExtractorGPU();
   m_pSiftMatcher   = new CSiftMatcherCudaSift();
}

void CObjectRecognizer::start()
{
}

void CObjectRecognizer::runComponent()
{
   // TODO: crash here or in startIceServer?:
   // vis.recognizer.srv: Aborting after catching an Ice::Exception from runComponent()
   // IllegalIdentityException: illegal identity: `ObjectRecognizer/'

   sleepComponent(1000);
   debug("CObjectRecognizer Server: running");
   while(isRunning()) {
     sleepComponent(1000);
   }
   debug("CObjectRecognizer Server: Done.");
}

long CObjectRecognizer::LoadObjectModel(const std::string& modelPath)
{
   return 0;
}

long CObjectRecognizer::GetSifts(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors)
{
   DTRACE("CObjectRecognizer::GetSifts");
   //int region[4];
   //region[0] = x0;
   //region[1] = y0;
   //region[2] = width;
   //region[3] = height;
   return 0;
}

void CObjectRecognizer::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& results)
{
   if (! m_pSiftMatcher || ! m_pSiftExtractor) return;

   std::vector<unsigned char>data;
   // Video::convertImageToGrayBytes(image, data);

   IplImage* pImg;
   TSiftVector sifts;

   pImg = Video::convertImageToIplGray(image);
   if (pImg) {
      m_pSiftExtractor->extractSifts(pImg, sifts);
   }

   std::ofstream fres;
   fres.open("/tmp/or_model_distrib.html", std::ofstream::out);
   fres << "NUM SIFTS: " << sifts.size() << "<br>";

   typeof(sifts.begin()) it;
   for (it = sifts.begin(); it != sifts.end(); it++) {
      CSiftFeature *pSift = *it;
      fres << "x: " << pSift->x << "  y: " << pSift->y << "<br>";
      delete pSift;
   }
   sifts.clear();
   fres.close();

   //IplImage* pTest;
   //Video::convertBytesToIpl(data, image.width, image.height, 1, &pTest);
   //cvSaveImage("/tmp/sift_inputimage.jpg", pTest);
   //cvReleaseImage(&pTest);
}

#if 0
static int tcount = 0;
void CObjectRecognizer::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& results)
{
   DTRACE("CObjectRecognizer::FindMatchingObjects");
   int region[4];
   region[0] = x0;
   region[1] = y0;
   region[2] = width;
   region[3] = height;

   PyGILState_STATE state = PyGILState_Ensure();
   tcount ++;
   DMESSAGE("processImage - NUM REQ: " << tcount);

   PyObject *pMatches = m_pyRecognizer.processImage(image, region);
   //if (pMatches != NULL) {
   //   m_pyRecognizer.parseMatches(pMatches, results);
   //   Py_DECREF(pMatches);
   //}

   tcount--;
   PyGILState_Release(state);
}
#endif


}} // namespace
// vim:sw=3:ts=8:et
