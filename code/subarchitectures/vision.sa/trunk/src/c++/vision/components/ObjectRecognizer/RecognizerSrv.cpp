/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"
#include "sifts/ExtractorSiftGpu.h"
#include "sifts/MatcherCudaSift.h"
#include "models/ModelLoader.h"

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

   typeof(m_models.begin()) itm;
   for(itm = m_models.begin(); itm != m_models.end(); itm++) {
      CObjectModel* pModel = *itm;
      if (pModel) delete pModel;
   }
   m_models.clear();
}

void CObjectRecognizer::startIceServer()
{
   orice::ObjectRecognizerInterfacePtr servant = new ObjectRecognizerI(this);
   registerIceServer<orice::ObjectRecognizerInterface, ObjectRecognizerI>(servant);
}

void CObjectRecognizer::loadModels(const std::string& from, const std::vector<std::string>& modelnames)
{
   sqlite3* db = NULL;
   int sqlrv = sqlite3_open(from.c_str(), &db);
   if (sqlrv != SQLITE_OK) {
      log(" *** CObjectRecognizer: Could not open %s", from.c_str());
      return;
   }
   CModelLoader loader;
   for(size_t i = 0; i < modelnames.size(); i++) {
      CObjectModel *pModel = new CObjectModel();
      try {
         loader.loadModel(db, modelnames[i], *pModel);
      }
      catch (Exception& e) {
         log("ERROR: %s", e.what());
         pModel->m_id = -1;
      }
      if (pModel->m_id > 0) {
         m_models.push_back(pModel);
         log("Read model: %s", modelnames[i].c_str());
      }
      else {
         delete pModel;
         log("Read model FAILED: %s", modelnames[i].c_str());
      }
   }

   if (db) {
      sqlite3_close(db);
   }
}

void CObjectRecognizer::configure(const map<string,string> & _config)
      throw(runtime_error)
{
   debug("CObjectRecognizer Server: configuring");
   CASTComponent::configure(_config);

   map<string,string>::const_iterator it;
   string modeldb;
   vector<string> modelnames;

   if ((it = _config.find("--modeldir")) != _config.end())
   {
      istringstream istr(it->second);
      istr >> modeldb;
   }

   if ((it = _config.find("--models")) != _config.end())
   {
      istringstream istr(it->second);
      string label;
      while(istr >> label) modelnames.push_back(label);
   }

   loadModels(modeldb, modelnames);

   //m_pyRecognizer.configureRecognizer(_config);

   //debug("CObjectRecognizer Server: starting");
   //m_pyRecognizer.initModule();
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

   std::ofstream fres;
   fres.open("/tmp/or_model_distrib.html", std::ofstream::out);
   double tm0 = fclocks();
   fres << "match start " << tm0 << "<br>";

   // TODO: get part of image (x0, y0, w, h)
   pImg = Video::convertImageToIplGray(image);
   if (pImg) {
      m_pSiftExtractor->extractSifts(pImg, sifts);

      typeof(m_models.begin()) it;
      for (it = m_models.begin(); it != m_models.end(); it++) {
         // Gather features from all objects
         std::vector<TSiftVector*> allfeatures;
         CObjectModel *pModel = *it;

         typeof(pModel->m_views.begin()) itv;
         for(itv = pModel->m_views.begin(); itv != pModel->m_views.end(); itv++) {
            CObjectView* pView = *itv;
            allfeatures.push_back(&pView->m_features);
         }

         // Match image features to all others
         std::vector<TFeatureMatchVector*> results;
         m_pSiftMatcher->matchSiftDescriptors(sifts, allfeatures, results);
         fres << "allfeatures: " << allfeatures.size() << "<br>";
         allfeatures.clear();

         // process results
         fres << "results: " << results.size() << "<br>";
         typeof(results.begin()) itres;
         for(itres = results.begin(); itres != results.end(); itres++) {
            TFeatureMatchVector *pMatches = *itres;

            int count = 0;
            typeof(pMatches->begin()) itmatch;
            fres << "result pack *************** count:" << pMatches->size() << "<br>"; 
            for(itmatch = pMatches->begin(); itmatch != pMatches->end(); itmatch++) {
               fres << "  " << itmatch->indexA << "," << itmatch->indexB << ","
                  << itmatch->distance << "<br>";
                  //<< itmatch->distance << " | ";
               count++;
               if (count > 30) { break; }
            }
            fres << "<br>";
         }
         results.clear();
      }
   }
   double tm1 = fclocks();
   fres << "match end " << tm1 << "  delta:" << tm1-tm0 << "<br>";

   //fres << "NUM SIFTS: " << sifts.size() << "<br>";

   //typeof(sifts.begin()) it;
   //for (it = sifts.begin(); it != sifts.end(); it++) {
   //   CSiftFeature *pSift = *it;
   //   fres << "x: " << pSift->x << "  y: " << pSift->y << "<br>";
   //   delete pSift;
   //}

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
