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

#include <algorithm>
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

   m_maxDistance = 1.0;
   m_maxAmbiguity = 0.8;

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

#ifdef FEAT_VISUALIZATION
   m_display.configureDisplayClient(_config);
#endif

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
#ifdef FEAT_VISUALIZATION
   m_display.connectIceClient(*this);
   m_display.setClientData(this);
   m_display.installEventReceiver();
   m_display.createForms();
#endif
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

class CViewEvaluator
{
   CDistanceCalculator *pDist;
   std::ostream *pOut;
public:
   double m_maxDistance;
   double m_maxAmbiguity;
   CViewEvaluator(double maxDistance=128, double maxAmbiguity=0.8) {
      m_maxDistance = maxDistance;
      m_maxAmbiguity = maxAmbiguity;
      pDist = NULL;
   }

   void setDistanceCalculator(CDistanceCalculator *pCalc) {
      pDist = pCalc;
   }

   void setOutput(std::ostream* pStream) {
      pOut = pStream;
   }

   // matches are sorted by distance (lo->hi)
   double getScore(TSiftVector& example, CObjectView& view, TFeatureMatchVector& matches)
   {
      double score = 0;
      int count = 0;
      int i = 0;
      typeof(matches.begin()) itmatch;
      for(itmatch = matches.begin(); itmatch != matches.end(); itmatch++) {
         i++;
         if (itmatch->ambiguity >= m_maxAmbiguity) continue;
         if (itmatch->distance > m_maxDistance) break; // all further distances are higher
         count++;
         score += std::min(1/(itmatch->distance + 1e-9), 100.0);
      }
      if (pOut) (*pOut) << "&nbsp;v" << view.m_id << ": good=" << count << "/" << matches.size();
      return score;
   }
};

struct CModelScore
{
   double score;
   std::vector<double> viewScore;
};

class CModelEvaluator
{
   CViewEvaluator *pViewEval;
   std::ostream *pOut;
public:
   CModelEvaluator() {
      pViewEval = NULL;
      pOut = NULL;
   }

   void setViewEvaluator(CViewEvaluator *pEvaluator) {
      pViewEval = pEvaluator;
   }

   void setOutput(std::ostream* pStream) {
      pOut = pStream;
   }

   // Order in matches is the same as order in model.m_views
   void evaluateModel(TSiftVector& example, CObjectModel& model, 
         std::vector<TFeatureMatchVector*>& matches, CModelScore& score)
   {
      if (! pViewEval) return; // XXX throw?
      assert(model.m_views.size() == matches.size());

      typeof(model.m_views.begin()) itv;
      int i = 0;
      if (pOut) (*pOut) << "view scores:<br>";
      for(itv = model.m_views.begin(); itv != model.m_views.end(); i++, itv++) {
         CObjectView* pView = *itv;
         double vsc = pViewEval->getScore(example, *pView, *matches[i]);
         if (pOut) (*pOut) << " -- " << sfloat(vsc, 2) << "<br>";
         score.viewScore.push_back(vsc);
      }
      if (i < 1) score.score = 0;
      else {
         typeof(score.viewScore.begin()) imax = std::max_element(score.viewScore.begin(), score.viewScore.end());
         score.score = *imax;
         //score.score = std::sum(score.viewScore.begin(), score.viewScore.end()); 
      }
   }
};

void CObjectRecognizer::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& results)
{
   if (! m_pSiftMatcher || ! m_pSiftExtractor) return;

   IplImage* pImg;
   TSiftVector sifts;

   CDistanceCalculator calc;
   CViewEvaluator vieweval(m_maxDistance, m_maxAmbiguity);
   CModelEvaluator modeval;

   vieweval.setDistanceCalculator(&calc);
   modeval.setViewEvaluator(&vieweval);

   std::ofstream fres;
   fres.open("/tmp/or_model_distrib.html", std::ofstream::out);
   vieweval.setOutput(&fres);
   modeval.setOutput(&fres);
   double tm0 = fclocks();
   fres << "match start " << tm0 << "<br>";
   fres << "maxDistance: " << m_maxDistance << " maxAmbiguity: " << m_maxAmbiguity << "<br><br>";

   // TODO: get part of image (x0, y0, w, h)
   pImg = Video::convertImageToIplGray(image);
   if (pImg) {
      m_pSiftExtractor->extractSifts(pImg, sifts);

      typeof(m_models.begin()) it;
      for (it = m_models.begin(); it != m_models.end(); it++) {
         std::vector<TSiftVector*> allfeatures;
         CObjectModel *pModel = *it;

         typeof(pModel->m_views.begin()) itv;
         for(itv = pModel->m_views.begin(); itv != pModel->m_views.end(); itv++) {
            CObjectView* pView = *itv;
            allfeatures.push_back(&pView->m_features);
         }

         // Match image features to all others
         std::vector<TFeatureMatchVector*> matches;
         m_pSiftMatcher->matchSiftDescriptors(sifts, allfeatures, matches);
         sortmatches(matches);

         //fres << pModel->m_name << " allfeatures: " << allfeatures.size() << "<br>";
         allfeatures.clear();

         // process results
         //fres << "results: " << matches.size() << "<br>";
         CModelScore score;
         modeval.evaluateModel(sifts, *pModel, matches, score);
         fres << "score: " << score.score << "<br>";
         fres << "<br>";
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

   if (1) {
      std::vector<unsigned char>data;
      Video::convertImageToGrayBytes(image, data);
      IplImage* pTest;
      Video::convertBytesToIpl(data, image.width, image.height, 1, &pTest);
      cvSaveImage("/tmp/sift_inputimage.jpg", pTest);
      cvReleaseImage(&pTest);
   }
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

#ifdef FEAT_VISUALIZATION
void CObjectRecognizer::COrDisplayClient::createForms()
{
   {
      std::ostringstream ss;
      ss << "Max distance: <select name='maxDistance'>";
      for (float f = 0.7; f <= 1.0; f += 0.01) {
         ss << "<option>" << sfloat(f, 2) << "</option>";
      }
      for (float f = 1.1; f <= 3.0; f += 0.1) {
         ss << "<option>" << sfloat(f, 2) << "</option>";
      }
      ss << "</select>";
      ss << "Max ambiguity: <select name='maxAmbiguity'>";
      for (float f = 0.7; f <= 1.0; f += 0.01) {
         ss << "<option>" << sfloat(f, 2) << "</option>";
      }
      ss << "</select>";
      ss << "<input type=\"submit\" name=\"submit\" value=\"Apply\"/>";
      setHtmlForm("Settings", "ObjectRecognizer", ss.str());
   }
}

void CObjectRecognizer::COrDisplayClient::handleForm(const std::string& id, const std::string& partId,
      const std::map<std::string, std::string>& fields)
{
   if (! pRecognizer) return;
   float f;
   typeof(fields.begin()) it;
   it = fields.find("maxDistance");
   if (it != fields.end()) {
      f = atof(it->second.c_str());
      if (f < 0.0) f = 0.0;
      if (f > 128) f = 128;
      pRecognizer->m_maxDistance = f;
   }

   it = fields.find("maxAmbiguity");
   if (it != fields.end()) {
      f = atof(it->second.c_str());
      if (f < 0.7) f = 0.7;
      if (f > 1.0) f = 1.0;
      pRecognizer->m_maxAmbiguity = f;
   }
}

bool CObjectRecognizer::COrDisplayClient::getFormData(const std::string& id, const std::string& partId,
      std::map<std::string, std::string>& fields)
{
   if (! pRecognizer) return false;
   fields["maxDistance"] = sfloat(pRecognizer->m_maxDistance, 2);
   fields["maxAmbiguity"] = sfloat(pRecognizer->m_maxAmbiguity, 2);
   return true;
}
#endif


}} // namespace
// vim:sw=3:ts=8:et
