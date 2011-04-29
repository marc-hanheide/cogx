/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"
#include "sifts/ExtractorSiftGpu.h"
#include "sifts/MatcherCudaSift.h"
#include "models/ModelLoader.h"

#include "VideoUtils.h"

#include "StringFmt.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VisionData.hpp>

#include <opencv/cv.h> // test
#include <opencv/highgui.h> // test

#include <algorithm>
#include <fstream>
#include <deque>

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CObjectRecognizer();
   }
}

#include "convenience.hpp"

using namespace std;
using namespace VisionData;
namespace orice = ObjectRecognizerIce;

namespace cogx { namespace vision {

CObjectRecognizer::CObjectRecognizer()
{
   m_pSiftExtractor = NULL;
   m_pSiftMatcher = NULL;

   m_maxDistance = 1.0;
   m_maxAmbiguity = 0.8;
   m_bWmFilters = true;
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

   std::vector<std::string> models;
   CModelLoader loader;

   if (modelnames.size() == 0 || modelnames[0] == "*") {
      loader.listModels(db, models);
   }
   else {
      models = modelnames;
   }

   for(size_t i = 0; i < models.size(); i++) {
      CObjectModel *pModel = new CObjectModel();
      try {
         loader.loadModel(db, models[i], *pModel);
      }
      catch (Exception& e) {
         log("ERROR: %s", e.what());
         pModel->m_id = -1;
      }
      if (pModel->m_id > 0) {
         m_models.push_back(pModel);
         log("Read model: %s", models[i].c_str());
      }
      else {
         delete pModel;
         log("Read model FAILED: %s", models[i].c_str());
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

   if ((it = _config.find("--modeldir")) != _config.end()) {
      istringstream istr(it->second);
      istr >> modeldb;
   }

   if ((it = _config.find("--models")) != _config.end()) {
      istringstream istr(it->second);
      string label;
      while(istr >> label) modelnames.push_back(label);
   }
   else {
      modelnames.push_back(string("*"));
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
 
   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   if (m_bWmFilters) {
      addChangeFilter(
            cast::createLocalTypeFilter<orice::ObjectRecognitionTask>(cast::cdl::ADD),
            new cast::MemberFunctionChangeReceiver<CObjectRecognizer>(this,
               &CObjectRecognizer::onAddRecognitionTask)
            );
   }
}

void CObjectRecognizer::abortRecognition(CRecognitionRequestWrapper& request, const std::string& cause)
{
   // TODO: add status to ObjectRecognitionTask; then overwrite with status=failed & cause
   deleteFromWorkingMemory(request.wmChange.address);
}

void CObjectRecognizer::onAddRecognitionTask(const cast::cdl::WorkingMemoryChange& _wmc)
{
   log("OR: Recognition task recieved.");

   orice::ObjectRecognitionTaskPtr pcmd;
   try {
      pcmd = getMemoryEntry<orice::ObjectRecognitionTask>(_wmc.address);

      // Lock the queue monitor and add a new request. The monitor will unlock
      // on scope-exit. notify() will wake up the timedWait() in the main loop
      // in runComponent.
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
      m_RrQueue.push_back(CRecognitionRequestWrapper(_wmc, pcmd));
      m_RrqMonitor.notify();
   }
   catch (cast::DoesNotExistOnWMException e) {
      log("ObjectRecognitionTask {%s} was removed before it could be processed", _wmc.address.id.c_str());
   }
}

// Process queued recognition requests
void CObjectRecognizer::processQueuedTasks(TRecognitionRequestVector &requests)
{
   log("%d recognition requests to process", requests.size());

   TRecognitionRequestVector::iterator it;
   for (it = requests.begin(); it != requests.end(); it++) {
      orice::ObjectRecognitionTaskPtr pTask = it->pTask;

      // load from wm the protoobject referenced by the request
      ProtoObjectPtr pProto;
      try {
         // TODO: pTask now has visualObjectAddr; use that instead of protoObjectAddr
         // or: if protoObjectAddr not empty use that, otherwise use visualObjectAddr
         pProto = getMemoryEntry<ProtoObject>(pTask->protoObjectAddr);
         Video::Image img = pProto->image;
      }
      catch (cast::DoesNotExistOnWMException e) {
         log("ProtoObject {%s} was removed before it could be processed", pTask->protoObjectAddr.id.c_str());
         abortRecognition(*it, "Failed: ProtoObject deleted");
         continue;
      }

      // process the protoobject image
      ObjectRecognizerIce::RecognitionResultSeq results;
      FindMatchingObjects(pProto->image, 0, 0, 0, 0, results);

      // write the result to recognition request
      pTask->matches = results;
      overwriteWorkingMemory(it->wmChange.address, pTask);
   }
}

// TODO: learning requests may take precedence over recognition requests so we shouldn't
// process too many requests in processQueuedTasks (which should probably be renamed to
// processRecognitionTasks).
void CObjectRecognizer::runComponent()
{
   // FIXME: there is (was?) a crash here or in startIceServer:
   //    vis.recognizer.srv: Aborting after catching an Ice::Exception from runComponent()
   //    IllegalIdentityException: illegal identity: `ObjectRecognizer/'

   sleepComponent(1000);
   debug("CObjectRecognizer Server: running");
   while(isRunning()) {
      if (m_bWmFilters) {
         TRecognitionRequestVector newRequests;

         {
            // SYNC: Lock the monitor
            IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_RrqMonitor);
            // SYNC: if queue empty, unlock the monitor and wait for notify() or timeout
            if (m_RrQueue.size() < 1) 
               m_RrqMonitor.timedWait(IceUtil::Time::seconds(2));
            // SYNC: Continue with a locked monitor

            newRequests = m_RrQueue;
            m_RrQueue.clear();
            // SYNC: unlock the monitor on scope exit
         }

         if (isRunning())
            processQueuedTasks(newRequests);
      }
      else {
         sleepComponent(1000);
      }
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
   DMESSAGE("**** NOT YET");
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
      if (model.m_views.size() != matches.size()) return; // XXX throw?

      typeof(model.m_views.begin()) itv;
      int i = 0;
      if (pOut) (*pOut) << "view scores:<br>";
      for (itv = model.m_views.begin(); itv != model.m_views.end(); i++, itv++) {
         CObjectView* pView = *itv;
         double vsc = pViewEval->getScore(example, *pView, *matches[i]);
         if (pOut) (*pOut) << " -- " << sfloat(vsc, 2) << "<br>";
         score.viewScore.push_back(vsc);
      }
      if (i < 1) score.score = 0;
      else {
         std::vector<double>::iterator imax;
         imax = std::max_element(score.viewScore.begin(), score.viewScore.end());
         score.score = *imax;
         //score.score = std::sum(score.viewScore.begin(), score.viewScore.end()); 
      }
   }
};

// "hash" function that converts string to RGB
std::string color(const std::string& val)
{
   union {
      unsigned long v;
      unsigned char b[4];
   };
   v = 1232357 + val.size() * 13;
   for(int i=0; i<val.size(); i++) {
      v = (v + val[i]) * 7 % 0x0f67f241;
      if (i > 10) break;
   }
   for(int i = 0; i < 4; i++) b[i] = b[i] % 192 + 32;
   char buf[32];
   sprintf(buf, "#%02x%02x%02x", (int) v & 0xff, (int) (v & 0xff00) >> 8, (int) (v & 0xff0000) >> 16);
   return std::string(buf);
}

// Display graph in v11n
void CObjectRecognizer::fancyDisplay(std::vector<CObjectModel*>& models, std::vector<CModelScore>& scores)
{
#ifdef FEAT_VISUALIZATION
   const unsigned int histSize = 32;
   typedef std::deque<double> TFloatQueue;
   static std::vector<TFloatQueue> scoreHist;
   while (models.size() > scoreHist.size()) {
      scoreHist.push_back(TFloatQueue());
   }
   for (int i = 0; i < scoreHist.size(); i++) {
      TFloatQueue& hist = scoreHist[i];
      while (hist.size() < histSize-1) hist.push_back(0);
      if (i < scores.size()) hist.push_back(scores[i].score);
      else hist.push_back(0);
      while (hist.size() > histSize) hist.pop_front();
   }
   double smin = 1e99;
   double smax = 1e-99;
   for (int i = 0; i < scoreHist.size(); i++) {
      TFloatQueue& hist = scoreHist[i];
      for(int k = 0; k < hist.size(); k++) {
         if (hist[k] < smin) smin = hist[k];
         if (hist[k] > smax) smax = hist[k];
      }
   }
   if (smin >= smax) return;
   if (smin > 0) smin = 0;

   std::ostringstream ss;
   ss << "<svg viewbox='0 0 242 162'>";

   ss << "<rect x='0' y='0' width='242' height='162' fill='white' stroke='blue' stroke-width='1' />";
   ss << "<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,120 241,120' />";
   ss << "<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,80 241,80' />";
   ss << "<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,40 241,40' />";

   double range = smax-smin;
   for (int i = 0; i < scoreHist.size(); i++) {
      TFloatQueue& hist = scoreHist[i];
      ss << "<polyline fill='none' stroke='" << color(models[i]->m_name) << "' stroke-width='1' points='";
      for(int k = 0; k < hist.size(); k++) {
         double p = 1.0 - ((hist[k] - smin) / range);
         ss << int(240.0*k/hist.size()+0.5) << "," << int(160*p+0.5) << " ";
      }
      ss << "' />\n";
   }

   ss << "<text x='2' y='16' font-size='12' fill='blue'>" << int(smax) << "</text>";
   ss << "<text x='2' y='160' font-size='12' fill='blue'>" << int(smin) << "</text>";

   ss << "</svg>";
   m_display.setObject("ObjectRecognizer.Graph", "999_lines", ss.str());
#endif
}

void CObjectRecognizer::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& results)
{
   if (! m_pSiftMatcher || ! m_pSiftExtractor) return;

   IplImage* pImg = NULL;
   TSiftVector sifts;

   CDistanceCalculator calc;
   CViewEvaluator vieweval(m_maxDistance, m_maxAmbiguity);
   CModelEvaluator modeval;

   vieweval.setDistanceCalculator(&calc);
   modeval.setViewEvaluator(&vieweval);

   std::vector<CModelScore> modelScores;

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

         //typeof(pModel->m_views.begin()) itv;
         //for(itv = pModel->m_views.begin(); itv != pModel->m_views.end(); itv++) {
         //   CObjectView* pView = *itv;
         //   allfeatures.push_back(&pView->m_features);
         //}
         pModel->getAllFeatures(allfeatures);

         // Match image features to all others
         std::vector<TFeatureMatchVector*> matches;
         m_pSiftMatcher->matchSiftDescriptors(sifts, allfeatures, matches);
         sortmatches(matches);

         //fres << pModel->m_name << " allfeatures: " << allfeatures.size() << "<br>";
         allfeatures.clear();

         // process results
         //fres << "results: " << matches.size() << "<br>";
         CModelScore score;
         score.pModel = pModel;
         modeval.evaluateModel(sifts, *pModel, matches, score);
         modelScores.push_back(score);
         fres << "score: " << score.score << "<br>";
         fres << "<br>";
      }
   }
   double tm1 = fclocks();
   fres << "match end " << tm1 << "  delta:" << tm1-tm0 << "<br>";

   fancyDisplay(m_models, modelScores);

   //fres << "NUM SIFTS: " << sifts.size() << "<br>";

   //typeof(sifts.begin()) it;
   //for (it = sifts.begin(); it != sifts.end(); it++) {
   //   CSiftFeature *pSift = *it;
   //   fres << "x: " << pSift->x << "  y: " << pSift->y << "<br>";
   //   delete pSift;
   //}

   sifts.clear();
   fres.close();

   // TODO: convert model scores to probabilities.
   //    - evaluateModel should produce better scores, based on observation models
   //    - score for each model should be converted to probability for that model
   //    - maxScore = max score of all models
   //    - unknown = 1 - maxScore; other scores are changed so that they sum up to 1;
   //      not ok when there are many (similar) recognitions:
   //         0.8, 0.8, 0.8, 0.8 => 0.2, 0.2, 0.2, 0.2, u=0.2
   //      => PDF for models probably doesn't make sense, but looks like I need to
   //         implement it for the planner to work 
   //      I could make maxScore (maxProb) part of the recognition result
   results.clear();
   vector<CModelScore>::iterator itsc;
   for (itsc = modelScores.begin(); itsc != modelScores.end(); itsc++) {
      ObjectRecognizerIce::RecognitionResult orr;
      if (! itsc->pModel) continue;
      if (itsc->pModel->m_name == "") orr.label = _str_(itsc->pModel->m_id);
      else orr.label = itsc->pModel->m_name;
      orr.probability = itsc->score;

      // TODO: copy pose probabilities to orr.poses/orr.posePd

      results.push_back(orr);
   }

   if (1) {
      std::vector<unsigned char>data;
      Video::convertImageToGrayBytes(image, data);
      IplImage* pTest = NULL;
      Video::convertBytesToIpl(data, image.width, image.height, 1, &pTest);
      cvSaveImage("/tmp/sift_inputimage.jpg", pTest);
      cvReleaseImage(&pTest);
   }
}

// Update a known model with features that are present in the image
//   TODO: It would be nice to know the object pose for the model update
//         Maybe a pose change relative to previous UpdateModel could be used
void CObjectRecognizer::UpdateModel(const std::string& modelName, const Video::Image& image)
{
   // find model with modelName; if it does not exist, create a new one
   // compare sifts to model views; if a good match is found, no need to learn
   // no match => add view to "unoriented views" (we need DB support for it)
   TSiftVector sifts;
   IplImage* pImg = Video::convertImageToIplGray(image);
   if (pImg) {
      m_pSiftExtractor->extractSifts(pImg, sifts);
   }
   cvReleaseImage(&pImg);

   typeof(m_models.begin()) itmod;
   CObjectModel *pModel = NULL;
   for (itmod = m_models.begin(); itmod != m_models.end(); itmod++) {
      if ((*itmod)->m_name == modelName) {
         pModel = *itmod;
         break;
      }
   }

   if (pModel == NULL) {
      // TODO: create a new model
      return;
   }

   CObjectView* pView = new CObjectView();
   // TODO: check if the model matches the features
   pModel->m_views.push_back(pView); // TODO: this view is special (no pose, learned online)
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
   typeof(fields.begin()) it;
   it = fields.find("maxDistance");
   if (it != fields.end())
      pRecognizer->m_maxDistance = parsefloat(it->second, 0, 128);

   it = fields.find("maxAmbiguity");
   if (it != fields.end())
      pRecognizer->m_maxAmbiguity = parsefloat(it->second, 0.7, 1.0);
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
