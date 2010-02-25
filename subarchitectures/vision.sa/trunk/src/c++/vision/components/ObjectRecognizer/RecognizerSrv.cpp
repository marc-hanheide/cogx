/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"


extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::ObjectRecognizer();
   }
}

using namespace std;

namespace cogx { namespace vision {

void ObjectRecognizer::startIceServer()
{
   Ice::Identity id;
   id.name = m_iceServerName;
   id.category = "ObjectRecognizer";
   getObjectAdapter()->add(new ObjectRecognizerI(this), id);
}

void ObjectRecognizer::configure(const map<string,string> & _config)
      throw(runtime_error)
{
   map<string,string>::const_iterator it;
   if((it = _config.find("--recognizerid")) != _config.end())
   {
      m_iceServerName = it->second;
   }

   m_pyRecognizer.configureRecognizer(_config);
}

void ObjectRecognizer::start()
{
   m_pyRecognizer.initModule();
   startIceServer();
}

void ObjectRecognizer::runComponent()
{
   //while(isRunning()) {
   //   sleepComponent(1000);
   //}
}

long ObjectRecognizer::GetSifts(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors)
{
   //int region[4];
   //region[0] = x0;
   //region[1] = y0;
   //region[2] = width;
   //region[3] = height;
   return 0;
}

void ObjectRecognizer::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& results)
{
   int region[4];
   region[0] = x0;
   region[1] = y0;
   region[2] = width;
   region[3] = height;

   PyGILState_STATE state = PyGILState_Ensure();

   PyObject *pMatches = m_pyRecognizer.processImage(image, region);
   if (pMatches != NULL) {
      m_pyRecognizer.parseMatches(pMatches, results);
      Py_DECREF(pMatches);
   }

   PyGILState_Release(state);
}


};}; // namespace
