/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"


extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::vision::CObjectRecognizer();
   }
}

using namespace std;
namespace orice = ObjectRecognizerIce;

namespace cogx { namespace vision {

CObjectRecognizer::CObjectRecognizer()
{
}

CObjectRecognizer::~CObjectRecognizer()
{
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
}

void CObjectRecognizer::start()
{
   debug("CObjectRecognizer Server: starting");
   m_pyRecognizer.initModule();
   startIceServer();
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
