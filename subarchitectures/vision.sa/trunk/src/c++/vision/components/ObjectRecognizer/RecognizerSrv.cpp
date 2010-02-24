/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerSrv.h"

extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx_vision_or::ObjectRecognizer();
   }
}

using namespace std;

namespace cogx_vision_or {

void ObjectRecognizer::startIceServer()
{
   Ice::Identity id;
   id.name = m_iceServerName;
   id.category = "ObjectRecognizer";
   getObjectAdapter()->add(new ObjectRecognizerI(this), id);
}

void ObjectRecognizer::configure(const std::map<std::string,std::string> & _config)
      throw(std::runtime_error)
{
   map<string,string>::const_iterator it;
   if((it = _config.find("--recognizerid")) != _config.end())
   {
      m_iceServerName = it->second;
   }
}

void ObjectRecognizer::start()
{
   startIceServer();
}

void ObjectRecognizer::runComponent()
{
   //while(isRunning()) {
   //   sleepComponent(1000);
   //}
}

long ObjectRecognizer::GetSifts(const Video::Image&, ObjectRecognizerIce::FloatSeq&,
      ObjectRecognizerIce::FloatSeq&)
{
   return 0;
}

void ObjectRecognizer::FindMatchingObjects(const Video::Image&, const cogx::Math::Rect2&,
      ObjectRecognizerIce::RecognitionResultSeq&)
{
}


}; // namespace
