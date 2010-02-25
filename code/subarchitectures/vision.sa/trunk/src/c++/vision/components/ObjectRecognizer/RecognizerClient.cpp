/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "RecognizerClient.h"

#include <sstream>
#include <Ice/Ice.h>
#include <cast/core/CASTUtils.hpp>

using namespace std;
using namespace cast;

namespace cogx { namespace vision {

ObjectRecognizerClient::ObjectRecognizerClient(	)
{
   m_serverHost = "localhost";
   m_serverName = "";
   m_serverPort = cdl::CPPSERVERPORT;
}

void ObjectRecognizerClient::configureRecognizer(const map<string,string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--recognizerhost")) != _config.end()) {
      m_serverHost = it->second;
   }

   if((it = _config.find("--recognizerid")) != _config.end()) {
      m_serverName = it->second;
   }

}

void ObjectRecognizerClient::connectIceClient(cast::CASTComponent& owner)
      throw(runtime_error)
{
   if (m_Server)
      throw runtime_error(exceptionMessage(__HERE__,
            "ObjectRecognizerClient already connected to server."));

   if (m_serverHost.empty())
      throw runtime_error(exceptionMessage(__HERE__, "no --recognizerhost given"));
   if (m_serverName.empty())
      throw runtime_error(exceptionMessage(__HERE__, "no --recognizerid given"));

   ostringstream serverAddr;
   Ice::Identity id;
 
   id.name = m_serverName;
   id.category = "ObjectRecognizer"; // same as in <URL:RecognizerSrv.cpp#::startIceServer>
   serverAddr << owner.getCommunicator()->identityToString(id)
         << ":default -h " << m_serverHost << " -p " << m_serverPort;
 
   Ice::ObjectPrx base = owner.getCommunicator()->stringToProxy(serverAddr.str());
   // mz: doing a checkedCast here freezes the server
   // stereoServer = Stereo::StereoInterfacePrx::checkedCast(base);
   m_Server = ObjectRecognizerIce::ObjectRecognizerInterfacePrx::uncheckedCast(base);
   if (!m_Server)
      throw runtime_error(exceptionMessage(__HERE__,
            "failed to connect to ObjectRecognizer server: %s",
            serverAddr.str().c_str()));
}


long ObjectRecognizerClient::GetSifts(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors)
{
   return m_Server->GetSifts(image, x0, y0, width, height, features, descriptors);
}


void ObjectRecognizerClient::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& result)
{
   m_Server->FindMatchingObjects(image, x0, y0, width, height, result);
}

};}; // namespace
