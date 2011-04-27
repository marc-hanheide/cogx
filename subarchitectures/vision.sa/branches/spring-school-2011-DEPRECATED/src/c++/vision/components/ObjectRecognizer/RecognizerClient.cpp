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
namespace orice = ObjectRecognizerIce;

namespace cogx { namespace vision {

CObjectRecognizerClient::CObjectRecognizerClient(	)
{
   m_serverName = "";
}

void CObjectRecognizerClient::configureRecognizer(const map<string,string> & _config)
{
   map<string,string>::const_iterator it;

   if((it = _config.find("--recognizerid")) != _config.end()) {
      m_serverName = it->second;
   }

   /* XXX: #r=models_from_client_cast
   if((it = _config.find("--modeldir")) != _config.end())
   {
      istringstream istr(it->second);
      istr >> m_modelDir;
   }

   if((it = _config.find("--models")) != _config.end())
   {
      istringstream istr(it->second);
      string label;
      while(istr >> label) m_modelLabels.push_back(label);
   }
   */

}

void CObjectRecognizerClient::connectIceClient(cast::CASTComponent& owner)
      throw(runtime_error)
{
   owner.debug("CObjectRecognizerClient connecting to CObjectRecognizerSrv.");
   if (m_Server)
      throw runtime_error(exceptionMessage(__HERE__,
            "CObjectRecognizerClient already connected to server."));

   if (m_serverName.empty())
      throw runtime_error(exceptionMessage(__HERE__, "no --recognizerid given"));

   m_Server = owner.getIceServer<orice::ObjectRecognizerInterface>(m_serverName);

   /* XXX: Models are currently loaded at the server; LoadObjectModel not implemented on server.
    * id=models_from_client_cast
   if (m_modelLabels.size() > 0) {
      ostringstream ostr;
      for(size_t i = 0; i < m_modelLabels.size(); i++)
         ostr << " '" << m_modelLabels[i] << "'";
      owner.log("Detecting objects: %s", ostr.str().c_str());

      owner.debug("Loading models");
      vector<string>::const_iterator it;
      for( it = m_modelLabels.begin(); it != m_modelLabels.end(); it++) {
         string modelpath = m_modelDir + "/" + *it;
         owner.debug(string("  *") + modelpath);
         LoadObjectModel(modelpath);
      }
   }
   */ 
}

long CObjectRecognizerClient::LoadObjectModel(const std::string& modelPath)
{
   return m_Server->LoadObjectModel(modelPath);
}

long CObjectRecognizerClient::GetSifts(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors)
{
   return m_Server->GetSifts(image, x0, y0, width, height, features, descriptors);
}


void CObjectRecognizerClient::FindMatchingObjects(const Video::Image& image,
      const int x0, const int y0, const int width, const int height,
      ObjectRecognizerIce::RecognitionResultSeq& result)
{
   m_Server->FindMatchingObjects(image, x0, y0, width, height, result);
}

}} // namespace
