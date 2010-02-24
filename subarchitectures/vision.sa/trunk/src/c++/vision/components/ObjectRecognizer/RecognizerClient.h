/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#ifndef RECOGNIZER_CLIENT_HPP_
#define RECOGNIZER_CLIENT_HPP_

#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <cast/core/CASTComponent.hpp>

#include "ObjectRecognizerSrv.hpp" // generated from ice

namespace cogx_vision_or {

// Client and server must implement the abstract interface ObjectRecognizerMethods.
// Methods are the ones implemented in ObjectRecognizerInterface,
// but without ice::context.
class ObjectRecognizerMethods 
{
public:
   virtual long GetSifts(const Video::Image&, ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&) = 0;
   virtual void FindMatchingObjects(const Video::Image&, const cogx::Math::Rect2&, ObjectRecognizerIce::RecognitionResultSeq&) = 0;
};


class ObjectRecognizerClient: public ObjectRecognizerMethods
{
private:
   std::string m_serverHost;
   std::string m_serverName;
   int m_serverPort;
   ObjectRecognizerIce::ObjectRecognizerInterfacePrx m_Server;

public:
   ObjectRecognizerClient();
   void configureRecognizer(const std::map<std::string,std::string> & _config);
   void connectIceClient(cast::CASTComponent& owner)
         throw(std::runtime_error);

public:
   // ObjectRecognizerMethods
   virtual long GetSifts(const Video::Image&, ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&);
   virtual void FindMatchingObjects(const Video::Image&, const cogx::Math::Rect2&, ObjectRecognizerIce::RecognitionResultSeq&);


};

}; // namespace
#endif
