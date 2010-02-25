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

#include <ObjectRecognizerSrv.hpp> // generated from ice

namespace cogx { namespace vision {

// Client and server must implement the abstract interface ObjectRecognizerMethods.
// Methods are the ones implemented in ObjectRecognizerInterface,
// but without ice::context.
class ObjectRecognizerMethods 
{
public:
   virtual long GetSifts(const Video::Image&,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&) = 0;
   virtual void FindMatchingObjects(const Video::Image&,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&) = 0;
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
   virtual long GetSifts(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&) = 0;
   virtual void FindMatchingObjects(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&) = 0;
   long GetSifts(const Video::Image& image,
         ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors)
   {
      return GetSifts(image, 0, 0, 0, 0, features, descriptors);
   }
   void FindMatchingObjects(const Video::Image& image, ObjectRecognizerIce::RecognitionResultSeq& result)
   {
      FindMatchingObjects(image, 0, 0, 0, 0, result);
   }


};

}; }; // namespace
#endif
