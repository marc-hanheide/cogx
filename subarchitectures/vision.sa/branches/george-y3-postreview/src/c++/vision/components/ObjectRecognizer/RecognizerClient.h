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
class CObjectRecognizerMethods 
{
public:
   virtual long LoadObjectModel(const std::string& modelPath) = 0;
   virtual long GetSifts(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&) = 0;
   virtual void FindMatchingObjects(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&) = 0;
};


class CObjectRecognizerClient: public CObjectRecognizerMethods
{
private:
   std::string m_serverName;
   ObjectRecognizerIce::ObjectRecognizerInterfacePrx m_Server;

   // models loaded with options parsed by configureRecognizer
   std::string m_modelDir;
   std::vector<std::string> m_modelLabels;

public:
   CObjectRecognizerClient();
   void configureRecognizer(const std::map<std::string,std::string> & _config);
   void connectIceClient(cast::CASTComponent& owner)
         throw(std::runtime_error);

public:
   // ObjectRecognizerMethods
   virtual long LoadObjectModel(const std::string& modelPath);
   virtual long GetSifts(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&);
   virtual void FindMatchingObjects(const Video::Image&,
         const int left, const int top, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&);
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
