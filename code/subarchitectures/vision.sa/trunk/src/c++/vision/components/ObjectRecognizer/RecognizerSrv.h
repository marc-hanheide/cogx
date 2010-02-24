/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#ifndef RECOGNIZER_SRV_HPP_
#define RECOGNIZER_SRV_HPP_

#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <cast/core/CASTComponent.hpp>

#include "ObjectRecognizerSrv.hpp" // generated from ice
#include "RecognizerClient.h"

namespace cogx_vision_or {

// ObjectRecognizer is the component that will be created when CAST starts.
// The ICE server interface (ObjectRecognizerI) will be created in start().
class ObjectRecognizer: public cast::CASTComponent, public ObjectRecognizerMethods
{
public:
   // CASTComponent methods
   virtual void configure(const std::map<std::string,std::string> & _config)
         throw(std::runtime_error);
   virtual void start();
   virtual void runComponent();

   // ObjectRecognizerMethods
   virtual long GetSifts(const Video::Image&,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&);
   virtual void FindMatchingObjects(const Video::Image&, const cogx::Math::Rect2&,
         ObjectRecognizerIce::RecognitionResultSeq&);

private:
   std::string m_iceServerName;

private:
   void startIceServer();

};


// The implementation of ObjectRecognizerInterface (ICE).
class ObjectRecognizerI: public ObjectRecognizerIce::ObjectRecognizerInterface
{
private:
   ObjectRecognizer *m_pRecognizer; // could also be ObjectRecognizerMethods

public:
   ObjectRecognizerI(ObjectRecognizer *pRecognizer) {
      m_pRecognizer = pRecognizer;
   }

   virtual long GetSifts(const Video::Image& image,
         ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors,
         const Ice::Current&)
   {
      return m_pRecognizer->GetSifts(image, features, descriptors);
   }

   virtual void FindMatchingObjects(const Video::Image& image, const cogx::Math::Rect2& region,
         ObjectRecognizerIce::RecognitionResultSeq& result, const Ice::Current&)
   {
      m_pRecognizer->FindMatchingObjects(image, region, result);
   }

};


}; // namespace
#endif
