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

#include "pythonproxy.h"
#include "RecognizerClient.h"

namespace cogx { namespace vision {

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
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&);
   virtual void FindMatchingObjects(const Video::Image&,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&);

private:
   std::string m_iceServerName;
   CPyProxy m_pyRecognizer;

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
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::FloatSeq& features, ObjectRecognizerIce::FloatSeq& descriptors,
         const Ice::Current&)
   {
      return m_pRecognizer->GetSifts(image, x0, y0, width, height, features, descriptors);
   }

   virtual void FindMatchingObjects(const Video::Image& image,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq& result, const Ice::Current&)
   {
      m_pRecognizer->FindMatchingObjects(image, x0, y0, width, height, result);
   }

};


};}; // namespace
#endif
