/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */
#ifndef RECOGNIZER_SRV_HPP_
#define RECOGNIZER_SRV_HPP_

#include "RecognizerClient.h"
#include "models/ObjectModel.h"
#include "sifts/Features.h"

#include "ObjectRecognizerSrv.hpp" // generated from ice

#include <cast/architecture/ManagedComponent.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <string>
#include <vector>
#include <map>
#include <stdexcept>


namespace cogx { namespace vision {

// CObjectRecognizer is the component that will be created when CAST starts.
// The ICE server interface (ObjectRecognizerI) will be created in start().
class CObjectRecognizer:
   public cast::ManagedComponent,
   public CObjectRecognizerMethods
{
private:
   //CPyProxy m_pyRecognizer;
   CSiftExtractor *m_pSiftExtractor;
   CSiftMatcher   *m_pSiftMatcher;
   std::vector<CObjectModel*> m_models;

   // options
   float m_maxDistance;
   float m_maxAmbiguity;

private:
#ifdef FEAT_VISUALIZATION
   class COrDisplayClient: public cogx::display::CDisplayClient
   {
      CObjectRecognizer* pRecognizer;
   public:
      COrDisplayClient() { pRecognizer = NULL; }
      void setClientData(CObjectRecognizer* pObjectRecognizer) { pRecognizer = pObjectRecognizer; }
      //void handleEvent(const Visualization::TEvent &event); [>override<]
      //std::string getControlState(const std::string& ctrlId); [>override<]
      void createForms();
      void handleForm(const std::string& id, const std::string& partId,
            const std::map<std::string, std::string>& fields); /*override*/
      bool getFormData(const std::string& id, const std::string& partId,
            std::map<std::string, std::string>& fields); /*override*/
   };
   COrDisplayClient m_display;
   cogx::display::CFormValues m_Settings;
#endif

private:
   void startIceServer();
   void loadModels(const std::string& from, const std::vector<std::string>& modelnames);

public:
   CObjectRecognizer();
   ~CObjectRecognizer();

   // CASTComponent methods
   virtual void configure(const std::map<std::string,std::string> & _config)
         throw(std::runtime_error);
   virtual void start();
   virtual void runComponent();

   // CObjectRecognizerMethods
   virtual long GetSifts(const Video::Image&,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::FloatSeq&, ObjectRecognizerIce::FloatSeq&);
   virtual long LoadObjectModel(const std::string& modelPath);
   virtual void FindMatchingObjects(const Video::Image&,
         const int x0, const int y0, const int width, const int height,
         ObjectRecognizerIce::RecognitionResultSeq&);
};


// The implementation of ObjectRecognizerInterface (ICE).
class ObjectRecognizerI: public ObjectRecognizerIce::ObjectRecognizerInterface
{
private:
   CObjectRecognizer *m_pRecognizer; // could also be CObjectRecognizerMethods

public:
   ObjectRecognizerI(CObjectRecognizer *pRecognizer) {
      m_pRecognizer = pRecognizer;
   }

   virtual Ice::Long LoadObjectModel(const std::string& modelPath, const Ice::Current&)
   {
      return m_pRecognizer->LoadObjectModel(modelPath);
   }

   virtual Ice::Long GetSifts(const Video::Image& image,
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
