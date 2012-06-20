/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#ifndef _PYTHON_PROXY_H_
#define _PYTHON_PROXY_H_

#include <Python.h>
#include <string>
#include <vector>
#include <map>

#include <VideoClient.h>
#include <VisionData.hpp>
#include <ObjectRecognizerSrv.hpp>

class CPyProxy
{
private:
   std::string m_SiftExtractor;
   std::string m_SiftMatcher;
   std::string m_ModelDir;
   std::vector<std::string> m_ModelNames;

public:
   CPyProxy();
   void configureRecognizer(const std::map<std::string,std::string> & _config);
   void initModule();
   PyObject* processImage(const Video::Image &image, const int *region);
   void parseMatches(PyObject *pMatches, VisionData::ObjectRecognitionMatchPtr &result);
   void parseMatches(PyObject *pMatches, ObjectRecognizerIce::RecognitionResultSeq& result);
};

#endif
