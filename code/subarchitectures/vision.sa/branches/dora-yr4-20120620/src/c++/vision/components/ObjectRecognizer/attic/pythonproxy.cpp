/*
 * @author:  Marko Mahnič
 * @created: feb 2010 
 */

#include "pythonproxy.h"

#include <numpy/arrayobject.h>
#include <dlfcn.h>
#ifndef PYTHON_LIB_NAME
#define PYTHON_LIB_NAME "libpython2.6.so"
#endif

#define log printf
#define println printf

#include "convenience.hpp"

using namespace std;
using namespace VisionData;

class CPyState 
{
   PyInterpreterState *mainInterpreterState;
   PyThreadState *mainThreadState;
   // If a different python interpreter state is needed for each thread
   PyThreadState *runThreadState;
   PyThreadState *filterThreadState;
   void checkMainThread() {
      if (mainThreadState == NULL) {
         // http://bugs.python.org/issue4434
         // quote: """ This works, but I believe that lib-dynload/*.so should depend on
         // libpython2.X.so so this HACK should not be necessary. """
         dlopen(PYTHON_LIB_NAME, RTLD_LAZY | RTLD_GLOBAL);
         Py_Initialize();
         
         // import numpy
         import_array();

         // from http://www.python.org/doc/current/c-api/init.html: 
         //   quote: """It is not safe to call this function when it is unknown which
         //   thread (if any) currently has the global interpreter lock."""
         PyEval_InitThreads();
         mainThreadState = PyThreadState_Get();
         mainInterpreterState = mainThreadState->interp;
         PyEval_ReleaseLock(); // Lock created by PyEval_InitThreads
      }
   }
public:
   CPyState() {
      mainInterpreterState = NULL;
      mainThreadState = NULL;
      runThreadState = NULL;
      filterThreadState = NULL;
      checkMainThread();
   }
   PyThreadState* getRunThread() {
      if (runThreadState == NULL) {
         checkMainThread();
         runThreadState = PyThreadState_New(mainInterpreterState);
      }
      return runThreadState;
   }
   void removeRunThread() {
      if (runThreadState != NULL) {
         PyThreadState_Clear(runThreadState);
         PyThreadState_Delete(runThreadState);
         runThreadState = NULL;
      }
   }
   PyThreadState* getFilterThread() {
      if (filterThreadState == NULL) {
         checkMainThread();
         filterThreadState = PyThreadState_New(mainInterpreterState);
      }
      return filterThreadState;
   }
   void removeFilterThread() {
      if (filterThreadState!= NULL) {
         PyThreadState_Clear(filterThreadState);
         PyThreadState_Delete(filterThreadState);
         filterThreadState = NULL;
      }
   }
};

CPyState RPyState;


CPyProxy::CPyProxy()
{
   m_SiftExtractor = "GPU";
   m_SiftMatcher = "NUMPY";
   m_ModelDir = "";
   m_ModelNames.clear();
}

void CPyProxy::configureRecognizer(const map<string,string> & _config)
{
   DTRACE("CPyProxy::configureRecognizer");
   map<string,string>::const_iterator it;

   if((it = _config.find("--modeldir")) != _config.end())
   {
      istringstream istr(it->second);
      istr >> m_ModelDir;
   }

   if((it = _config.find("--models")) != _config.end())
   {
      istringstream istr(it->second);
      string label;
      while(istr >> label) m_ModelNames.push_back(label);

      ostringstream ostr;
      for(size_t i = 0; i < m_ModelNames.size(); i++)
         ostr << " '" << m_ModelNames[i] << "'";
      log("Detecting objects: %s", ostr.str().c_str());
   }

   if((it = _config.find("--matcher-cuda")) != _config.end()) m_SiftMatcher = "CUDA";
   else if((it = _config.find("--matcher-numpy")) != _config.end()) m_SiftMatcher = "NUMPY";
}

void CPyProxy::initModule()
{
   DTRACE("CPyProxy::initModule");
   ostringstream pycode;
   pycode
      << "import castinit, numpy" << endl
      << "from castmodule import mod_recognizer as main" << endl
      << "from ObjectRecognizer import objectmodel, objectmatcher" << endl
      << "from ObjectRecognizer.featuresetup import CSiftSetup" << endl;

   if (m_ModelNames.size() > 0) {
      vector<string>::const_iterator it;
      for( it = m_ModelNames.begin(); it != m_ModelNames.end(); it++) {
         pycode
            << "main.Manager.addModel('" << *it << "', '" << m_ModelDir << "')" << endl;
      }
   }

   pycode
      << "main.reconfigSift(extractor=CSiftSetup." << m_SiftExtractor 
      << ", matcher=CSiftSetup." << m_SiftMatcher << ")" << endl;

   PyGILState_STATE state = PyGILState_Ensure();
   PyRun_SimpleString(pycode.str().c_str());
   PyGILState_Release(state);
}


// @returns pMatches: matches found with pose estimation
PyObject* CPyProxy::processImage(const Video::Image &image, const int *region)
{
   DTRACE("CPyProxy::processImage");
   PyObject *pMatches = NULL;
   int nchn = 3; // This should be a field in Image...
   int ndims;
   npy_intp dims[] = {image.height, image.width, nchn};
   log("processImage %dx%d(%d)\n", dims[1], dims[0], dims[2]);

   if (nchn == 3) ndims = 3;
   else if (nchn == 1) ndims = 2;

   // matches = ObjectRecognizer.main.findMatchingObject(image)
   PyObject *pName = PyString_FromString("castmodule.mod_recognizer");
   PyObject *pModule = PyImport_Import(pName);
   Py_DECREF(pName);
   if (pModule) {
      PyObject *pFunc = NULL;
      // TODO: string funcname;
      //if (testmode == 1)
      //   pFunc = PyObject_GetAttrString(pModule, "testCppInterface");
      //else if (testmode == 2)
      //   pFunc = PyObject_GetAttrString(pModule, "findMatchingObject");
      //else if (testmode == 3)
      //   pFunc = PyObject_GetAttrString(pModule, "testMatching");
      //else
         pFunc = PyObject_GetAttrString(pModule, "findMatchingObject");

      if (! pFunc || !PyCallable_Check(pFunc)) {
         log("Failed to find the required python function.\n");
      }
      else {
         PyObject *pArgs = PyTuple_New(2);
         PyObject* pimarray = NULL;

         // HACK: no need to copy: vision::image byte order is the same as numpy-array byte order (tested 20090608)
         void *pdata = const_cast<unsigned char*>(&(image.data[0]));
         pimarray = PyArray_SimpleNewFromData(ndims, dims, NPY_UBYTE, pdata);
         PyTuple_SetItem(pArgs, 0, pimarray);

         if (region[2] > 0 && region[3] > 0) {
            PyObject *pRegion = Py_BuildValue("(iiii)", region[0], region[1],
                  region[0] + region[2], region[1] + region[3]);
            PyTuple_SetItem(pArgs, 1, pRegion);
            log("Processing region x=%d, y=%d, w=%d, h=%d\n", region[0], region[1],  region[2],  region[3]);
         }
         else {
            log("No region defined.\n");
            PyTuple_SetItem(pArgs, 1, Py_BuildValue("")); // None!
         }

         pMatches = PyObject_CallObject(pFunc, pArgs);
         Py_DECREF(pArgs);
      }
      Py_XDECREF(pFunc);
      Py_DECREF(pModule);
   }

   return pMatches;
}


// parse the result and update the RecogntionTask
// pMatches: see <url:#r=MatchesFormat>
void CPyProxy::parseMatches(PyObject *pMatches, ObjectRecognitionMatchPtr &imatch)
{
   DTRACE("CPyProxy::parseMatches ObjectRecognitionMatchPtr");
   if (PyTuple_Check(pMatches)) {
      int len = PyTuple_Size(pMatches);
      ostringstream ostr;
      if (len != 3) {
         println("Tuple of wrong size. Something went wrong.");
      }
      else {
         // if (testmode) ostr << "Result (" << len << ":";
         for (int i = 0; i < 3; i++) {
            PyObject *pList = PyTuple_GetItem(pMatches, i);
            //if (testmode)
            //   ostr << (PyList_Check(pList) ? " list " : " NOT-A-LIST ");
            int len = PyList_Size(pList);
            if (i == 0) {
               for (int j = 0; j < len; j++) {
                  PyObject *label = PyList_GetItem(pList, j);
                  imatch->objectId.push_back(string(PyString_AsString(label)));
               }
            }
            else if (i == 1) {
               for (int j = 0; j < len; j++) {
                  PyObject *prob = PyList_GetItem(pList, j);
                  imatch->probability.push_back(PyFloat_AsDouble(prob));
               }
            }
            else if (i == 2) {
               // TODO Pose PDFs: list of arrays
            }
         }
         //if (testmode) {
         //   ostr << ")";
         //   println(ostr.str());
         //}
      }
   }
}


// pMatches format: ( [name, ...], [prob, ...] [pose, ...] )     id=MatchesFormat
//    where pose = ndarray(n, 4, float), line = [ %, phi, lambda, theta ]
//    the last pose may be None (for unknown object)
//    TODO: pose = list_[%, rotation_matrix] instead of array_[[ %, phi, lambda, theta ]]
void CPyProxy::parseMatches(PyObject *pMatches, ObjectRecognizerIce::RecognitionResultSeq& results)
{
   DTRACE("CPyProxy::parseMatches RecognitionResultSeq");
   if (PyTuple_Check(pMatches)) {
      int len = PyTuple_Size(pMatches);
      ostringstream ostr;
      if (len != 3) {
         println("Tuple of wrong size. Something went wrong.");
      }
      else {
         PyObject *pNames = PyTuple_GetItem(pMatches, 0);
         PyObject *pProbs = PyTuple_GetItem(pMatches, 1);
         PyObject *pPoses = PyTuple_GetItem(pMatches, 2);
         results.resize(len);
         for (int i = 0; i < len; i++) {
            ObjectRecognizerIce::RecognitionResult &res = results.at(i);

            PyObject *label = PyList_GetItem(pNames, i);
            res.label = string(PyString_AsString(label));

            PyObject *prob = PyList_GetItem(pProbs, i);
            res.probability = PyFloat_AsDouble(prob);
            
            PyObject *posearray = PyList_GetItem(pPoses, i);
            if (posearray != Py_None) {
               int nposes = PyArray_DIM(posearray, 0);
               int nattrs = PyArray_DIM(posearray, 1); // nattrs should be 4
               res.poses.resize(nposes);
               res.posePd.resize(nposes);
               for (int ip = 0; ip < nposes; ip++) {
                  res.posePd[ip] = *(float*)PyArray_GETPTR2(posearray, ip, 0);
                  res.poses[ip].phi = *(float*)PyArray_GETPTR2(posearray, ip, 1);
                  res.poses[ip].lambda = *(float*)PyArray_GETPTR2(posearray, ip, 2);
                  res.poses[ip].theta = *(float*)PyArray_GETPTR2(posearray, ip, 3);
               }
            }
         }
      }
   }
}

