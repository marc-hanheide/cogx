/*
 * @author:  Marko Mahnič
 * @created: jun 2009 
 */
   
// Python, NumPy
#include <Python.h>
#include <numpy/arrayobject.h>
#include <dlfcn.h>
#ifndef PYTHON_LIB_NAME
#define PYTHON_LIB_NAME "libpython2.6.so"
#endif

// cast
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include <Video.hpp>

#include "Recognizer.h"
#include "../../VisionUtils.h"

using namespace std;
using namespace cast;
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
         // """ This works, but I believe that lib-dynload/*.so should depend on
         // libpython2.X.so so this HACK should not be necessary. """
         dlopen(PYTHON_LIB_NAME, RTLD_LAZY | RTLD_GLOBAL);
         Py_Initialize();
         
         // import numpy
         import_array();

         // from http://www.python.org/doc/current/c-api/init.html: 
         //   It is not safe to call this function when it is unknown which thread (if any)
         //   currently has the global interpreter lock.
         PyEval_InitThreads();
         mainThreadState = PyThreadState_Get();
         mainInterpreterState = mainThreadState->interp;
         PyEval_ReleaseLock(); // Lock Created by PyEval_InitThreads
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

extern "C" {
   cast::CASTComponentPtr newComponent()
   {
      return new CRecognizer();
   }
}

CRecognizer::CRecognizer()
{
}

void CRecognizer::configure(const map<string,string> & _config)
{
   // first let the base classes configure themselves
   configureVideoCommunication(_config);

   PyGILState_STATE state = PyGILState_Ensure();
   PyRun_SimpleString("import castinit");
   PyRun_SimpleString("import numpy");
   PyRun_SimpleString("import ObjectRecognizer");
   PyRun_SimpleString("import ObjectRecognizer.main as main");
   PyRun_SimpleString("main.Manager.addModel('TwEarlGrey', '/home/mmarko/Documents/doc/Devel/CogX/code/apps/xdata/models/TwEarlGrey')");
   PyGILState_Release(state);
 
   /*
   map<string,string>::const_iterator it;
   if((it = _config.find("--detect-labels")) != _config.end())
   {
      istringstream istr(it->second);
      string label;
      while(istr >> label)
         m_labels.push_back(label);

      ostringstream ostr;
      for(size_t i = 0; i < m_labels.size(); i++)
         ostr << " '" << m_labels[i] << "'";
      log("detecting objects: %s", ostr.str().c_str());
   }
   */
}

void CRecognizer::start()
{
   log("Recognizer starting");
   startVideoCommunication(*this);

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   addChangeFilter(
         createLocalTypeFilter<ObjectRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<CRecognizer>(
            this, &CRecognizer::onRecognitionTaskAdded)
         );

   addChangeFilter(
        createLocalTypeFilter<ObjectRecognitionTask>(cdl::DELETE),
        new MemberFunctionChangeReceiver<CRecognizer>(
           this, &CRecognizer::onRecognitionTaskRemoved)
        );
}

void CRecognizer::_test_addRecognitionTask()
{
   string id(newDataID());
   ObjectRecognitionTaskPtr task = new ObjectRecognitionTask();
   println("Adding new task");
   addToWorkingMemory(id, getSubarchitectureID(), task);
}

// #include "embedded_py.inc"
void CRecognizer::runComponent()
{
   log("Recognizer runComponent");

   sleepComponent(2000);
   log("Recognizer component thread is awake.");

   while(1) {
      _test_addRecognitionTask();

      PyGILState_STATE state = PyGILState_Ensure();
      PyRun_SimpleString(
            "from time import time,ctime,sleep\n"
            "print 'Today is', ctime(time())\n"
            "sleep(0.001)\n" // This will allow other python threads to run
            );
      PyGILState_Release(state);
      sleepComponent(2000); // This will keep the GIL blocked if not explicitly released
   }
}

void CRecognizer::onRecognitionTaskAdded(const cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task added.");
   ObjectRecognitionTaskPtr cmd = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
   std::vector<string>::iterator s;
   Video::ImageSeq images;
   getImages(images);

   // ATM: Ignore SOIs, use only first camera, whole image
   // TODO: Notify on failure!
   if (images.size() < 1) return;
   Video::Image img = images[0];
   if (img.width < 8 || img.height < 8) return;

   //for (s = cmd->soiIds.begin(); s != cmd->soiIds.end(); s++) {
   //   println(s->c_str());
   //   cdl::WorkingMemoryAddress addr;
   //   addr.subarchitecture = getSubarchitectureID();
   //   addr.id = *s;
   //   SOIPtr soi = getMemoryEntry<SOI>(addr[> CHECK Create a valid address <]);
   //   std::vector<Video::Image>::iterator pim;
   //   for (pim = images.begin(); pim != images.end(); pim++) {
   //      // TODO Project on all available cameras, select best
   //      ROIPtr roi = projectSOI( pim->camPars, *soi); 
   //   }
   //}

   // TODO: Process each SOI and write the result back

   PyGILState_STATE state = PyGILState_Ensure();

   PyObject* pimarray = NULL;
   int nchn = 3; // This should be a field in Image...
   int ndims;
   npy_intp dims[] = {img.height, img.width, nchn};

   if (nchn == 3) ndims = 3;
   else if (nchn == 1) ndims = 2;

   // matches = ObjectRecognizer.main.findMatchingObject(image)
   PyObject *pName = PyString_FromString("ObjectRecognizer.main");
   PyObject *pModule = PyImport_Import(pName);
   Py_DECREF(pName);
   if (pModule) {
      PyObject *pFunc = PyObject_GetAttrString(pModule, "findMatchingObject");
      if (pFunc && PyCallable_Check(pFunc)) {
         PyObject *pArgs = PyTuple_New(1);

         // no need to copy: vision::image byte order is the same as numpy-array byte order (tested 20090608)
         pimarray = PyArray_SimpleNewFromData(ndims, dims, NPY_UBYTE, &(img.data[0]));
         PyTuple_SetItem(pArgs, 0, pimarray);

         PyObject *pMatches = PyObject_CallObject(pFunc, pArgs);
         Py_DECREF(pArgs);
         // TODO: parse the result and update the RecogntionTask
         if (pMatches != NULL) Py_DECREF(pMatches);
      }
      else 
         println("Failed to find the required python function.");

      Py_XDECREF(pFunc);
      Py_DECREF(pModule);
   }

   PyGILState_Release(state);

   sleepComponent(500);
   deleteFromWorkingMemory(_wmc.address);
}

void CRecognizer::onRecognitionTaskRemoved(const cdl::WorkingMemoryChange & _wmc)
{
   log("Recognition task removed.");
}

// vim:set fileencoding=utf-8 sw=3 ts=8 et:
