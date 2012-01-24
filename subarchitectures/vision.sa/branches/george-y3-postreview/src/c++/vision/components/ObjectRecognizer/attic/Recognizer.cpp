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
   testmode = 0; // Set manually for debugging
   videoServerName = "";
   videoHostName = "localhost";
}

void CRecognizer::configure(const map<string,string> & _config)
{
   string dir(".");
   vector<string> models;
   map<string,string>::const_iterator it;

   if((it = _config.find("--videoname")) != _config.end()) {
      videoServerName = it->second;
   }

   if((it = _config.find("--videohost")) != _config.end()) {
      videoHostName = it->second;
   }

   //if((it = _config.find("--testmode")) != _config.end())
   //{
   //   string mode;
   //   istringstream istr(it->second);
   //   istr >> mode;
   //   if (mode == "1") testmode = 1;
   //   else if (mode == "2") testmode = 2;
   //   else if (mode == "3") testmode = 3;
   //   else testmode = 0;
   //   log("TEST MODE: %ld", testmode);
   //}

   if((it = _config.find("--modeldir")) != _config.end())
   {
      istringstream istr(it->second);
      istr >> dir;
   }

   if((it = _config.find("--models")) != _config.end())
   {
      istringstream istr(it->second);
      string label;
      while(istr >> label) models.push_back(label);

      ostringstream ostr;
      for(size_t i = 0; i < models.size(); i++)
         ostr << " '" << models[i] << "'";
      log("detecting objects: %s", ostr.str().c_str());
   }

   ostringstream pycode;
   pycode
      << "import castinit, numpy" << endl
      << "from castmodule import mod_recognizer as main" << endl
      << "from ObjectRecognizer import objectmodel, objectmatcher" << endl
      << "from ObjectRecognizer.featuresetup import CSiftSetup" << endl;

   if (models.size() > 0) {
      vector<string>::const_iterator it;
      for( it = models.begin(); it != models.end(); it++) {
         pycode
            << "main.Manager.addModel('" << *it << "', '" << dir << "')" << endl;
      }
   }

   string siftExtr = "GPU";
   string siftMatch = "NUMPY";

   if((it = _config.find("--matcher-cuda")) != _config.end()) siftMatch = "CUDA";
   else if((it = _config.find("--matcher-numpy")) != _config.end()) siftMatch = "NUMPY";

   pycode
      << "main.reconfigSift(extractor=CSiftSetup." << siftExtr 
      << ", matcher=CSiftSetup." << siftMatch << ")" << endl;

   PyGILState_STATE state = PyGILState_Ensure();
   PyRun_SimpleString(pycode.str().c_str());
   PyGILState_Release(state);
}

void CRecognizer::start()
{
   log("Recognizer starting");

   // get connection to the video server
   videoServer = getIceServer<Video::VideoInterface>(videoServerName);

   // Global change filter expecting message from vision.sa
   // (ID set in CAST file, subarchitecture entry)
   addChangeFilter(
         createLocalTypeFilter<ObjectRecognitionTask>(cdl::ADD),
         new MemberFunctionChangeReceiver<CRecognizer>(
            this, &CRecognizer::onAddRecognitionTask)
         );

   //if (testmode) {
   //   addChangeFilter(
   //        createLocalTypeFilter<ObjectRecognitionTask>(cdl::DELETE),
   //        new MemberFunctionChangeReceiver<CRecognizer>(
   //           this, &CRecognizer::onDeleteRecognitionTask)
   //        );

   //   addChangeFilter(
   //        createLocalTypeFilter<ObjectRecognitionTask>(cdl::OVERWRITE),
   //        new MemberFunctionChangeReceiver<CRecognizer>(
   //           this, &CRecognizer::onChangeRecognitionTask)
   //        );
   //}
}

//void CRecognizer::_test_addRecognitionTask()
//{
//   string id(newDataID());
//   ObjectRecognitionTaskPtr task = new ObjectRecognitionTask();
//   println("Adding new task");
//   addToWorkingMemory(id, getSubarchitectureID(), task);
//   sleepComponent(5000);
//}

static int Processing = 0;
void CRecognizer::runComponent()
{
   sleepComponent(2000);

   while(isRunning()) {
      //if (testmode && ! Processing)
      //   _test_addRecognitionTask();

      sleepComponent(100);
   }
   log("runComponent Done.");
}

void CRecognizer::onAddRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
   log("OR: Recognition task recieved.");
   // TODO: add to queue and process in main loop
   doRecognize(_wmc);
}

//void CRecognizer::onDeleteRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
//{
//   log("Recognition task removed.");
//}

//void CRecognizer::onChangeRecognitionTask(const cdl::WorkingMemoryChange & _wmc)
//{
//   log("Recognition task modified.");
//   ObjectRecognitionTaskPtr pcmd = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
//   std::vector<string>::iterator itstr;
//   std::vector<double>::iterator itdbl;

//   ostringstream msg;
//   std::vector<VisionData::ObjectRecognitionMatchPtr>::iterator pmatch;
//   for (pmatch = pcmd->matches.begin(); pmatch < pcmd->matches.end(); pmatch++) {
//      println("Match for '%s' ID: [%s]", (*pmatch)->sourceType.c_str(), (*pmatch)->sourceId.id.c_str());
//      msg.str("");
//      for (itstr = (*pmatch)->objectId.begin(); itstr != (*pmatch)->objectId.end(); itstr++) {
//         msg << *itstr << ", ";
//      }
//      println("Labels: %s", msg.str().c_str());

//      msg.str("");
//      for (itdbl = (*pmatch)->probability.begin(); itdbl != (*pmatch)->probability.end(); itdbl++) {
//         msg << *itdbl << ", ";
//      }
//      println("Probabilities: %s", msg.str().c_str());
//   }

//   // TODO: dump the request!
//   //if (testmode) {
//   //   sleepComponent(500);
//   //   deleteFromWorkingMemory(_wmc.address);
//   //}
//}

void CRecognizer::abortRecognition(const cast::cdl::WorkingMemoryChange & _wmc, ObjectRecognitionTaskPtr pcmd)
{
   // TODO set some flag in ObjectRecognitionTask?
   log("Recognition task aborted.");
   overwriteWorkingMemory(_wmc.address, pcmd);
}

// @returns pMatches: matches found with pose estimation
PyObject* CRecognizer::pyProcessImage(Video::Image &image, const int *region)
{
   PyObject *pMatches = NULL;
   int nchn = 3; // This should be a field in Image...
   int ndims;
   npy_intp dims[] = {image.height, image.width, nchn};

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
         log("Failed to find the required python function.");
      }
      else {
         PyObject *pArgs = PyTuple_New(2);
         PyObject* pimarray = NULL;

         // HACK: no need to copy: vision::image byte order is the same as numpy-array byte order (tested 20090608)
         pimarray = PyArray_SimpleNewFromData(ndims, dims, NPY_UBYTE, &(image.data[0]));
         PyTuple_SetItem(pArgs, 0, pimarray);

         if (region[2] > 0 && region[3] > 0) {
            PyObject *pRegion = Py_BuildValue("(iiii)", region[0], region[1],
                  region[0] + region[2], region[1] + region[3]);
            PyTuple_SetItem(pArgs, 1, pRegion);
            log("Processing region x=%d, y=%d, w=%d, h=%d", region[0], region[1],  region[2],  region[3]);
         }
         else {
            log("No region defined.", region[0], region[1],  region[2],  region[3]);
            PyTuple_SetItem(pArgs, 1, Py_BuildValue("")); // None!
         }

         Processing = 1;
         pMatches = PyObject_CallObject(pFunc, pArgs);
         Py_DECREF(pArgs);
         Processing = 0;
      }
      Py_XDECREF(pFunc);
      Py_DECREF(pModule);
   }

   return pMatches;
}

// parse the result and update the RecogntionTask
// Format: ( [name, ...], [prob, ...] [pose, ...] )
//    where pose = ndarray(n, 4), line = [ %, phi, lambda, theta ]
//    the last pose may be None (for unknown object)
void CRecognizer::pyParseMatches(PyObject *pMatches, ObjectRecognitionMatchPtr &imatch)
{
   if (PyTuple_Check(pMatches)) {
      int len = PyTuple_Size(pMatches);
      ostringstream ostr;
      if (len != 3) {
         println("Tuple of wrong size. Something went wrong.");
      }
      else {
         if (testmode) ostr << "Result (" << len << ":";
         for (int i = 0; i < len; i++) {
            PyObject *pList = PyTuple_GetItem(pMatches, i);
            if (testmode)
               ostr << (PyList_Check(pList) ? " list " : " NOT-A-LIST ");
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
         if (testmode) {
            ostr << ")";
            println(ostr.str());
         }
      }
   }
}

void CRecognizer::doRecognize(const cdl::WorkingMemoryChange & _wmc)
{
   ObjectRecognitionTaskPtr pcmd;
   try {
      pcmd = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
   }
   catch (DoesNotExistOnWMException e) {
      log("ObjectRecognitionTask ID: %s was removed before it could be processed", _wmc.address.id.c_str());
      abortRecognition(_wmc, pcmd);
      return;
   }

   Video::Image img;
   int region[4]; // x,y,w,h
   string source = "";
   bool foundRegion = false;
   cdl::WorkingMemoryAddress soiProcessed;

   if (pcmd->protoObjectAddr.id.length() > 0) {
      try {
         ProtoObjectPtr objPtr = getMemoryEntry<VisionData::ProtoObject>(pcmd->protoObjectAddr);
         source = "ProtoObj";
         foundRegion = true;
         img = objPtr->image;
      }
      catch (DoesNotExistOnWMException e) {
         log("ProtoObject ID: %s was removed before it could be processed", pcmd->protoObjectAddr.id.c_str());
         abortRecognition(_wmc, pcmd);
         return;
      }
   }
   else {
      std::vector<string>::iterator s;
      std::vector<Video::Image>::iterator pim;
      Video::ImageSeq images;
      videoServer->getImages(images);

      if (images.size() < 1) {
         println("No images are available.");
         abortRecognition(_wmc, pcmd);
         return;
      }

      for (int i = 0; i < 4; i++) region[i] = -1;

      if (pcmd->soiIds.size() > 1)
         log("Multiple SOI request. Only the first SOI will be processed.");
      else if (pcmd->soiIds.size() == 0) {
         source = "Image";
         log("Request without SOI.");
         for (pim = images.begin(); pim != images.end(); pim++) {
            if (pim->width < 8 || pim->height < 8) continue;
            log("The whole image from camera %d will be processed.", pim->camId);
            img = *pim;
            foundRegion = true;
         }
      }
      else {
         source = "SOI";
         s = pcmd->soiIds.begin();
         soiProcessed.subarchitecture = getSubarchitectureID();
         soiProcessed.id = *s;
         SOIPtr psoi;
         try {
            psoi = getMemoryEntry<SOI>(soiProcessed);
         }
         catch (DoesNotExistOnWMException e) {
            log("SOI ID: %s was removed before it could be processed", soiProcessed.id.c_str());
            abortRecognition(_wmc, pcmd);
            return;
         }
         for (pim = images.begin(); pim != images.end(); pim++) {
            if (pim->width < 8 || pim->height < 8) continue;
            // Project on all available cameras, select best
            ROIPtr roi = projectSOI( pim->camPars, *psoi); 

            // clip to image
            if (roi->rect.pos.x < 0) {
               roi->rect.width -= roi->rect.pos.x;
               roi->rect.pos.x = 0;
            }
            if (roi->rect.pos.y < 0) {
               roi->rect.height -= roi->rect.pos.y;
               roi->rect.pos.y = 0;
            }
            if (roi->rect.pos.x + roi->rect.width > pim->width) {
               roi->rect.width -= (roi->rect.pos.x + roi->rect.width - pim->width);
            }
            if (roi->rect.pos.y + roi->rect.height > pim->height) {
               roi->rect.height -= (roi->rect.pos.y + roi->rect.height - pim->height);
            }
            if (roi->rect.width <= 4 || roi->rect.height <= 4) continue;
            if (roi->rect.width * roi->rect.height > region[2] * region[3]) {
               region[0] = roi->rect.pos.x;
               region[1] = roi->rect.pos.y;
               region[2] = roi->rect.width;
               region[3] = roi->rect.height;
               img = *pim;
               foundRegion = true;
            }
         }
      }
   }

   if ( ! foundRegion) {
      log("No visible region could be found. (tried: '%s')", source.c_str());
      abortRecognition(_wmc, pcmd);
      return;
   }

   PyGILState_STATE state = PyGILState_Ensure();

   PyObject *pMatches = pyProcessImage(img, region);
   if (pMatches == NULL) {
      abortRecognition(_wmc, pcmd);
   }
   else {
      VisionData::ObjectRecognitionMatchPtr imatch = new VisionData::ObjectRecognitionMatch;
      pyParseMatches(pMatches, imatch);
      Py_DECREF(pMatches);

      imatch->sourceType = source;
      imatch->sourceId = soiProcessed;
      pcmd->matches.push_back(imatch);
      overwriteWorkingMemory(_wmc.address, pcmd);
   }

   PyGILState_Release(state);

   Processing = 0;
}

// vim:set fileencoding=utf-8 sw=3 ts=8 et:
