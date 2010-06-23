// vim:set fileencoding=utf-8 sw=3 ts=8 et:vim
/*
 * @author:  Marko Mahnič
 * @created: jul 2009 
 */

#include <iostream>  
#include <cmath>
#include <Python.h>
#include <numpy/arrayobject.h>
#include <dlfcn.h>

#include <gl.h>
#include <SiftGPU/SiftGPU.h>

#define PyArray_NDIMS(obj) (((PyArrayObject *)(obj))->nd)

class CSift
{
   int initialized;
   SiftGPU* _extract;
   SiftMatchGPU* _match;
public:
   int maxMatch;
   CSift() {
      initialized = 0;
      _extract = NULL;
      _match = NULL;
      maxMatch = 4096;
   }
   // Cleanup when library is terminated and global objects are destroyed.
   ~CSift() {
      // if (_match) { delete _match; _match = NULL; }
      // BUG: CAUSES SEGFAULT. if (_extract) { delete _extract; _extract = NULL; }
   }
   int checkInit()
   {
      if (initialized < 0) return 0;
      if (!initialized) {
         _extract = new SiftGPU();

         // TODO: parameters
         // -d 3: best value for the number of scales per octave (lowe04ijcv)
         // -no 8: limit the number of octaves processed (default: unlimited)
         // -fo 0: first octave; -1 to upscale initial image
         // char * argv[] = {"-fo", "-1",  "-v", "1"};// "-cg"
         char * argv[] = {"-fo", "-1",  "-v", "1",  "-d", "3", "-no", "8"}; //
         int argc = sizeof(argv)/sizeof(char*);
         _extract->ParseParam(argc, argv);

         if(_extract->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
            initialized = -1;
            return 0;
         }

         _match = new SiftMatchGPU(maxMatch); // TODO parameter
         _match->VerifyContextGL(); //must call once

         initialized = 1;
      }
      return initialized;
   }
   SiftGPU* extract() {
      return _extract;
   }
   SiftMatchGPU* match() {
      return _match;
   }
};
static CSift Sift;

static
PyObject* PySG_ExtractFeatures(PyObject *self, PyObject *args)
{
   PyObject* pimg = NULL;
   if (!PyArg_ParseTuple(args, "O", &pimg))
      return NULL;
   if (!PyArray_Check(pimg)) {
      PyErr_SetString(PyExc_ValueError, "siftGpu-ExtractFeatures: Expecting a PyArray Object");
      return NULL;
   }
   int h = PyArray_DIM(pimg, 0);
   int w = PyArray_DIM(pimg, 1); 
   int nc = 1;
   if (PyArray_NDIMS(pimg) > 2) nc = PyArray_DIM(pimg, 2); 
   if (nc != 1 && nc != 3) {
      PyErr_SetString(PyExc_ValueError, "siftGpu-ExtractFeatures: Expecting a RGB or GS Image (dtype=uchar)");
      return NULL;
   }

   if (!Sift.checkInit()) {
      PyErr_SetString(PyExc_ValueError, "siftGpu: Initialization error");
      return NULL;
   }
   // GLUT_RGB = 0
   // Matches OpenGL's right now.
#define IL_COLOUR_INDEX     0x1900
#define IL_COLOR_INDEX      0x1900
#define IL_RGB              0x1907
#define IL_RGBA             0x1908
#define IL_BGR              0x80E0
#define IL_BGRA             0x80E1
#define IL_LUMINANCE        0x1909
#define IL_LUMINANCE_ALPHA  0x190A
   int il = nc == 3 ? IL_BGR : IL_LUMINANCE;
   void *data = PyArray_DATA((PyArrayObject*) pimg);
   int rv = Sift.extract()->RunSIFT(w, h, (unsigned char*) data, il, GL_UNSIGNED_BYTE);
   // Py_DECREF(pcont)

   // convert sift data to 2 ndarrays: keypoints (x,y,scale,orientation,...) + features (128D)
   int nf = Sift.extract()->GetFeatureNum();
   int nattr = 4;
   npy_intp dims[2] = { nf, nattr };
   PyObject *pKeypoints = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
   float *kp_data = (float*) PyArray_DATA((PyArrayObject*) pKeypoints);

   dims[1] = 128;
   PyObject *pDescript = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
   unsigned char *dscr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) pDescript);

   Sift.extract()->GetFeatureVector((::SiftGPU::SiftKeypoint*)kp_data, (float*)dscr_data);
   Sift.extract()->AllocatePyramid(8, 8); // release GPU memory
   
   // pass the results to Python
   PyObject *pRes = PyTuple_New(2);
   PyTuple_SetItem(pRes, 0, pKeypoints);
   PyTuple_SetItem(pRes, 1, pDescript);

   return pRes;
}

static
int checkDescriptorArray(PyObject *arr)
{
   if (!PyArray_Check(arr) || PyArray_NDIMS(arr) != 2 || PyArray_DIM(arr, 1) != 128) {
      PyErr_SetString(PyExc_ValueError, 
            "siftGpu-MatchFeatures: Expecting a PyArray Object of size (N,128), dtype=float");
      return -1;
   }
   return PyArray_DIM(arr, 0);
}

static
int uploadDescriptorArray(int index, PyObject *parr)
{
   int np = PyArray_DIM(parr, 0);
   // TODO: PyArrayObject* pcont = PyArray_ContiguousFromObject(pobj, PyArray_FLOAT, 2, 2);
   //   if (pcont != NULL)
   Sift.match()->SetDescriptors(index, np, (float*) PyArray_DATA(parr), -1);
   // Py_DECREF(pcont)

   return np; // count of copied elements
}

static
PyObject* PySG_MatchDescriptors(PyObject *self, PyObject *args)
{
   PyObject* ndaDescrA = NULL;
   PyObject* ndaDescrB = NULL;
   if (!PyArg_ParseTuple(args, "OO", &ndaDescrA, &ndaDescrB))
      return NULL;
   int nf1 = checkDescriptorArray(ndaDescrA);
   int nf2 = nf1 < 0 ? -1 : checkDescriptorArray(ndaDescrB);
   if (nf1 < 0 || nf2 < 0) return NULL;

   if (!Sift.checkInit()) {
      PyErr_SetString(PyExc_ValueError, "siftGpu: Initialization error");
      return NULL;
   }

   int na = uploadDescriptorArray(0, ndaDescrA);
   int nb = uploadDescriptorArray(1, ndaDescrB);

   int nmatch = na < Sift.maxMatch ? na : Sift.maxMatch;
   std::cout << "Will Match Features: " << nmatch << "/" << na << ":" << nb << std::endl;
   int (*match_buf)[2] = new int[nmatch][2];
   nmatch = Sift.match()->GetSiftMatch(nmatch, match_buf); // , 1024, 0.8, 0); TODO: parameters
   std::cout << "Matches found: " << nmatch << std::endl;
 
   int type = PyArray_INT;
   if (sizeof(int) == 2) type = PyArray_SHORT;

   npy_intp dims[2] = { nmatch, 3 };
   PyObject *pPairs = PyArray_SimpleNew(2, dims, type);
   void *pair_data = (void *) PyArray_DATA((PyArrayObject*) pPairs);
   memcpy(pair_data, match_buf, sizeof(match_buf[0]) * nmatch);
   delete match_buf;

   return pPairs;
}

/* ****************************************************************************** 
 * Python Module Stuff
 *
 * *****************************************************************************/

PyMethodDef methods[] = {
    {"extractFeatures", PySG_ExtractFeatures, METH_VARARGS, 
       "extractFeatures(image)\n   image - a numpy ndaray\n"
       "Returns a tuple (keypoints, descriptors) with SIFT features\n"
       "extracted from the image."
    },
    {"matchDescriptors", PySG_MatchDescriptors, METH_VARARGS, 
       "extractFeatures(descriptorsA, descriptorsB)\n   descriptorsX - a numpy ndaray Nx128\n"
       "Returns an array Nx2 with indices of matching descriptors."
    },
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initsiftgpu()
{
    (void) Py_InitModule("siftgpu", methods);   
    import_array();
}

