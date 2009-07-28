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
#include <cutil.h>

#include "cudaImage.h"
#include "cudaSift.h"

#include "featureUploadH.h"

#define PyArray_NDIMS(obj) (((PyArrayObject *)(obj))->nd)

// TODO: cast-global initialization of cuda. What about multiple threads?
static int initialized = 0;
static
void checkInit()
{
   if (!initialized) {
      InitCuda();
      initialized = 1;
   }
}

static
PyObject* PyCS_ExtractFeatures(PyObject *self, PyObject *args)
{
   PyObject* pimg = NULL;
   if (!PyArg_ParseTuple(args, "O", &pimg))
      return NULL;
   if (!PyArray_Check(pimg)) {
      PyErr_SetString(PyExc_ValueError, "cudaSift-ExtractFeatures: Expecting a PyArray Object");
      return NULL;
   }
   int h = PyArray_DIM(pimg, 0);
   int w = PyArray_DIM(pimg, 1); 
   int nc = 1;
   if (PyArray_NDIMS(pimg) > 2) nc = PyArray_DIM(pimg, 2); 
   if (nc != 1 && nc != 3) {
      PyErr_SetString(PyExc_ValueError, "cudaSift-ExtractFeatures: Expecting a RGB or GS Image (dtype=uchar)");
      return NULL;
   }

   checkInit();
   CudaImage img1;
   void *data = PyArray_DATA((PyArrayObject*) pimg);
   AllocCudaImage(&img1, w, h, w, true, true);
   unsigned char *pdata = (unsigned char*) data;
   float *ph = img1.h_data;
   int sz = w*h;
   if (nc == 1) {
      while (sz > 0) {
         *ph = 1.0f/256.0f * (*pdata);
         ph++; pdata++; sz--;
      }
   }
   else {
      // convert RGB to grayscale; FIXME: the correct order of channels might be BGR!
      while (sz > 0) {
         *ph = 1.0f/256.0f * (0.3f * pdata[0] + 0.59f * pdata[1] + 0.11f * pdata[2]);
         ph++; pdata += 3; sz--;
      }
   }
   Download(&img1);
   SiftData siftData1;
   InitSiftData(&siftData1, 128, true, true); 

   /* void ExtractSift(SiftData *siftData, CudaImage *img, int numLayers,
      int numOctaves, double initBlur, float thresh, float subsampling=1.0) */
   int nLayers = 3;
   int nOctaves = 3;
   double initBlur = 0.3f;
   float thresh = 0.04;
   float subsampling = 1.0;
   ExtractSift(&siftData1, &img1, nLayers, nOctaves, initBlur, thresh, subsampling);
   FreeCudaImage(&img1);

   // convert sift data to 2 ndarrays: keypoints (x,y,scale,orientation,...) + features (128D)
   int nf = siftData1.numPts;
   int nattr = 4;
   npy_intp dims[2] = { nf, nattr };
   PyObject *pKeypoints = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
   float *kp_data = (float*) PyArray_DATA((PyArrayObject*) pKeypoints);

   dims[1] = 128;
   PyObject *pDescript = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
   unsigned char *dscr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) pDescript);
   
   int i;
   int lendata = 128 * sizeof(float);
   for (i = 0; i < nf; i++) {
      memcpy(dscr_data, (void*)&siftData1.h_data[i].data[0], lendata);
      dscr_data += lendata;
      *kp_data++ = siftData1.h_data[i].xpos;
      *kp_data++ = siftData1.h_data[i].ypos;
      *kp_data++ = siftData1.h_data[i].scale;
      *kp_data++ = siftData1.h_data[i].orientation;
         if (i < 0) 
         printf("%.2f %.2f %.2f %.2f\n", 
               siftData1.h_data[i].xpos, siftData1.h_data[i].ypos,
               siftData1.h_data[i].scale,siftData1.h_data[i].orientation);
   }
   FreeSiftData(&siftData1);

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
      PyErr_SetString(PyExc_ValueError, "cudaSift-MatchFeatures: Expecting a PyArray Object of size (N,128)");
      return -1;
   }
   return PyArray_DIM(arr, 0);
}

static
PyObject* PyCS_MatchDescriptors(PyObject *self, PyObject *args)
{
   PyObject* ndaDescrA = NULL;
   PyObject* ndaDescrB = NULL;
   if (!PyArg_ParseTuple(args, "OO", &ndaDescrA, &ndaDescrB))
      return NULL;
   int nf1 = checkDescriptorArray(ndaDescrA);
   int nf2 = checkDescriptorArray(ndaDescrB);
   if (nf1 < 0 || nf2 < 0) return NULL;

   checkInit();
   SiftData siftData1, siftData2;
   int i;
#define ROUND(n) ((n + 7) / 8) * 8
   InitSiftData(&siftData1, ROUND(nf1), true, true); 
   InitSiftData(&siftData2, ROUND(nf2), true, true); 
#undef ROUND

   int lendata = 128 * sizeof(float);
   memset((void*)&siftData1.h_data[0], 0, lendata*nf1);
   unsigned char *dscr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) ndaDescrA);
   for (i = 0; i < nf1; i++) {
      memcpy((void*)&siftData1.h_data[i].data[0], dscr_data, lendata);
      dscr_data += lendata;
   }
   siftData1.numPts = nf1;

   memset((void*)&siftData2.h_data[0], 0, lendata*nf2);
   dscr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) ndaDescrB);
   for (i = 0; i < nf2; i++) {
      memcpy((void*)&siftData2.h_data[i].data[0], dscr_data, lendata);
      dscr_data += lendata;
   }
   siftData2.numPts = nf2;

   // move data from host to device; we need a new .cu module
   UploadSiftData(&siftData1);
   UploadSiftData(&siftData2);
  
   MatchSiftData(&siftData1, &siftData2); // result in siftData1
   FreeSiftData(&siftData2);
 
   // return a ndarray with matches
   int count = 0;
   for (i = 0; i < nf1; i++) {
      if (siftData1.h_data[i].match >= 0 && siftData1.h_data[i].match < nf2)
         count++;
   }

   /*
   npy_intp dims[2] = { count, 2 };
   PyObject *pPairs = PyArray_SimpleNew(2, dims, NPY_INT16);
   short int *p_data = (short int *) PyArray_DATA((PyArrayObject*) pPairs);
   for (i = 0; i < nf1; i++) {
      if (siftData1.h_data[i].match >= 0 && siftData1.h_data[i].match < nf2) {
         *p_data++ = i;
         *p_data++ = siftData1.h_data[i].match;
      }
   }
   */
   npy_intp dims[2] = { count, 3 };
   PyObject *pPairs = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
   float *p_data = (float *) PyArray_DATA((PyArrayObject*) pPairs);
   for (i = 0; i < nf1; i++) {
      if (siftData1.h_data[i].match >= 0 && siftData1.h_data[i].match < nf2) {
         *p_data++ = i;
         *p_data++ = siftData1.h_data[i].match;
         *p_data++ = siftData1.h_data[i].score;
      }
   }

   FreeSiftData(&siftData1);
   return pPairs;
}

/* ****************************************************************************** 
 * Python Module Stuff
 *
 * *****************************************************************************/

PyMethodDef methods[] = {
    {"extractFeatures", PyCS_ExtractFeatures, METH_VARARGS, 
       "extractFeatures(image)\n   image - a numpy ndaray\n"
       "Returns a tuple (keypoints, descriptors) with SIFT features\n"
       "extracted from the image."
    },
    {"matchDescriptors", PyCS_MatchDescriptors, METH_VARARGS, 
       "extractFeatures(descriptorsA, descriptorsB)\n   descriptorsX - a numpy ndaray Nx128\n"
       "Returns an array Nx2 with indices of matching descriptors."
    },
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initsiftcuda()
{
    (void) Py_InitModule("siftcuda", methods);   
    import_array();
}

