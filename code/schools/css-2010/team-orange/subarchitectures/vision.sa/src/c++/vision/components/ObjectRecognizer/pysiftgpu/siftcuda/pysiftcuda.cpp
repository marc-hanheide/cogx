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
#include <vector>

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
void copyPyArrayToSiftData(PyObject *array, int nPoints, SiftData &data)
{
#define ROUND(n) ((n + 7) / 8) * 8
   InitSiftData(&data, ROUND(nPoints), true, true); 
#undef ROUND

   int lendata = 128 * sizeof(float);
   memset((void*)&data.h_data[0], 0, lendata*nPoints);
   unsigned char *dscr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) array);
   for (int i = 0; i < nPoints; i++) {
      memcpy((void*)&data.h_data[i].data[0], dscr_data, lendata);
      dscr_data += lendata;
   }
   data.numPts = nPoints;
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
   copyPyArrayToSiftData(ndaDescrA, nf1, siftData1);
   copyPyArrayToSiftData(ndaDescrB, nf2, siftData2);

   // move data from host to device; we need a new .cu module
   UploadSiftData(&siftData1);
   UploadSiftData(&siftData2);
  
   MatchSiftData(&siftData1, &siftData2); // result in siftData1
   FreeSiftData(&siftData2);
 
   // return a ndarray with matches
   int i, count = 0;
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

static
PyObject* PyCS_HomographyMatchDescriptors(PyObject *self, PyObject *args)
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
   copyPyArrayToSiftData(ndaDescrA, nf1, siftData1);
   copyPyArrayToSiftData(ndaDescrB, nf2, siftData2);

   // move data from host to device; we need a new .cu module
   UploadSiftData(&siftData1);
   UploadSiftData(&siftData2);
  
   MatchSiftData(&siftData1, &siftData2); // result in siftData1
   float homography[9];
   int numMatches;
   const float maxPixDist = 3.0;
   FindHomography(&siftData1, homography, &numMatches, 1000, 0.85f, 0.95f, maxPixDist);

   int numPts = siftData1.numPts;
   SiftPoint *sift1 = siftData1.h_data;
   SiftPoint *sift2 = siftData2.h_data;
   std::vector<int> ma, mb;
   for (int j=0; j<numPts; j++) { 
      int k = sift1[j].match;
      float x = sift1[j].xpos;
      float y = sift1[j].ypos;
      float den = homography[6]*x + homography[7]*y + homography[8]; 
      float x2 = (homography[0]*x + homography[1]*y + homography[2]) / den;
      float y2 = (homography[3]*x + homography[4]*y + homography[5]) / den;
      float erx = x2 - sift2[k].xpos;
      float ery = y2 - sift2[k].ypos;
      float er2 = erx*erx + ery*ery;
      if (er2 < maxPixDist*maxPixDist && sift1[j].score > 0.85f && sift1[j].ambiguity < 0.95f) {
         ma.push_back(j);
         mb.push_back(k); 
      }
   }
   FreeSiftData(&siftData1);
   FreeSiftData(&siftData2);

   //PyObject *pRes = PyTuple_New(2);
   PyObject *pRes = NULL;
   int i, count = ma.size();
   if (count < 1) pRes = Py_BuildValue("O", Py_None, Py_None);
   else {
      // copy ma, mb to PyObject
      npy_intp dims[2] = { count, 2 };
      PyObject *pPairs = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
      float *p_data = (float *) PyArray_DATA((PyArrayObject*) pPairs);
      for (i = 0; i < count; i++) {
         *p_data++ = ma[i];
         *p_data++ = mb[i];
      }

      // copy homography to PyObject
      dims[0] = 3;
      dims[1] = 3;
      PyObject *pHgr = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
      unsigned char *hgr_data = (unsigned char*) PyArray_DATA((PyArrayObject*) pHgr);
      memcpy(hgr_data, homography, 9 * sizeof(float));

      // pass the results to Python
      pRes = Py_BuildValue("OO", pPairs, pHgr);
      //PyTuple_SetItem(pRes, 0, pPairs);
      //PyTuple_SetItem(pRes, 1, pHgr);
   }
   return pRes;
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
       "matchDescriptors(descriptorsA, descriptorsB)\n   descriptorsX - a numpy ndaray Nx128\n"
       "Returns an array Nx2 with indices of matching descriptors."
    },
    {"homographyMatchDescriptors", PyCS_HomographyMatchDescriptors, METH_VARARGS, 
       "homographyMatchDescriptors(descriptorsA, descriptorsB)\n   descriptorsX - a numpy ndaray Nx128\n"
       "Returns a tuple (matches, homography)\n"
       "  matches - Nx2 array with indices of descriptors that match a homography.\n"
       "  homography - calculated with RANSAC on the matching descriptors."
    },
    {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC initsiftcuda()
{
    (void) Py_InitModule("siftcuda", methods);   
    import_array();
}
