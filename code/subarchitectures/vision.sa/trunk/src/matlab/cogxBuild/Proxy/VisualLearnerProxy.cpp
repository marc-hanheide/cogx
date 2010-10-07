/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
/** 
 * @brief Proxy for the VisualLearner component.
 *
 * @author Marko Mahnic
 */
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <VisionData.hpp>
#include "../../../c++/vision/VisionUtils.h"

#include "MatlabHelper.h"
#include "VisualLearnerProxy.h"
#include "libVisualLearnerCtf.h"
#include "initlib.h"

using namespace std;
using namespace VisionData;

namespace matlab {

cogx::CTypeEnumerator Enumerator;
std::map<std::string, int> labelConceptMap; // name -> 1=color, 2=shape
std::string ClfStartConfig;

class CInitializer
{
public:
   CInitializer() {
      printf("VisualLearnerProxy initializing\n");
      InitVisualLearnerLib();

      printf("ClfStartConfig='%s'\n", ClfStartConfig.c_str());

      // Load global variables 
      mwArray clfConfig(ClfStartConfig.c_str());
      CLFstart(clfConfig);
   }
   ~CInitializer() {
      printf("VisualLearnerProxy terminating\n");
      TermVisualLearnerLib();
   }

   void initEnumeration() {
      try {
         Enumerator.clear();
         labelConceptMap.clear();
         mwArray avNames, scConcept;
         getGlobalArray(1, avNames, "Coma", "Coma.avNames");
         getGlobalArray(1, scConcept, "Coma", "Coma.SCC");
         mwArray dims = avNames.GetDimensions();
         double dim0 = dims.Get(mwSize(1), 1);
         for (int i = 0; i < dim0; i++) {
            mwString mws = avNames.Get(mwSize(1), i+1).ToString();
            int concept = scConcept.Get(mwSize(2), i+1, 2);
            string label((const char*)mws);
            Enumerator.addMapping(label, i+1);
            labelConceptMap[label] = concept;
            printf(" ... %d .. %s .. c%d \n", i+1, label.c_str(), concept);
         }
      }
      catch (...) {
         printf(" **** FAILED to extract from Matlab: Coma.avNames, Coma.SCC\n");
         Enumerator.clear();
         Enumerator.addMapping("red", 1);
         Enumerator.addMapping("green", 2);
         Enumerator.addMapping("blue", 3);
         Enumerator.addMapping("yellow", 4);
         Enumerator.addMapping("black", 5);
         Enumerator.addMapping("white", 6);
         Enumerator.addMapping("orange", 7);
         Enumerator.addMapping("pink", 8);
         Enumerator.addMapping("compact", 9);
         Enumerator.addMapping("elongated", 10);
      }
   }

} *pInitializer=NULL;

class CTerminator
{
public:
   ~CTerminator() {
      if (pInitializer) delete pInitializer;
      pInitializer = NULL;
   }
} Terminator;

static void CheckInit()
{
   if (!pInitializer) {
      pInitializer = new CInitializer();
      pInitializer->initEnumeration();
   }
}


static void addLabels(mwArray &ans, double weight, vector<string> &labels, vector<double> &probs)
{
   //unsigned n_dims = ans.NumberOfDimensions();
   // TODO: assert n_dims = 1
   mwArray dims = ans.GetDimensions();
   double dim0 = dims.Get(mwSize(1), 1);
   for (int i = 0; i < dim0; i++) {
      double label = ans.Get(mwSize(1), i+1);
      labels.push_back(Enumerator.getName((int)label));
      probs.push_back(weight);
   }
}

static void copyLabels(const mwArray &ans, vector<string> &labels, vector<double> &probs)
{
   //unsigned n_dims = ans.NumberOfDimensions();
   // TODO: assert n_dims = 2
   mwArray dims = ans.GetDimensions();
   double dim0 = dims.Get(mwSize(1), 1);
   for (int i = 0; i < dim0; i++) {
      double label  = ans.Get(mwSize(2), i+1, 1);
      double weight = ans.Get(mwSize(2), i+1, 2);
      labels.push_back(Enumerator.getName((int)label));
      probs.push_back(weight);
   }
}

void protoObjectToMwArray(const ProtoObject &Object, mwArray &image, mwArray &mask, mwArray &points3d)
{
   // Convert image patch
   image = CMatlabHelper::iplImage2array(&(Object.image.data[0]), 
         Object.image.width, Object.image.height, 3, 1); // WISH: number of channels in image
   // printf("Object Image: %dx%d\n", Object.image.width, Object.image.height);

   // Convert image mask
   mask = CMatlabHelper::iplImage2array( &(Object.mask.data[0]), 
         Object.mask.width, Object.mask.height, 1, 1);
   // printf("Object Mask: %dx%d\n", Object.mask.width, Object.mask.height);

   // Convert 3D points
   int npts = Object.points.size();
   const int ncol = 6;
   if (npts < 1) {
      printf("**** not enough points\n");
      points3d = mwArray();
   }
   else {
      printf("**** we have %d 3D points\n", npts);
      // x, y, z, r, g, b
      mwSize dimensions[2] = {npts, ncol};
      points3d = mwArray(2, dimensions, mxDOUBLE_CLASS, mxREAL);
      double *data = new double[npts * ncol];
      for (int i = 0; i < npts; i++) {
         const SurfacePoint &p = Object.points[i];
         data[i + npts*0] = p.p.x;
         data[i + npts*1] = p.p.y;
         data[i + npts*2] = p.p.z;
         data[i + npts*3] = p.c.r;
         data[i + npts*4] = p.c.g;
         data[i + npts*5] = p.c.b;
      }
      points3d.SetData(data, npts*ncol);
      delete data;
   }
}

void VL_LoadAvModels(const char* filename)
{
   CheckInit();
   mwArray fname (filename);
   LRloadAVmodels(fname);
}

void VL_setEnumeration(const cogx::CTypeEnumerator& typeEnum)
{
   Enumerator = typeEnum;
}

void VL_setClfStartConfig(const std::string& absConfigPath)
{
   ClfStartConfig = absConfigPath;
   CheckInit();
}

//void FE_recognise_attributes(ROI &Roi)
void VL_recognise_attributes(const ProtoObject &Object, vector<string> &labels,
      vector<int> &labelConcepts, vector<double> &probs, vector<double> &gains)
{
   CheckInit();

#if 0 /* DEBUG */
   int npix = Object.mask.width * Object.mask.height;
   Video::Image test;
   test.width = Object.mask.width;
   test.height = Object.mask.height;
   for (int i = 0; i < npix; i++) {
      for (int j = 0; j < 3; j++) test.data.push_back(Object.mask.data[i] * 63);
   }
   IplImage *pimg = convertImageToIpl(test);
   cvShowImage("vlproxy", pimg);
   cvReleaseImage(&pimg);
#endif /* END-DEBUG */

   mwArray image, mask, pts3d;
   protoObjectToMwArray(Object, image, mask, pts3d);
   
#if 1 /* DEBUG */
   // printf("**** WEIRD\n");
   sleep(0.01);
   mwArray dims = pts3d.GetDimensions();
   int dim0 = dims.Get(mwSize(1), 1);
   int dim1 = dims.Get(mwSize(1), 2);
   printf("**** ML: converted pts3d size: %dx%d\n", dim0, dim1);
   dims = image.GetDimensions();
   dim0 = dims.Get(mwSize(1), 1);
   dim1 = dims.Get(mwSize(1), 2);
   printf("**** ML: converted image size: %dx%d\n", dim0, dim1);
#endif /* END-DEBUG */

   // Extract features and recognise.
   mwArray rCqnt, mlGain;
   cogxVisualLearner_recognise(2, rCqnt, mlGain, image, mask, pts3d);

   labels.clear();
   probs.clear();
   gains.clear();

   //unsigned n_dims = rCqnt.NumberOfDimensions();
   // TODO: assert n_dims = 2; assert gain.size == rCqnt.size
   dims = rCqnt.GetDimensions();
   dim0 = dims.Get(mwSize(1), 1);
   for (int i = 0; i < dim0; i++) {
      double label = rCqnt.Get(mwSize(2), i+1, 1);
      double fprob = rCqnt.Get(mwSize(2), i+1, 2);
      double fgain = mlGain.Get(mwSize(2), i+1, 2);
      string strLabel = Enumerator.getName((int)label);
      labels.push_back(strLabel);
      labelConcepts.push_back(labelConceptMap[strLabel]);
      probs.push_back(fprob);
      gains.push_back(fgain);
   }
}

void VL_update_model(ProtoObject &Object, std::vector<string>& labels, std::vector<double>& weights)
{
   CheckInit();
   mwArray image, mask, pts3d;
   protoObjectToMwArray(Object, image, mask, pts3d);

   long cntPlus = 0, cntMinus = 0;
   for (unsigned i = 0; i < labels.size(); i++) {
      if (weights[i] > 0) cntPlus++;
      if (weights[i] < 0) cntMinus++;
   }

   if (cntPlus > 0) {
      mwArray avw(cntPlus, 2, mxDOUBLE_CLASS, mxREAL);
      int row = 1;
      for (unsigned i = 0; i < labels.size(); i++) {
        if (weights[i] > 0) {
           avw(row, 1) =  (double) Enumerator.getEnum(labels[i]);
           avw(row, 2) =  (double) weights[i];
           row++;
        }
      }
      cogxVisualLearner_update(avw, image, mask, pts3d);
   }

   if (cntMinus > 0) {
      mwArray avw(cntMinus, 2, mxDOUBLE_CLASS, mxREAL);
      int row = 1;
      for (unsigned i = 0; i < labels.size(); i++) {
        if (weights[i] < 0) {
           avw(row, 1) =  (double) Enumerator.getEnum(labels[i]);
           avw(row, 2) =  (double) -weights[i];
           row++;
        }
      }
      cogxVisualLearner_unlearn(avw, image, mask, pts3d);
   }
}

void VL_introspect(vector<string>& labels, vector<int>& labelConcepts, vector<double>& gains)
{
   CheckInit();
   mwArray newGains;
   cogxVisualLearner_introspect(1, newGains);

   labels.clear();
   labelConcepts.clear();
   gains.clear();
   mwArray dims = newGains.GetDimensions();
   int dim0 = dims.Get(mwSize(1), 1);
   for (int i = 0; i < dim0; i++) {
      double label = newGains.Get(mwSize(2), i+1, 1);
      double fgain = newGains.Get(mwSize(2), i+1, 2);
      string strLabel = Enumerator.getName((int)label);
      labels.push_back(strLabel);
      labelConcepts.push_back(labelConceptMap[strLabel]);
      gains.push_back(fgain);

      printf(" ... label '%s' gain %lf\n", strLabel.c_str(), fgain);
   }
}

} // namespace

// typedef unsigned char BYTE;

// Assumption: channels in raw data are BGR, 8 bits per channel (uchar), 4 byte alignment.
// Channels in CvMat are stored in the same matrix cell (CvScalar).
// NOT TESTED!
//~ static CvMat* rawImage2cvmat(Image &image)
//~ {
//~    printf("rawImage2cvmat\n");
//~    const unsigned align = 4;
//~    unsigned _width = image.m_width;
//~    unsigned _height = image.m_height; 
//~    unsigned _depth = image.m_nChannels;
//~    BYTE *pdata = (BYTE *) &(image.m_image[0]);
//~    unsigned stride = ((_width * _depth * sizeof(BYTE) + align - 1) / align) * align;
//~    
//~    int cvc = 0;
//~    if (_depth == 1)  cvc = CV_8UC1;
//~    else if (_depth == 3) cvc = CV_8UC3;
//~    
//~    CvMat *A = (CvMat*) cvAlloc(sizeof(CvMat));
//~    cvInitMatHeader(A, _height, _width, cvc, pdata, stride);
//~    
//~    return A;
//~ }

/// Copies image data from \p ImageFrame structure to a \p IplImage object.
/// Assumption: rows are 4-byte aligned.
//IplImage* rawImage2iplImage(Image &Image)
//{
//   IplImage *img = cvCreateImage(cvSize(Image.width, Image.height), 
//      IPL_DEPTH_8U, 3); // WISH: number of Channels in Image
//   BYTE *dst = (BYTE*) img->imageData;
//   BYTE *src = (BYTE*)&(Image.image[0]);
//   memcpy(dst, src, Image.image.size());

//   return img;
//}

// Channels in CvMat are stored in the same matrix cell (CvScalar)
// Channels in mwArray are separated, (currently double)
// Because direct access to mwArray's data is not available, the data needs
// to be pre-transformed in an intermediate array.
// NOT TESTED! A faster version is possible (see iplImage2array)
//~ static mwArray cvmat2array(CvMat *mat, int nChannels)
//~ {
//~    printf("cvmat2array\n");
//~    if (mat == NULL)
//~       return  mwArray(mxDOUBLE_CLASS, mxREAL); // an empty array
//~    
//~    int width = mat->width;
//~    int height = mat->height;
//~    unsigned nPixelsInPlane = width * height;
//~    unsigned nArrayElements = nPixelsInPlane * nChannels;
//~    double *pArrayData = new double[nArrayElements];
//~    for (int y = 0; y < height; y++) {
//~       for (int x = 0; x < width; x++) {
//~          CvScalar color = cvGet2D(mat, y, x);
//~          for (int c = 0; c < nChannels; c++) 
//~             *(pArrayData + y * width + x + c * nPixelsInPlane) = color.val[c];
//~       }
//~    }
//~    mwSize dimensions[3] = {height, width, nChannels};
//~    mwArray array(3, dimensions, mxDOUBLE_CLASS, mxREAL);
//~    array.SetData((mxDouble*) pArrayData, nArrayElements);
//~    
//~    if (pArrayData != NULL) delete pArrayData;
//~    return array;
//~ }

// @param matrix3x3 Array with the parameters of the perspective transformation, storage: [Row0; Row1; Row2]
//static CvMat* getTrafoAndOutputBox(BBox2D &bbox, float* matrix3x3, BBox2D &outBox)
//{
//   // printf("getTrafoAndOutputBox\n");
//   struct _roi_position { double x0, y0, x1, y1; } ROI;
//   ROI.x0 = bbox.m_center.m_x - bbox.m_size.m_x / 2.0; 
//   ROI.y0 = bbox.m_center.m_y - bbox.m_size.m_y / 2.0; 
//   ROI.x1 = bbox.m_center.m_x + bbox.m_size.m_x / 2.0; 
//   ROI.y1 = bbox.m_center.m_y + bbox.m_size.m_y / 2.0; 
//   CvPoint2D32f bbRoi[4]; // corners of ROI
//   bbRoi[0].x = ROI.x0;  bbRoi[0].y = ROI.y0; 
//   bbRoi[1].x = ROI.x1;  bbRoi[1].y = ROI.y0; 
//   bbRoi[2].x = ROI.x1;  bbRoi[2].y = ROI.y1;
//   bbRoi[3].x = ROI.x0;  bbRoi[3].y = ROI.y1; 
//   CvMat* matbbRoi = cvCreateMat(4, 1, CV_32FC3);
//   for (int i=0; i < 4; i++) cvSet2D(matbbRoi, i, 0, cvScalar (bbRoi[i].x, bbRoi[i].y, 1));

//   CvMat H = cvMat(3, 3, CV_32FC1, matrix3x3);
//   CvMat *matTraRoi = cvCreateMat(4, 1, CV_32FC3); // Transformed ROI, a polygon

//   // cvPerspectiveTransform doesn't work as expected, so: Transform + normalize
//   cvTransform(matbbRoi, matTraRoi, &H); // NOT cvPerspectiveTransform
//   for (int i=0; i < 4; i++) { // Normalize
//      CvScalar coord = cvGet2D(matTraRoi, i, 0);
//      for (int c = 0; c < 3; c++) coord.val[c] /= coord.val[2];
//      cvSet2D(matTraRoi, i, 0, coord);
//   }      

//   CvPoint2D32f traRoi[4];
//   for (int i=0; i < 4; i++) {
//      CvScalar coord = cvGet2D(matTraRoi, i, 0);
//      traRoi[i].x = coord.val[0];
//      traRoi[i].y = coord.val[1];
//   }
//   cvReleaseMat(&matbbRoi);
//   cvReleaseMat(&matTraRoi);
//   // cvReleaseMat(&H);

//   // Limits of the transformed ROI
//   float traRoiXmin = traRoi[0].x; // top-left of polygon BB
//   float traRoiYmin = traRoi[0].y;
//   float traRoiXmax = traRoi[0].x; // bottom-right of polygon BB
//   float traRoiYmax = traRoi[0].y;
//   for (int i=1; i < 4; i++) {
//      if (traRoiXmin > traRoi[i].x) traRoiXmin = traRoi[i].x;
//      if (traRoiYmin > traRoi[i].y) traRoiYmin = traRoi[i].y;
//      if (traRoiXmax < traRoi[i].x) traRoiXmax = traRoi[i].x;
//      if (traRoiYmax < traRoi[i].y) traRoiYmax = traRoi[i].y;
//   }
//   // ... store in the resulting BB
//   outBox.m_center.m_x = (traRoiXmin + traRoiXmax) / 2;
//   outBox.m_center.m_y = (traRoiYmin + traRoiYmax) / 2;
//   outBox.m_size.m_x = (traRoiXmax - traRoiXmin);
//   outBox.m_size.m_y = (traRoiYmax - traRoiYmin);

//   // Shift ROI and transformed ROI to image (0, 0)
//   for (int i=0; i < 4; i++) {
//      bbRoi[i].x -= ROI.x0;
//      bbRoi[i].y -= ROI.y0;
//      traRoi[i].x -= traRoiXmin;
//      traRoi[i].y -= traRoiYmin;
//   }

//   // calculate the new transformation
//   CvMat *H1 = cvCreateMat(3, 3, CV_32FC1);
//   cvWarpPerspectiveQMatrix(bbRoi, traRoi, H1);

//   return H1;
//}

// 1. copy data to IplImage A
// 2. calculate the size of the transformed image and create IplImage B
// 3. create CvMat C from matrix3x3
// 4. transform with cvWarpPerspective (B = CA)
// 5. create matlab array out, same size as B
// 6. copy B to out; beware: channels are separeted in out!
//static mwArray transformRaw2array(Image &image, BBox2D &bbox, float* matrix3x3)
//{
//   // printf("transformRaw2array\n");
//   IplImage *A = rawImage2iplImage(image);

//   BBox2D outBox;
//   CvMat *H1 = getTrafoAndOutputBox(bbox, matrix3x3, outBox);
//   IplImage *B = cvCreateImage(cvSize(outBox.m_size.m_x, outBox.m_size.m_y), IPL_DEPTH_8U, image.m_nChannels); 

//   cvWarpPerspective(A, B, H1); 
//   cvReleaseImage(&A);
//   cvReleaseMat(&H1);

//   mwArray out = CMatlabHelper::iplImage2array(B);
//   cvReleaseImage(&B);

//   return out;
//}

// Extract features from ROI. 
// Transform the image before extraction.
// @param matrix3x3 Array with the parameters of the perspective transformation, storage: [Row0; Row1; Row2]
/*void FE_extract_features_transform(ROI &Roi, float* matrix3x3)
  {
// Add the ROI to Matlab engine
mwArray x = transformRaw2array(Roi.m_region, Roi.m_bbox, matrix3x3);
mwArray b0 = transformRaw2array(Roi.m_mask, Roi.m_bbox, matrix3x3);

// Segment the Roi.
mwArray b;
cosyFeatureExtractor_limitvalue(1, b, b0, mwArray(1.0));

// Extract features.
mwArray f;
cosyFeatureExtractor_extract(1, f, x, b);

// Add extracted features to the Roi.
CMatlabHelper::array2idl(f, Roi.m_features);
}*/


