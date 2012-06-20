#ifndef MATLABHELPER_UT6PZ9BU
#define MATLABHELPER_UT6PZ9BU

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <MatlabData.hpp>

// If you get linking problems with mclmcrrt, check how the matlab header files are
// included in a "Matlab Compiler Shared C++ Library" project (check the header file 
// that is generated in the compilation process).
#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"

namespace matlab {

class CMatlabHelper
{
   public:
      // Create a Matrix from a mwArray
      static void array2idl(mwArray &array, Matlab::Matrix& _matrix);

      // Create a mwArray from a Matrix
      static mwArray idl2array(const Matlab::Matrix& _matrix);

      // Create a mwArray from an IplImage
      static mwArray iplImage2array(IplImage *image);
      static mwArray iplImage2array(const unsigned char *pImageData, unsigned width, unsigned height, 
            unsigned nChannels, unsigned align = 4);

      /// @param pImageData Pointer to source data, the IplImage raw image bytes.
      /// @param pArrayData Pointer to destination data, raw array data. If the value is NULL, it will be allocated.
      ///
      /// Assumption: 
      ///    @p pImageData is an interleaved 8-bit image with @p align -byte alignment of rows.
      ///    The order of channels in a color image is BGR.
      /// Assumption: 
      ///    @pArrayData array stores data column by column, channels are separated.
      ///    The order of channels in a color image is RGB.
      ///
      /// @return The number of elements in @p pArrayData.
      template<typename TMatElem>
         static unsigned iplImage2rawArray(const unsigned char *pImageData, TMatElem *&pArrayData,
               unsigned width, unsigned height, unsigned nChannels, unsigned align = 4);

      static void iplImage2matrix(const unsigned char *pImageData, Matlab::Matrix& _matrix,
            unsigned width, unsigned height, unsigned nChannels, unsigned align = 4);

      // Prepare 1d vector matrix;
      template<typename T>
         static void sequence2matrix(const T* _data, unsigned _length, 
               Matlab::Matrix& _matrix);

      // Prepare 1d vector matrix;
      template<typename T>
         static void sequence2matrix(const T* _data, Matlab::Matrix& _matrix, 
               unsigned _width, unsigned _height = 1, unsigned _depth = 1);

      //		template<typename T>
      //			void matrix2sequence(const Matlab::Matrix& _matrix, T* _data);

};

template<typename TMatElem>
unsigned CMatlabHelper::iplImage2rawArray(
      const unsigned char *pImageData, TMatElem *&pArrayData,
      unsigned width, unsigned height, unsigned nChannels, unsigned align)
{
   if (align < 1) align = 1;
   unsigned stride = ((width * nChannels * sizeof(unsigned char) + align - 1) / align) * align;
   unsigned nPixelsInPlane = width * height;
   unsigned nArrayElements = nPixelsInPlane * nChannels;
   if (pArrayData == NULL) pArrayData = new TMatElem[nArrayElements];
   for (unsigned ch = 0; ch < nChannels; ch++) {
      for (unsigned y = 0; y < height; y++) {
         // channel ch of first pixel in row y
         const unsigned char *pIplData = pImageData + stride * y + ch ;
         // row-col swapping, channel separation, channel mixing (BGR->RGB)
         TMatElem *pDst = pArrayData + y + (nChannels - ch - 1) * nPixelsInPlane;
         for (unsigned x = 0; x < width; x++) {
            *pDst = *pIplData;
            pIplData += nChannels;
            pDst += height; // next column, same row
         }
      }
   }
   return nArrayElements;
}

template<typename T>
void CMatlabHelper::sequence2matrix(
      const T* _data, unsigned _length, Matlab::Matrix& _matrix)
{
   // Set the dimensions.
   _matrix.dimensions.resize(2);
   _matrix.dimensions[0] = 1;
   _matrix.dimensions[1] = _length;

   // Set the data.
   _matrix.data.resize(_length);
   for (unsigned i = 0; i < _length; i++) {
      _matrix.data[i] = _data[i];
   }
}

template<typename T>
void CMatlabHelper::sequence2matrix(
      const T* _data, Matlab::Matrix& _matrix,
      unsigned _width, unsigned _height /*= 1*/, unsigned _depth /*= 1*/)
{
   unsigned dataLength = _width * _height * _depth;

   // Set the dimensions.
   _matrix.dimensions.resize(3);
   _matrix.dimensions[0] = _height;
   _matrix.dimensions[1] = _width;
   _matrix.dimensions[2] = _depth;
   _matrix.data.resize(dataLength);

   unsigned planeSize = _width * _height;

   const unsigned char* pData = _data;
   double* pResult = &(_matrix.data[0]);

   for (unsigned c = 0; c < _depth; c++, pData += planeSize, pResult += planeSize) {
      for (unsigned y = 0; y < _height; y++) {
         for (unsigned x = 0; x < _width; x++) {
            *(pResult + x * _height + y) = *(pData + y * _width + x);
         }
      }
   }
}

} // namespace

#endif /* end of include guard: MATLABHELPER_UT6PZ9BU */
