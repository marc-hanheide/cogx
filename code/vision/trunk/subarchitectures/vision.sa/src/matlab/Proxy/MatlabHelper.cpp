#include "MatlabHelper.h"

void CMatlabHelper::array2idl(mwArray &array, Matlab::Matrix& _matrix)
{
   // TODO: test - array.IsNumeric();
   unsigned n_dims = array.NumberOfDimensions();
   _matrix.dimensions_.length(n_dims);
   mwArray dims = array.GetDimensions();
   dims.GetData((mxInt32*)(&_matrix.dimensions_[0]), n_dims);

   unsigned length = array.NumberOfElements();
   _matrix.data_.length(length);
   if (length > 0) 
      array.GetData((mxDouble*)(&_matrix.data_[0]), length);
}

// Similar to mwArray-Get functions this one returns a mwArray.
mwArray CMatlabHelper::idl2array(const Matlab::Matrix& _matrix)
{
   mwSize ndims = _matrix.dimensions_.length();
   const double* data_ptr = (ndims > 0) ? &(_matrix.data_[0]) : NULL;
   mwSize* dimensions = (mwSize*)(&_matrix.dimensions_[0]);
   
   if (data_ptr == NULL) 
      return mwArray(mxDOUBLE_CLASS, mxREAL); // an empty array
   else {
      mwArray array(ndims, dimensions, mxDOUBLE_CLASS, mxREAL);
      array.SetData((mxDouble*) data_ptr, array.NumberOfElements());
      return array;
   }
}

// Assumption: 
//    @p image is an interleaved 8-bit image with 4-byte alignment of rows.
//    The order of channels in a color image is BGR.
// Assumption: 
//    ML array stores data column by column, channels are separated.
//    The order of channels in a color image is RGB.
mwArray CMatlabHelper::iplImage2array(IplImage *image)
{
   if (image == NULL)
      return  mwArray(mxDOUBLE_CLASS, mxREAL); // an empty array

   return iplImage2array((unsigned char*)image->imageData, 
      image->width, image->height, image->nChannels, 4);
}

// Assumption: 
//    @p image is an interleaved 8-bit image.
//    The order of channels in a color image is BGR.
// Assumption: 
//    ML array stores data column by column, channels are separated.
//    The order of channels in a color image is RGB.
mwArray CMatlabHelper::iplImage2array(unsigned char *pImageData, unsigned width, unsigned height, 
   unsigned nChannels, unsigned align)
{
   if (pImageData == NULL)
      return  mwArray(mxDOUBLE_CLASS, mxREAL); // an empty array

   if (align < 1) align = 1;
   if (nChannels < 1) nChannels = 1;
   double *pArrayData = NULL;
   unsigned nArrayElements = CMatlabHelper::iplImage2rawArray<double>(
      pImageData, pArrayData,  width, height, nChannels, align);
   
   mwSize dimensions[3] = {height, width, nChannels};
   mwArray array(3, dimensions, mxDOUBLE_CLASS, mxREAL);
   array.SetData((mxDouble*) pArrayData, nArrayElements);
   
   if (pArrayData != NULL) delete pArrayData;
   return array;
}

void CMatlabHelper::iplImage2matrix(unsigned char *pImageData, Matlab::Matrix& matrix,
   unsigned width, unsigned height, unsigned nChannels, unsigned align)
{
   unsigned dataLength = width * height * nChannels;

   // Set the dimensions.
   matrix.dimensions_.length(3);
   matrix.dimensions_[0] = height;
   matrix.dimensions_[1] = width;
   matrix.dimensions_[2] = nChannels;
   matrix.data_.length(dataLength);

   double* pResult = &(matrix.data_[0]);
   CMatlabHelper::iplImage2rawArray<double>(
      pImageData, pResult, width, height, nChannels, align);
}


