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
