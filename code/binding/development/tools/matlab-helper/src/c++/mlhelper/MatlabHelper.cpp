#include <iostream>
#include "MatlabHelper.h"

MatlabHelper::MatlabHelper(unsigned _buffer_length /* = 0 */) :
  uol_matlab_engine(_buffer_length)
{
  this->eval("format compact;");
} // MatlabHelper::MatlabHelper

void MatlabHelper::mat2idl(const char* _matrix_name, 
			   Matlab::Matrix& _matrix)
{
  mxArray* mx = this->get_variable(_matrix_name);
  
  unsigned n_dims = mxGetNumberOfDimensions(mx);
  _matrix.dimensions_.length(n_dims);
  memcpy(&(_matrix.dimensions_[0]), mxGetDimensions(mx), 
	 n_dims * sizeof(int));
  
  unsigned length = mxGetNumberOfElements(mx);
  _matrix.data_.length(length);
  if (length > 0)
    {
      memcpy(&_matrix.data_[0], mxGetPr(mx), length * sizeof(double));
    } // if
} // MatlabHelper::mat2idl

void MatlabHelper::idl2mat(const char* _matrix_name, 
			   const Matlab::Matrix& _matrix)
{
  this->put_variable1(_matrix_name, 
		      ((_matrix.data_.length() > 0) ? &(_matrix.data_[0]) : NULL),
		      _matrix.dimensions_.length(), (int*)(&_matrix.dimensions_[0]));
} // MatlabHelper::idl2mat
