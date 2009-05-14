#ifndef __MatlabHelper_h__
#define __MatlabHelper_h__

#include "uol_matlab_engine.h"
#include <mlhelper/idl/Matlab.hh>

class MatlabHelper : public uol_matlab_engine
{
protected:
  /// Reads the matrix into IDL structure.
  void mat2idl(const char* _matrix_name, Matlab::Matrix& _matrix);

  /// Puts the matrix from IDL structure.
  void idl2mat(const char* _matrix_name, 
	       const Matlab::Matrix& _matrix);

  // Prepare 1d vector matrix;
  template<typename T>
  void sequence2matrix(const T* _data, unsigned _length, 
		       Matlab::Matrix& _matrix) const;
 
  // Prepare 1d vector matrix;
  template<typename T>
  void sequence2matrix(const T* _data, Matlab::Matrix& _matrix, 
		       unsigned _width, unsigned _height = 1, unsigned _depth = 1) const;

  //		template<typename T>
  //			void matrix2sequence(const Matlab::Matrix& _matrix, T* _data);

protected:
  MatlabHelper(unsigned _buffer_length = 0);
}; // class MatlabHelper

template<typename T>
void MatlabHelper::sequence2matrix(const T* _data, unsigned _length,
				   Matlab::Matrix& _matrix) const
{
  // Set the dimensions.
  _matrix.dimensions_.length(2);
  _matrix.dimensions_[0] = 1;
  _matrix.dimensions_[1] = _length;

  // Set the data.
  _matrix.data_.length(_length);
  for (unsigned i = 0; i < _length; i++)
    {
      _matrix.data_[i] = _data[i];
    } // for
} // MatlabHelper::sequence2matrix

template<typename T>
void MatlabHelper::sequence2matrix(const T* _data, Matlab::Matrix& _matrix, 
				   unsigned _width, unsigned _height /*= 1*/, unsigned _depth /*= 1*/) const
{
  unsigned dataLength = _width * _height * _depth;

  // Set the dimensions.
  _matrix.dimensions_.length(3);
  _matrix.dimensions_[0] = _height;
  _matrix.dimensions_[1] = _width;
  _matrix.dimensions_[2] = _depth;
  _matrix.data_.length(dataLength);

  unsigned planeSize = _width * _height;

  const unsigned char* pData = _data;
  double* pResult = &(_matrix.data_[0]);

  for (unsigned c = 0; c < _depth; c++, 
	 pData += planeSize, pResult += planeSize)
    {
      for (unsigned y = 0; y < _height; y++)
	{
	  for (unsigned x = 0; x < _width; x++)
	    {
	      *(pResult + x * _height + y) = *(pData + y * _width + x);
	    } // for
	} // for
    } // for
} // Matrix::sequence2matrix

#endif
