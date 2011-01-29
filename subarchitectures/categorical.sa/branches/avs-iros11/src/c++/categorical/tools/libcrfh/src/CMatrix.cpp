// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * \file CMatrix.cpp
 * \author Andrzej Pronobis
 *
 * Contains implementation of the CMatrix class.
 */

#include <QtCore/QFile>
#include "global.h"

#include "CMatrix.h"


// -----------------------------------------
CMatrix::CMatrix(int rows, int cols):_rows(rows), _cols(cols)
{
  if ((rows==0) || (cols==0))
    _data = 0;
  else
    _data = aMalloc<double>(rows*cols);
}



// -----------------------------------------
CMatrix::~CMatrix()
{
  if (_data)
    aFree(_data);
}



// -----------------------------------------
void CMatrix::print()
{
  for (int i=0; i<_cols; i+=6)
  {
    int lastColumn = aMin<int>(i+6, _cols);
    aout<<"Columns "<<i<<" to "<<lastColumn-1<<":"<<endl<<fixed;
    for (int j=0; j<_rows; ++j)
    {
      for (int k=i; k<lastColumn; ++k)
        aout<<getElem(j, k)<<" ";
      aout<<endl;
    }
    aout<<endl<<reset;
  }
}


// -----------------------------------------
void CMatrix::resize(int rows, int cols)
{
  if ((rows!=_rows) || (cols!=_cols))
  {
    if ((rows==0) || (cols==0))
    {
      _data = 0;
      aout<<"ERROR: Incorrect sie of a matrix!"<<endl;
    }
    else
      _data = aRealloc<double>(_data, rows*cols);
    _rows = rows;
    _cols = cols;
  }
}


// -----------------------------------------
double CMatrix::getMax() const
{
  int elems = _rows*_cols;
  double max=_data[0];

  for(int i=1; i<elems; ++i)
    if (_data[i]>max) max=_data[i];

  return max;
}


// -----------------------------------------
double CMatrix::getMin() const
{
  int elems = _rows*_cols;
  double min=_data[0];

  for(int i=1; i<elems; ++i)
    if (_data[i]<min) min=_data[i];

  return min;
}


// -----------------------------------------
double CMatrix::getSum() const
{
  int elems = _rows*_cols;
  double sum=0;

  for(int i=0; i<elems; ++i)
    sum+=_data[i];

  return sum;
}

// -----------------------------------------
void CMatrix::convolveWith(const CMatrix &m)
{
  // Do noting for empty matrices or kernels
  if ((isEmpty()) || (m.isEmpty())) return;

  // Get some info about the filter
  int mRows=m.getRows();
  int mCols=m.getCols();

  int mPreMidRow=(mRows-1)/2;
  int mPreMidCol=(mCols-1)/2;
  int mPostMidRow=mRows-1-mPreMidRow;
  int mPostMidCol=mCols-1-mPreMidCol;

  // Create new data
  double *newData = aMalloc<double>(_rows*_cols);
  double *newDataPtr = newData;

  // Compute convolution
  for (int i=0; i<_rows; ++i)
    for (int j=0; j<_cols; ++j)
    {
      int startRow=aMax<int>(i-mPreMidRow,0);
      int endRow=aMin<int>(i+mPostMidRow, _rows-1);
      int startCol=aMax<int>(j-mPreMidCol,0);
      int endCol=aMin<int>(j+mPostMidCol, _cols-1);

      (*newDataPtr)=0;

      for (int k=startRow; k<=endRow; ++k)
        for (int l=startCol; l<=endCol; ++l)
          (*newDataPtr) += getElem(k,l)*m.getElem(mPreMidRow-(i-k), mPreMidCol-(j-l));

      ++newDataPtr;
    }

  // Substitute the old data with the new data
  aFree(_data);
  _data = newData;
}



// -----------------------------------------
void CMatrix::convolveWithFft(const CMatrix &m)
{

}


// -----------------------------------------
CMatrix *CMatrix::convolve(const CMatrix &mA, const CMatrix &mB, CMatrix *mC)
{
  // Do noting for empty matrices
  if (mA.isEmpty() || mB.isEmpty()) return mC;

  // Get some info about the filter
  int mARows=mA.getRows();
  int mACols=mA.getCols();
  int mBRows=mB.getRows();
  int mBCols=mB.getCols();

  int mBPreMidRow=(mBRows-1)/2;
  int mBPreMidCol=(mBCols-1)/2;
  int mBPostMidRow=mBRows-1-mBPreMidRow;
  int mBPostMidCol=mBCols-1-mBPreMidCol;

  // Create the resulting matrix
  if (mC)
    mC->resize(mARows, mACols);
  else
    mC = new CMatrix(mARows, mACols);
  double *newDataPtr = mC->getRawData();

  // Compute convolution
  for (int i=0; i<mARows; ++i)
    for (int j=0; j<mACols; ++j)
    {
      int startRow=aMax<int>(i-mBPreMidRow,0);
      int endRow=aMin<int>(i+mBPostMidRow, mARows-1);
      int startCol=aMax<int>(j-mBPreMidCol,0);
      int endCol=aMin<int>(j+mBPostMidCol, mACols-1);

      (*newDataPtr)=0;

      for (int k=startRow; k<=endRow; ++k)
        for (int l=startCol; l<=endCol; ++l)
          (*newDataPtr) += mA.getElem(k,l)*mB.getElem(mBPreMidRow-(i-k), mBPreMidCol-(j-l));

      ++newDataPtr;
    }

  return mC;
}


// -----------------------------------------
//CMatrix *CMatrix::convolveFft(const CMatrix &mA, const CMatrix &mB, CMatrix *mC)
//{
//}

// -----------------------------------------
CMatrix &CMatrix::operator/=(double value)
{
  int elems = _rows*_cols;
  for (int i=0; i<elems; ++i)
    _data[i]/=value;

  return *this;
}


// -----------------------------------------
CMatrix &CMatrix::operator*=(double value)
{
  int elems = _rows*_cols;
  for (int i=0; i<elems; ++i)
    _data[i]*=value;

  return *this;
}


// -----------------------------------------
CMatrix *CMatrix::getTransposed(CMatrix *result) const
{
  // Create the output matrix
  if (result)
    result->resize(_cols, _rows);
  else
    result = new CMatrix(_cols, _rows);

  // Compute transpose
  for (int i=0; i<_rows; ++i)
    for (int j=0; j<_cols; ++j)
      result->setElem(j,i, getElem(i,j));

  // Return result
  return result;
}


// -----------------------------------------
CMatrix *CMatrix::getDeepCopy(CMatrix *result) const
{
  // Create the output matrix
  if (result)
    result->resize(_rows,_cols);
  else
    result = new CMatrix(_rows,_cols);

  aMemCopy<double>(result->getRawData(), getRawData(), _cols*_rows);

  // Return result
  return result;
}



// -----------------------------------------
int CMatrix::saveToMatFile(QString fileName, QByteArray variableName)
{

  // Prepare the header
  CMatFileHeader header;
  header.type = 0000; // 1000;
  header.mrows = _rows;
  header.ncols = _cols;
  header.imagf = 0;
  header.namelen = 1+variableName.size();

  // Open file
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly))
    return -1;

  file.write(reinterpret_cast<char *>(&header), sizeof(header));
  file.write(variableName.data(), header.namelen);
  for (int i=0; i<_cols; ++i)
    for (int j=0; j<_rows; ++j)
      file.write(reinterpret_cast<char *>(_data+j*_cols+i), sizeof(double));

  // Close file
  file.close();
  return 0;
}




