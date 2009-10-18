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
 * \file CFilter.cpp
 * \author Andrzej Pronobis
 *
 * Contains implementation of the CFilter class.
 */

#include "math.h"
#include "global.h"
#include "CMatrix.h"

#include "CFilter.h"

#define PI 3.141592653589793




CFilter *CFilter::createFilter(const CFilterInfo &fi)
{
  switch(fi.getFilterType())
  {
  case FT_GAUSSIAN:
    return new CGaussianFilter(reinterpret_cast<const CGaussianFilterInfo &>(fi).getSigma2());
  case FT_CARTESIAN:
    return new CCartesianFilter(reinterpret_cast<const CCartesianFilterInfo &>(fi).getDx(),
                                reinterpret_cast<const CCartesianFilterInfo &>(fi).getDy());
  default:
    return 0;
  }
}


// -----------------------------------------
void CGaussianFilter::createGaussHorizontalKernel(double sigma2)
{
  double sigma = sqrt(sigma2);
  int gaussSize = aMax<int>(static_cast<int>(ceil(sigma*GAUSSIAN_SIGMAS)), 1);
  int kernelSize = 2*gaussSize+1;

  // Allocate memory for the kernel
  _horizontalKernel.resize(1, kernelSize);

  // Be robust to sigma = 0
  if (sigma2==0)
  {
    _horizontalKernel.setElem(0,0,0);
    _horizontalKernel.setElem(0,1,1);
    _horizontalKernel.setElem(0,2,0);
    return;
  }

// Fill in the values
  double sum=0;
  for (int i=0; i<kernelSize; ++i)
  {
    int tmp = i-gaussSize;
    double value = exp(-(tmp*tmp)/(2*sigma2));
    _horizontalKernel.setElem(0, i, value);
    sum+=value;
  }

  // Normalize
  _horizontalKernel/=sum;
}



// -----------------------------------------
void CGaussianFilter::createGaussVerticalKernel(double sigma2)
{
  double sigma = sqrt(sigma2);
  int gaussSize = aMax<int>(static_cast<int>(ceil(sigma*GAUSSIAN_SIGMAS)), 1);
  int kernelSize = 2*gaussSize+1;

  // Allocate memory for the kernel
  _verticalKernel.resize(kernelSize, 1);

  // Be robust to sigma = 0
  if (sigma2==0)
  {
    _verticalKernel.setElem(0,0,0);
    _verticalKernel.setElem(1,0,1);
    _verticalKernel.setElem(2,0,0);
    return;
  }

// Fill in the values
  double sum=0;
  for (int i=0; i<kernelSize; ++i)
  {
    int tmp = i-gaussSize;
    double value = exp(-(tmp*tmp)/(2*sigma2));
    _verticalKernel.setElem(i, 0, value);
    sum+=value;
  }

  // Normalize
  _verticalKernel/=sum;
}


// -----------------------------------------
CMatrix *CGaussianFilter::apply(const CMatrix &input, CMatrix *result) const
{
  result=CMatrix::convolve(input, _verticalKernel, result);
  result->convolveWith(_horizontalKernel);
  return result;
}



// -----------------------------------------
void CCartesianFilter::createXKernel(int dx)
{
  if (dx==1)
  {
    // Allocate memory for the kernel
    _xKernel.resize(1, 3);

    // Fill in the kernel matrix
    _xKernel.setElem(0, 0, -0.5);
    _xKernel.setElem(0, 1, 0);
    _xKernel.setElem(0, 2, 0.5);
  }
  else if (dx==2)
  {
    // Allocate memory for the kernel
    _xKernel.resize(1, 3);

    // Fill in the kernel matrix
    _xKernel.setElem(0, 0, 1);
    _xKernel.setElem(0, 1, -2);
    _xKernel.setElem(0, 2, 1);
  }
}


// -----------------------------------------
void CCartesianFilter::createYKernel(int dy)
{
  if (dy==1)
  {
    // Allocate memory for the kernel
    _yKernel.resize(3, 1);

    // Fill in the kernel matrix
    _yKernel.setElem(0, 0, -0.5);
    _yKernel.setElem(1, 0, 0);
    _yKernel.setElem(2, 0, 0.5);
  }
  else if (dy==2)
  {
    // Allocate memory for the kernel
    _yKernel.resize(3, 1);

    // Fill in the kernel matrix
    _yKernel.setElem(0, 0, 1);
    _yKernel.setElem(1, 0, -2);
    _yKernel.setElem(2, 0, 1);
  }
}



// -----------------------------------------
CMatrix *CCartesianFilter::apply(const CMatrix &input, CMatrix *result) const
{
  if (_xKernel.getCols())
  {
    result = CMatrix::convolve(input, _xKernel, result);
    if (_yKernel.getRows())
    {
      result->convolveWith(_yKernel);
    }
  }
  else if (_yKernel.getRows())
  {
    result = CMatrix::convolve(input, _yKernel, result);
  }

  return result;
}





