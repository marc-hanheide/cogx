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
 * \file CCrfh.cpp
 * \author Andrzej Pronobis
 *
 * Contains implementation of the CCrfh class.
 */

#include <QtCore/QVector>
#include <QtCore/QTextStream>
#include <limits>
#include "global.h"
#include "CMatrix.h"
#include "CDescriptorList.h"

#include "CCrfh.h"

using namespace std;


// -----------------------------------------
CCrfh::CCrfh(QList<CMatrix *> outputs, const CDescriptorList &descrList, int skipBorderPixels)
{
  // Check whether the descriptor list matches the output list
  if (outputs.size()!=descrList.size())
  {
    aout<<"ERROR: The size of the descriptor list does not match the size of the outputs list. "<<endl;
    return;
  }

  // Read all the necessary information and store it in the vectors
  QVector<CMatrix *> dataVect;
  QVector<double> minVect;
  QVector<double> maxVect;
  QVector<int> binsVect;
  int rows = outputs[0]->getRows();
  int cols = outputs[0]->getCols();
  int ndims = outputs.size();

  for (int i=0; i<ndims; ++i)
  {
    dataVect.append(outputs[i]);
    binsVect.append(descrList[i]->getBins());
    minVect.append(descrList[i]->getMin());
    maxVect.append(descrList[i]->getMax());
    if ( (rows != outputs[i]->getRows()) || (cols !=outputs[i]->getCols()) )
    {
      aout<<"ERROR: the filter outputs have different dimensions. "<<endl;
      return;
    }
  }

  // Direct access to vectors' arrays
  CMatrix **data = dataVect.data();
  double *min = minVect.data();
  double *max = maxVect.data();
  int *bins = binsVect.data();

  // Calculate scaling factors
  QVector<double> factorsVect;
  for (int i=0; i<ndims; ++i)
  {
    factorsVect.append( ((double)bins[i] - numeric_limits<double>::epsilon()) /
                        (max[i] - min[i]) );
  }
  double *factors = factorsVect.data();


  // Create the histogram
  _max=-1;
  for (int i=rows-1-skipBorderPixels; i>=skipBorderPixels; --i)  // Iterate through all pixels
    for (int j=cols-1-skipBorderPixels; j>=skipBorderPixels; --j) // without borders
    {
      // Number of bin for dimension k:
      // (int)((data[k][i]-min[k])*factors[k])

      int k = ndims-1;
      long index = (int)((data[k]->getElem(i,j)-min[k])*factors[k]);
      for (k=ndims-2; k>=0; --k)
        index = index * bins[k] + (int)((data[k]->getElem(i,j)-min[k])*factors[k]);
      double tmp=value(index)+1;
      if (tmp>_max) _max=tmp;
      insert(index, tmp);
    }

  // Store sum of all bins
  _sum=(rows-2*skipBorderPixels)*(cols-2*skipBorderPixels);
}

// -----------------------------------------
void CCrfh::serialize(QTextStream &stream)
{
  for (QMap<int, double>::const_iterator i=constBegin(); i!=constEnd(); ++i)
  {
    stream << i.key() << ":" << i.value()<<" ";
  }
}



// -----------------------------------------
void CCrfh::filter(double min_val)
{
  double tmp=min_val*_sum;

  QMap<int, double>::iterator i = begin();
  while (i != end())
  {
    if (i.value()<tmp)
    {
      //_sum-=i.value(); // Decreses the classification performance. We should not do it.
      i = erase(i);
    }
    else
      ++i;
  }
}


// -----------------------------------------
void CCrfh::normalize()
{
  for (QMap<int, double>::iterator i=begin(); i!=end(); ++i)
  {
    i.value()/=_sum;
  }
}


// -----------------------------------------
CSvmNode *CCrfh::getLibSvmVector()
{
  CSvmNode *vector = aMalloc<CSvmNode>(size()+1);

  int nr=0;
  for (QMap<int, double>::const_iterator i=constBegin(); i!=constEnd(); ++i, ++nr)
  {
    vector[nr].index=i.key();
    vector[nr].value=i.value();
  }

  vector[size()].index=-1;
  vector[size()].value=0.0;

  return vector;
}

