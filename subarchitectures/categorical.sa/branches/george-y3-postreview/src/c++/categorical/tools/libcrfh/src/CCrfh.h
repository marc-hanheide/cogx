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
 * \file CCrfh.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CCrfh class.
 */

#ifndef _CCRFH_H_
#define _CCRFH_H_

#include <QtCore/QMap>
#include <QtCore/QList>

class CMatrix;
class CDescriptorList;
class QTextStream;

/**
* Node of a libsvm sparse vector.
*/
struct CSvmNode
{
  int index;
  double value;
};


class CCrfh : public QMap<int, double>
{

public:

  /** Constructor. Creates a histogram from a set of
      outputs of descriptors. */
  CCrfh(QList<CMatrix *> outputs, const CDescriptorList &descrList, int skipBorderPixels);

  /** Zeroes small values in the histogram. The function removes those
      values that divided by maximum value are smaller than min_val. */
  void filter(double min_val);

  /** Normalizes the histogram - divides each bin by the sum of all. */
  void normalize();

  /** Serializes the histogram to a file in the libSVM format. */
  void serialize(QTextStream &stream);

  /** Returns a libSVM compatible sparse vector containing the histogram. */
  CSvmNode *getLibSvmVector();


private:

  /** Sum of all values before normalization. */
  double _sum;

  /** Maximal value before normalization. */
  double _max;

};


#endif


