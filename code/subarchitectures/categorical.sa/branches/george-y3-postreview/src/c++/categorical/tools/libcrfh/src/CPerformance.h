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
 * \file CPerformance.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CPerformance class.
 */

#ifndef _CPERFORMANCE_H_
#define _CPERFORMANCE_H_

#include <QtCore/QStack>

/**
* Class providing methods that can be used
* to measure performance of the program.
*/
class CPerformance
{

public:

  /** See Matlab's help. */
  static void tic();

  /** See Matlab's help. */
  static double toc(bool verbose=true);

  /** Returns the number of seconds that elapsed since
      the epoch as a double with 1us accuracy. */
  static double getSecsSinceEpoch();

private:

  /** A stack holding time samples acquired by
      the function tic(). */
  static QStack<double> ticStack;

};


#endif
