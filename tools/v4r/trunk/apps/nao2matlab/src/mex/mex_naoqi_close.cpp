/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include "mex.h"

#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

#include "bridge/naoqi.h"
#include "mxhlp.h"

void mexFunction ( int nlhs, mxArray *plhs[], int nrhs,  const mxArray *prhs[] ) {

    if  (nrhs != 1) {
        mexPrintf ( "usage: command ()\n");
        mexErrMsgTxt ( "wrong number of paramters\n" );
    }
    AK::NaoQi *pNaoQi = (AK::NaoQi *) AK::getMxArray2Ptr( prhs[0] );
    delete pNaoQi;
}
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
