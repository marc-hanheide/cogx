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

    if  (nrhs%3 != 1) {
        mexPrintf ( "usage: command(name, value, name, value, ...) \n");
        mexErrMsgTxt ( "wrong number of paramters\n" );
    }

		
    std::vector<std::string> names;
    std::vector<float> stiffnessLists;
    std::vector<float> timeLists;
    for( int i = 1; i < nrhs; ){
      std::string name = mxArrayToString(prhs[i++]);
      names.push_back(name);
      double stiffnes = AK::getMxArray2Double ( prhs[i++] );
      stiffnessLists.push_back(stiffnes);
      double time = AK::getMxArray2Double  ( prhs[i++] );
      timeLists.push_back(time);
    }
    
 
    AK::NaoQi *pNaoQi = (AK::NaoQi *) AK::getMxArray2Ptr( prhs[0] );
    pNaoQi->stiffnessInterpolation  (names, stiffnessLists, timeLists);
}
