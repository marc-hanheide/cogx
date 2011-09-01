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
    std::cout << "nrhs" << nrhs << std::endl;
    if ( nrhs < 3 ) {
        mexPrintf ( "usage mex: command(ptrNaoQi, name1, name2, ... , useSensorValues ) \n" );
        mexPrintf ( "number of parameters: %i\n", nrhs );
        mexErrMsgTxt ( "wrong number of paramters\n" );
    }

    AK::NaoQi *pNaoQi = (AK::NaoQi *) AK::getMxArray2Ptr ( prhs[0] );
    std::vector<std::string> names;
    for ( int i = 1; i < nrhs-1; ) {
        std::string name = mxArrayToString ( prhs[i++] );
        names.push_back ( name );
    }
    bool useSensorValues = AK::getMxArray2Double ( prhs[nrhs-1] );


    std::vector<float> angles = pNaoQi->getAngles ( names, useSensorValues);
    //std::cout << "<" << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << pos[3] << ", " << pos[4] << ", " << pos[5] << std::endl;

    plhs[0] = AK::getVector2MxArray(angles);

}
