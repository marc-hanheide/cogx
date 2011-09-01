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

    if (( nrhs < 5 ) || ( nrhs > 6 )) {
        mexPrintf ( "usage mex: command(ptrNaoQi, resolution, colorSpace, fps, camera, <format [1 == Matlab, 0 = C++]> ) \n" );
        mexPrintf ( "number of parameters: %i\n", nrhs );
        mexErrMsgTxt ( "wrong number of paramters\n" );
    }

    AK::NaoQi *pNaoQi = (AK::NaoQi *) AK::getMxArray2Ptr ( prhs[0] );
    int resolution = AK::getMxArray2Int ( prhs[1] );
    int colorSpace = AK::getMxArray2Int ( prhs[2] );
    int fps = AK::getMxArray2Int ( prhs[3] );
    int camera = AK::getMxArray2Int ( prhs[4] );
		AK::NaoQi::ImageMemFormat data_format = AK::NaoQi::IN_COLUMS;
		if(nrhs > 4){
			data_format = (AK::NaoQi::ImageMemFormat) AK::getMxArray2Int ( prhs[5] );
		}
				
    int rows, cols, channels;
		
		if(pNaoQi->getCameraImageSize(&cols, &rows, &channels)){
			pNaoQi->subscribeCamera(resolution, colorSpace, fps, camera );
			pNaoQi->getCameraImageSize(&cols, &rows, &channels);
		}
    
    mwSize dims[] =  {rows, cols, channels };
    mxArray *pImg = mxCreateNumericArray ( 3, dims, mxUINT8_CLASS, ( mxComplexity ) 0 );
		pNaoQi->getCameraImage((char* )  mxGetPr ( pImg ), data_format);
    plhs[0] = pImg;
}
