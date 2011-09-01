/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader                                    *
 *   bader@acin.tuwien.ac.at                                               *
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


#include <mex.h>
#include <v4r/utils/mexhlp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sharedmemmanager.hpp>


void mexFunction ( int nlhs, mxArray *plhs[], int nrhs,  const mxArray *prhs[] ) {
    if  (nrhs != 0) {
        mexPrintf ( "Wrong number of arguments: nrhs = %i. ", nrhs );
        mexErrMsgTxt ( "usage: mex_shmlist()\n" );
    }
    std::vector< std::string > shmNames;
    AK::SharedMemManager::getSingleton()->listNames(shmNames);
    plhs[0]= V4R::getVector2MxArray(shmNames); 
}
// kate: indent-mode cstyle; space-indent on; indent-width 0;
