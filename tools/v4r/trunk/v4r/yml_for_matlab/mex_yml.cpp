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
#include <opencv/cv.h>
#include <opencv/highgui.h>

void mexFunction ( int nlhs, mxArray *plhs[], int nrhs,  const mxArray *prhs[] ) {

    if  (nrhs != 2) {
        mexPrintf ( "usage: <file> <variable> \n" );
        mexErrMsgTxt ( "Wrong command" );
    }

    mexPrintf("1\n");
    std::string file = mxArrayToString ( prhs[0] );
    std::string name = mxArrayToString ( prhs[1] );
    {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        mexPrintf("1\n");
        cv::Mat_<double> M = cv::Mat_<double>::eye(3,3);
        mexPrintf("1.1\n");
        fs << name << M;
        mexPrintf("1.2\n");
    }
    cv::Mat M;
    {
        cv::FileStorage fs(file, cv::FileStorage::READ);
        mexPrintf("2\n");
        mexPrintf("%s, %s\n", file.c_str(), name.c_str());
        mexPrintf("2.1\n");
        fs[name] >> M;
        mexPrintf("2.2\n");
    }
    mwSize dims[2];
    dims[2] = M.rows;
    dims[1] = M.cols;

    int type = CV_MAT_TYPE(M.type());
    mexPrintf ( "[%i x %i] ***", M.rows, M.cols );
    switch (type) {
    case CV_8U:
        mexPrintf ( " Type: CV_8U\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxUINT8_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_8S:
        mexPrintf ( " Type: CV_8S\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxINT8_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_16U:
        mexPrintf ( " Type: CV_16U\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxUINT16_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_16S:
        mexPrintf ( " Type: CV_16S\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxINT16_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_32S:
        mexPrintf ( " Type: CV_32S\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxINT32_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_32F:
        mexPrintf ( " Type: CV_32F\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxSINGLE_CLASS, ( mxComplexity ) 0 );
        break;
    case CV_64F:
        mexPrintf ( " Type: CV_64F\n");
        plhs[0] = mxCreateNumericArray ( 2, dims, mxDOUBLE_CLASS, ( mxComplexity ) 0 );
        break;
    default:
        mexPrintf ( " Type: NA\n");
    }
    double *v = mxGetPr ( plhs[0] );
    memcpy(v, M.data, M.cols * M.step);

}
// kate: indent-mode cstyle; space-indent on; indent-width 0;
