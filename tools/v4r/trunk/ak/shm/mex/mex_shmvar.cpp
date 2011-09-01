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
#include <ak/sharedmem/sharedmemmanager.hpp>
#include <ak/sharedmem/sharedmem.hpp>
#include <ak/sharedmem/sharedmemheader.hpp>

void mexFunction ( int nlhs, mxArray *plhs[], int nrhs,  const mxArray *prhs[] ) {
    if  (nrhs != 1) {
        mexPrintf ( "Wrong number of arguments: nrhs = %i. ", nrhs );
        mexErrMsgTxt ( "usage: mex_shmvar('ShmName')\n" );
    }

    std::string shmName = mxArrayToString ( prhs[0] );
    AK::SharedMemHeaderNA* pHeader = (AK::SharedMemHeaderNA*)  AK::SharedMemManager::getSingleton()->findName(shmName);
    if (pHeader == NULL) {
        mexPrintf ( "shared name = %s. --> ", shmName.c_str() );
        mexErrMsgTxt ( "No such shared name!\n" );
    }
    if(!pHeader->timed_lock(5000)){
        mexErrMsgTxt ( "Could not look shmvar in time (5sec)\n" );
    }
    unsigned int columns, rows, typeEntry;
    rows = AK::SharedMem::getVectorOrArraySize(pHeader);
    typeEntry = AK::SharedMem::getEntryType(pHeader->getVarType(), &columns);
    mwSize dims[] = {rows, columns};
    mxArray *pMx;
    void *pShmData = (void*) AK::SharedMem::getFirst(pHeader);
    if(pShmData){

    switch (typeEntry) {
    case AK::SharedMem::VAR_INT8:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxINT8_CLASS, ( mxComplexity ) 0 );
        int8_t *pSrc = (int8_t*) pShmData;
        int8_t *pDes = (int8_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_INT16:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxINT16_CLASS, ( mxComplexity ) 0 );
        int16_t *pSrc = (int16_t*) pShmData;
        int16_t *pDes = (int16_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_INT32:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxINT32_CLASS, ( mxComplexity ) 0 );
        int32_t *pSrc = (int32_t*) AK::SharedMem::getFirst(pHeader);
        int32_t *pDes = (int32_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_INT64:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxINT64_CLASS, ( mxComplexity ) 0 );
        int64_t *pSrc = (int64_t*) AK::SharedMem::getFirst(pHeader);
        int64_t *pDes = (int64_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_UINT8:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxUINT8_CLASS, ( mxComplexity ) 0 );
        uint8_t *pSrc = (uint8_t*) AK::SharedMem::getFirst(pHeader);
        uint8_t *pDes = (uint8_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_UINT16:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxUINT16_CLASS, ( mxComplexity ) 0 );
        uint16_t *pSrc = (uint16_t*) AK::SharedMem::getFirst(pHeader);
        uint16_t *pDes = (uint16_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_UINT32:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxUINT32_CLASS, ( mxComplexity ) 0 );
        uint32_t *pSrc = (uint32_t*) AK::SharedMem::getFirst(pHeader);
        uint32_t *pDes = (uint32_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_UINT64:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxUINT64_CLASS, ( mxComplexity ) 0 );
        uint64_t *pSrc = (uint64_t*) AK::SharedMem::getFirst(pHeader);
        uint64_t *pDes = (uint64_t*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_F32:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxSINGLE_CLASS, ( mxComplexity ) 0 );
        float *pSrc = (float*) AK::SharedMem::getFirst(pHeader);
        float *pDes = (float*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    case AK::SharedMem::VAR_F64:
    {
        pMx = mxCreateNumericArray ( 2, dims, mxDOUBLE_CLASS, ( mxComplexity ) 0 );
        double *pSrc = (double*) AK::SharedMem::getFirst(pHeader);
        double *pDes = (double*) mxGetPr ( pMx );
        for ( unsigned int i = 0; i < rows * columns; i++ )  pDes[i] = pSrc[i];
    }
    break;
    default:
        mexErrMsgTxt ( "unsupportet Format\n" );
    }
    pHeader->unlock();
    plhs[0] = pMx;
    }

}
// kate: indent-mode cstyle; space-indent on; indent-width 0;
