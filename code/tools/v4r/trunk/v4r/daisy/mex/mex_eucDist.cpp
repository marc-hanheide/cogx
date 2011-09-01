/**
 * @file mxSqEucDist.c
 * @author Philipp Blauensteiner, Markus Bader
 * @date 23. Aug. 2008
 * @version 0.2
 * @brief
 **/




#include <mex.h>
#include <math.h>

void usage()
{
    mexErrMsgTxt("usage: [DISTMAP MINDIST MC MR] = MXSQEUCDIST(MAP,COMP)");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	
    if (nlhs == 0){
        usage();
		}
    if (nrhs != 2){
        usage();
		}
	
    mwSize  nDim = mxGetNumberOfDimensions(prhs[0]);
    const mwSize *pDim = mxGetDimensions(prhs[0]);
    
        
    
    mwSize descLength = pDim[0];
    mwSize width = pDim[1];
    mwSize height = pDim[2];
    
    if (mxGetNumberOfElements(prhs[1])!=descLength)
        usage();

    if ( mxIsSingle(prhs[0]) != 1  &&  mxIsSingle(prhs[1]) != 1 ){
        mexErrMsgTxt("All parameters must be a single (float)!"); 
    }

    float *pMap = (float *) mxGetData(prhs[0]);
    float *pComp =(float *) mxGetData(prhs[1]);
    
    mwSize dims[2];
    int colLen = 3;
    int rowLen = width*height;

    plhs[0] = mxCreateNumericMatrix(rowLen, colLen,mxSINGLE_CLASS, mxREAL);
    float *pSqDist = (float*) mxGetData(plhs[0]);
    float *pX = pSqDist + (width * height * 1);
    float *pY = pSqDist + (width * height * 2);
    
    float fSqDist;
		int h, w, d;
		int mc = -1, mr = -1;
		float min = descLength*descLength*1000.0;
    for ( h = 1; h < height+1; h++)
    {
        for ( w = 1; w < width+1; w++)
        {
						fSqDist = 0;
						pComp =  (float *) mxGetData(prhs[1]);
            for ( d = 0 ; d < descLength; d++)
            {
                fSqDist += ((*pMap)-(*pComp)) * ((*pMap)-(*pComp));
                pMap++; 
								pComp++;
            }
						if(min > fSqDist){
							mc = w;
							mr = h;
							min = fSqDist;
						}
            *pSqDist++ = fSqDist; 
            *pX++ = w; 
            *pY++ = h;
        }
    }

    if (nlhs > 1)
        plhs[1] = mxCreateDoubleScalar(min);
    
    if (nlhs>2)
        plhs[2] = mxCreateDoubleScalar(mc);
    
    if (nlhs>3)
        plhs[3] = mxCreateDoubleScalar(mr);
}
