/**
 * @file mxDaisy.cpp
 * @author Philipp Blauensteiner, Markus Bader
 * @date 23. Aug. 2008
 * @version 0.2
 * @brief
 **/

#include <mex.h>
#include <v4r/daisy/daisy.h>
#include "math.h"

using namespace std;
using namespace kutility;

bool file_exists ( const char * filename ) {
    if ( FILE * file = fopen ( filename, "r" ) ) {
        fclose ( file );
        return true;
    }
    return false;
}


void usage() {
    mexErrMsgTxt ( "USAGE: [DESC] = mxDaisyInterface(IMG, OPTIONS)\n" );
}

void mexFunction ( int nhls, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
   
    char* strFileName;
    mexPrintf("HAllo\n");
    
    int w, h;
    int opy = -1;
    int opx = -1;

    float rad   = 15;
    int radq  =  3;
    int thq   =  8;
    int histq =  8;
    int verbose_level = 0;

    int orientation_resolution = 18;
    bool rotation_inv = false;

    char buffer[10];
    char* filename=NULL;
    daisy* desc = NULL;


    if ( nrhs<1 || nrhs>2 ) {
        usage();
    }

    // get options, if specified
    if ( nrhs==2 ) {
        if ( !mxIsStruct ( prhs[1] ) ) {
            usage();
            return;
        }
        mxArray* temp;
        temp = mxGetField ( prhs[1],0,"R" );
        if ( !mxIsNumeric ( temp ) ) {
            usage();
            return;
        }
        rad = static_cast<float> ( mxGetPr ( temp ) [0] );

        temp = mxGetField ( prhs[1],0,"RQ" );
        if ( !mxIsNumeric ( temp ) ) {
            usage();
            return;
        }
        radq = static_cast<int> ( round ( mxGetPr ( temp ) [0] ) );

        temp = mxGetField ( prhs[1],0,"THQ" );
        if ( !mxIsNumeric ( temp ) ) {
            usage();
            return;
        }
        thq = static_cast<int> ( round ( mxGetPr ( temp ) [0] ) );

        temp = mxGetField ( prhs[1],0,"HQ" );
        if ( !mxIsNumeric ( temp ) ) {
            usage();
            return;
        }
        histq = static_cast<int> ( round ( ( mxGetPr ( temp ) ) [0] ) );

    }

    h = mxGetDimensions ( prhs[0] ) [0] ;
    w = mxGetDimensions ( prhs[0] ) [1] ;
    //convert matlab image
    unsigned char *img = ( unsigned char* ) malloc ( 3*h*w );
    {
        unsigned char *des = img;
        unsigned char *tmp = ( unsigned char* ) mxGetData ( prhs[0] ) ;
        unsigned char *src = tmp ;
        for ( int y = 0; y < h; y++ ) {
            for ( int x = 0; x < w; x++ ) {
                src = tmp + ( ( x*h+y ) *3 );
                *des++ = *src++;
                *des++ = *src++;
                *des++ = *src++;
            }
        }
    }
    std::cout << "new daisy()" << std::endl;
    desc = new daisy();
    desc->set_image ( img,h,w );
    free ( img );
    desc->verbose ( verbose_level );
    desc->set_parameters ( rad, radq, thq, histq );

    desc->initialize_single_descriptor_mode();
    desc->compute_descriptors();

    float* descriptors = desc->get_dense_descriptors();


    mwSize dims[3];
    dims[2] = h;
    dims[1] = w;
    dims[0] = desc->descriptor_size();

    plhs[0] = mxCreateNumericArray ( 3,dims,mxSINGLE_CLASS,mxREAL );
    float* d = reinterpret_cast<float*> ( mxGetData ( plhs[0] ) );
    memcpy ( d,descriptors,dims[0]*dims[1]*dims[2]*sizeof ( float ) );



    delete desc;

}

