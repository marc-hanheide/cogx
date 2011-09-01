

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <v4r/daisy/daisy.h>
using namespace kutility;

struct DaisyImagePair {
    daisy *pDescA;
    IplImage *pImgA;
    char pWndA[0x1F];
    daisy *pDescB;
    IplImage *pImgB;
    char pWndB[0x1F];
};

/**
* @brief draws a cross
* @param pImg target image
* @param rPoint
* @param rColor ( CV_RED, CV_BLUE, CV_GREEN, ...)
* @param iSize
**/
inline void DrawCross ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 3 ) {
    if ( ( rPoint.x + iSize >= pImg->width ) || ( rPoint.x - iSize < 0 ) ||
         ( rPoint.y + iSize >= pImg->height ) || ( rPoint.y - iSize < 0 ) ) {
        return;
    }
    CvPoint pt1 = cvPoint ( rPoint.x + iSize, rPoint.y + iSize );
    CvPoint pt2 = cvPoint ( rPoint.x - iSize, rPoint.y - iSize );
    CvPoint pt3 = cvPoint ( rPoint.x - iSize, rPoint.y + iSize );
    CvPoint pt4 = cvPoint ( rPoint.x + iSize, rPoint.y - iSize );
    cvLine ( pImg, pt1, pt2, rColor );
    cvLine ( pImg, pt3, pt4, rColor );
};


void findMatch ( DaisyImagePair *pPair, int pSrcX, int pSrcY, int *pMatchX, int *pMatchY ) {
    int descLength = pPair->pDescA->descriptor_size();
    float* pCompA = new float[descLength];
    pPair->pDescA->get_descriptor ( pSrcY, pSrcX, pCompA );
    float* pCompB = new float[descLength];
    float min = descLength*descLength*1000.0;
    *pMatchX = -1, *pMatchY = -1;
    for ( int y = 0; y < pPair->pDescA->height(); y++ ) {
        for ( int x = 0; x < pPair->pDescA->width(); x++ ) {

            float fSqDist = 0;
            float* da = pCompA;
            pPair->pDescB->get_descriptor ( y, x, pCompB );
            float* db = pCompB;
            for ( int i = 0 ; i < descLength; i++ ) {
                fSqDist += ( da[i] - db[i] ) * ( da[i] - db[i] );
            }
            if ( min > fSqDist ) {
                *pMatchX = x;
                *pMatchY = y;
                min = fSqDist;
            }
        }
    }
    std::cout << min << std::endl;
}

static void mouseCallBack ( int evt, int x, int y, int flags, void *param ) {
    DaisyImagePair *pPair = ( DaisyImagePair* ) param;
    int matchX, matchY;
    if ( evt == CV_EVENT_LBUTTONDOWN ) {
        printf ( ">>>>>>>>>>>>>>>> click left <<<<<<<<<<<<<<<<<<<\n" );
        DrawCross ( pPair->pImgA, cvPoint ( x,y ), cvScalar ( 0xff,0x00,0x00,0x00 ), 5 );
        std::cout << "Look  for: "  << x << ", " << y << std::endl;
        cvShowImage ( pPair->pWndA, pPair->pImgA );
        clock_t t_findMatch = clock();
        findMatch ( pPair, x, y, &matchX, &matchY );
        clock_t dt_findMatch = clock() - t_findMatch;
        printf ( "%5i cycles or %5i ms for two the findMatch function\n", ( int ) dt_findMatch, ( int ) ( ( dt_findMatch * 1000 ) / CLOCKS_PER_SEC ) );
        DrawCross ( pPair->pImgB, cvPoint ( matchX, matchY ), cvScalar ( 0x00,0x00,0xff,0x00 ), 5 );
        cvShowImage ( pPair->pWndB, pPair->pImgB );
        std::cout << "Match for: "  << matchX << ", " << matchY << std::endl;
    }
    if ( evt == CV_EVENT_RBUTTONDOWN ) {
        printf ( ">>>>>>>>>>>>>>>> click right <<<<<<<<<<<<<<<<<<<\n" );
        return;
    }
}

int main ( int argc, char** argv ) {
    if ( ( argc != 3 ) ) {
        printf ( "usage: %s <imageA> <imageB>\n", argv[0] );
        return 1;
    }
    DaisyImagePair pair;
    sprintf ( pair.pWndA,"%s", "ImageA" );
    sprintf ( pair.pWndB,"%s", "ImageB" );
    char* pFileImageA = argv[1];
    char* pFileImageB = argv[2];
    printf ( "A: %s\n", pFileImageA );
    printf ( "B: %s\n", pFileImageB );
    IplImage *ptImgA = cvLoadImage ( pFileImageA, 1 );
    IplImage *ptImgB = cvLoadImage ( pFileImageB, 1 );
    cvNamedWindow ( pair.pWndA, 1 );
    cvNamedWindow ( pair.pWndB, 1 );
    cvMoveWindow ( pair.pWndA, 100,0 );
    cvMoveWindow ( pair.pWndB, 100+400,0 );
    pair.pImgA = cvCreateImage ( cvSize ( ptImgA->width/2, ptImgA->height/2 ), IPL_DEPTH_8U, 3 );
    pair.pImgB = cvCreateImage ( cvSize ( ptImgB->width/2, ptImgB->height/2 ), IPL_DEPTH_8U, 3 );
    cvResize ( ptImgA, pair.pImgA );
    cvResize ( ptImgB, pair.pImgB );
	
    IplImage *pGrayA = cvCreateImage ( cvSize ( pair.pImgA->width, pair.pImgA->height ), IPL_DEPTH_8U, 1 );
    IplImage *pGrayB = cvCreateImage ( cvSize ( pair.pImgB->width, pair.pImgB->height ), IPL_DEPTH_8U, 1 );

    cvCvtColor ( pair.pImgA, pGrayA, CV_BGR2GRAY );
    cvCvtColor ( pair.pImgB, pGrayB, CV_BGR2GRAY );

    cvShowImage ( pair.pWndA, pair.pImgA );
    cvShowImage ( pair.pWndB, pair.pImgB );

    int rad   = 15;
    int radq  =  2;
    int thq   =  4;
    int histq =  4;
    int verbose_level = 0;

    pair.pDescA = new daisy();
    pair.pDescB = new daisy();

    clock_t t_set_image = clock();
    pair.pDescA->set_image ( ( unsigned char * ) pGrayA->imageData, pair.pImgA->height, pair.pImgA->width );
    pair.pDescB->set_image ( ( unsigned char * ) pGrayB->imageData, pair.pImgB->height, pair.pImgB->width );
    clock_t dt_set_image = clock() - t_set_image;
    printf ( "%5i cycles or %5i ms for two the set_image function\n", ( int ) dt_set_image, ( int ) ( ( dt_set_image * 1000 ) / CLOCKS_PER_SEC ) );
    pair.pDescA->verbose ( verbose_level );
    pair.pDescB->verbose ( verbose_level );
    pair.pDescA->set_parameters ( rad, radq, thq, histq );
    pair.pDescB->set_parameters ( rad, radq, thq, histq );



    clock_t t_initialize_single_descriptor_mode = clock();
    pair.pDescA->initialize_single_descriptor_mode();
    pair.pDescB->initialize_single_descriptor_mode();
    clock_t dt_initialize_single_descriptor_mode = clock() - t_initialize_single_descriptor_mode;
    printf ( "%5i cycles or %5i ms for two the initialize_single_descriptor_mode function\n", ( int ) dt_initialize_single_descriptor_mode, ( int ) ( ( dt_initialize_single_descriptor_mode * 1000 ) / CLOCKS_PER_SEC ) );

    clock_t t_compute_descriptors = clock();
    pair.pDescA->compute_descriptors();
    pair.pDescB->compute_descriptors();
    clock_t dt_compute_descriptorse = clock() - t_compute_descriptors;
    printf ( "%5i cycles or %5i ms for two the compute_descriptors function\n", ( int ) dt_compute_descriptorse, ( int ) ( ( dt_compute_descriptorse * 1000 ) / CLOCKS_PER_SEC ) );

    cvSetMouseCallback ( pair.pWndA, ( CvMouseCallback ) mouseCallBack, ( void* ) &pair );
    printf ( "%s\n", argv[1] );
    cvWaitKey ( 100000 );
    cvReleaseImage ( &pair.pImgA );
    cvReleaseImage ( &pair.pImgB );
    cvReleaseImage ( &ptImgA );
    cvReleaseImage ( &ptImgB );
    cvReleaseImage ( &pGrayA );
    cvReleaseImage ( &pGrayB );
    cvDestroyWindow ( pair.pWndA );
    cvDestroyWindow ( pair.pWndB );

    delete pair.pDescA;
    delete pair.pDescB;
    return 0;
}
