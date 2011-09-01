/**
 * @file main
 * @author Johann Prankl
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#include <vector>
#include <time.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <limits.h>
#include "v4r/LPSegment/LPSegment.hh"


IplImage* img0 = 0, *img = 0, *dbg = 0;
CvPoint prev_pt = {-1,-1};
P::vector<cv::Point2d> contour, segContour;

void on_mouse( int event, int x, int y, int flags, void* )
{
    if( !img )
        return;

    if( event == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON) )
        prev_pt = cvPoint(-1,-1);
    else if( event == CV_EVENT_LBUTTONDOWN )
        prev_pt = cvPoint(x,y);
    else if( event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) )
    {
        contour.push_back(cv::Point2d((double)x, (double)y));
        CvPoint pt = cvPoint(x,y);
        if( prev_pt.x < 0 )
        {
            prev_pt = pt;
        }
        cvLine( img, prev_pt, pt, cvScalarAll(255), 2, 8, 0 );
        prev_pt = pt;
        cvShowImage( "image", img );
    }
}


/******************************** main *********************************/
int main(int argc, char** argv)
{
  P::LPSegment seg;
  
  if( argc == 2 && (img0=cvLoadImage(argv[1],1)) != 0 )
  {
    printf( "Hot keys: \n"
            "\tESC - quit the program\n"
            "\tr - restore the original image\n"
            "\ts - run segmentation\n"
            "\t\t(before running it, paint a prior contour)\n" );

    cvNamedWindow( "image", 1 );

    img = cvCloneImage( img0 );
    dbg = cvCloneImage( img0 );

    cvShowImage( "image", img );
    cvSetMouseCallback( "image", on_mouse, 0 );
    for(;;)
    {
        int c = cvWaitKey(0);

        if( (char)c == 27 )
            break;

        if( (char)c == 'r' )
        {
            contour.clear();
            cvCopy( img0, img );
            cvCopy( img0, dbg );
            cvShowImage( "image", img );
        }

        if( (char)c == 's')
        {
          ///do...
          if (contour.size()>0)
          {
            seg.SetDebugImage(dbg);
            cv::Mat matImage0(img0);
            seg.Operate(matImage0, segContour,contour[0], 50, 25,cv::Mat());
          }

          //draw...
          cvCopy( dbg, img );
          cv::Mat matImage = img;
          seg.Draw(matImage,segContour);
            
          contour.clear();
          cvShowImage( "image", img );
        }
    }

  }
  else
  {
    printf("%s image\n",argv[0]);
  }
  return 0;
}












