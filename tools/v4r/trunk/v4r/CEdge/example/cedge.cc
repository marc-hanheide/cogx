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
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <limits.h>
#include <PNamespace.hh>
#include <Array.hh>
#include <Vector2.hh>
#include <CEdge.hh>
#include <SDraw.hh>
#include <Line.hh>
#include <FormArcs.hh>
#include <FormLines.hh>
#include <CreateMSLD.hh>


#define CREATE_LINES
//#define CREATE_ARCS
#define CREATE_MSLD
#define SUB_PX

#define LINE_LENGTH_THR 15

using namespace P;



/******************************** main *********************************/
int main(int argc, char** argv)
{
  int c=INT_MAX;
  IplImage* img0 = 0, *img = 0, *dx=0, *dy=0, *edge=0, *dbg=0, *grey=0;

  CEdge getEdge;
  FormArcs formArcs;
  FormLines formLines;
  CreateMSLD createMSLD;

  Array<Segment *> segments;
  Array<Line *> lines;
  Array<Arc *> arcs;
  Segment *seg;
  char filename[PATH_MAX];
 
  printf( "Hot keys: \n"
          "\tESC - quit the program\n"
          "\tr - restore the original image\n"
          "\ts - run edge detection\n");

  cvNamedWindow( "image", 1 );
  cvNamedWindow( "result", 1 );

  if (argc != 2)
  {
    printf("%s image\n",argv[0]);
    exit(0);
  }
  
  for(unsigned z=0;;z++)
  {
    snprintf(filename,PATH_MAX,argv[1],z);

    if((img0=cvLoadImage(filename,1)) != 0 )
    {
      

      if (img==0)
      {
        img = cvCreateImage(cvGetSize(img0), img0->depth, img0->nChannels );
        edge = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
        dx = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
        dy = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
        edge = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
        dbg = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 3 );
        grey = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
      }


      cvCopy(img0, img );
      cvConvertImage(img0, grey);


      if( (char)c == 27 )
        break;

      if( (char)c == 'r' )
      {
          cvCopy( img0, img );
          cvZero(dbg);
          cvShowImage( "image", img );
          
      }

      if( (char)c == 's')
      {
        ///do...
        struct timespec start1, end1, start2, end2, start3, end3;
        clock_gettime(CLOCK_REALTIME, &start1);
        getEdge.Set(false, 3);
        getEdge.Sobel(grey, dx, dy); 
        getEdge.Canny(dx, dy, edge, 30, 60);
        getEdge.LinkEdge(edge, segments, dx, dy);
        clock_gettime(CLOCK_REALTIME, &end1);
        cout<<"Time detect edge segments [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

        #ifdef CREATE_LINES
        clock_gettime(CLOCK_REALTIME, &start2);
        DeleteLines(lines);
        formLines.Operate(segments,lines);
        #ifdef SUB_PX
        formLines.GetSubPixel(dx,dy,lines,10);
        #endif
        clock_gettime(CLOCK_REALTIME, &end2);
        cout<<"Time create lines [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
        #endif

        #ifdef CREATE_ARCS
        clock_gettime(CLOCK_REALTIME, &start3);
        DeleteArcs(arcs);
        formArcs.Operate(segments, arcs);
        clock_gettime(CLOCK_REALTIME, &end3);
        cout<<"Time create arcs [s]: "<<P::timespec_diff(&end3, &start3)<<endl;
        #endif

        #ifdef CREATE_MSLD
        clock_gettime(CLOCK_REALTIME, &start3);
        createMSLD.Operate(grey,lines);
        clock_gettime(CLOCK_REALTIME, &end3);
        cout<<"Time create MSLD [s]: "<<P::timespec_diff(&end3, &start3)<<endl;
        #endif
        //draw segments...
        cvZero(dbg);
        int mean=0;
        for (unsigned i=0; i<segments.Size(); i++)
        {
          uchar r = 100+rand()%155;
          uchar g = 100+rand()%155;
          uchar b = 100+rand()%155;
          seg = segments[i];
          mean+=segments[i]->edgels.Size();
          for (unsigned j=0; j<segments[i]->edgels.Size(); j++)
          {
            SetPx8UC3(dbg, seg->edgels[j].p.x, seg->edgels[j].p.y, r, g, b);
          }
        }
        cout<<"Number of edges: "<<segments.Size()<<endl;
        cout<<"Mean edge length: "<<(double)mean / (double)segments.Size()<<endl;

        //draw lines....
        unsigned cnt=0;
        for (unsigned i=0; i<lines.Size(); i++)
          if (lines[i]->len > LINE_LENGTH_THR)
          {
            lines[i]->Draw(img);
cout<<"x/y-x/y: "<<lines[i]->point[0]<<"-"<<lines[i]->point[1]<<endl;
            cnt++;
          }
        cout<<"Number of lines: "<<lines.Size()<<endl;
        cout<<"Number of lines longer than "<<LINE_LENGTH_THR<<" Px: "<<cnt<<endl;

        //draw arcs...
        for (unsigned i=0; i<arcs.Size(); i++)
        {
          arcs[i]->Draw(img,1);
        }
        cout<<"Number of arcs: "<<arcs.Size()<<endl;
          
        //cvSaveImage("log/testLinking.jpg",dbg);   

        DeleteLines(lines);
        DeleteSegments(segments);
        DeleteArcs(arcs);

      }
      cvShowImage( "image", img );
      cvShowImage( "result", dbg);

      c = cvWaitKey(0);

    }
    cvReleaseImage( &img0 );
  }

  cvReleaseImage( &img);
  cvReleaseImage( &dx);
  cvReleaseImage( &dy);
  cvReleaseImage( &edge);
  cvReleaseImage( &dbg);


  return 0;
}












