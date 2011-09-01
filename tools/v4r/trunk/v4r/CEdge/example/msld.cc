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
#include <Math.hh>


#define CREATE_MSLD
#define RANSAC_ETA0 0.01
#define RANSAC_INL_DIST 2.
#define ANGLE_THR 10.
#define SUB_PX

using namespace P;

void MatchLines(Array<Line*> &lns1, Array<Line*> &lns2, Array<unsigned> &idx1, Array<unsigned> &idx2);
void DrawLines(IplImage *img1, IplImage *img2, Array<Line*> &lns1, Array<Line*> &lns2, Array<unsigned> &idx1, Array<unsigned> &idx2);
void GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx);





/******************************** main *********************************/
int main(int argc, char** argv)
{

  if (argc != 3)
  {
    printf("%s image1 image2\n",argv[0]);
    exit(0);
  }


  int c=INT_MAX;
  IplImage* img0 = 0, *img1 = 0, *dx=0, *dy=0, *edge=0, *img2=0, *grey=0;

  CEdge getEdge;
  FormArcs formArcs;
  FormLines formLines;
  CreateMSLD createMSLD;

  Array<Segment *> segments1, segments2;
  Array<Line *> lines1, lines2;

  char filename1[PATH_MAX];
  char filename2[PATH_MAX];
 

  cvNamedWindow( "image1", 1 );
  cvNamedWindow( "image2", 1 );

  if (argc != 3)
    exit(0);
  
  snprintf(filename1,PATH_MAX,argv[1], c);
  snprintf(filename2,PATH_MAX,argv[2], c);

  struct timespec start1, end1, start2, end2, start3, end3;

  /************************** IMAGE 1 ********************************/
  if((img0=cvLoadImage(filename1,1)) != 0 )
  {
    if (img1==0)
    {
      img1 = cvCreateImage(cvGetSize(img0), img0->depth, img0->nChannels );
      
      edge = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
      dx = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
      dy = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
      grey = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
    }


    cvCopy(img0, img1 );
    cvConvertImage(img0, grey);


    ///do...
    clock_gettime(CLOCK_REALTIME, &start1);
    getEdge.Set(false, 3);
    getEdge.Sobel(grey, dx, dy); 
    getEdge.Canny(dx, dy, edge, 70, 150);
    getEdge.LinkEdge(edge, segments1, dx, dy);
    clock_gettime(CLOCK_REALTIME, &end1);
    cout<<"Time detect edge segments [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

    clock_gettime(CLOCK_REALTIME, &start2);
    DeleteLines(lines1);
    formLines.Operate(segments1,lines1);
    clock_gettime(CLOCK_REALTIME, &end2);
    cout<<"Time create lines [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
    clock_gettime(CLOCK_REALTIME, &start2);
    #ifdef SUB_PX
    formLines.GetSubPixel(dx,dy, lines1, 10.);
    #endif
    cout<<"Time refine lines subpixel [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
    clock_gettime(CLOCK_REALTIME, &start2);

    clock_gettime(CLOCK_REALTIME, &start3);
    createMSLD.Operate(grey, lines1);
    clock_gettime(CLOCK_REALTIME, &end3);
    cout<<"Time create MSLD [s]: "<<P::timespec_diff(&end3, &start3)<<endl;

    //draw lines....
    //unsigned cnt=0;
    /*for (unsigned i=0; i<lines1.Size(); i++)
      {
        lines1[i]->Draw(img1);
        cnt++;
      }*/
    cout<<"Number of lines: "<<lines1.Size()<<endl;

    //cvSaveImage("log/testLinking.jpg",dbg);   

    cvReleaseImage( &img0 );
    cvReleaseImage( &dx);
    cvReleaseImage( &dy);
    cvReleaseImage( &edge);
    cvReleaseImage( &grey);
  }

  /************************** IMAGE 1 ********************************/
  if((img0=cvLoadImage(filename2,1)) != 0 )
  {
    if (img2==0)
    {
      img2 = cvCreateImage(cvGetSize(img0), img0->depth, img0->nChannels );
      
      edge = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
      dx = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
      dy = cvCreateImage(cvGetSize(img0), IPL_DEPTH_16S, 1 );
      grey = cvCreateImage(cvGetSize(img0), IPL_DEPTH_8U, 1 );
    }


    cvCopy(img0, img2 );
    cvConvertImage(img0, grey);


    ///do...
    clock_gettime(CLOCK_REALTIME, &start1);
    getEdge.Set(false, 3);
    getEdge.Sobel(grey, dx, dy); 
    getEdge.Canny(dx, dy, edge, 70, 150);
    getEdge.LinkEdge(edge, segments2, dx, dy);
    clock_gettime(CLOCK_REALTIME, &end1);
    cout<<"Time detect edge segments [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

    clock_gettime(CLOCK_REALTIME, &start2);
    DeleteLines(lines2);
    formLines.Operate(segments2,lines2);
    clock_gettime(CLOCK_REALTIME, &end2);
    cout<<"Time create lines [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
    clock_gettime(CLOCK_REALTIME, &start2);
    #ifdef SUB_PX
    formLines.GetSubPixel(dx,dy, lines2, 10.);
    #endif
    cout<<"Time refine lines subpixel [s]: "<<P::timespec_diff(&end2, &start2)<<endl;
    clock_gettime(CLOCK_REALTIME, &start2);

    clock_gettime(CLOCK_REALTIME, &start3);
    createMSLD.Operate(grey, lines2);
    clock_gettime(CLOCK_REALTIME, &end3);
    cout<<"Time create MSLD [s]: "<<P::timespec_diff(&end3, &start3)<<endl;

    //draw lines....
    /*unsigned cnt=0;
    for (unsigned i=0; i<lines2.Size(); i++)
      {
        lines2[i]->Draw(img2);
        cnt++;
      }*/
    cout<<"Number of lines: "<<lines2.Size()<<endl;

    //cvSaveImage("log/testLinking.jpg",dbg);   

    cvReleaseImage( &img0 );
    cvReleaseImage( &dx);
    cvReleaseImage( &dy);
    cvReleaseImage( &edge);
    cvReleaseImage( &grey);
  }

  Array<unsigned> idx1, idx2;
  MatchLines(lines1, lines2, idx1, idx2);

  DrawLines(img1, img2, lines1, lines2, idx1, idx2);

  unsigned cnt=0;  
  for (unsigned i=0; i<lines1.Size(); i++)
    if (lines1[i]->vec!=0)
      cnt++;
  cout<<"Number of lines with descriptor in image 1: "<<cnt<<endl;
  cnt=0;
  for (unsigned i=0; i<lines2.Size(); i++)
    if (lines2[i]->vec!=0)
      cnt++;
  cout<<"Number of lines with descriptor in image 2: "<<cnt<<endl;
  cout<<"Number of matched lines: "<<idx1.Size()<<endl;


  cvShowImage( "image1", img1 );
  cvShowImage( "image2", img2);

  while (((char)cvWaitKey(10)) != 27 ){};

  cvReleaseImage( &img1);
  cvReleaseImage( &img2);

  DeleteLines(lines1);
  DeleteLines(lines2);
  DeleteSegments(segments1);
  DeleteSegments(segments2);



  return 0;
}





void MatchLines(Array<Line*> &lns1, Array<Line*> &lns2, Array<unsigned> &idx1, Array<unsigned> &idx2)
{
  unsigned idx, bidx;
  float dist, mindist;
  idx1.Clear();
  idx2.Clear();

  Line *l;

  for (unsigned i=0; i<lns1.Size(); i++)
  {
    if (lns1[i]->vec!=0)
    {
      mindist=FLT_MAX;
      for (unsigned j=0; j<lns2.Size(); j++)     // forward matching
      {
        if (lns2[j]->vec!=0)
        {
          dist = DistSqr(lns1[i]->vec,lns2[j]->vec, lns1[i]->size);
          if (dist < mindist)
          {
            mindist=dist;
            idx=j;
          }
        }
      }
      if (mindist < 0.55)                        // if fixed threshold is passed
      {
        l = lns2[idx];
        mindist=FLT_MAX;
        for (unsigned j=0; j<lns1.Size(); j++)   // check backward
        {
          if (lns1[j]->vec!=0)
          {
            dist = DistSqr(l->vec,lns1[j]->vec, l->size);
            if (dist < mindist)
            {
              mindist=dist;
              bidx=j;
            }
          }
        }

        if (bidx == i)
        {
          idx1.PushBack(i);
          idx2.PushBack(idx);
        }
      } 
    }
  }


}


void DrawLines(IplImage *img1, IplImage *img2, Array<Line*> &lns1, Array<Line*> &lns2, Array<unsigned> &idx1, Array<unsigned> &idx2)
{
  if (idx1.Size()!=idx2.Size())
    return;

  CvScalar col;
  Line *l1, *l2;
  Vector2 m1, m2;

  for (unsigned i=0; i<idx1.Size(); i++)
  {
    col = CV_RGB(rand()%255, rand()%255, rand()%255);

    l1 = lns1[idx1[i]];
    SDraw::DrawLine(img1, l1->point[0].x, l1->point[0].y,l1->point[1].x, l1->point[1].y, col,2);

    l2 = lns2[idx2[i]];
    SDraw::DrawLine(img2, l2->point[0].x, l2->point[0].y,l2->point[1].x, l2->point[1].y, col,2);

    m1 = MidPoint(l1->point[0],l1->point[1]);
    m2 = MidPoint(l2->point[0],l2->point[1]);
    SDraw::DrawLine(img2, m1.x, m1.y, m2.x, m2.y, CV_RGB(100,100,255));
  }
}













