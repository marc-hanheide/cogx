/**
 * $Id$
 * Johann Prankl, 2010-03-30 
 * prankl@acin.tuwien.ac.at
 */


#include "DetectMSLD.hh"
#include "SDraw.hh"


namespace P
{


float DetectMSLD::MIN_LINE_LENGTH = 25.; //15.;       // only use lines longer than...
bool DetectMSLD::SUB_PIXEL_LINES = true;        // detect lines with subpixel accuracy


DetectMSLD::DetectMSLD()
 : dx(0), dy(0), edge(0),cannyLow(60), cannyHigh(120), filter(true)
{
}

DetectMSLD::~DetectMSLD()
{
  if (dx!=0) cvReleaseImage(&dx);
  if (dy!=0) cvReleaseImage(&dy);
  if (edge!=0) cvReleaseImage(&edge);

  DeleteSegments(segments);
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/
/**
 * detect lines and compute MSLD descriptor
 */
void DetectMSLD::Detect(IplImage *img, Array<Line*> &lines)
{
  DeleteSegments(segments);
  DeleteLines(lines);

  if (img->depth != IPL_DEPTH_8U || img->nChannels!=1)
    throw Except(__HERE__,"Wrong image type!");

  if (dx!=0)
  if (!IsImageSizeEqual(img, dx))
  {
    cvReleaseImage(&dx); dx=0;
    cvReleaseImage(&dy); dy=0;
    cvReleaseImage(&edge); edge=0;
  }
  if (dx==0)
  {
    dx = cvCreateImage(cvGetSize(img), IPL_DEPTH_16S, 1 );
    dy = cvCreateImage(cvGetSize(img), IPL_DEPTH_16S, 1 );
    edge = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1 );
  }

  getEdge.Set(false, 3);
  getEdge.Sobel(img, dx, dy);
  getEdge.Canny(dx, dy, edge, cannyLow, cannyHigh);
  getEdge.LinkEdge(edge, segments, dx, dy);

  lns.Clear();
  formLines.Operate(segments,lns);
  
  if (SUB_PIXEL_LINES)
    formLines.GetSubPixel(dx, dy, lns, MIN_LINE_LENGTH);

  createMSLD.Operate(img, lns, MIN_LINE_LENGTH);

  //just filter lines without descriptor
  if (filter)
  {
    for (unsigned i=0; i<lns.Size(); i++)
      if (lns[i]->vec!=0)
        lines.PushBack(lns[i]);
      else
        delete(lns[i]);
  }
  else
  {
    lines = lns;
  }

  lns.Clear();
}

/**
 * Match lines
 */
void DetectMSLD::Match(Array<Line*> &lines1, Array<Line*> &lines2, int (*matches)[2], int buf_size, int &num, double thrGlob)
{
  unsigned idx, bidx;
  float dist, mindist;

  Line *l2, *l1;
  
  if (buf_size<(int)lines1.Size())
    throw Except(__HERE__,"Not enough memory to store matches!");

  num=0;
  for (unsigned i=0; i<lines1.Size(); i++)
  {
    if (lines1[i]->vec!=0)
    {
      l1 = lines1[i];
      mindist=FLT_MAX;
      for (unsigned j=0; j<lines2.Size(); j++)
      {
        if (lines2[j]->vec!=0 && l1->size==lines2[j]->size)
        {
          dist = DistSqr(l1->vec,lines2[j]->vec, l1->size);
          if (dist < mindist)
          {
            mindist=dist;
            idx=j;
          }
        }
      }
      if (mindist < thrGlob)
      {
        l2 = lines2[idx];
        mindist=FLT_MAX;
        for (unsigned j=0; j<lines1.Size(); j++)
        {
          if (lines1[j]->vec!=0 && lines1[j]->size==l2->size)
          {
            dist = DistSqr(l2->vec,lines1[j]->vec, l2->size);
            if (dist < mindist)
            {
              mindist=dist;
              bidx=j;
            }
          }
        }
        if (bidx == i)
        {
          matches[num][0]=i;
          matches[num][1]=idx;
          num++;
        }
      }
    }
  }
}




/***
 * Draw tracks
 */
void DetectMSLD::Draw(IplImage *img, Array<Line*> &lines)
{
  for (unsigned i=0; i<lines.Size(); i++)
    lines[i]->Draw(img);
}



}

