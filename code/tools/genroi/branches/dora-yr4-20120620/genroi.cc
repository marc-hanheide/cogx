/**
 * Load an image foo.jpg and click on the four corners of a ROI.
 * ROI is saved in foo.jpg.roi
 *
 * @author  Michael Zillich,
 * @date February 2009
 *
 * compile:
 *   g++ genroi.cc -o genroi -lcv -lhighgui
 */

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

// ascii code for the escape key
#define ESCAPE 27

static const int TEXT_WHITE = 1;
static const int TEXT_BLACK = 2;
static const int TEXT_NONE = 3;
static const int NUM_CORNERS = 4;
static CvPoint corners[NUM_CORNERS];
static int corner_cnt = 0;
static IplImage *img = 0;
static IplImage *display = 0;
static CvFont font;
static int display_text = TEXT_WHITE;

static void ToggleText()
{
  display_text++;
  if(display_text > TEXT_NONE)
    display_text = TEXT_WHITE;
}

static void ResetCorners()
{
  corner_cnt = 0;
}

static void SaveCorners(const string &imgname)
{
  if(corner_cnt == NUM_CORNERS)
  {
    string roiname(imgname);
    roiname += ".roi";
    ofstream roifile(roiname.c_str());
    for(int i = 0; i < corner_cnt; i++)
      roifile << corners[i].x << " " << corners[i].y << endl;
    printf("writen file '%s'\n", roiname.c_str());
  }
  else
    printf("only got %d of %d corners!\n", corner_cnt, NUM_CORNERS);
}

static void DrawCorners()
{
  for(int i = 0; i < corner_cnt; i++)
    cvCircle(display, corners[i], 2, CV_RGB(0, 255, 0));
  if(display_text != TEXT_NONE)
  {
    ostringstream str;
    str << "got " << corner_cnt << " of " << NUM_CORNERS << " corners";
    if(corner_cnt < NUM_CORNERS)
      str << " / left-click to add";
    else
      str << " / 's' to save";
    CvScalar col = (display_text == TEXT_WHITE ? cvScalar(255, 255, 255) :
        cvScalar(0, 0, 0));
    cvPutText(display, str.str().c_str(), cvPoint(10, 30), &font, col);
  }
}

static void Redraw()
{
  cvCopy(img, display);
  DrawCorners();
  cvShowImage("genroi", display);
}

static void on_mouse(int event, int x, int y, int flags, void *dummy)
{
  switch(event)
  {
    case CV_EVENT_LBUTTONDOWN:
      if(corner_cnt < NUM_CORNERS)
      {
        corners[corner_cnt].x = x;
        corners[corner_cnt].y = y;
        corner_cnt++;
        Redraw();
      }
      break;
    default:
      break;
  }
}

int main(int argc, char **argv)
{
  bool done = false;

  if(argc != 2)
  {
    printf("Load an image foo.jpg and click on the four corners of a ROI.\n"
           "ROI corners are saved in clicked order in foo.jpg.roi\n"
           "usage: %s <image>\n"
           "  image .. image file, any format\n",
           argv[0]);
    exit(EXIT_FAILURE);
  }

  printf("left-click to add corners\n"
         "hot keys:\n"
         "  r .. reset corners\n"
         "  s .. save ROI to imagename.roi\n"
         "  t .. toggle text\n"
         "  q, ESC .. quit\n");

  img = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
  if(img == 0)
  {
    printf("failed to load image '%s'", argv[1]);
    exit(EXIT_FAILURE);
  }
  display = cvCloneImage(img);

  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,
	     0.7, 0.7, 0.0,
	     2, 8);

  cvNamedWindow("genroi", 0);
  cvResizeWindow("genroi", img->width, img->height);
  cvSetMouseCallback("genroi", on_mouse, (void*)0);

  Redraw();

  while(!done)
  {
    int c = cvWaitKey(100);
    switch(c)
    {
      case ESCAPE:
      case 'q':
        done = true;
        break;
      case 'r':
        ResetCorners();
        Redraw();
        break;
      case 's':
        SaveCorners(argv[1]);
        break;
      case 't':
        ToggleText();
        Redraw();
      default:
        break;
    }
  }

  cvDestroyWindow("genroi");
  cvReleaseImage(&img);
  cvReleaseImage(&display);
  exit(EXIT_SUCCESS);
}

