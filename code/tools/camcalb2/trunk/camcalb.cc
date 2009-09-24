 /**
 * $Id$
 *
 * @author  Michael Zillich,
 *      <A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 * @date August 2006
 *
 * TODO: remove finger image
 * TODO: better usage text
 * TODO: when saving poses for images: match pose number to image number:
 *       calibimg_034.jpg -> t034, R034
 * TODO: set window title to current image
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvcompat.h>
#include "ell_markers.hh"
#include "CamPars.hh"

using namespace std;

// ascii code for the escape key
#define ESCAPE 27

// These are the parameters calculated by cvCalibrateCamera()
class CamParsPoses : public CamPars
{
private:
  void WriteVector(FILE *file, float *v, int n);
protected:
  virtual void Write(FILE *file);
public:
  float cam_matrix[9];
  float distortion[4];
  float *trans_vecs;   // translation parts of pose (camera to model) per image
  float *rot_matrs;    // rotation parts of pose (camera to model) per image
  int num_points;      // points per image
  int num_images;      // number of calibration images and thus camera poses
  vector<string> img_names;  // names of images corresponding to poses
  CvPoint2D32f *img_points;
  CvPoint3D32f *model_points;
  bool save_poses;     // save poses when writing file

  CamParsPoses();
  virtual ~CamParsPoses();
  void AllocatePoses();
  void OpenCV2Internal();
  void DrawProjectModelPoints(IplImage *img, int num, CvScalar col);
  void DrawImagePoints(IplImage *img, int num, CvScalar col);
  void EstimateExtrinsicAndDrawModelPoints(IplImage *img, int num, CvScalar col);
  void ProjectPoint(float *R, float *t,
      float X, float Y, float Z, float *x, float *y);
};

CamParsPoses::CamParsPoses()
{
  num_images = 0;
  trans_vecs = 0;
  rot_matrs =  0;
  save_poses = false;
  for(int k = 0; k < 4; k++)
    distortion[k] = 0.0;
  for(int k = 0; k < 8; k++)
    cam_matrix[k] = 0.0;
  cam_matrix[8] = 1.0;
}

CamParsPoses::~CamParsPoses()
{
  delete[] trans_vecs;
  delete[] rot_matrs;
  delete[] img_points;
  delete[] model_points;
}

void CamParsPoses::AllocatePoses()
{
  trans_vecs = new float[num_images*3];
  rot_matrs =  new float[num_images*9];
}

void CamParsPoses::Write(FILE *file)
{
  CamPars::Write(file);
  if(save_poses)
  {
    fprintf(file,
        "\n# poses of calibration object w.r.t. camera for each image\n");
    fprintf(file,
        "# 3x1 translation vector t [m] and 3x3 rotation matrix R "
        "(in row major order)\n");
    fprintf(file, "nposes = %d\n", num_images);
    for(int i = 0; i < num_images; i++)
    {
      fprintf(file, "# %s\n", img_names[i].c_str());
      fprintf(file, "pose%d = ", i);
      WriteVector(file, &trans_vecs[3*i], 3);
      fprintf(file, "  ");
      WriteVector(file, &rot_matrs[9*i], 9);
      fprintf(file, "\n");
    }
  }
}

void CamParsPoses::WriteVector(FILE *file, float *v, int n)
{
  fprintf(file, "[");
  for(int i = 0; i < n; i++)
  {
    fprintf(file, "%.3f", v[i]);
    if(i != n-1) fputc(' ', file);
    //else fputc('\n', file);
  }
  fprintf(file, "]");
}

/**
 * Copy OpenCv calibration parameters to own internal paramters
 */
void CamParsPoses::OpenCV2Internal()
{
  fx = cam_matrix[0];
  fy = cam_matrix[4];
  cx = cam_matrix[2];
  cy = cam_matrix[5];
  k1 = distortion[0];
  k2 = distortion[1];
  p1 = distortion[2];
  p2 = distortion[3];
}

/**
 * Project a 3D model point from world co-ordinates to image pixel
 * co-ordinates.
 * For projection equation see OpenCV manual (OpenCVMan.pdf), p.6-1:
 *  m = A*[Rt]*M
 * X, Y, Z .. model point M in world co-ordinates
 * x, y, z .. image point m in homogenous image co-ordinates
 * R       .. rotation matrix of camera pose
 * t       .. translation vector of camera pose
 * A       .. camera matrix
 */
void CamParsPoses::ProjectPoint(float *R, float *t,
    float X, float Y, float Z, float *x, float *y)
{
  float tx, ty, tz, z;
  float *A = cam_matrix;
  // R*M + t
  tx = R[0]*X + R[1]*Y + R[2]*Z + t[0];
  ty = R[3]*X + R[4]*Y + R[5]*Z + t[1];
  tz = R[6]*X + R[7]*Y + R[8]*Z + t[2];
  // A*(..)
  *x = A[0]*tx + A[1]*ty + A[2]*tz;
  *y = A[3]*tx + A[4]*ty + A[5]*tz;
  z = A[6]*tx + A[7]*ty + A[8]*tz;
  // from homogenous to cartesian image co-ordinates
  *x = *x/z;
  *y = *y/z;
}

void CamParsPoses::DrawProjectModelPoints(IplImage *img, int num, CvScalar col)
{
  if(num < num_images)
  {
    int start = num*num_points;
    int stop = start + num_points;
    for(int i = start; i < stop; i++)
    {
      float x, y;
      ProjectPoint(&rot_matrs[num], &trans_vecs[num],
          model_points[i].x, model_points[i].y, model_points[i].z,
          &x, &y);
      CvPoint p1 = cvPoint((int)x, (int)y);
      cvCircle(img, p1, 1, col);
    }
  }
}

void CamParsPoses::DrawImagePoints(IplImage *img, int num, CvScalar col)
{
  if(num < num_images)
  {
    int start = num*num_points;
    int stop = start + num_points;
    for(int i = start; i < stop; i++)
    {
      CvPoint p1 = cvPoint((int)img_points[i].x, (int)img_points[i].y);
      cvCircle(img, p1, 1, col);
    }
  }
}

/**
 * Does a pose estimation (i.e. only extrinsic camera parameters) from the
 * num-th image and draws projected points (as little circles) into given image.
 */
void CamParsPoses::EstimateExtrinsicAndDrawModelPoints(IplImage *img, int num,
    CvScalar col)
{
  if(num < num_images)
  {
    float f[2] = {fx, fy};
    CvPoint2D32f c = {cx, cy};
    float t[3], r[3];
    float R[9];
    CvMat Rmat = cvMat(3, 3, CV_32FC1, R);
    CvMat rmat = cvMat(3, 1, CV_32FC1, r);
    CvSize img_size = {w, h};
    cvFindExtrinsicCameraParams(num_points, img_size,
        &img_points[num*num_points], &model_points[num*num_points],
        f, c, distortion, r, t);
    printf("trans %d-th image: %f %f %f\n", num, t[0], t[1], t[2]);     
    printf("rot   %d-th image: %f %f %f\n", num, r[0], r[1], r[2]); 
    cvRodrigues2(&rmat, &Rmat);

    int start = num*num_points;
    int stop = start + num_points;
    for(int i = start; i < stop; i++)
    {
      float x, y;
      ProjectPoint(R, t,
          model_points[i].x, model_points[i].y, model_points[i].z,
          &x, &y);
      CvPoint p1 = cvPoint((int)x, (int)y);
      cvCircle(img, p1, 2, col);
    }
  }
}

/**
 * Note that OpenCV cvEllipse seems to be off by up to two pixels!
 */
void DrawEllipse(IplImage *img, Ellipse &e, CvScalar col)
{
  int npoints = 100, i;
  double ang = 0., delta_ang = 2.*M_PI/npoints;
  double tx, ty;
  CvPoint p, q;
  for(i = 0; i < npoints; i++)
  {
    tx = e.a*cos(ang - atan2(e.a*sin(e.phi), e.b*cos(e.phi)));
    ty = e.b*sin(ang - atan2(e.a*sin(e.phi), e.b*cos(e.phi)));
    q = cvPoint((int)(e.x + tx*cos(e.phi) - ty*sin(e.phi)),
                (int)(e.y + tx*sin(e.phi) + ty*cos(e.phi)));
    if(i > 0)
      cvLine(img, p, q, col);
    p = q;
    ang += delta_ang;
  }
}

void DrawEllipses(IplImage *img, vector<Ellipse> &ells, CvScalar col)
{
  for(unsigned i = 0; i < ells.size(); i++)
    DrawEllipse(img, ells[i], col);
}

void DrawPairs(IplImage *img, vector<EllipsePair> &pairs, CvScalar col)
{
  for(unsigned i = 0; i < pairs.size(); i++)
  {
    DrawEllipse(img, pairs[i].ell1, col);
    DrawEllipse(img, pairs[i].ell2, col);
  }
}

void DrawMarker(IplImage *img, CvPoint2D32f &p, CvScalar col)
{
  CvPoint p1 = cvPoint((int)p.x, (int)p.y);
  cvCircle(img, p1, 1, col);
  // cvLine(img, cvPoint(x - size, y), cvPoint(x + size, y), col);
  // cvLine(img, cvPoint(x, y - size), cvPoint(x, y + size), col);
}

void DrawMarkers(IplImage *img, vector<CvPoint2D32f> &markers, int start,
    int num, CvScalar col)
{
  for(int i = start; i < start + num; i++)
    DrawMarker(img, markers[i], col);
}

void Calibrate(vector<CvPoint2D32f> &markers, int img_width, int img_height,
  Model &model, CamParsPoses &parms)
{
  try
  {
    CvSize img_size = {img_width, img_height};
    int num_points = model.NumPoints();
    // array with number of points for each image (always the same in our case)
    int *points_per_img = new int[parms.num_images];
    // array with markers in each image
    parms.img_points = new CvPoint2D32f[parms.num_images*num_points];
    // array with (3D) model markers
    parms.model_points = new CvPoint3D32f[parms.num_images*num_points];
    // we have the same number of points in each image
    for(int i = 0; i < parms.num_images; i++)
      points_per_img[i] = num_points;
    // fill image points
    for(int i = 0; i < parms.num_images*num_points; i++)
      parms.img_points[i] = markers[i];
    // fill model points
    int k = 0;
    for(int i = 0; i < parms.num_images; i++)
      for(int y = 0; y < model.ny; y++)
      {
        // note that the "top right" corner (nx, ny) is missing
        int nx = (y < model.ny - 1 ? model.nx : model.nx - 1);
        for(int x = 0; x < nx; x++)
        {
          parms.model_points[k].x = x*model.dx;
          parms.model_points[k].y = y*model.dy;
          parms.model_points[k].z = 0.0;
          k++;
        }
      }
    parms.AllocatePoses();
    // the actual calibration
    cvCalibrateCamera(
       parms.num_images,
       points_per_img,
       img_size,
       parms.img_points,
       parms.model_points,
       parms.distortion,
       parms.cam_matrix,
       parms.trans_vecs,
       parms.rot_matrs,
       0    // useIntrinsicGuess, leave 0
    );
    parms.OpenCV2Internal();
    delete[] points_per_img;
  }
  catch (std::bad_alloc & exc)
  {
    fprintf(stderr, "failed to allocate a ridiculously small amount memory\n");
  }
}


#if 0
/*
 * Draw projected model (cirle center) center points.
 * For projection equation see OpenCV manual (OpenCVMan.pdf), p.6-1:
 *  m = A*[Rt]*M
 * where M model point in world co-ordinates
 * and m image point in homogenous image co-ordinates
 */
void DrawProjectedMarkers(, vector<CvPoint2D32f> &markers,
  int model_nx, int model_ny, double model_dx, double model_dy,
  CamParameters &parms)
{
  // project pattern grid
  for(int i = 0; i < model_nx; i++)
    DrawLine(model_dx*i, 0., 0.,
        model_dx*i, model_dy*(model_ny-1), 0.);
  for(int j = 0; j < model_ny; j++)
    DrawLine(0., model_dy*j, 0.,
        model_dx*(model_nx-1), model_dy*j, 0.);
}

/**
 * Draw the co-ordinate axes of the calibration model.
 */
void DrawProjectedCoordinateAxes(int model_dx)
{
  // length of axes drawn is derived from circle distance
  float l = 1.5*model_dx;
  // length and width of arrows
  float la = 0.85*l, wa = 0.05*l;

  DrawProjectedLine(0, 0, 0,  la, 0, 0);
  DrawProjectedLine(la, wa, 0,  la, -wa, 0);
  DrawProjectedLine(la, wa, 0,  l, 0, 0);
  DrawProjectedLine(la, -wa, 0,  l, 0, 0);
  DrawProjectedText(l, 0, 0, "x");

  DrawProjectedLine(0, 0, 0,  0, la, 0);
  DrawProjectedLine(wa, la, 0,  -wa, la, 0);
  DrawProjectedLine(wa, la, 0,  0, l, 0);
  DrawProjectedLine(-wa, la, 0,  0, l, 0);
  DrawProjectedText(0, l, 0, "y");

  DrawProjectedLine(0, 0, 0,  0, 0, la);
  DrawProjectedLine(wa, 0, la,  -wa, 0, la);
  DrawProjectedLine(wa, 0, la,  0, 0, l);
  DrawProjectedLine(-wa, 0, la,  0, 0, l);
  DrawProjectedText(0, 0, l, "z");
}

void DrawProjecedLine(IplImage *img,
    float X1, float Y1, float Z1,
    float X2, float Y2, float Z2,
    CamParameters &parms)
{
  // let R point to rotation matrix of last image
  float *R = &parms.rot_matrs[9*(parms.num_images - 1)];
  // let t point to translation vector of last image
  float *t = &parms.trans_vecs[3*(parms.num_images - 1)];
  // camera matrix
  float *A = parms.cam_matrix;
  float x1, y1, x2, y2;

  ProjectPoint(R, t, A, X1, Y1, Z1, &x1, &y1);
  ProjectPoint(R, t, A, X2, Y2, Z2, &x2, &y2);
  cvLine(img, 
  get_window()->draw_line(gc, (int)floor(x1+0.5), (int)floor(y1+0.5),
      (int)floor(x2+0.5), (int)floor(y2+0.5));
}

void DrawProjectedText(const Glib::RefPtr<const Gdk::GC>& gc,
      float X, float Y, float Z, const char *text)
{
  // let R point to rotation matrix of last image
  float *R = &parms.rot_matrs[9*(parms.num_images - 1)];
  // let t point to translation vector of last image
  float *t = &parms.trans_vecs[3*(parms.num_images - 1)];
  // camera matrix
  float *A = parms.cam_matrix;
  float x, y;
  Glib::RefPtr<Pango::Layout> caption = create_pango_layout(text);

  ProjectPoint(R, t, A, X, Y, Z, &x, &y);
  get_window()->draw_layout(gc, (int)x, (int)y , caption);
}
#endif

int main(int argc, char **argv)
{
  bool done = false;
  char *basename = 0;
  int cnt, cnt_min, cnt_max;
  CamParsPoses parms;
  vector<CvPoint2D32f> markers;
  const char *parm_filename = 0;
  // TODO: read from file or command line
  Model model = {7, 5, 40., 40.};

  if(argc != 7)
  {
    printf(
      "usage: %s imagebase start stop nominal_focal_length_mm save_poses calibfile\n",
      argv[0]);
    exit(EXIT_FAILURE);
  }
  basename = argv[1];
  cnt_min = atoi(argv[2]);
  cnt_max = atoi(argv[3]);
  parms.f = atof(argv[4]);
  parms.save_poses = (bool)atoi(argv[5]);
  parm_filename = argv[6];
  parms.num_points = model.NumPoints();
  cnt = cnt_min;

  cvNamedWindow("camcalb", 0);
  printf("press <space> to move to next image ...\n");
  while(cnt <= cnt_max && !done)
  {
    int c;
    IplImage *img = 0;
    vector<Ellipse> ells;
    vector<EllipsePair> pairs;
    char imgname[1024];

    snprintf(imgname, 1024, basename, cnt);
    cnt++;
    img = cvLoadImage(imgname, 1);
    if(img == 0)
    {
      fprintf(stderr, "failed to open image '%s'\n", imgname);
      exit(EXIT_FAILURE);
    }
    parms.w = img->width;
    parms.h = img->height;
    cvResizeWindow("camcalb", parms.w, parms.h);
    if(DetectEllipseMarkers((char*)img->imageData, img->width, img->height,
       model, markers, ells, pairs))
    {
      DrawMarkers(img, markers, parms.num_images*model.NumPoints(),
          model.NumPoints(), CV_RGB(0,255,0));
      parms.img_names.push_back(imgname);
      parms.num_images++;
    }
    else
    {
      // DrawEllipses(img, ells, CV_RGB(255,0,0));
      DrawPairs(img, pairs, CV_RGB(255,0,0));
    }
    cvShowImage("camcalb", img);
    cvReleaseImage(&img);

    c = cvWaitKey();
    switch(c)
    {
      case ESCAPE:
      case 'q':
        done = true;
        break;
      default:
        ;
    }
  }
  Calibrate(markers, parms.w, parms.h, model, parms);
  {
    // reload and undistort the 0-th image
    IplImage *img = cvLoadImage(parms.img_names[0].c_str(), 1);
    IplImage *undist = cvCloneImage(img);
    cvUnDistortOnce(img, undist, parms.cam_matrix, parms.distortion, 1);
    // and draw projected model points
    parms.DrawProjectModelPoints(undist, 0, CV_RGB(255,0,255));
    // Just to see how well pose estimation (i.e. estimation of only the
    // extrinsic camera parameters) works from a single image:
    // parms.EstimateExtrinsicAndDrawModelPoints(undist, 0, CV_RGB(0,255,0));
    printf("showing undistorted image with projected model points\n");
    cvShowImage("camcalb", undist);
    printf("press <space> to continue ...\n");
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&undist);
  }
  parms.Save(parm_filename);
  printf("camera parameters saved to %s\n", parm_filename);
  cvDestroyWindow("camcalb");

  exit(0);
}

int main_live(int argc, char **argv)
{
  bool bayer = true;
  bool done = false;
  CvCapture* capture = 0;
  IplImage *frame = 0, *img = 0;
  vector<CvPoint2D32f> centers;
  vector<Ellipse> ells;
  vector<EllipsePair> pairs;
  Model model = {7, 5, 40., 40.};

  if(argc != 2)
  {
    printf("usage: %s image\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  capture = cvCreateCameraCapture(0);
  if(!capture)
  {
    fprintf(stderr,"Could not initialize capturing.\n");
    exit(EXIT_FAILURE);
  }
  if(bayer)
    cvSetCaptureProperty(capture, CV_CAP_PROP_CONVERT_RGB, 0.0);

  cvNamedWindow("camcalb", 0);

  while(!done)
  {
    int c;

    frame = cvQueryFrame(capture);
    // sometimes grabbing fails
    if(frame)
    {
      img = cvCloneImage(frame);
      if(bayer)
      {
        IplImage *buf = cvCreateImage(cvSize(img->width, img->height),
            IPL_DEPTH_8U, 3);
        cvCvtColor(img, buf, CV_BayerRG2RGB);
        cvReleaseImage(&img);
        img = buf;  // don't forget to free later
      }
      DetectEllipseMarkers((char*)img->imageData, img->width, img->height,
        model, centers, ells, pairs);
      //DrawEllipses(img, ells);
      //DrawPairs(img, pairs);
      //DrawMarkers(img, centers);
      cvShowImage("camcalb", img);
      cvReleaseImage(&img);
    }
    c = cvWaitKey(10);
    switch(c)
    {
      case 's':
        printf("snapshot\n");
        break;
      case ESCAPE:
      case 'q':
        done = true;
        break;
      default:
        ;
    }
  }
  cvReleaseCapture(&capture);
  cvDestroyWindow("camcalb");
  exit(0);
}
