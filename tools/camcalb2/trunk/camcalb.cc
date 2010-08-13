 /**
 * $Id$
 *
 * @author  Michael Zillich,
 *      <A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 * @date August 2006
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
  void WriteRowVector(FILE *file, CvMat *m, int i) const;
protected:
  virtual void Write(FILE *file) const;
public:
  CvMat *cam_matrix;   // 3x3 camera matrix
  CvMat *dist_coeffs;  // 4x1 distortion coefficients
  CvMat *trans_vecs;   // translation parts of pose (camera to model) per image
  CvMat *rot_vecs;     // rotation parts of pose (camera to model) per image
  double mean_reproj_err;   // reprojection error
  int num_points;      // points per image
  int num_images;      // number of images
  CvMat *img_points;   // Nx2 array of 2D marker points, with N =
                       // num_points*num_images
  CvMat *model_points; // Nx3 array of 3D model points
  vector<string> img_names;  // names of images corresponding to poses

  CamParsPoses();
  virtual ~CamParsPoses();
  /**
   * Add an image to the list of calibration images.
   * @param img_name file name of the image to add
   * @param img the actual image, but note: we just store the filename and only
   *            use the actual image to get and check image size
   */
  void AddImage(const string &img_name, const IplImage *img);
  /**
   * Perform the actual calibration.
   * @param markers nxm marker point positions, with n = the number of
   *                calibration image and m the number of model points (same for
   *                each image)
   * @param model the calibration model
   */
  bool Calibrate(vector<CvPoint2D64f> &markers, Model &model);
  /**
   * Copy OpenCv calibration parameters to own internal paramters.
   */
  void OpenCVParametersToInternal();
  /**
   * Draw projected model points for given image number.
   * @param img IPL image to draw into
   * @param img_num image number in the sequence of calibration images
   * @param col color to draw the points
   * @param distorted_image whether the input image is distorted, or already
   *                        undistorted (in which case distortion is ommited in
   *                        the projection).
   */
  void DrawProjectedModelPoints(IplImage *img, int img_num, CvScalar col,
      bool distorted_image);
  /**
   * Draw detected marker image points points for given image number.
   * @param img IPL image to draw into
   * @param img_num image number in the sequence of calibration images
   * @param col color to draw the points
   */
  void DrawImagePoints(IplImage *img, int num, CvScalar col);
};

CamParsPoses::CamParsPoses()
{
  num_points = 0;
  img_points = 0;
  trans_vecs = 0;
  rot_vecs = 0;
  mean_reproj_err = HUGE;

  cam_matrix = cvCreateMat(3, 3, CV_64FC1);
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      cvmSet(cam_matrix, i, j, 0.0);
  cvmSet(cam_matrix, 2, 2, 1.0);

  dist_coeffs = cvCreateMat(4, 1, CV_64FC1);
  for(int i = 0; i < 4; i++)
    cvmSet(dist_coeffs, i, 0,  0.0);

}

CamParsPoses::~CamParsPoses()
{
  cvReleaseMat(&trans_vecs);
  cvReleaseMat(&rot_vecs);
  cvReleaseMat(&img_points);;
  cvReleaseMat(&model_points);
  cvReleaseMat(&cam_matrix);
  cvReleaseMat(&dist_coeffs);
}

void CamParsPoses::Write(FILE *file) const
{
  CamPars::Write(file);
  fprintf(file, "# mean reprojection error\n");
  fprintf(file, "err = %f\n", mean_reproj_err);
  fprintf(file,
      "\n# poses of calibration object w.r.t. camera for each image\n");
  fprintf(file,
      "# 3x1 translation vector t [m] and 3x1 rotation vector\n");
  fprintf(file, "nposes = %d\n", num_images);
  for(int i = 0; i < num_images; i++)
  {
    fprintf(file, "# %s\n", img_names[i].c_str());
    fprintf(file, "pose%d = ", i);
    WriteRowVector(file, trans_vecs, i);
    fprintf(file, "  ");
    WriteRowVector(file, rot_vecs, i);
    fprintf(file, "\n");
  }
}

void CamParsPoses::WriteRowVector(FILE *file, CvMat *m, int i) const
{
  fprintf(file, "[");
  for(int j = 0; j < m->cols; j++)
  {
    fprintf(file, "%.3f", (double)cvmGet(m, i, j));
    if(j != m->cols-1) fputc(' ', file);
    //else fputc('\n', file);
  }
  fprintf(file, "]");
}

void CamParsPoses::OpenCVParametersToInternal()
{
  fx = cvmGet(cam_matrix, 0, 0);
  fy = cvmGet(cam_matrix, 1, 1);
  cx = cvmGet(cam_matrix, 0, 2);
  cy = cvmGet(cam_matrix, 1, 2);
  k1 = cvmGet(dist_coeffs, 0, 0);
  k2 = cvmGet(dist_coeffs, 1, 0);
  p1 = cvmGet(dist_coeffs, 2, 0);
  p2 = cvmGet(dist_coeffs, 3, 0);
}

void CamParsPoses::AddImage(const string &img_name, const IplImage *img)
{
  assert(img != 0);
  assert(!img_name.empty());
  if(img_names.empty())
  {
    w = img->width;
    h = img->height;
  }
  else
  {
    assert(w == img->width && h == img->height);
  }
  img_names.push_back(img_name);
}

void CamParsPoses::DrawProjectedModelPoints(IplImage *img, int num,
    CvScalar col, bool distorted_image)
{
  if(num < num_images)
  {
    double _tvec[3];
    double _rvec[3];
    CvMat tvec = cvMat(3, 1, CV_64FC1, _tvec);
    CvMat rvec = cvMat(3, 1, CV_64FC1, _rvec);
    cvGetRow(trans_vecs, &tvec, num);
    cvGetRow(rot_vecs, &rvec, num);
    CvMat *model = cvCreateMat(num_points, 3, CV_64FC1);
    CvMat *proj_points = cvCreateMat(num_points, 2, CV_64FC1);
    double _tmp_dist_coeffs[4] = {0., 0., 0., 0.};
    CvMat tmp_dist_coeffs = cvMat(4, 1, CV_64FC1, _tmp_dist_coeffs);

    // copy the model points corresponding to the first calibration image (note
    // that of course they are the same for all images)
    for(int i = 0; i < num_points; i++)
    {
      cvmSet(model, i, 0, cvmGet(model_points, i, 0));
      cvmSet(model, i, 1, cvmGet(model_points, i, 1));
      cvmSet(model, i, 2, cvmGet(model_points, i, 2));
    }

    // if the input image is distorted, use the distortion coefficients from the
    // calibration, otherwise leave them 0.
    if(distorted_image)
      cvCopy(&dist_coeffs, &tmp_dist_coeffs);

    cvProjectPoints2(model, &rvec, &tvec, cam_matrix, &tmp_dist_coeffs,
        proj_points);

    for(int i = 0; i < proj_points->rows; i++)
    {
      CvPoint p1 = cvPoint((int)cvmGet(proj_points, i, 0),
                           (int)cvmGet(proj_points, i, 1));
      cvCircle(img, p1, 1, col);
    }

    cvReleaseMat(&model);
    cvReleaseMat(&proj_points);
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
      CvPoint p1 = cvPoint((int)cvmGet(img_points, i, 0),
                           (int)cvmGet(img_points, i ,1));
      cvCircle(img, p1, 1, col);
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

void DrawMarker(IplImage *img, CvPoint2D64f &p, CvScalar col)
{
  CvPoint p1 = cvPoint((int)p.x, (int)p.y);
  cvCircle(img, p1, 1, col);
  // cvLine(img, cvPoint(x - size, y), cvPoint(x + size, y), col);
  // cvLine(img, cvPoint(x, y - size), cvPoint(x, y + size), col);
}

void DrawMarkers(IplImage *img, vector<CvPoint2D64f> &markers, int start,
    int num, CvScalar col)
{
  for(int i = start; i < start + num; i++)
    DrawMarker(img, markers[i], col);
}

void DrawEdgels(IplImage *img, vector<CvPoint2D64f> &all_edgels, 
    CvScalar col)
{
  for(size_t i = 0; i < all_edgels.size(); i++)
  {
    CvPoint p1 = cvPoint((int)all_edgels[i].x, (int)all_edgels[i].y);
    cvLine(img, p1, p1, col);
  }
}

bool CamParsPoses::Calibrate(vector<CvPoint2D64f> &markers, Model &model)
{
  num_images = (int)img_names.size();
  num_points = model.NumPoints();
  int total = num_points*num_images;
  if(num_images <= 0)
  {
    printf("calibrate: no images given\n");
    return false;
  }
  assert(total == (int)markers.size());

  img_points = cvCreateMat(total, 2, CV_64FC1);
  model_points = cvCreateMat(total, 3, CV_64FC1);
  trans_vecs = cvCreateMat(num_images, 3, CV_64FC1);
  rot_vecs = cvCreateMat(num_images, 3, CV_64FC1);

  // we have the same number of points in each image
  // array with number of points for each image (always the same in our case)
  int _point_counts[num_images];
  CvMat point_counts = cvMat(num_images, 1, CV_32SC1, _point_counts);
  for(int i = 0; i < num_images; i++)
    _point_counts[i] = num_points;
  // fill image points
  for(int i = 0; i < total; i++)
  {
    cvmSet(img_points, i, 0, markers[i].x);
    cvmSet(img_points, i, 1, markers[i].y);
  }
  // fill model points
  int k = 0;
  for(int i = 0; i < num_images; i++)
    for(int y = 0; y < model.ny; y++)
    {
      // note that the "top right" corner (nx, ny) is missing
      int nx = (y < model.ny - 1 ? model.nx : model.nx - 1);
      for(int x = 0; x < nx; x++)
      {
        cvmSet(model_points, k, 0, x*model.dx);
        cvmSet(model_points, k, 1, y*model.dy);
        cvmSet(model_points, k, 2, 0.0);
        k++;
      }
    }

  CvSize img_size = {w, h};
  // the actual calibration
  mean_reproj_err = cvCalibrateCamera2(model_points,
                              img_points,
                              &point_counts,
                              img_size,
                              cam_matrix,
                              dist_coeffs,
                              rot_vecs,
                              trans_vecs);
  mean_reproj_err /= (double)total;

  OpenCVParametersToInternal();

  return true;
}

void PrintUsage(const char *argv0)
{
  printf(
    "usage: %s imagebase start stop focal_length outfile\n"
    "  imagebase .. base name of image files: e.g. img%%03d.jpg, which would\n"
    "               load img000.jpg, img001.jpg, ... img019.jpg\n"
    "  start .. first image number, e.g. 0\n"
    "  stop  .. last image number, e.g. 19\n"
    "  focal_length .. nominal focal length in [mm]. Note that of course the\n"
    "                  actual focal length can not be determined (only the\n"
    "                  ratio of focal length and pixel size\n"
    "  outfile .. output file name\n",
    argv0);
}

int main(int argc, char **argv)
{
  bool done = false;
  char *basename = 0;
  int cnt, cnt_min, cnt_max;
  CamParsPoses parms;
  vector<CvPoint2D64f> markers;
  const char *parm_filename = 0;
  // TODO: read from file or command line
  Model model = {7, 5, 40., 40.};

  if(argc != 6)
  {
    PrintUsage(argv[0]);
    exit(EXIT_FAILURE);
  }
  basename = argv[1];
  cnt_min = atoi(argv[2]);
  cnt_max = atoi(argv[3]);
  parms.f = atof(argv[4]);
  parm_filename = argv[5];
  parms.num_points = model.NumPoints();
  cnt = cnt_min;

  cvNamedWindow("camcalb");
  printf("press <space> to move to next image ...\n");
  while(cnt <= cnt_max && !done)
  {
    int c;
    IplImage *img = 0;
    vector<Ellipse> ells;
    vector<EllipsePair> pairs;
    vector<CvPoint2D64f> all_edgels;
    char imgname[1024];

    snprintf(imgname, 1024, basename, cnt);
    cnt++;
    img = cvLoadImage(imgname, 1);
    if(img == 0)
    {
      fprintf(stderr, "failed to open image '%s'\n", imgname);
      exit(EXIT_FAILURE);
    }
    if(DetectEllipseMarkers((char*)img->imageData, img->width, img->height,
       model, markers, ells, pairs, all_edgels))
    {
      DrawMarkers(img, markers, parms.img_names.size()*model.NumPoints(),
          model.NumPoints(), CV_RGB(0,255,0));
      parms.AddImage(imgname, img);
    }
    else
    {
      DrawEdgels(img, all_edgels, CV_RGB(0,255,0));
      DrawEllipses(img, ells, CV_RGB(0,0,255));
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
  if(parms.Calibrate(markers, model))
  {
    printf("mean reprojection error: %f\n", parms.mean_reproj_err);
    // reload and undistort the 0-th image
    IplImage *img = cvLoadImage(parms.img_names[0].c_str(), 1);
    IplImage *undist = cvCloneImage(img);
    cvUndistort2(img, undist, parms.cam_matrix, parms.dist_coeffs);
    // and draw projected model points
    parms.DrawProjectedModelPoints(undist, 0, CV_RGB(255,0,255), false);
    printf("showing undistorted image with projected model points\n");
    cvShowImage("camcalb", undist);
    printf("press <space> to continue ...\n");
    cvWaitKey();
    cvReleaseImage(&img);
    cvReleaseImage(&undist);
    parms.Save(parm_filename);
    printf("camera parameters saved to %s\n", parm_filename);
  }
  else
  {
    printf("failed to calibrate\n");
  }
  cvDestroyWindow("camcalb");

  exit(EXIT_SUCCESS);
}

