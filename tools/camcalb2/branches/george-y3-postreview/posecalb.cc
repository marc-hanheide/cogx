/**
 * posecalb
 *
 * @author  Michael Zillich,
 *      <A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 * @date April 2007
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvcompat.h>
#include "ell_markers.hh"
#include "CamPars.hh"

// ascii code for the escape key
#define ESCAPE 27

using namespace std;

class Camera : public CamPars
{
private:
  virtual void Read(FILE *file) throw(runtime_error);
public:
  float cam_matrix[9];
  float distortion[4];
};

void Camera::Read(FILE *file) throw(runtime_error)
{
  CamPars::Read(file);
  cam_matrix[0] = fx;
  cam_matrix[1] = 0.;
  cam_matrix[2] = cx;
  cam_matrix[3] = 0.;
  cam_matrix[4] = fy;
  cam_matrix[5] = cy;
  cam_matrix[6] = 0.;
  cam_matrix[7] = 0.;
  cam_matrix[8] = 1.;
  distortion[0] = k1;
  distortion[1] = k2;
  distortion[2] = p1;
  distortion[3] = p2;
}

class CalibObj
{
public:
  CvPoint3D32f *model_points;
  CvPoint2D32f *img_points;
  int npoints;
  int cnt;
  bool have_pose;
  float t[3], r[3];    // pose (translation and rotation vector)
  float ti[3], ri[3];  // inverse pose
  IplImage *img;  // whence the image points come from

  CalibObj();
  ~CalibObj();
  void Load(const char *filename);
  void SavePose(const char *filename);
  void WriteRotationMatrix(FILE *file, float r[3]);
  void FindPose(Camera &cam, IplImage *img);
  void FindPosePOSIT(Camera &cam, IplImage *img);
  void ProjectPoint(float *R, float *t,
      float X, float Y, float Z, float *x, float *y, float *cam_matrix);
  void InvertPose(float t[3], float r[3], float ti[3], float ri[3]);
  void DrawProjectedPoints(Camera &cam, IplImage *img);
  void DrawObjectPose(Camera &cam, IplImage *img);
  void DrawImagePoints(IplImage *img);
};

CalibObj::CalibObj()
{
  model_points = 0;
  img_points = 0;
  npoints = 0;
  cnt = 0;
  t[0] = t[1] = t[2] = 0.;
  r[0] = r[1] = r[2] = 0.;
  have_pose = false;
  img = 0;
}

CalibObj::~CalibObj()
{
  delete[] model_points;
  delete[] img_points;
}

void CalibObj::Load(const char *filename)
{
  FILE *file = fopen(filename, "r");
  fscanf(file, "%d", &npoints);
  model_points = new CvPoint3D32f[npoints];
  img_points = new CvPoint2D32f[npoints];
  for(int i = 0; i < npoints; i++)
  {
    fscanf(file, "%f %f %f", &model_points[i].x, &model_points[i].y,
      &model_points[i].z);
    img_points[i].x = img_points[i].y = 0.;
  }
  fclose(file);
}

void CalibObj::SavePose(const char *filename)
{
  FILE *file = fopen(filename, "w");
  time_t tim = time(NULL);

  fprintf(file, "# pose calibration file generated %s\n", ctime(&tim));
  fprintf(file,
      "# Pose of camera w.r.t. calibration object:\n"
      "# t .. 3x1 translation vector in [m]\n"
      "# r .. 3x1 rotation vector\n"
      "#      direction = axis of rotation, length = angle in [rad]\n\n");
  fprintf(file, "pose = [%f %f %f] [%f %f %f]\n", t[0], t[1], t[2],
      r[0], r[1], r[2]); 
  WriteRotationMatrix(file, r);
  fprintf(file, "\n# Pose of calibration object w.r.t. camera:\n");
  fprintf(file, "# pose = [%f %f %f] [%f %f %f]\n", ti[0], ti[1], ti[2],
      ri[0], ri[1], ri[2]); 
  WriteRotationMatrix(file, ri);
  fclose(file);
}

/**
 * Save rotation matrix, just as comment.
 */
void CalibObj::WriteRotationMatrix(FILE *file, float r[3])
{
  float R[9];
  CvMat Rmat = cvMat(3, 3, CV_32FC1, R);
  CvMat rmat = cvMat(3, 1, CV_32FC1, r);
  cvRodrigues2(&rmat, &Rmat);
  fprintf(file, "# rotation matrix:\n");
  for(int i = 0; i < 3; i++)
  {
    fprintf(file, "# |");
    for(int j = 0; j < 3; j++)
      fprintf(file, " %6.3f", cvmGet(&Rmat, i, j));
    fprintf(file, " |\n");
  }
}

void CalibObj::FindPose(Camera &cam, IplImage *img)
{
  float f[2] = {cam.fx, cam.fy};
  CvPoint2D32f c = {cam.cx, cam.cy};
  CvSize img_size = {img->width, img->height};

  // this finds the pose of the calibration object w.r.t. the camera, i.e.
  // the inverse of the camera pose
  cvFindExtrinsicCameraParams(npoints, img_size,
      img_points, model_points, f, c, cam.distortion, ri, ti);
  InvertPose(ti, ri, t, r);
  have_pose = true;
}

/**
 * Use POSIT for pose estimation.
 * TODO: does not seem to work as expected. Check!
 */
void CalibObj::FindPosePOSIT(Camera &cam, IplImage *img)
{
  CvPOSITObject *model;
  CvTermCriteria term = cvTermCriteria(CV_TERMCRIT_EPS, 0, 1e-6);
  CvPoint2D32f points_mm[npoints];
  float Ri[9];  // 3x3 rotation matrix, as vector

  // transform image points from [pixel] to physical image co-ordinates in [mm]
  for(int i = 0; i < npoints; i++)
  {
    points_mm[i].x = (img_points[i].x - cam.cx)*cam.sx();
    points_mm[i].y = (img_points[i].y - cam.cy)*cam.sy();
  }
  model = cvCreatePOSITObject(model_points, npoints);
  cvPOSIT(model, points_mm, cam.f, term, Ri, ti);
  CvMat Rimat = cvMat(3, 3, CV_32FC1, Ri);
  CvMat rimat = cvMat(3, 1, CV_32FC1, ri);
  cvRodrigues2(&Rimat, &rimat);
  InvertPose(ti, ri, t, r);
  have_pose = true;
  cvReleasePOSITObject(&model);
}

void CalibObj::DrawProjectedPoints(Camera &cam, IplImage *img)
{
  /* TODO: although the correct pose is calculated, this reprojection
   * does not work. find out why!
  CvMat rmat = cvMat(3, 1, CV_32FC1, r);
  CvMat tmat = cvMat(3, 1, CV_32FC1, t);
  CvMat cam_mat = cvMat(3, 3, CV_32FC1, cam.cam_matrix);
  CvMat dist_mat = cvMat(1, 4, CV_32FC1, cam.distortion);
  CvPoint2D32f proj_points[npoints];
  CvMat mod_mat = cvMat(3, npoints, CV_32FC1, model_points);
  CvMat proj_mat = cvMat(2, npoints, CV_32FC1, proj_points);
  cvProjectPoints2(&mod_mat, &rmat, &tmat, &cam_mat, &dist_mat, &proj_mat);
  for(int i = 0; i < npoints; i++)
  {
    CvPoint p1 = cvPoint((int)proj_points[i].x, (int)proj_points[i].y);
    cvCircle(img, p1, 2, CV_RGB(255,0,255));
    p1 = cvPoint((int)img_points[i].x, (int)img_points[i].y);
    cvCircle(img, p1, 2, CV_RGB(0, 255, 0));
  }*/
  
  float Ri[9];
  CvMat Rimat = cvMat(3, 3, CV_32FC1, Ri);
  CvMat rimat = cvMat(3, 1, CV_32FC1, ri);
  cvRodrigues2(&rimat, &Rimat);
  for(int i = 0; i < npoints; i++)
  {
    float x, y;
    ProjectPoint(Ri, ti,
        model_points[i].x, model_points[i].y, model_points[i].z,
        &x, &y, cam.cam_matrix);
    CvPoint p1 = cvPoint((int)x, (int)y);
    cvCircle(img, p1, 2, CV_RGB(255,0,255));
  }
}

void CalibObj::DrawObjectPose(Camera &cam, IplImage *img)
{
  // find maximum extent of model in any axis direction
  float _min = HUGE;
  float _max = -HUGE;
  for(int i = 0; i < npoints; i++)
  {
    _min = min(_min, model_points[i].x);
    _min = min(_min, model_points[i].y);
    _min = min(_min, model_points[i].z);
    _max = max(_max, model_points[i].x);
    _max = max(_max, model_points[i].y);
    _max = max(_max, model_points[i].z);
  }
  // choose a visually pleasing proportion of the maxmum extent as length of
  // coordinate axes
  float len = 0.2*(_max - _min);
  float Ri[9];
  CvMat Rimat = cvMat(3, 3, CV_32FC1, Ri);
  CvMat rimat = cvMat(3, 1, CV_32FC1, ri);
  cvRodrigues2(&rimat, &Rimat);
  for(int i = 0; i < 3; i++)
  {
    float ox, oy, x, y;
    ProjectPoint(Ri, ti,
        0., 0., 0.,
        &ox, &oy, cam.cam_matrix);
    ProjectPoint(Ri, ti,
        i == 0 ? len : 0., i == 1 ? len : 0., i == 2 ? len : 0.,
        &x, &y, cam.cam_matrix);
    cvLine(img, cvPoint(ox, oy), cvPoint(x, y),
        CV_RGB(i == 0 ? 255 : 0, i == 1 ? 255 : 0, i == 2 ? 255 : 0));
  }
}

void CalibObj::DrawImagePoints(IplImage *img)
{
  for(int i = 0; i < npoints; i++)
  {
    CvPoint p1 = cvPoint((int)img_points[i].x, (int)img_points[i].y);
    cvCircle(img, p1, 2, CV_RGB(0, 255, 0));
  }
}

void CalibObj::ProjectPoint(float *R, float *t,
    float X, float Y, float Z, float *x, float *y, float *cam_matrix)
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

/**
 * w = R o + t
 * o = R^T w - R^T t
 */
void CalibObj::InvertPose(float *t, float *r, float *ti, float *ri)
{
  float Ri[9];
  CvMat Rimat = cvMat(3, 3, CV_32FC1, Ri);
  CvMat rimat = cvMat(3, 1, CV_32FC1, ri);

  ri[0] = -r[0];
  ri[1] = -r[1];
  ri[2] = -r[2];
  cvRodrigues2(&rimat, &Rimat);
  ti[0] = -Ri[0]*t[0] - Ri[1]*t[1] - Ri[2]*t[2];
  ti[1] = -Ri[3]*t[0] - Ri[4]*t[1] - Ri[5]*t[2];
  ti[2] = -Ri[6]*t[0] - Ri[7]*t[1] - Ri[8]*t[2];
}

void on_mouse(int event, int x, int y, int flags, void *param)
{
  CalibObj *cal = (CalibObj*)param;
  switch(event)
  {
    case CV_EVENT_LBUTTONDOWN:
      if(cal->cnt < cal->npoints)
      {
        cal->img_points[cal->cnt].x = (float)x;
        cal->img_points[cal->cnt].y = (float)y;
        cal->cnt++;
        cal->DrawImagePoints(cal->img);
        cvShowImage("posecalb", cal->img);
      }
      break;
    default:
      break;
  }
}

int main(int argc, char **argv)
{
  Camera cam;
  CalibObj cal;
  IplImage *img = 0, *undist = 0;
  bool done = false;

  if(argc != 5)
  {
    printf("usage: %s <camera calib> <model> <image> <pose>\n"
        "  camera calib .. internal camera calibration parameter file\n"
        "  model .. model file with number of points and 3D points in [m]\n"
        "  image .. calibration image, any format\n"
        "  pose .. output pose file\n",
        argv[0]);
    exit(1);
  }

  cam.Load(argv[1]);
  cal.Load(argv[2]);
  img = cvLoadImage(argv[3], 1);
  cal.img = img;

  cvNamedWindow("posecalb", 0);
  cvResizeWindow("posecalb", img->width, img->height);
  cvSetMouseCallback("posecalb", on_mouse, (void*)&cal);

  undist = cvCloneImage(img);
  cvUnDistortOnce(img, undist, cam.cam_matrix, cam.distortion, 1);
  cvShowImage("posecalb", img);

  printf("Click on the points as given in the model file in correct order.\n");
  while(!done)
  {
    int c = cvWaitKey(100);
    switch(c)
    {
      case ESCAPE:
      case 'q':
        done = true;
        break;
      default:
        // if all points available for pose calculation, and have no pose yet
        if(cal.cnt == cal.npoints && !cal.have_pose)
        {
          printf("Have all points, calculating pose ...\n");
          cal.FindPose(cam, img);
          cal.SavePose(argv[4]);
          printf("Pose written to '%s'\n", argv[4]);
          printf("Press <q> or <ESC> to exit.\n");
          //cal.FindPosePOSIT(cam, img);
          cal.DrawProjectedPoints(cam, undist);
          cal.DrawObjectPose(cam, undist);
          cvShowImage("posecalb", undist);
        }
        break;
    }
  }

  cvDestroyWindow("posecalb");
  cvReleaseImage(&undist);
  cvReleaseImage(&img);
}

