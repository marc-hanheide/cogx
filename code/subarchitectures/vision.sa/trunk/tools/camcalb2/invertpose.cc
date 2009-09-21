#include <stdlib.h>
#include <stdio.h>
#include <opencv/cv.h>

/**
 * Return inverse pose Ri, ti
 * with (global) world point w, (local) object point o:
 * w = R o + t
 * o = R^T w - R^T t
 * thus:
 * Ri = R^T
 * ti = -R^T t
 */
static void InvertPose(float *t, float *r, float *ti, float *ri)
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

int main(int argc, char **argv)
{
  float t[3], r[3], ti[3], ri[3];
  if(argc != 7)
  {
    printf("usage: %s tx ty tz rx ry rz\n"
           "  tx, ty, tz .. translation vector\n"
           "  rx, ry, rz .. rotation vector\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  t[0] = (float)atof(argv[1]);
  t[1] = (float)atof(argv[2]);
  t[2] = (float)atof(argv[3]);
  r[0] = (float)atof(argv[4]);
  r[1] = (float)atof(argv[5]);
  r[2] = (float)atof(argv[6]);
  InvertPose(t, r, ti, ri);
  printf("given pose (tx  ty  tz  rx ry rz):\n  %.6f %.6f %.6f  %.6f %.6f %.6f\n",
      t[0], t[1], t[2], r[0], r[1], r[2]);
  printf("inverse pose (tx  ty  tz  rx ry rz):\n  %.6f %.6f %.6f  %.6f %.6f %.6f\n",
      ti[0], ti[1], ti[2], ri[0], ri[1], ri[2]);

  exit(EXIT_SUCCESS);
}

