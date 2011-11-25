#ifndef CAM_PARS_HH
#define CAM_PARS_HH

#include <stdio.h>
#include <math.h>
#include <stdexcept>

/**
 * Camera parameters for a pinhole camera model.
 *
 * The pinhole camera model is as follows:
 * m = A*(R*M + t)
 *
 * M .. 3x1 world point [mm]
 * A .. 3x3 camera intrinsic matrix
 *          |f_x  0  c_x|
 *      A = | 0  f_y c_y|
 *          | 0   0   1 |
 *      c_x, c_y .. principal point [pix]
 *      f_x, f_y .. focal length in [pix]
 *                  f_x = f/s_x, f_y = f/s_y
 *                  with f the focal length in [mm] and s_x, s_y the pixel
 *                  sizes in [mm/pix]
 * R,t .. 3x3 rotation matrix and 3x1 translation vector [mm]
 *      pose of camera w.r.t. world
 * m .. 3x1 image point in homogenous co-ordinates
 *      image point in cartesian pixel co-ordinates is given as
 *      (x, y) = (m_x/m_z, m_y/m_z)
 * Note that only the ratios focal length to pixel size can be determined.
 * Given a (known) nominal focal length f (e.g. from camera data sheet) we can
 * calculate:
 * s_x = f/f_x .. horizontal pixel size [mm/pix]
 * s_y = f/f_y .. vertical pixel size [mm/pix]
 * r = s_x/s_y .. pixel aspect ratio
 *
 * Distortion is modelled as follows:
 * x_d = x + x*(k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*x^2)
 * y_d = y + y*(k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*y^2)
 *
 * where (x, y) are ideal, distortion-free image physical co-ordinates,
 * (x_d, y_d) are distorted image physical co-ordinates and r^2 = x^2 + y^2
 *
 * w, h .. image width and height
 */
class CamPars
{
private:
  bool IsName(const char *str, size_t len, const char *name);

protected:
  virtual void Write(FILE *file);
  virtual void Read(FILE *file) throw(std::runtime_error);

public:
  double fx, fy;
  double cx, cy;
  double k1, k2, p1, p2;
  double f;
  int w, h;

  CamPars(double f = 1.);
  virtual ~CamPars() {}
  double sx() {return (isnormal(fx) ? f/fx : 0.);}
  double sy() {return (isnormal(fy) ? f/fy : 0.);}
  double r() {return (isnormal(fx) ? fy/fx : 0.);}
  void Save(const char *filename) throw(std::runtime_error);
  void Load(const char *filename) throw(std::runtime_error);
};

#endif
