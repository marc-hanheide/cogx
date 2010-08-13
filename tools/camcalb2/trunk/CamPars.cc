#include <string.h>
#include <math.h>
#include "CamPars.hh"

using namespace std;

const char *formula_header =
"# Intrinsic parameters for pinhole camera model.\n"
"# The pinhole camera model is as follows:\n"
"# m = A*(R*M + t)\n"
"#\n"
"# M .. 3x1 world point [mm]\n"
"# A .. 3x3 camera intrinsic matrix\n"
"#          |f_x  0  c_x|\n"
"#      A = | 0  f_y c_y|\n"
"#          | 0   0   1 |\n"
"#      c_x, c_y .. principal point [pix]\n"
"#      f_x, f_y .. focal length in [pix]\n"
"#                  f_x = f/s_x, f_y = f/s_y\n"
"#                  with f the focal length in [mm] and s_x, s_y the pixel\n"
"#                  sizes in [mm/pix]\n"
"# R,t .. 3x3 rotation matrix and 3x1 translation vector [mm]\n"
"#      pose of camera w.r.t. world (= extrinsic parameters)\n"
"# m .. 3x1 image point in homogenous co-ordinates\n"
"#      image point in cartesian pixel co-ordinates is given as\n"
"#      (x, y) = (m_x/m_z, m_y/m_z)\n"
"# Note that only the ratios focal length to pixel size can be determined.\n"
"# Given a (known) nominal focal length f (e.g. from camera data sheet) we\n"
"# can calculate:\n"
"# s_x = f/f_x .. horizontal pixel size [mm/pix]\n"
"# s_y = f/f_y .. vertical pixel size [mm/pix]\n"
"# r = s_x/s_y .. pixel aspect ratio\n"
"#\n"
"# Distortion is modelled as follows:\n"
"# x_d = x + x*(k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*x^2)\n"
"# y_d = y + y*(k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*y^2)\n"
"#\n"
"# where (x, y) are ideal, distortion-free image physical co-ordinates,\n"
"# (x_d, y_d) are distorted image physical co-ordinates and r^2 = x^2 + y^2\n"
"#\n"
"# w, h .. image width and height\n\n";


CamPars::CamPars(double fnew)
{
  fx = fy = cx = cy = 0.;
  k1 = k2 = p1 = p2 = 0.;
  f = fnew;
  w = h = 0;
}

void CamPars::Write(FILE *file) const
{
  fprintf(file, "# focal lengths in [pix]\n");
  fprintf(file, "fx = %.6f\n", fx);
  fprintf(file, "fy = %.6f\n", fy);
  fprintf(file, "# principal point [pix]\n");
  fprintf(file, "cx = %.6f\n", cx);
  fprintf(file, "cy = %.6f\n", cy);

  fprintf(file, "\n# (nominal) focal length [mm]\n");
  fprintf(file, "f  = %.6f\n", f);
  fprintf(file, "# pixel size [mm/pix]\n");
  fprintf(file, "sx = %.6f\n", sx());
  fprintf(file, "sy = %.6f\n", sy());
  fprintf(file, "# aspect ratio: pixel size x/y\n");
  fprintf(file, "r  = %.6f\n", r());

  fprintf(file, "# image width and height\n");
  fprintf(file, "w =  %d\n", w);
  fprintf(file, "h =  %d\n", h);

  fprintf(file, "\n# distortion parameters\n");
  fprintf(file, "k1 = %.6f\n", k1);
  fprintf(file, "k2 = %.6f\n", k2);
  fprintf(file, "p1 = %.6f\n", p1);
  fprintf(file, "p2 = %.6f\n", p2);
  fprintf(file, "\n");
}

/**
 * Check if str corresponds to name (same length and same content).
 * Note that str is NOT '\0' delimited!
 */
bool CamPars::IsName(const char *str, size_t len, const char *name) const
{
  return len == strlen(name) && strncmp(str, name, len) == 0;
}

/**
 * Read name-value pairs, separated by whitespace.
 * Ignore comments.
 */
void CamPars::Read(FILE *file) throw(runtime_error)
{
  const char comment = '#';
  const char separator = '=';
  const char *spaces = " \t\n";
  const size_t bufsize = 1024;
  char buf[bufsize];
  while(fgets(buf, bufsize, file) != NULL)
  {
    size_t linelen = strlen(buf);
    // move to first non-space (= comment or start of name)
    size_t start = strspn(buf, spaces);
    if(start < linelen && buf[start] != comment)
    {
      // move to first space (= end of name)
      size_t stop = start + strcspn(&buf[start], spaces);
      size_t namelen = stop - start;
      char *name = &buf[start];
      // move to separator
      char *value = index(&buf[stop], separator);
      if(value == 0)
        throw runtime_error("CamPars::Read: missing '='");
      // move one beyond separator
      value++;
      int scan = 0;
      if(IsName(name, namelen, "fx"))
        scan = sscanf(value, "%lf", &fx);
      else if(IsName(name, namelen, "fy"))
        scan = sscanf(value, "%lf", &fy);
      else if(IsName(name, namelen, "cx"))
        scan = sscanf(value, "%lf", &cx);
      else if(IsName(name, namelen, "cy"))
        scan = sscanf(value, "%lf", &cy);
      else if(IsName(name, namelen, "f"))
        scan = sscanf(value, "%lf", &f);
      else if(IsName(name, namelen, "w"))
        scan = sscanf(value, "%d", &w);
      else if(IsName(name, namelen, "h"))
        scan = sscanf(value, "%d", &h);
      else if(IsName(name, namelen, "k1"))
        scan = sscanf(value, "%lf", &k1);
      else if(IsName(name, namelen, "k2"))
        scan = sscanf(value, "%lf", &k2);
      else if(IsName(name, namelen, "p1"))
        scan = sscanf(value, "%lf", &p1);
      else if(IsName(name, namelen, "p2"))
        scan = sscanf(value, "%lf", &p2);
    }
  }
  // check if values are valid
  // Note that the distortion parameters can be <0 >0 =0
  if(fx <= 0.)
    throw runtime_error("CamPars::Read: invalid fx");
  if(fy <= 0.)
    throw runtime_error("CamPars::Read: invalid fy");
  if(cx <= 0.)
    throw runtime_error("CamPars::Read: invalid cx");
  if(cy <= 0.)
    throw runtime_error("CamPars::Read: invalid cy");
  if(f <= 0.)
    throw runtime_error("CamPars::Read: invalid f");
  if(w <= 0)
    throw runtime_error("CamPars::Read: invalid w");
  if(h <= 0)
    throw runtime_error("CamPars::Read: invalid h");
}

void CamPars::Save(const char *filename) const throw(runtime_error)
{
  FILE *file;
  file = fopen(filename, "w");
  if(file == 0)
    throw runtime_error("CamPars::Save: failed to open file");
  // file header
  time_t t = time(NULL);
  fprintf(file, "# camera parameter file generated %s\n", ctime(&t));
  fputs(formula_header, file);
  // parameters
  Write(file);
  fclose(file);
}

void CamPars::Load(const char *filename) throw(runtime_error)
{
  FILE *file;
  file = fopen(filename, "r");
  if(file == 0)
    throw runtime_error("CamPars::Load: failed to open file");
  Read(file);
  fclose(file);
}

