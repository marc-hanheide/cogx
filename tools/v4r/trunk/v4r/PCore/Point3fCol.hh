/**
 * $Id$
 */

#ifndef P_POINT3FCOL_HH
#define P_POINT3FCOL_HH

#include <opencv2/core/core.hpp>

namespace P
{

typedef union
{
  struct
  {
    unsigned char b; // Blue channel
    unsigned char g; // Green channel
    unsigned char r; // Red channel
    unsigned char a; // Alpha channel
  };
  float fl;
  long lo;
} RGBValue;


class Point3fCol : public cv::Point3f
{
public:
  RGBValue col;

  Point3fCol(){};
  Point3fCol(const cv::Vec4f &pt) 
  {
    x = pt[0], y = pt[1], z = pt[2], col.fl=pt[3];
  };
  Point3fCol(const cv::Vec3f &pt) 
  {
    x = pt[0], y = pt[1], z = pt[2];
  };
  Point3fCol(const cv::Point3f &pt) 
  {
    x = pt.x, y = pt.y, z = pt.z;
  };
  ~Point3fCol(){};

  Point3fCol& operator=(const cv::Vec4f &pt)
  {
    this->x = pt[0];
    this->y = pt[1];
    this->z = pt[2];
    this->col.fl = pt[3];
    return *this;
  }

  inline float X() {return x;}
  inline float Y() {return y;}
  inline float Z() {return z;}
  inline void X(float _x) {x = _x;}
  inline void Y(float _y) {y = _y;}
  inline void Z(float _z) {z = _z;}

  inline uchar R() {return col.r;}
  inline uchar G() {return col.g;}
  inline uchar B() {return col.b;}
  inline void R(uchar r) {col.r = r;}
  inline void G(uchar g) {col.g = g;}
  inline void B(uchar b) {col.b = b;}
  inline void SetCol(uchar r, uchar g, uchar b) {col.r = r, col.r = r, col.r = r, col.a = 0;}
  
  static inline float Col(uchar r, uchar g, uchar b);
};


/*********************** INLINE METHODES **************************/

/**
 * Set colour float value
 */
inline float Point3fCol::Col(uchar r, uchar g, uchar b)
{
  RGBValue col;
  col.r=r, col.g=g, col.b=b, col.a=0;
  return col.fl;
}


}

#endif

