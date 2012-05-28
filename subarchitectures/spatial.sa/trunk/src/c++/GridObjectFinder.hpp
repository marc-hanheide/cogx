//
// = FILENAME
//    GridObjectFinder.hpp
//
// = FUNCTION
//    Helper class used to extract candidate objects
//    from a discrete grid map, given bitmaps
//    for the objects' shapes
//    Uses naïve brute-force search
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2010 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef GridObjectFinder_hpp
#define GridObjectFinder_hpp
#include <cv.h> 
// #include <opencv/cv.h>  -- incorrect according to the opencv pkg-config flags
#include <vector>
#include <Navigation/LocalGridMap.hh>

struct PlaneData;

namespace spatial {

enum ObjectFinderSymmetry {
  NO_SYMMETRY, HALF_SYMMETRY, QUARTER_SYMMETRY, CIRCULAR_SYMMETRY
};

class GridObjectFinder {
public:
  GridObjectFinder(IplImage *objectGrid, int objectXCenter, int objectYCenter,
      ObjectFinderSymmetry symmetry, double angStep = M_PI / 16);
  ~GridObjectFinder();

  void findObject(IplImage *image, int *outX, int *outY, double *outAngle,
      double *outConfidence = 0);
  void findObject(Cure::LocalGridMap<unsigned char> &lgm, int *outX, int *outY,
      double *outAngle, double *outConfidence = 0);

private:
  std::vector<IplImage *> m_aspectImages;
  double m_angStep;
};

GridObjectFinder *createTableFinder(double width, double height,
    double cellSize);
}
;
#endif //GridObjectFinder_hpp
