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
#include <opencv/cv.h>
#include <vector>
//#include <highgui.h>

struct PlaneData;

namespace spatial {

  enum ObjectFinderSymmetry { NO_SYMMETRY, HALF_SYMMETRY, QUARTER_SYMMETRY, 
    CIRCULAR_SYMMETRY };
  
  class GridObjectFinder
  {
    public:
      GridObjectFinder(IplImage *objectGrid, int objectXCenter,
	  int objectYCenter, ObjectFinderSymmetry symmetry);
      ~GridObjectFinder();

      void findObject(IplImage *image, double *outX, double *outY, double *outAngle,
	  double *outConfidence = 0);


    private:
      std::vector<IplImage *> m_aspectImages;
  };
};
#endif //GridObjectFinder_hpp
