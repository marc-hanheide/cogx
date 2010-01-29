/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DETECT_GPUSIFT_HH
#define P_DETECT_GPUSIFT_HH

#include <limits.h>
#include <GL/glut.h>
#include <dlfcn.h>
#include "PNamespace.hh"
#include "KeypointDescriptor.hh"
#include "Array.hh"
#include "SiftGPU.h"


namespace P
{

typedef float SIFTDescriptor[128];



class DetectGPUSIFT
{
private:

  SiftGPU *sift;

public:
  DetectGPUSIFT();
  ~DetectGPUSIFT();

  void Operate(IplImage *img, Array<KeypointDescriptor*> &keys);

  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys);
};


/************************** INLINE METHODES ******************************/



}

#endif

