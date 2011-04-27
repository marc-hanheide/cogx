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
#include "CodebookEntry.hh"
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
  //void Match(Array<KeypointDescriptor*> &keys, Array<CodebookEntry *> &cb, int (*matches)[2], int buf_size, int &num); 

  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys);
};


/************************** INLINE METHODES ******************************/



}

#endif

