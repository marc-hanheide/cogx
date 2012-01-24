/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#include <errno.h>
#include "DetectSIFT.hh"
#include "SDraw.hh"

namespace P 
{

DetectSIFT::DetectSIFT()
{
}

DetectSIFT::~DetectSIFT()
{
}




/************************************** PRIVATE ************************************/
/**
 * SaveImage
 */
void DetectSIFT::SavePGMImage(const char *filename, IplImage *grey)
{
  if (grey->nChannels!=1 && grey->depth != IPL_DEPTH_8U)
    throw Except(__HERE__,"Wrong image format!");
  
  unsigned storage_size = grey->width*grey->height*grey->depth;
  
  FILE *file = fopen(filename, "w");
  if(file == NULL)
    throw Except(__HERE__, "failed to open file %s:", filename,strerror(errno));
  fprintf(file,"P5\n%u %u\n255\n", grey->width, grey->height);

  fwrite(grey->imageData, 1, storage_size, file);

  fclose(file);
}










/************************************** PUBLIC ************************************/

void DetectSIFT::Operate(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  if (img->depth != IPL_DEPTH_8U && img->nChannels!=1)
    throw Except(__HERE__,"Wrong image type!");

  SavePGMImage("./detbin/grey.pgm", img);
  
  int pid,status;
  switch(pid = fork()){
    case 0: execl("/bin/sh","/bin/sh","-c","./detbin/sift <./detbin/grey.pgm >./detbin/grey.key",0);

    default: while(wait(&status) != pid);
  }

  LoadLoweKeypoints("./detbin/grey.key", keys, 0);

  //..or only correct type
  for (unsigned i=0; i<keys.Size(); i++) 
    ((KeypointDescriptor*)keys[i])->type = KeypointDescriptor::LOWE_DOG_SIFT; 
}


/***
 * Draw tracks
 */
void DetectSIFT::Draw(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  for (unsigned i=0; i<keys.Size(); i++)
  {
      keys[i]->Draw(img,*keys[i],CV_RGB(255,0,0));
  }
}



}

