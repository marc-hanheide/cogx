/**
 * Small main program without gui.
 * Just creates vision core and processes single image.
 */
#include <stdio.h>
#include <assert.h>
#include "VisionCore.hh"

int main(int argc, char **argv)
{
  Z::VisionCore *vcore = 0;
  if(argc != 2)
  {
    printf("usage: %s image\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  vcore = new Z::VisionCore(Z::VIDEO_TYPE_FILESEQ);
  assert(vcore != 0);
  vcore->LoadImage(argv[1]);
  vcore->ProcessImage();
  delete vcore;
  exit(EXIT_SUCCESS);
}

