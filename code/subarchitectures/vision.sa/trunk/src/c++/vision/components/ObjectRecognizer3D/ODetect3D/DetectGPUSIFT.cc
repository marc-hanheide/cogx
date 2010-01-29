/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 *
 * TODO:
 * - add gpu sift matcher
 */


#include "DetectGPUSIFT.hh"
#include "SDraw.hh"


namespace P 
{

DetectGPUSIFT::DetectGPUSIFT()
{

  //init sift
  //char * argv[] = {"-m", "-s", "-v", "1"};
  char * argv[] = {"-m", "-fo","-1", "-s", "-v", "1", "-pack"};
  //char * argv[] = {"-m", "-s", "-w", "3", "-fo", "-1", "-loweo"};
  //char * argv[] = {"-fo","-1","-v", "1"};

  int argc = sizeof(argv)/sizeof(char*);
  sift = new SiftGPU;
  sift->ParseParam(argc, argv);

  //create an OpenGL context for computation
  if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    throw Except(__HERE__,"SiftGPU is not fully supported");
}

DetectGPUSIFT::~DetectGPUSIFT()
{
  delete sift;
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/

void DetectGPUSIFT::Operate(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  for (unsigned i=0; i<keys.Size(); i++)
    delete keys[i];
  keys.Clear();

  if (img->depth != IPL_DEPTH_8U && img->nChannels!=1)
    throw Except(__HERE__,"Wrong image type!");
  
  if(sift->RunSIFT(img->width, img->height,(unsigned char *)img->imageData,GL_LUMINANCE))
  {
    KeypointDescriptor *k;
    
    int num = sift->GetFeatureNum();
    
    if (num>0)
    {
      Array<SiftGPU::SiftKeypoint> ks(num);
      Array<SIFTDescriptor> desc(num);
      sift->GetFeatureVector(&ks[0], (float*)&desc[0]);

      //copy sift
      for (unsigned i=0; i<desc.Size(); i++)
      {
        k = new KeypointDescriptor(KeypointDescriptor::DOG_SIFT, ks[i].x,ks[i].y,ks[i].s, -ks[i].o);
        k->AllocVec(128);
        CopyVec((float*)&desc[i], k->vec, 128);

        keys.PushBack(k);
      }
    }else cout<<"No SIFT found"<<endl;
  }else throw Except(__HERE__, "SiftGPU Error!");

}


/***
 * Draw tracks
 */
void DetectGPUSIFT::Draw(IplImage *img, Array<KeypointDescriptor*> &keys)
{
  for (unsigned i=0; i<keys.Size(); i++)
  {
      keys[i]->Draw(img,*keys[i],CV_RGB(255,0,0));
  }
}



}

