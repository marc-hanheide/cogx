#include <iostream>
#include "ImageFrameStream.hpp"

using namespace std;
using namespace Vision;

XDROutputStream& operator<<(XDROutputStream &stream, const ImageFrame &img)
{
  stream << (int)img.m_width << (int)img.m_height << (int)img.m_time.m_s <<
    (int)img.m_time.m_us << (int)img.m_camNum;
  // note: taking the address of a Corba sequence is dangerous. But then, Corba
  // is always dangerous.
  // Note further that we assume (as noted in Vision.idl) that image format is
  // RGB24, i.e. 3 bytes per pixel.
  stream.writeBinary((char*)&img.m_image[0], img.m_width*img.m_height*3);
  return stream;
}

XDRInputStream& operator>>(XDRInputStream &stream, ImageFrame &img)
{
  int w, h, s, us, n;
  stream >> w >> h >> s >> us >> n;
  img.m_width = w;
  img.m_height = h;
  img.m_time.m_s = s;
  img.m_time.m_us = us;
  img.m_camNum = n;
  // note: taking the address of a Corba sequence is dangerous. But then, Corba
  // is always dangerous.
  // Note further that we assume (as noted in Vision.idl) that image format is
  // RGB24, i.e. 3 bytes per pixel.
  if(img.m_image.length() != img.m_width*img.m_height*3)
    img.m_image.length(img.m_width*img.m_height*3);
  stream.readBinary((char*)&img.m_image[0], img.m_width*img.m_height*3);
  return stream;
}

