/**
 * @file Image.cc
 * @author Zillich
 * @date November 2006
 * @version 0.1
 * @brief Yet another image class.
 **/

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "Except.hh"
#include "Image.hh"

namespace Z
{

/**
 * @brief Constructor of class Image.
 */
Image::Image()
{
  width = 0;
  height = 0;
  bytes_per_pixel = 0;
  bytes_per_line = 0;
  col_fmt = Z::MAX_COLOR_FMT;
  own_data = false;
  data = 0;
}

/**
 * @brief Constructor of class Image.
 * @param d Data of image.
 * @param w Width of image.
 * @param h Height of image.
 * @param bpp Bytes per pixel (Bits per Pixel = bpp * 8 !!!)
 * @param c Color format.
 * @param own ???
 * @param bpl Bits per ???
 */
Image::Image(char *d, unsigned w, unsigned h, unsigned bpp, ColorFormat c,
             bool own, unsigned bpl)
{
  own_data = false;
  data = 0;
  Set(d, w, h, bpp, c, own, bpl);
}

/**
 * @brief Destructor of class Image.
 */
Image::~Image()
{
  if(own_data)
    delete[] data;
}

/**
 * @brief Set image properties.
 * @param d Data of image.
 * @param w Width of image.
 * @param h Height of image.
 * @param bpp Bytes per pixel (Bits per Pixel = bpp * 8 !!!)
 * @param c Color format.
 * @param own ???
 * @param bpl Bits per ???
 */
void Image::Set(char *d, unsigned w, unsigned h, unsigned bpp,
    ColorFormat c, bool own, unsigned bpl)
{
  if(own_data)
    delete[] data;
  width = w;
  height = h;
  bytes_per_pixel = bpp;
  if(bpl == 0)
    bytes_per_line = width*bytes_per_pixel;
  else
    bytes_per_line = bpl;
  col_fmt = c;
  own_data = own;
  if(own_data)
  {
    data = new char[StorageSize()];
    if(data == 0)
      throw Except(__HERE__, "failed to allocate %d bytes", StorageSize());
    if(d != 0)
      memcpy(data, d, StorageSize());
  }
  else
  {
    if(d != 0)
      data = d;
    else
      throw Except(__HERE__, "invalid NULL pointer");
  }
}

void Image::Copy(const Image &img)
{
  if(width == img.width &&
     height == img.height &&
     bytes_per_pixel == img.bytes_per_pixel &&
     bytes_per_line == img.bytes_per_line &&
     col_fmt == img.col_fmt)
    memcpy(data, img.data, StorageSize());
}

/**
 * @brief Save as binary (raw) PPM image.
 * The data is stored as 24 bit RGB.
 * @param filename Filename.
 */
void Image::SavePPM(const char *filename) const
{
  FILE *file = fopen(filename, "w");
  if(file == NULL)
    throw Except(__HERE__, "failed to open file %s:", filename,strerror(errno));
  fprintf(file,"P6\n%u %u\n255\n", width, height);
  if(bytes_per_pixel == 3)
  {
    if(col_fmt == RGB24)
      fwrite(data, 1, StorageSize(), file);
    else
      throw Except(__HERE__, "unsupported color format");
  }
  else
    throw Except(__HERE__, "unsupported color format");
  fclose(file); 
}

/**
 * @brief Save as binary (raw) PPM image. (Flip vert)
 * The data is stored as 24 bit RGB.
 * @param filename Filename.
 */
void Image::SavePPM_FlipVert(const char *filename) const
{
  FILE *file = fopen(filename, "w");
  if(file == NULL)
    throw Except(__HERE__, "failed to open file %s:", filename,strerror(errno));
  fprintf(file,"P6\n%u %u\n255\n", width, height);
  if(bytes_per_pixel == 3)
  {
    if(col_fmt == RGB24)
    {
      int y;
      for(y = height-1; y >= 0; y--)
        fwrite(data + y*bytes_per_line, 1, bytes_per_line, file);
    }
    else
      throw Except(__HERE__, "unsupported color format");
  }
  else
    throw Except(__HERE__, "unsupported color format");
  fclose(file); 
}

}

