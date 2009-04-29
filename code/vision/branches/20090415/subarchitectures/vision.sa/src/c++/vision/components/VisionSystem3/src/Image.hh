/**
 * @file Image.hh
 * @author Zillich
 * @date 2007
 * @version 0.1
 * @brief Yet another image class.
 **/

#ifndef Z_IMAGE_HH
#define Z_IMAGE_HH

#include "Color.hh"

namespace Z
{

/**
 * @brief Yet another image class.
 */
class Image
{
protected:
  unsigned width;
  unsigned height;
  unsigned bytes_per_pixel;
  unsigned bytes_per_line;
  ColorFormat col_fmt;
  bool own_data;  ///< data belongs to me or is just a pointer
//  char *data; => copied to public

public:
  char *data;
  // construction and destruction
  Image();
  Image(char *d, unsigned w, unsigned h, unsigned bpp, ColorFormat c,
        bool own, unsigned bpl = 0);
  ~Image();
  void Set(char *d, unsigned w, unsigned h, unsigned bpp, ColorFormat c,
           bool own, unsigned bpl = 0);
  // access functions
  unsigned Width() const {return width;}
  unsigned Height() const {return height;}
  unsigned BytesPerPixel() const {return bytes_per_pixel;}
  unsigned BytesPerLine() const {return bytes_per_line;}
  unsigned StorageSize() const {return bytes_per_line*height;}
  ColorFormat ColorFmt() const {return col_fmt;}
  char* Data() {return data;}
  const char* Data() const {return data;}
  char* Data(int x, int y) {return &data[y*bytes_per_line + x*bytes_per_pixel];}
  const char* Data(int x, int y) const {return &data[y*bytes_per_line + x*bytes_per_pixel];}
  void Copy(const Image &img);
  bool SameSize(const Image &img)
  {return Width() == img.Width() && Height() == img.Height();}
  // loading and saving
  void SavePPM(const char *filename) const;
  void SavePPM_FlipVert(const char *filename) const;
};

}

#endif

