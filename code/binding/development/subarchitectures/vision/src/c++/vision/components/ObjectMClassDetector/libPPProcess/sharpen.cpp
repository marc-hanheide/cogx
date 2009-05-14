/*
 * "$Id: sharpen.c 23603 2007-09-21 13:27:33Z neo $"
 *
 *   Sharpen filters for GIMP - The GNU Image Manipulation Program
 *
 *   Copyright 1997-1998 Michael Sweet (mike@easysw.com)
 * 	 (slightly adapted version by Christian Wojek (wojek@mis.tu-darmstadt.de) )
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#include <string.h>

#define CLAMP(x,l,u) ((x)<(l)?(l):((x)>(u)?(u):(x)))
#define CLAMP0255(a)  CLAMP(a,0,255)

typedef int intneg;
typedef int intpos;

typedef unsigned char uchar;

void     compute_luts   (int sharpen_percent, intneg* neg_lut, intpos* pos_lut);
void     sharpen        (uchar *img, int width, int height, int rowStep, int img_bpp, int sharpen_percent);

void     gray_filter  (int width, uchar *src, uchar *dst, intneg *neg0,
                              intneg *neg1, intneg *neg2, intneg *pos_lut);
void     graya_filter (int width, uchar *src, uchar *dst, intneg *neg0,
                              intneg *neg1, intneg *neg2, intneg *pos_lut);
void     rgb_filter   (int width, uchar *src, uchar *dst, intneg *neg0,
                              intneg *neg1, intneg *neg2, intneg *pos_lut);
void     rgba_filter  (int width, uchar *src, uchar *dst, intneg *neg0,
                              intneg *neg1, intneg *neg2, intneg *pos_lut);


void compute_luts (int sharpen_percent, intneg* neg_lut, intpos* pos_lut) {
  int i;       /* Looping var */
  int fact;    /* 1 - sharpness */

  fact = 100 - sharpen_percent;
  if (fact < 1)
    fact = 1;

  for (i = 0; i < 256; i ++)
    {
      pos_lut[i] = 800 * i / fact;
      neg_lut[i] = (4 + pos_lut[i] - (i << 3)) >> 3;
    }
}

/*
 * 'sharpen()' - Sharpen an image using a convolution filter.
 */

void sharpen (uchar *img, int img_width, int img_height, int rowStep, int img_bpp, int sharpen_percent)
{
  uchar       *src_rows[4];    /* Source pixel rows */
  uchar       *src_ptr;        /* Current source pixel */
  uchar       *dst_row;        /* Destination pixel row */
  uchar		  *dst_img;		   /* Buffer to hold result */
  intneg       *neg_rows[4];    /* Negative coefficient rows */
  intneg       *neg_ptr;        /* Current negative coefficient */
  int          i;              /* Looping vars */
  int          y;              /* Current location in image */
  int          row;            /* Current row in src_rows */
  int          count;          /* Current number of filled src_rows */
  int          width;          /* Byte width of the image */
  int          x1;             /* Selection bounds */
  int          y1;
  int          x2;
  int          y2;
  int          sel_width;      /* Selection width */
  int          sel_height;     /* Selection height */
  //int          img_bpp;      /* Bytes-per-pixel in image */
  void          (*filter)(int, uchar *, uchar *, intneg *, intneg *, intneg *, intneg *);
  
  static intneg neg_lut[256];   /* Negative coefficient LUT */
  static intpos pos_lut[256];   /* Positive coefficient LUT */

  filter = 0;
  
  x1 = 0;
  y1 = 0;
  
  x2 = img_width;
  y2 = img_height; 

  sel_width  = img_width;
  sel_height = img_height;

  /*
   * Setup for filter...
   */
  
  compute_luts (sharpen_percent, neg_lut, pos_lut);

  width = sel_width * img_bpp;

  for (row = 0; row < 4; row ++)
    {
      src_rows[row] = new uchar[width];
      neg_rows[row] = new intneg[width];
    }

  dst_row = new uchar[width];
  dst_img = new uchar[rowStep * img_height];

  /*
   * Pre-load the first row for the filter...
   */

  memcpy(src_rows[0], img, width);
  
  for (i = width, src_ptr = src_rows[0], neg_ptr = neg_rows[0];
       i > 0;
       i --, src_ptr++, neg_ptr++)
    *neg_ptr = neg_lut[*src_ptr];

  row   = 1;
  count = 1;

  /*
   * Select the filter...
   */

  switch (img_bpp)
    {
    case 1 :
      filter = gray_filter;
      break;
    case 2 :
      filter = graya_filter;
      break;
    case 3 :
      filter = rgb_filter;
      break;
    case 4 :
      filter = rgba_filter;
      break;
    };

  /*
   * Sharpen...
   */

  for (y = y1; y < y2; y ++)
    {
      /*
       * Load the next pixel row...
       */

      if ((y + 1) < y2)
        {
          /*
           * Check to see if our src_rows[] array is overflowing yet...
           */

          if (count >= 3)
            count --;

          /*
           * Grab the next row...
           */

          memcpy(src_rows[row], img + (y + 1) * rowStep, width);
          
          for (i = width, src_ptr = src_rows[row], neg_ptr = neg_rows[row];
               i > 0;
               i --, src_ptr ++, neg_ptr ++)
        	   *neg_ptr = neg_lut[*src_ptr];

          count ++;
          row = (row + 1) & 3;
        }
      else
        {
          /*
           * No more pixels at the bottom...  Drop the oldest samples...
           */

          count --;
        }

      /*
       * Now sharpen pixels and save the results...
       */

      if (count == 3)
        {
          (* filter) (sel_width, src_rows[(row + 2) & 3], dst_row,
                      neg_rows[(row + 1) & 3] + img_bpp,
                      neg_rows[(row + 2) & 3] + img_bpp,
                      neg_rows[(row + 3) & 3] + img_bpp,
                      pos_lut);

          /*
           * Set the row...
           */          
          memcpy(dst_img + y * rowStep, dst_row, width);          
          
        }
      else if (count == 2)
        {
          if (y == y1) /* first row */
        	  memcpy(dst_img + y * rowStep, src_rows[0], width);            
          else /* last row  */
              memcpy(dst_img + y * rowStep, src_rows[(sel_height - 1) & 3], width);
        }
    }

  memcpy(img, dst_img, rowStep * img_height);
  
  /*
   * OK, we're done.  Free all memory used...
   */

  for (row = 0; row < 4; row ++)
    {
      delete[](src_rows[row]);
      delete[](neg_rows[row]);
    }

  delete[](dst_row);
  delete[](dst_img);
}

/*
 * 'gray_filter()' - Sharpen grayscale pixels.
 */

void gray_filter (int    width,     /* I - Width of line in pixels */
             uchar *src,       /* I - Source line */
             uchar *dst,       /* O - Destination line */
             intneg *neg0,      /* I - Top negative coefficient line */
             intneg *neg1,      /* I - Middle negative coefficient line */
             intneg *neg2,      /* I - Bottom negative coefficient line */
             intneg *pos_lut)
{
  intpos pixel;         /* New pixel value */

  *dst++ = *src++;
  width -= 2;

  while (width > 0)
    {
      pixel = (pos_lut[*src++] - neg0[-1] - neg0[0] - neg0[1] -
               neg1[-1] - neg1[1] -
               neg2[-1] - neg2[0] - neg2[1]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      neg0 ++;
      neg1 ++;
      neg2 ++;
      width --;
    }

  *dst++ = *src++;
}

/*
 * 'graya_filter()' - Sharpen grayscale+alpha pixels.
 */

void graya_filter (int   width,     /* I - Width of line in pixels */
              uchar *src,      /* I - Source line */
              uchar *dst,      /* O - Destination line */
              intneg *neg0,     /* I - Top negative coefficient line */
              intneg *neg1,     /* I - Middle negative coefficient line */
              intneg *neg2,     /* I - Bottom negative coefficient line */
              intneg *pos_lut)
{
  intpos pixel;         /* New pixel value */

  *dst++ = *src++;
  *dst++ = *src++;
  width -= 2;

  while (width > 0)
    {
      pixel = (pos_lut[*src++] - neg0[-2] - neg0[0] - neg0[2] -
               neg1[-2] - neg1[2] -
               neg2[-2] - neg2[0] - neg2[2]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      *dst++ = *src++;
      neg0 += 2;
      neg1 += 2;
      neg2 += 2;
      width --;
    }

  *dst++ = *src++;
  *dst++ = *src++;
}

/*
 * 'rgb_filter()' - Sharpen RGB pixels.
 */

void rgb_filter (int    width,      /* I - Width of line in pixels */
            uchar *src,        /* I - Source line */
            uchar *dst,        /* O - Destination line */
            intneg *neg0,       /* I - Top negative coefficient line */
            intneg *neg1,       /* I - Middle negative coefficient line */
            intneg *neg2,       /* I - Bottom negative coefficient line */
            intneg *pos_lut)
{
  intpos pixel;         /* New pixel value */

  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
  width -= 2;

  while (width > 0)
    {
      pixel = (pos_lut[*src++] - neg0[-3] - neg0[0] - neg0[3] -
               neg1[-3] - neg1[3] -
               neg2[-3] - neg2[0] - neg2[3]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      pixel = (pos_lut[*src++] - neg0[-2] - neg0[1] - neg0[4] -
               neg1[-2] - neg1[4] -
               neg2[-2] - neg2[1] - neg2[4]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      pixel = (pos_lut[*src++] - neg0[-1] - neg0[2] - neg0[5] -
               neg1[-1] - neg1[5] -
               neg2[-1] - neg2[2] - neg2[5]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      neg0 += 3;
      neg1 += 3;
      neg2 += 3;
      width --;
    }

  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
}

/*
 * 'rgba_filter()' - Sharpen RGBA pixels.
 */

void rgba_filter (int   width,      /* I - Width of line in pixels */
             uchar *src,       /* I - Source line */
             uchar *dst,       /* O - Destination line */
             intneg *neg0,      /* I - Top negative coefficient line */
             intneg *neg1,      /* I - Middle negative coefficient line */
             intneg *neg2,      /* I - Bottom negative coefficient line */
             intneg *pos_lut)
{
  intpos pixel;         /* New pixel value */

  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
  width -= 2;

  while (width > 0)
    {
      pixel = (pos_lut[*src++] - neg0[-4] - neg0[0] - neg0[4] -
               neg1[-4] - neg1[4] -
               neg2[-4] - neg2[0] - neg2[4]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      pixel = (pos_lut[*src++] - neg0[-3] - neg0[1] - neg0[5] -
               neg1[-3] - neg1[5] -
               neg2[-3] - neg2[1] - neg2[5]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      pixel = (pos_lut[*src++] - neg0[-2] - neg0[2] - neg0[6] -
               neg1[-2] - neg1[6] -
               neg2[-2] - neg2[2] - neg2[6]);
      pixel = (pixel + 4) >> 3;
      *dst++ = CLAMP0255 (pixel);

      *dst++ = *src++;

      neg0 += 4;
      neg1 += 4;
      neg2 += 4;
      width --;
    }

  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
  *dst++ = *src++;
}
