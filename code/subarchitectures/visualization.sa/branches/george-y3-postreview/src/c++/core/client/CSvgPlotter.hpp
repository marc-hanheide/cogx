/*
 * Author: Marko Mahnič
 * Created: 2011-01-14
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef CSVGPLOTTER_6VQB4RDO
#define CSVGPLOTTER_6VQB4RDO

#include <plotter.h> // libplot-dev
#include <string>
#include <sstream>

namespace cogx { namespace display {

// A helper class for libplot.
//
// The funcion getScreenSvg() will create an SVG string suitable for drawing on
// screen with pixel coordinates. This means that it will remove viewport/image
// size information from the <svg> tag.
//
// When the viewport information is removed from the SVG image QtSvg can
// calculate the viewport R from vector data in the SVG image. To precisely
// position such an image to the screen (in screen coordinates), the image has
// to be translated with T.translate(R.left, R.top) in QtSvg.
//
// The alternative would be to calculate the viewport and image size based on
// some predefined screen size. The drawback of this approach is that at least
// in QtSvg all graphic elements are clipped to the viewport. In our application
// it is possible that some elemets extend beyond the limits of the screen and
// would thus be clipped.
//
// IMPORTANT: All drawing commands should use -y instead of y. By default
// Plotter uses non-flipped coordinates, ie. y+ is up. Screen and SVG
// coordiantes are flipped, ie. y+ is down. For compatibility with other
// plotters SVGPlotter uses non-flipped coordinates and applies the
// transformation scale(1,-1) to the resulting plot. To compensate, all texts
// are also (unconditionally) transformed with scale(1,-1) otherwise they would
// all be flipped vertically.
//
// Unfortunately we can't just apply scale(1,-1) to the plot because this will
// make all texts upside-down. Scaling each text with fscale(1,-1) also doesn't
// work because it moves the text to a different location. The only (known) way 
// to remove scale(1,-1) from SVG texts is to use a negative ysize in PAGESIZE,
// which also implies negative y coordinates.
//
// SUGGESTION: use #define YY(y) -(y) in your application.
class CSvgStringPlotter: public SVGPlotter
{
   std::ostringstream* pStream;
   double _framewidth;
   std::string _framecolor;
public:
   CSvgStringPlotter(std::ostringstream& ssvg) 
      : SVGPlotter(ssvg)
   {
      parampl("PAGESIZE", (char*)"ysize=-8in");
      pStream = &ssvg;
      _framewidth = 0;
      _framecolor = "white";
   }

   // Open the plotter and remove the background rectangle.
   int openpl(void)
   {
      int rv = SVGPlotter::openpl();
      bgcolorname("none");
      erase();
      return rv;
   }

   // Put a frame around the text so that it will be included in the calculated
   // viewport in QtSvg.
   //
   // x0 and y0 are the coordinates of the top-left corner.
   // Default frame width is 0, use fframewidth() to show it.
   // Default frame color is white. Use framecolor() to change it.
   void fframedtext(double x0, double y0, const std::string& str)
   {
      double fontsize = 20;
      double angle = 0;
      if (drawstate) {
         fontsize = drawstate->font_size;
         angle = drawstate->text_rotation;
      }
      double true_size = ffontsize (fontsize);
      double width = flabelwidth (str.c_str());
      savestate();
      ftranslate(x0, y0);
      frotate(angle);
      textangle(0);
      savestate();
      pencolorname(_framecolor.c_str());
      flinewidth(_framewidth);
      box(0, 0, width+_framewidth, -(true_size+_framewidth));
      restorestate();
      move(_framewidth/2, -(_framewidth/2));
      alabel('l', 't', str.c_str());
      restorestate();
   }

   void fframewidth(double width)
   {
      _framewidth = width;
      if (_framewidth < 0) _framewidth = 0;
   }

   void framecolor(const std::string& color)
   {
      _framecolor = color;
   }

   // Assumption: closepl() was called.
   std::string getSvg()
   {
      return pStream->str();
   }

   // Create an SVG string suitable for drawing on screen with pixel
   // coordinates. This means that the size and viewport data are removed from
   // the <svg> tag.
   //
   // QtSvg: To draw precisely on screen, calculate the bounding rect R of the
   // resulting SVG and apply the translation T.translate(R.left, R.top) to the
   // object.
   //
   // Assumption: closepl() was called.
   std::string getScreenSvg()
   {
      string str = pStream->str();
      ostringstream svgfix;
      size_t pos, ppos = 0;
      for (int i = 0; i < 3; i++) {
         pos = str.find("\n", ppos+1);
         if (pos == str.npos) break; // error
         if (i == 1)
            svgfix << str.substr(0, pos+1);
         ppos = pos;
      }
      svgfix << "<svg version=\"1.1\" baseProfile=\"full\" id=\"body\" preserveAspectRatio=\"none\" "
         "xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
         "xmlns:ev=\"http://www.w3.org/2001/xml-events\">\n";
      svgfix << str.substr(ppos+1);
      return svgfix.str();
   }
};

}} // namespace
#endif /* end of include guard: CSVGPLOTTER_6VQB4RDO */
