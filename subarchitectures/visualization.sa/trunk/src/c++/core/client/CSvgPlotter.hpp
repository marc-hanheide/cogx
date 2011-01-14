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
// screen with pixel coordinates.
class CSvgStringPlotter: public SVGPlotter
{
   std::ostringstream* pStream;
public:
   CSvgStringPlotter(std::ostringstream& ssvg) 
      : SVGPlotter(ssvg)
   {
      pStream = &ssvg;
   }

   // Set the viewport to the default range, flip the Y coordinate and erase -- removes background rect.
   // Useful when coordinates are pixel coordinates for drawing on screen.
   // Assumption: openpl() was called. Will erase() the object.
   void removeViewport()
   {
      // back to the default viewport
      fspace (0.0, 0.0, 1.0, 1.0);

      // flip the Y coordinate
      fscale (1.0, -1.0);
      ftranslate(0.0, -1.0);

      // remove background rectangle
      bgcolorname("none");
      erase();
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
   // Assumption: removeViewport() and closepl() were called.
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
