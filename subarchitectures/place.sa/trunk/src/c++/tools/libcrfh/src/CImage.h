// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
* \file CImage.h
* \author Andrzej Pronobis
*
* Contains declaration of the CImage class.
*/

#ifndef _CIMAGE_H_
#define _CIMAGE_H_

#include <QtCore/QByteArray>
#include "global.h"

class QFile;
class CMatrix;

/** Image type. */
enum ImageType
{
  /** Empty. */
  IT_EMPTY = 0,

  /** 24 bits per pixel, triplet of red, green, blue values. */
  IT_RGB24,

  /** 8bits, one channel */
  IT_BYTE
};


/**
* Class storing an image.
*/
class CImage
{

public:

  /** Constructor. Creates an empty image. */
  CImage();

  /** Constructor. */
  CImage(int width, int height, ImageType type);

  /** Constructor. Creates an image from an external buffer.*/
  CImage(char *data, int width, int height, ImageType type);

  /** Conversion from CMatrix. */
  CImage(const CMatrix &matrix, int noborder = 0);

  /** Destructor. */
  inline ~CImage()
  {
    if (_data)
      aFree(_data);
  }

public:

  /** Resizes the image if the current size if different than given. */
  void resize(int width, int height, ImageType type);

  /** Returns pointer to the internal data buffer. */
  inline char *getRawData() const
  {
    return _data;
  }


public:

  /** Returns the intensity channel L as a matrix of doubles. */
  CMatrix *getL(CMatrix *L = 0) const;

  /** Returns the chromatic channel C1 as a matrix of doubles. */
  CMatrix *getC1(CMatrix *L = 0) const;

  /** Returns the chromatic channel C1 as a matrix of doubles. */
  CMatrix *getC2(CMatrix *L = 0) const;

  /** Returns the red channel as a matrix of doubles. */
  CMatrix *getR(CMatrix *L = 0) const;

  /** Returns the green channel as a matrix of doubles. */
  CMatrix *getG(CMatrix *L = 0) const;

  /** Returns the blue channel as a matrix of doubles. */
  CMatrix *getB(CMatrix *L = 0) const;


public:

  /** Returns an image with cropped borders. */
  CImage *getCropped(int left, int top, int right, int bottom, CImage *result=0);

public:

  /** Loads the image from a PPM file. */
  int loadFromFile(QString fileName);

  /** Save image to a PPM file. */
  int saveToFile(QString fileName) const;

  /** Creates the image from an external buffer.*/
  void createFromBuffer(const char *data, int width, int height, ImageType type);


private:

  /** Returns the number of bytes per pixel for a given image type. */
  int getPixelSize(ImageType type);

private:

  /** Internal buffer. */
  char *_data;

  /** Size of the image. */
  int _width, _height;

  /** Image type. */
  ImageType _type;

private:

  QByteArray readNotCommentedWord(QFile &file);

};



#endif

