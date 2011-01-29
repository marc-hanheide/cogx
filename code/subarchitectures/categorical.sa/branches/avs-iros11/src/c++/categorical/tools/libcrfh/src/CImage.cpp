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
* \file CImage.cpp
* \author Andrzej Pronobis
*
* Contains implementation of the CImage class.
*/

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include "math.h"

#include "global.h"
#include "CMatrix.h"

#include "CImage.h"


// -----------------------------------------
CImage::CImage() : _data(0), _width(0), _height(0), _type(IT_EMPTY)
{
}


// -----------------------------------------
CImage::CImage(int width, int height, ImageType type) :  _width(width), _height(height), _type(type)
{
  _data = aMalloc<char>(width*height*getPixelSize(type));
}


// -----------------------------------------
CImage::CImage(char *data, int width, int height, ImageType type) :  _width(width), _height(height), _type(type)
{
  _data = aMalloc<char>(width*height*getPixelSize(type));
  _data = aMemCopy<char>(_data, data, width*height*getPixelSize(type));
}


// -----------------------------------------
CImage::CImage(const CMatrix &matrix, int noborder)
{
  _type = IT_BYTE;
  _width = matrix.getCols() - 2*noborder;
  _height = matrix.getRows() - 2*noborder;
  int endX = matrix.getCols() - noborder;
  int endY = matrix.getRows() - noborder;

  // Get max and min
  double max=matrix.getElem(noborder,noborder);
  double min=matrix.getElem(noborder,noborder);

  for (int i=noborder; i<endY; ++i)
    for (int j=noborder; j<endX; ++j)
    {
      if (matrix.getElem(i,j)>max) max=matrix.getElem(i,j);
      if (matrix.getElem(i,j)<min) min=matrix.getElem(i,j);
    }

  // Calculate scale
  double scale = 255.0/(max-min);

  //  aout<<max<<" "<<min<<" "<<scale<<endl;

  _data = aMalloc<char>(_width*_height);

  for (int i=noborder; i<endY; ++i)
    for (int j=noborder; j<endX; ++j)
      _data[(i-noborder)*_width+(j-noborder)] = (char)((int)(((matrix.getElem(i, j)-min)*scale)+0.5));
}


// -----------------------------------------
QByteArray CImage::readNotCommentedWord(QFile &file)
{
  char c='#';
  bool ok=true;
  QByteArray word="";

  // Avoid comment
  while((ok) && (c=='#'))
  {
    // Read white space
    while((ok=file.getChar(&c)) && (aIsWhiteChar(c)));

    // Read the whole line if it is a comment
    if ((ok)&&(c=='#')) file.readLine();
  }

  // EOF?
  if (!ok) return "";

  // Read the whole word
  for ( ; (ok) && (!aIsWhiteChar(c)) ; ok=file.getChar(&c))
    word+=c;

  // Return the word
  return word;
}


// -----------------------------------------
int CImage::loadFromFile(QString fileName)
{
  QFile file(fileName);

  if (!file.open(QIODevice::ReadOnly))
    return -1;

  // Read header
  QByteArray magicNumber = readNotCommentedWord(file);
  ImageType it;
  if (magicNumber == "P6")
    it = IT_RGB24;
  else if (magicNumber == "P5")
    it = IT_BYTE;
  else
    return -2;

  int width = readNotCommentedWord(file).toInt();
  int height = readNotCommentedWord(file).toInt();
  if ((!width) || (!height)) return -3;

  int colors = readNotCommentedWord(file).toInt();
  if ((colors<1) || (colors>255)) return -4;

  // Read data
  int size = width*height*getPixelSize(it);
  char *data = aMalloc<char>(size);
  if (file.read(data, size)!=size)
  {
    aFree(data);
    return -5;
  }
  file.close();

  // Everything fine, store all the data inside the class
  _type = it;
  _width = width;
  _height = height;
  if (_data)
    aFree(_data);
  _data = data;

  return 0;
}


// -----------------------------------------
int CImage::saveToFile(QString fileName) const
{
  QFile file(fileName);

  if (!file.open(QIODevice::WriteOnly))
    return -1;

  file.write("P6\n");
  file.write("# Created by the libCRFH library\n");
  file.write(QByteArray::number(_width)+" "+QByteArray::number(_height)+"\n");
  file.write("255\n");

  if (_type==IT_RGB24)
    file.write(_data, _width*_height*3);
  else if (_type==IT_BYTE)
  {
    int elems = _width*_height;
    for (int i=0; i<elems; ++i)
    {
      file.putChar(_data[i]);
      file.putChar(_data[i]);
      file.putChar(_data[i]);
    }
  }
  else
    aout<<"ERROR: Image type not supported!";

  file.close();
  return 0;
}


// -----------------------------------------
CMatrix *CImage::getL(CMatrix *L) const
{
  if (L)
    L->resize(_height, _width);
  else
    L = new CMatrix(_height, _width);

  if (_type==IT_RGB24)
  {
    int elems = _width*_height;
    for (int i=0; i<elems; ++i)
    {
      int index = i*3;
      L->setElem(i, static_cast<double>(
                    static_cast<int>(static_cast<unsigned char>(_data[index]))+
                    static_cast<int>(static_cast<unsigned char>(_data[index+1]))+
                    static_cast<int>(static_cast<unsigned char>(_data[index+2])))/3.0);
    }
  }
  else if (_type==IT_BYTE)
  {
    int elems = _width*_height;
    for (int i=0; i<elems; ++i)
    {
      L->setElem(i, static_cast<double>(
                    static_cast<int>(static_cast<unsigned char>(_data[i])) ));
    }
  }
  else
    aout<<"ERROR: Image type not supported!";

  return L;
}


// -----------------------------------------
CMatrix *CImage::getC1(CMatrix *L) const
{
  if (L)
    L->resize(_height, _width);
  else
    L = new CMatrix(_height, _width);

  if (_type==IT_RGB24)
  {
    int elems = _width*_height;
    for (int i=0; i<elems; ++i)
    {
      int index = i*3;
      L->setElem(i, static_cast<double>(
                    static_cast<int>(static_cast<unsigned char>(_data[index]))-
                    static_cast<int>(static_cast<unsigned char>(_data[index+1])))/2.0);
    }
  }
  else
    aout<<"ERROR: Image type not supported!";

  return L;
}


// -----------------------------------------
CMatrix *CImage::getC2(CMatrix *L) const
{
  if (L)
    L->resize(_height, _width);
  else
    L = new CMatrix(_height, _width);

  if (_type==IT_RGB24)
  {
    int elems = _width*_height;
    for (int i=0; i<elems; ++i)
    {
      int index = i*3;
      L->setElem(i, static_cast<double>(
                                        (static_cast<int>(static_cast<unsigned char>(_data[index]))+
                                         static_cast<int>(static_cast<unsigned char>(_data[index+1])))/2.0-
                                         static_cast<int>(static_cast<unsigned char>(_data[index+2]))
                                       ));
    }
  }
  else
    aout<<"ERROR: Image type not supported!";

  return L;
}


// -----------------------------------------
int CImage::getPixelSize(ImageType type)
{
  switch(type)
  {
  case IT_BYTE:
    return 1;
  case IT_RGB24:
    return 3;
  default:
    return 0;
  };
}


// -----------------------------------------
void CImage::resize(int width, int height, ImageType type)
{
  int bytes = getPixelSize(type);
  if ((!bytes) || (width<=0) || (height<=0))
  {
    if (_data)
    {
      aFree(_data);
      _data=0;
    }
    aout<<"ERROR: Incorrect image type ("<<bytes<<" bytes per pixel) or size("<<width<<"x"<<height<<")!"<<endl;
  }

  bytes*=width;
  bytes*=height;

  if ( bytes!=(_width*_height*getPixelSize(_type)) )
  {
    _data = aRealloc<char>(_data, bytes);
    _width = width;
    _height = height;
    _type = type;
  }
}


// -----------------------------------------
CImage *CImage::getCropped(int left, int top, int right, int bottom, CImage *result)
{
  if (result)
    result->resize(_width-left-right, _height-top-bottom, _type);
  else
    result = new CImage(_width-left-right, _height-top-bottom, _type);

  int pixelSize = getPixelSize(_type);
  int shift = _width*pixelSize;
  int newShift = (_width-left-right)*pixelSize;
  int endWidth = (_width-right)*pixelSize;
  int endHeight = (_height-bottom);
  char *resultData = result->getRawData();

  for(int i=top; i<endHeight; ++i)
    for (int j=left*pixelSize; j<endWidth; j+=pixelSize)
      aMemCopy(resultData+(i-top)*newShift+(j-left*pixelSize), _data+i*shift+j, pixelSize);

  return result;
}


// -----------------------------------------
void CImage::createFromBuffer(const char *data, int width, int height, ImageType type)
{
  resize(width, height, type);
  _data = aMemCopy<char>(_data, data, width*height*getPixelSize(type));
}

