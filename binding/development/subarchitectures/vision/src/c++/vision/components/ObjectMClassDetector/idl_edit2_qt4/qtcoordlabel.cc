/*********************************************************************/
/*                                                                   */
/* FILE         qtcoordlabel.cc                                      */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      Implement a Qt widget, derived from QLabel, that can */
/*              display image coordinates and pixel colors. Designed */
/*              in conjunction with QtResizeImg.                     */
/*                                                                   */
/* BEGIN        Wed May 16 2001                                      */
/* LAST CHANGE  Fri Aug 30 2001                                      */
/*                                                                   */
/*********************************************************************/

#include <qlabel.h>
#include <qstring.h>

#include "qtcoordlabel.hh"

QtCoordLabel::QtCoordLabel( QString text, QWidget *parent, const char *name )
  : QLabel ( parent, name )
{
  m_strCaption = text;
  display( "(  0,  0)" );
}
QtCoordLabel::QtCoordLabel( QWidget *parent, const char *name )
  : QLabel ( parent, name )
{
  m_strCaption = "";
  display( "(  0,  0)" );
}


void QtCoordLabel::setCaption( QString text )
{
  m_strCaption = text;
}


void QtCoordLabel::setCoordValue( int x )
{
  QString strCoord;
  strCoord.sprintf("(%3d)", x);
  display( strCoord );
}


void QtCoordLabel::setCoordValue( int x, int y )
{
  QString strCoord;
  strCoord.sprintf("(%3d,%3d)", x, y);
  display( strCoord );
}


void QtCoordLabel::setCoordValue( int x, int y, QRgb rgbVal )
{
  QString strCoord;
  strCoord.sprintf("(%3d,%3d: [%3d,%3d,%3d])", x, y, 
		   qRed(rgbVal), qGreen(rgbVal), qBlue(rgbVal) );
  display( strCoord );
}


void QtCoordLabel::setCoordValue( int x, int y, QRgb rgbVal, float grayVal )
{
  QString strCoord;
  strCoord.sprintf("(%3d,%3d: %5.2f [%3d,%3d,%3d])", x, y,
		   grayVal,
		   qRed(rgbVal), qGreen(rgbVal), qBlue(rgbVal) );
  display( strCoord );
}


void QtCoordLabel::display( QString text )
{
  setText( m_strCaption + " " + text);
}
