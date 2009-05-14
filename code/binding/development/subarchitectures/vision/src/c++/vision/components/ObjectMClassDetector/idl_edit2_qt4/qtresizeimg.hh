/*********************************************************************/
/*                                                                   */
/* FILE         qtresizeimg.hh                                       */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      Define a Qt widget, derived from QWidget, that can   */
/*              display an automatically resizable image, and that   */
/*              can send out image coordinates and pixel colors for  */
/*              the pixel currently under the mouse pointer. Design- */
/*              ed in conjunction with QtCoordLabel.                 */
/*                                                                   */
/* BEGIN        Wed May 09 2001                                      */
/* LAST CHANGE  Thu Apr 18 2002                                      */
/*                                                                   */
/*********************************************************************/

#ifndef LEIBE_QTRESIZEIMG_H
#define LEIBE_QTRESIZEIMG_H

#include <qwidget.h>
#include <qsize.h>
#include <qpoint.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qcolor.h>
//Added by qt3to4:
#include <QMouseEvent>
#include <QPaintEvent>


class QString;

class QtResizeImg : public QWidget
{
	Q_OBJECT
public:
	QtResizeImg( QWidget *parent=0, const char *name=0 );
	QtResizeImg( int w, int h , int depth = -1, QWidget *parent=0, 
							 const char *name=0 );
	QtResizeImg( const QSize &, int depth = -1, QWidget *parent=0, 
							 const char *name=0 );
	~QtResizeImg();
	
	int    imgWidth()  const { return m_nWidth;  }
	int    imgHeight() const { return m_nHeight; }
	QSize  imgSize() const { return QSize( m_nWidth, m_nHeight); }
	int    imgDepth()  const { return m_nDepth;  }
	float  imgScaleX() const { return m_fScaleX; }
	float  imgScaleY() const { return m_fScaleY; }
	bool   imageLoaded() const { return m_bImageLoaded; }
	QImage getImage() const { return m_pixmap->convertToImage(); }
	QPoint imgSelectedPoint() const { return QPoint( m_nSelectX, m_nSelectY ); }
	int    imgSelectedRadius() const { return m_nSelectRadius; }
	QSize sizeHint() const;
	QSizePolicy sizePolicy() const;
  
public slots:
	void   setSize( int w, int h );
  void   setSize( QSize size );
	void   setScale( float sx, float sy );
	void   loadImage( const QImage&  img );
	void   loadImage( const QPixmap&  pixmap );
	void   loadImage( const QString& filename );
	void   loadTestImage();
	void   display();
	void   setSelectRadius( int r );
	void   setSelectRadius( int x, int y );
	void   setSelectColor( QColor &color ) { m_cSelectColor = color; }
	void   selectImgRegion( int x, int y );
  
signals:
	void   leftClickOnPixel(int, int);
	void   rightClickOnPixel(int, int);
	void   leftDragOnPixel(int, int);
	void   rightDragOnPixel(int, int);
	void   mouseMovedOnPixel(int, int);
	void   mouseMovedOnPixel(int, int, QRgb);
	void   mouseMovedOnPixel(int, int, QRgb, float);
	void   adjustSizeNeeded(int, int);
	void   selectRadiusChanged(int);
	
protected:
	void  paintEvent( QPaintEvent * );
	void  mousePressEvent( QMouseEvent * );
	void  mouseMoveEvent( QMouseEvent * );

public:
	void  drawPoint( int x, int y, QColor color, bool bUpdateImage = false );
	void  drawCross( int x, int y, QColor color, bool bUpdateImage = false );
  void  drawCircle( int x, int y, float radius, QColor color, 
                    bool bUpdateImage = false );
	void  drawLine( int startx, int starty, int endx, int endy, QColor color,
                  bool bUpdateImage = false );
	void  drawRect( int startx, int starty, int width, int height, QColor color,
                  bool bUpdateImage = false );

private:
	void  Init();
	void  paintSelectedRegion( QPainter * );
	
	int      m_nWidth;
	int      m_nHeight;
	int      m_nDepth;
	
	float    m_fScaleX;
	float    m_fScaleY;
	
	QPixmap *m_pixmap;
	QPixmap *m_pmScaled;
	bool     m_bImageLoaded;
	QImage   m_img;
	
	bool     m_bGrayImageLoaded;
	
	int      m_nSelectX;
	int      m_nSelectY;
	int      m_nSelectRadius;
	QColor   m_cSelectColor;
	QPoint   m_ptCenter;
	QRect    m_rectSelect;
	void  updateRectSelect();
	void  updateCenterSelect();
};

#endif // LEIBE_QTRESIZEIMG_H
 
