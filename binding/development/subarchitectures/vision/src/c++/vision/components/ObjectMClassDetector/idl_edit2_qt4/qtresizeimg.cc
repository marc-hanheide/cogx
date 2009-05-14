/*********************************************************************/
/*                                                                   */
/* FILE         qtresizeimg.cc                                       */
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

#include <qimage.h>
#include <qstring.h>
#include <qpixmap.h>
#include <qpainter.h>
#include <qregion.h>
#include <qmatrix.h>
//Added by qt3to4:
#include <QPaintEvent>
#include <QMouseEvent>

#include <math.h>
#include <stdio.h>

#include "qtresizeimg.hh"


/***********************************************************/
/*                      Constructors                       */
/***********************************************************/
QtResizeImg::QtResizeImg( QWidget *parent, const char *name )
  : QWidget( parent, name )
{
  m_nWidth  = 10000;
  m_nHeight = 10000;
  m_nDepth  = -1;
  m_fScaleX = 1.0;
  m_fScaleY = 1.0;
  Init();
}


QtResizeImg::QtResizeImg( int w, int h , int depth, 
		      QWidget *parent, const char *name )
  : QWidget( parent, name )
{
  m_nWidth  = w;
  m_nHeight = h;
  m_nDepth  = depth;
  m_fScaleX = 1.0;
  m_fScaleY = 1.0;
  Init();
}


QtResizeImg::QtResizeImg( const QSize& size, int depth, 
			  QWidget *parent, const char *name )
  : QWidget( parent, name )
{
  m_nWidth  = size.width();
  m_nHeight = size.height();
  m_nDepth  = depth;
  m_fScaleX = 1.0;
  m_fScaleY = 1.0;
  Init();
}


void QtResizeImg::Init()
{
  m_pixmap   = new QPixmap( m_nWidth, m_nHeight);//, m_nDepth );
  m_pmScaled = new QPixmap( qRound(m_nWidth*m_fScaleX), qRound(m_nHeight*m_fScaleY));// ,m_nDepth );
  m_bImageLoaded = FALSE;

  m_bGrayImageLoaded = FALSE;

  m_nSelectX = -1;
  m_nSelectY = -1;
  m_nSelectRadius = 20;
  m_cSelectColor = Qt::magenta;
}

QtResizeImg::~QtResizeImg()
  /* standard destructor */
{
  if( m_pixmap )   delete m_pixmap;
  if( m_pmScaled ) delete m_pmScaled;
}


/***********************************************************/
/*                          Slots                          */
/***********************************************************/
void QtResizeImg::setSize( int w, int h )
  /* set the image size (independent of scale) */
{
  m_nWidth = w;
  m_nWidth = h;
}


void QtResizeImg::setSize( QSize size )
  /* set the image size (independent of scale) */
{
  m_nWidth = size.width();
  m_nHeight = size.height();
}


void QtResizeImg::setScale( float sx, float sy )
  /* set the image scale */
{
  m_fScaleX = sx;
  m_fScaleY = sy;
}


void QtResizeImg::loadImage( const QImage &img )
  /* load a new image from a QImage structure */
{
  m_img = img.copy();
  m_pixmap->convertFromImage( m_img, 0 );
  m_bImageLoaded = true;
  setSize( m_img.size() );

  m_bGrayImageLoaded = false;
}

void QtResizeImg::loadImage( const QPixmap &pixmap )
  /* load a new image from a QPixmap structure */
{
  m_img = pixmap.convertToImage();
  (*m_pixmap) = pixmap;
  //m_pixmap->convertFromImage( m_img, 0 );
  m_bImageLoaded = true;
  setSize( m_img.size() );

  m_bGrayImageLoaded = false;
}


void QtResizeImg::loadImage( const QString& filename )
  /* load a new image from disk */
{
  m_img.load(filename);
  
  if ( imgSize() != m_img.size() ) {
    setSize( m_img.size() );
    m_pixmap->resize( m_img.size() );
    adjustSize();
    emit adjustSizeNeeded( imgWidth(), imgHeight() );
  }
  m_pixmap->convertFromImage( m_img );
  m_bImageLoaded = true;

  m_bGrayImageLoaded = false;
}


void QtResizeImg::display()
  /* display the current image */
{
  if ( imageLoaded() ) {
    //printf("Displaying...\n");
    repaint();
  }
}


void QtResizeImg::setSelectRadius( int r )
  /* specify a radius for the selection */
{
  m_nSelectRadius = r;
  updateRectSelect();
  emit selectRadiusChanged( r );

  repaint();
}


void QtResizeImg::setSelectRadius( int x, int y )
  /* Specify a radius for the selection by taking the distance from  */
  /* the center point to (x,y).                                      */
{
  int dist_x = m_nSelectX - x;
  int dist_y = m_nSelectY - y;
  m_nSelectRadius = qRound( sqrt(dist_x*dist_x + dist_y*dist_y) );
  updateRectSelect();
  emit selectRadiusChanged( m_nSelectRadius );

  repaint();
}


void QtResizeImg::selectImgRegion( int x, int y )
  /* select an image region centered on the point (x,y) */
{
  QRegion updateRegion( m_rectSelect );
  
  m_nSelectX = x;
  m_nSelectY = y;
  updateCenterSelect();
  updateRectSelect();
  updateRegion = updateRegion.unite( m_rectSelect );

  if (m_nSelectRadius > 0) {
    //repaint( m_rectSelect );
    repaint( updateRegion, false );
  }
}


void QtResizeImg::updateRectSelect()
  /* update the rectangle structure for the selected image region    */
  /* (in scaled coordinates)                                         */
{
  m_rectSelect.setLeft ( qRound((m_nSelectX - m_nSelectRadius) * m_fScaleX) );
  m_rectSelect.setTop  ( qRound((m_nSelectY - m_nSelectRadius) * m_fScaleY) );
  m_rectSelect.setWidth( qRound((2*m_nSelectRadius+1) * m_fScaleX) );
  m_rectSelect.setHeight( qRound((2*m_nSelectRadius+1) * m_fScaleY) );
}


void QtResizeImg::updateCenterSelect()
  /* update the center point structure for the selected image region */
  /* (in scaled coordinates)                                         */
{
  m_ptCenter.setX( qRound(m_nSelectX * m_fScaleX) );
  m_ptCenter.setY( qRound(m_nSelectY * m_fScaleY) );  
}


/***********************************************************/
/*                         Events                          */
/***********************************************************/
void QtResizeImg::paintEvent( QPaintEvent* event )
{
  if ( !imageLoaded() )
    return;

//  if ( !event->rect().intersects( m_pixmap->rect() ) )
//    return;

  QPainter p(this);
  p.setClipRect(event->rect());
  
  //  if ( m_pixmap->size() == size() ) {
  //  p.drawPixmap(0,0,*m_pixmap);

  //} else {
  if ( m_pmScaled->size() != size() ) {
    QMatrix m;
    float rw = ((float) width() )/((float) m_pixmap->width() );
    float rh = ((float) height() )/((float) m_pixmap->height() );
    if (rw>0 && rh>0)
      if (rw < rh) {
	m.scale(rw,rw);
	m_fScaleX = rw;
	m_fScaleY = rw;
      } else {
	m.scale(rh,rh);
	m_fScaleX = rh;
	m_fScaleY = rh;
      }
    *m_pmScaled = m_pixmap->xForm(m);
  }


  if ( !event->rect().intersects( m_pmScaled->rect() ) )
    return;
    
  //}  
  
  // draw the selected region
  if ( (m_nSelectX >= 0) && (m_nSelectY >=0) && (m_nSelectRadius >0 ) ) {
    QPainter tmp(m_pmScaled);
    paintSelectedRegion( &tmp );
    tmp.end();
  }
  
  p.drawPixmap(0,0,*m_pmScaled);
}


void QtResizeImg::paintSelectedRegion( QPainter *p )
  /* paint the selected region */
{
    p->setPen( m_cSelectColor );

    p->drawLine( m_ptCenter.x()-3, m_ptCenter.y(), m_ptCenter.x()+3, m_ptCenter.y() );
    p->drawLine( m_ptCenter.x(), m_ptCenter.y()-3, m_ptCenter.x(), m_ptCenter.y()+3 );
    p->drawEllipse( m_rectSelect );
    p->drawRect( m_rectSelect );
}


void  QtResizeImg::drawPoint( int x, int y, QColor color, bool bUpdateImage )
  /* draw a small point into the image at the specified location */
{
  QPainter p(this);

  p.setPen( color );

  int px = qRound(x * m_fScaleX);
  int py = qRound(y * m_fScaleY);
  p.drawPoint( px, py );

	if( bUpdateImage ) {
		QPainter pImg(m_pixmap);

		pImg.setPen( color );
		pImg.drawPoint( x, y );
	}
}


void  QtResizeImg::drawCross( int x, int y, QColor color, bool bUpdateImage )
  /* draw a small cross into the image at the specified location */
{
  QPainter p(this);

  p.setPen( color );

  int px = qRound(x * m_fScaleX);
  int py = qRound(y * m_fScaleY);
  p.drawLine( px-3, py, px+3, py );
  p.drawLine( px, py-3, px, py+3 );

	if( bUpdateImage ) {
		QPainter pImg(m_pixmap);

		pImg.setPen( color );
		pImg.drawLine( x-3, y, x+3, y );
		pImg.drawLine( x, y-3, x, y+3 );
	}
}


void  QtResizeImg::drawCircle( int x, int y, float radius, QColor color, 
															 bool bUpdateImage )
  /* draw a circle into the image at the specified location */
{
  QPainter p(this);

  p.setPen( color );

  int px = qRound(x * m_fScaleX);
  int py = qRound(y * m_fScaleY);
  int pradius = qRound(radius * m_fScaleY);

  p.drawEllipse( px-pradius, py-pradius, 2*pradius, 2*pradius );

	if( bUpdateImage ) {
		QPainter pImg(m_pixmap);

		pImg.setPen( color );
		pImg.drawEllipse( x-(int)radius, y-(int)radius, 
											(int) (2*radius), (int) (2*radius) );
	}
}


void  QtResizeImg::drawLine( int startx, int starty, int endx, int endy,
                             QColor color, bool bUpdateImage )
  /* draw a line into the image at the specified location */
{
  QPainter p(this);

  p.setPen( color );

  int startpx = qRound(startx * m_fScaleX);
  int startpy = qRound(starty * m_fScaleY);
  int endpx = qRound(endx * m_fScaleX);
  int endpy = qRound(endy * m_fScaleY);
  
  p.drawLine( startpx, startpy, endpx, endpy );


	if( bUpdateImage ) {
		QPainter pImg(m_pixmap);

		pImg.setPen( color );
		pImg.drawLine( startx, starty, endx, endy );
	}
}


void  QtResizeImg::drawRect( int startx, int starty, int width, int height,
                             QColor color, bool bUpdateImage )
  /* draw a line into the image at the specified location */
{
  QPainter p(this);

  p.setPen( color );

  int startpx = qRound(startx * m_fScaleX);
  int startpy = qRound(starty * m_fScaleY);
  int widthp  = qRound(width * m_fScaleX);
  int heightp = qRound(height * m_fScaleY);
  
  p.drawRect( startpx, startpy, widthp, heightp );

	if( bUpdateImage ) {
		QPainter pImg(m_pixmap);

		pImg.setPen( color );
		pImg.drawRect( startx, starty, width, height );
	}
}


void  QtResizeImg::mousePressEvent( QMouseEvent* event )
  /* emits signals for left and right mouse clicks on pixels */
{
  int px, py;

  //px = qRound( event->x() / m_fScaleX );
  //py = qRound( event->y() / m_fScaleY );
  px = (int) floor( event->x() / m_fScaleX );
  py = (int) floor( event->y() / m_fScaleY );

  if ( event->button() == Qt::LeftButton ) {
    //printf("emitting leftClickOnPixel(%3d,%3d)\n", px, py);
    emit leftClickOnPixel( px, py );
    
  } else if ( event->button() == Qt::RightButton ) {
    //printf("emitting rightClickOnPixel(%3d,%3d)\n", px, py);
    emit rightClickOnPixel( px, py );
  }
}


void  QtResizeImg::mouseMoveEvent( QMouseEvent* event )
  /* if activated by "setMouseTracking( TRUE )", emit signals for    */
  /* every mouse movement. Else, just emit signals when a button is  */
  /* pressed while the mouse is moved.                               */
{
  int px, py;

  if ( !rect().contains( event->pos() ) )
    return;

  //px = qRound( event->x() / m_fScaleX );
  //py = qRound( event->y() / m_fScaleY );
  px = (int) floor( event->x() / m_fScaleX );
  py = (int) floor( event->y() / m_fScaleY );
  emit mouseMovedOnPixel( px, py );
  if ( imageLoaded() && (px>=0) && (py>=0) && 
       (py < m_nHeight) && (px < m_nWidth) ) {
    emit mouseMovedOnPixel( px, py, *((uint *)m_img.scanLine(py) + px) );
  }

  if ( event->state() == Qt::LeftButton ) {
    emit leftDragOnPixel( px, py );
    
  } else if ( event->state() == Qt::RightButton ) {
    emit rightDragOnPixel( px, py );
  }
}


/***********************************************************/
/*                     Miscellaneous                       */
/***********************************************************/
QSize QtResizeImg::sizeHint() const
  /* the preferred size is the size of the image */
{
    return QSize( 10000, 10000);//imgWidth(), imgHeight() );
}


QSizePolicy QtResizeImg::sizePolicy() const
  /* the user is allowed to expand the image */
{
  return QSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding, true );
  //return QSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
}


void QtResizeImg::loadTestImage()
  /* load a test image */
{
  loadImage("test.jpg");
  display();
}
