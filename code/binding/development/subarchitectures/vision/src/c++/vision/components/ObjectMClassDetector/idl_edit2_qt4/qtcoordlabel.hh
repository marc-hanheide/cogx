/*********************************************************************/
/*                                                                   */
/* FILE         qtcoordlabel.hh                                      */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      Define a Qt widget, derived from QLabel, that can    */
/*              display image coordinates and pixel colors. Designed */
/*              in conjunction with QtResizeImg.                     */
/*                                                                   */
/* BEGIN        Wed May 16 2001                                      */
/* LAST CHANGE  Fri Aug 30 2001                                      */
/*                                                                   */
/*********************************************************************/

#ifndef LEIBE_QTCOORDLABEL_H
#define LEIBE_QTCOORDLABEL_H

#include <qobject.h>
#include <qlabel.h>
#include <qstring.h>
#include <qcolor.h>


class QtCoordLabel : public QLabel
{
  Q_OBJECT

public: 
    QtCoordLabel( QString text, QWidget *parent=0, const char *name=0 );
    QtCoordLabel( QWidget *parent=0, const char *name=0 );
public slots:
    void setCaption( QString text );
    void setCoordValue( int );
    void setCoordValue( int, int );
    void setCoordValue( int, int, QRgb );
    void setCoordValue( int, int, QRgb, float );

private:
    void display( QString text );
    QString m_strCaption;
};


#endif //LEIBE_QTCOORDLABEL_H
