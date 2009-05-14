#ifndef IDL_EDITIMGDISP_HH
#define IDL_EDITIMGDISP_HH

#include "qtresizeimg.hh"
//Added by qt3to4:
#include <QPaintEvent>
#include "imgdescrlist.hh"


////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp (inherits QResizeImg)
//______________________________________________________________________________
// This is the IDL_EditImgDisp Widget (IDL-Editor's Image Display).         
// It shows always a certain image and the rectangles given by the 
// current ImgDescr. 
// Also you can create and edit rectangle by mouse drags.
// See further comments after class definition.
////////////////////////////////////////////////////////////////////////////////
class IDL_EditImgDisp : public QtResizeImg
{
	Q_OBJECT

public:
  IDL_EditImgDisp( QWidget *parent=0, const char *name=0 );

  void setCurImgDescr( ImgDescr* cID,ImgDescr* bID=NULL,int cRect = -1 );
  void setCurRect    ( int cRect);
  int getCurRect();

  QColor normColor;	
  QColor selColor;	
  QColor backColor;	

signals:
  void changed();
  
public slots:
  void selectRect(bool forward = true);
  void deleteRect();
  void setThresholdLow(double low);
  void setThresholdHigh(double high);

protected slots:
  void onDragLeft(int x, int y);
  void onDragRight(int x, int y);
  void onClickLeft(int x, int y);
  void onClickRight(int x, int y);

protected:
  void  paintEvent( QPaintEvent * );
			
private:
  ImgDescr* curImgDescr; 	
  ImgDescr* curBackImgDescr; 	
  int curRect;
  Rect dragRect;
  double threshLow;
  double threshHigh;
};
////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp's Constructor
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// public IDL_Edit( QWidget *parent=0, const char *name=0 );
//______________________________________________________________________________
// This is the one and only constructor. 
// Simply use it like a QWidget constructor.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp's public funtions and members
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void setCurImgDescr(ImgDescr* cID,ImgDescr* bID=NULL,int cRect = -1);
//______________________________________________________________________________
// Use:
//   Updates the current ImgDescr and the Background ImgDescr as well.
//   You can also set the currently selected rectangle.
// Parameters: 
//   cID: the current ImgDescr. (if NULL no current Image)
//   bID: the current background ImgDescr (if NULL no cur. backgr. image descr)
//   cRect: the currently selected rectangle (see setCurRect())
//
//
//
// void setCurRect(int cRect);
//______________________________________________________________________________
// Use:
//   Sets the currently selected rectangle
// Parameters:
//   cRect: index of the rectangle in the RectList of the ImgDescr.
//     if cRect is set to -1 no rectangle is selected.
// Precond: none; due to security checks meaningless input will lead to 
//   no selection
//
//
// public mebers:
//______________________________________________________________________________
//
//  QColor normColor;	the color value for the normal, selected and backgr.
//  QColor selColor;	rectangles. If you change these values make sure the
//  QColor backColor;	effect is taking place by calling e.g. repaint()
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp's signals
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void changed();
//______________________________________________________________________________
// Use:
//   This signal is sent if the currentlyselected rectangle changed
//   This means the rectangle or the selection can change.
//   This signal isn't emited by the object's creation.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp's public slots and functions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void selectRect();
//______________________________________________________________________________
// Use:
//   Selects the next or previous rectangle of the current image Description. 
//   The selection series forms a cycle
// Parameters:
//   forward: It cycles forwards (true, default) or backwards (false) 
//
//
//
// void deleteRect();
//______________________________________________________________________________
// Use:
//   Deletes the currently selected rectangle.
//   The newly selected image is the next one or the last one.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditImgDisp's protected slots and functions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void onDragLeft(int x, int y);
//______________________________________________________________________________
// Use:
//   Creates and updates a new rectangle
//   The dragged rectangle has exceed a certain size before a new rectangle is 
//   created and added to the RectList.
//   This is a protection against the unintentionally creation of rectangles
//	 It's made sure that the rectangles always are completely in the image.
//   The caller is the Object itself. (QtResizeImg)
//
//
//
// void onDragRight(int x, int y);
//______________________________________________________________________________
// Use:
//   Updates the selected rectangle
//	 It's made sure that the rectangles always are completely in the image.
//   The caller is the Object itself. (QtResizeImg)
//
//
//
// void onClickLeft(int x, int y);
//______________________________________________________________________________
// Use:
//   Prepares to create a new rectangle
//   The caller is the Object itself. (QtResizeImg)
//
//
//
// void onClickRight(int x, int y);
//______________________________________________________________________________
// Use:
//   Prepares to change the selected rectangle
//   The caller is the Object itself. (QtResizeImg)
//
//
//
// void paintEvent( QPaintEvent * );
//______________________________________________________________________________
// Use:
//   Extends the QtResizeImg Version.
//   The Rectangles are painted automaticaly.
//   Rectangles width or height equal to zero are drawn like rectangles with 
//   with or heigth equal to one to make sure they appear on screen.
//   The rectangles are always drawn in the order of the RectList.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_ImgDisp's private members
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// private members
//______________________________________________________________________________
// 
//	ImgDescr* curImgDescr; 	    current Image Description (can be NULL)
//	ImgDescr* curBackImgDescr; 	current background Image Descr. (can be NULL)
//	int curRect;                currently selected rectangle (can be -1)
//	Rect dragRect;				dragged but invisible rectangle
////////////////////////////////////////////////////////////////////////////////

#endif
