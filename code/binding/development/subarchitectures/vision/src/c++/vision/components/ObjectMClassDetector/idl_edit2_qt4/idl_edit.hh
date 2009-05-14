#ifndef IDL_EDIT_HH
#define IDL_EDIT_HH

#include <qwidget.h>
#include <qimage.h>
#include <qstring.h>
#include <q3listbox.h>
#include <qpushbutton.h>
#include <q3vbox.h>
#include <qlabel.h>
#include <qlineedit.h>

#include "qtresizeimg.hh"
//Added by qt3to4:
#include <QKeyEvent>
#include "imgdescrlist.hh"
#include "idl_editimgdisp.hh"

const bool                     bFixObjDim  = true;
const double                   DIST        = 0.5;
const double                   COVER       = 0.5;
const double                   OVERLAP     = 0.5;
const double                   ASPECTRATIO = 0.647; // = 110.0/170.0
const ImgDescrList::FixDimType eFixObjDim  = ImgDescrList::FIX_OBJHEIGHT; 


////////////////////////////////////////////////////////////////////////////////
//
// class IDL_EditListBox (inherits QListBox)
//______________________________________________________________________________
// This class behaves like a normal QListBox, only the incoming key events
// for "delete", "left" and "right" are ignored and send back to the parent.
// This was done by overwriting keyPressEvent 
// No futher comments.
////////////////////////////////////////////////////////////////////////////////
class IDL_EditListBox : public Q3ListBox
{
   Q_OBJECT
public:   
   IDL_EditListBox( QWidget *parent=0, const char *name=0);
   
protected:
	void keyPressEvent(QKeyEvent* );

};


////////////////////////////////////////////////////////////////////////////////
//****************************************************************************//
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
//
// class IDL_Edit (inherits QHBox)
//______________________________________________________________________________
// This is the IDL_Edit Widget(Image Description List Editor).         
// It creates the GUI and manages all necessary data.
// To handle the IDL datas an ImgDescrList is used
// To edit the rectangles an IDL_EditImgDisp Widget is used
// To choose within the list of images IDL_EditListBox is used.
// See further comments after class definition.
////////////////////////////////////////////////////////////////////////////////
class IDL_Edit : public Q3HBox
{
   Q_OBJECT
   
public:
    IDL_Edit( QWidget *parent=0, const char *name=0 );
    
protected slots:

	void updateImage(Q3ListBoxItem* item);
	void updateInfoLabel();

  void saveCurrentImage();
  void saveAllImages();
  void saveCorrect();
  void saveMissed();
  void setSaveDir();
  
  void selectImgs();
	void nextImg();
	void prevImg();
	void removeImg();
  
	void clearAllRects();
  
  void openIDL();
  void saveIDL();
  void saveCopy();
	void addIDsFrom();
  
  void openBackIDL();
  void closeBackIDL();
	
  void setNormColor();
  void setSelColor();
  void setBackColor();

  void thresholdsChanged();

protected:
	void keyPressEvent(QKeyEvent* );
	
private:
  double threshLow;
  double threshHigh;

	void loadImage(const QString& fname);
        void updateListBox();
	void updateCaption();

	QString IDL_File;
	QString IDL_Dir;
        QString saveDir;

    ImgDescrList myIDL;
	ImgDescr* curImgDescr;

    ImgDescrList myBackIDL;
	ImgDescr* curBackImgDescr;
	
	QImage  curImg;    
	IDL_EditImgDisp* imgDisp;
	IDL_EditListBox *lb_files;

	QLabel* lb_info;

	QPushButton* bt_saveIDL;
	QPushButton* bt_selectImgs;
	QPushButton* bt_addIDsFrom;

  QLineEdit *leScoreFilterLow;
  QLineEdit *leScoreFilterHigh;
  
};

////////////////////////////////////////////////////////////////////////////////
//
// class IDL_Edit's Constructor
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// public IDL_Edit( QWidget *parent=0, const char *name=0 );
//______________________________________________________________________________
// This is the one and only constructor. It creates the GUI
// Simply use it like a QWidget constructor.
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
//
// class IDL_Edit's protected slots and functions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void updateImage(QListBoxItem* item);
//______________________________________________________________________________
// Use:
//   Has to be called if the current selected image changes
//   It is usually connected to the ListBox
//   It loads (if possible) the according image and updates the imgDisp
// Parameters: 
//   item: the current item. (can also be NULL)
//
//
//
// void updateInfoLabel();
//______________________________________________________________________________
// Use:
//   Updates the InfoLabel.
//   Has to be called if the current Rectangle changes. 
//   It is usually called by the imgDisp.
//
//
//
// void selectImgs();
//______________________________________________________________________________
// Use:
//   Shows up an Open File Dialog and adds the selected images to your IDL.
//   It is usually called by the click() event of the according button.
//
//
//
// void nextImgs();
//______________________________________________________________________________
// Use:
//   Changes the current image to the next image in lb_files
//   Usually calleb by the keyPressEvent-Handler.
//
//
//
// void nextImgs();
//______________________________________________________________________________
// Use:
//   Changes the current image to the next image in lb_files
//   Usually calleb by the keyPressEvent-Handler.
//
//
//
// void removeImg();
//______________________________________________________________________________
// Use:
//   Removes the current image out of the IDL and the ListBox
//
//
//
// void clearAllRects();
//______________________________________________________________________________
// Use:
//   Removes all the rectangle in the whole IDL.
//   Usually called by the according button
//
//
//
// void openIDL();
//______________________________________________________________________________
// Use:
//   Shows up an Save File Dialog to Open or Create an IDL-File.
//   It sets IDL_Dir and IDL_File if possible
//   Then it tries to open or create the IDL_File and does all necessary update
//   Usually called by the according button
//
//
//
// void saveIDL();
//______________________________________________________________________________
// Use:
//   Save the IDL_Dir+IDL_File
//   Usually called by the according button
//
//
//
// void saveCopy();
//______________________________________________________________________________
// Use:
//   Shows up an Save File Dialog to Save the current IDL whereever you want.
//   Usually called by the according button
//
//
//
// void addIDsFrom();
//______________________________________________________________________________
// Use:
//   Shows up an Open File Dialog to open another IDL-File.
//   The file opened has to be in the same directory like the current IDL File
//   or in one of its subdirectories.
//   Then a reading of the IDL-File is tried and (if succesful) the IDLs are merged 
//   like in the according function of ImgDescrList. The name of the subdirectory
//   (if there's any) is the prefix argument.
//   All necessary updates are done.
//   Usually called by the according button
//
//
//
// void openBackIDL();
//______________________________________________________________________________
// Use:
//   Shows up an Open File Dialog to open the Background IDL-File.
//   All necessary updates are done.
//   Usually called by the according button
//
//
//
// void closeBackIDL();
//______________________________________________________________________________
// Use:
//   Get rid of the Background IDL.
//   All necessary updates are done.
//   Usually called by the according button
//
//
//
// void setNormColor(); void setSelColor(); void setBackColor();
//______________________________________________________________________________
// Use:
//   Sets the color of the (normal, selected, background) rectangles in
//   the imgDisp.
//   All necessary updates are done.
//   Usually called by the according button
//
//
//
// void keyPressEvent(QKeyEvent* );
//______________________________________________________________________________
// Use: 
//   Handles the Key Events for "delete" and the four arrow keys.
//
////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
//
// class IDL_Edit's private functions and members
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// void loadImage(QString fname);
//______________________________________________________________________________
// Use:
//   Loads the current image and displays it.
//
//
//
// void updateListBox();
//______________________________________________________________________________
// Use:
//   Recreates the entries of the listbox according to the IDL.
//   It is usually called by openIDL() or addIDsFrom()
//
//
//
// void updateCaption();
//______________________________________________________________________________
// Use:
//   updates the main window caption
//   It is usually called by openIDL()
//
//
//
// member variables
//______________________________________________________________________________
//
//	QString IDL_File;           Name of the IDL File 
//	QString IDL_Dir;            Directory of the IDL File
//	
//  ImgDescrList myIDL :        the IDL (initially empty)
//	ImgDescr* curImgDescr;      the ImgDescr of the currently selected image
//
//  ImgDescrList myBackIDL:     the Background IDL (initially empty)
//	ImgDescr* curBackImgDescr;  myBackIDL's ImgDescr of the cur. selected image
//	
//	QImage  curImg;    
//	IDL_EditImgDisp* imgDisp;	the Image Display used.
//	IDL_EditListBox *lb_files;  the ListBox used
//
//	QLabel* lb_info;			the Info Label
//
//	QPushButton* bt_saveIDL;	Some buttons initially set to be not enabled
//	QPushButton* bt_select;
//	QPushButton* bt_addIDsFrom;
////////////////////////////////////////////////////////////////////////////////

#endif
