
#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>

#include <q3vbox.h>
#include <qapplication.h>
#include <qslider.h>
#include <qlcdnumber.h>
#include <qfont.h>
#include <qlayout.h>
#include <qsizepolicy.h>
#include <qlabel.h>
#include <q3filedialog.h>
#include <qcolordialog.h>
#include <qtabwidget.h>
#include <qpainter.h>

#include "qtcoordlabel.hh"
//Added by qt3to4:
#include <QKeyEvent>
#include <QPixmap>
#include <Q3Frame>
#include "idl_edit.hh"


const string INFOTEXT = "IDL_Viewer V1.0 \n" 
  "05.02.04"
  "by Dirk Zimmer \n\n\n"
  "Key Commands: \n\n"
  "Up/Down: select Image\n\n "
  "Left/Right: select Rectangle\n\n"
  "Del:  delete Rectangle \n";



IDL_EditListBox::IDL_EditListBox( QWidget *parent, const char *name)
  :Q3ListBox( parent, name)
{
}


void IDL_EditListBox::keyPressEvent(QKeyEvent* e)
{
  switch(e->key()) {
    case Qt::Key_Delete: 
      e->ignore(); 
      break;

    case Qt::Key_Left: 
      e->ignore(); 
      break;

    case Qt::Key_Right: 
      e->ignore(); 
      break;

    default: 
      Q3ListBox::keyPressEvent(e);
  }
}


IDL_Edit::IDL_Edit( QWidget *parent, const char *name )
  : Q3HBox( parent, name )
{
  threshLow=-10000;
  threshHigh=10000;

  curImgDescr = NULL;
  curBackImgDescr = NULL;
	
  setSpacing(20);
  setFocusPolicy(Qt::StrongFocus);
	
  Q3VBox* menuBox = new Q3VBox(this,"menubox");
  menuBox->setMaximumWidth(256);
	
  /************************/
  /* Make a 'Quit' button */
  /************************/
  QPushButton *bt_quit = new QPushButton( "Quit", menuBox, "bt_quit" );
  bt_quit->setFont( QFont( "Times", 18, QFont::Bold ) );
  connect( bt_quit, SIGNAL(clicked()), qApp, SLOT(quit()) );

  /****************************/
  /* Make a button 'Open IDL' */
  /****************************/
	QPushButton *bt_openIDL = new QPushButton( "Open IDL-File...", 
                                             menuBox, "bt_openIDL" );
  connect( bt_openIDL, SIGNAL(clicked()), this, SLOT(openIDL()) );

  /****************************/
  /* Make a button 'Save IDL' */
  /****************************/
  bt_saveIDL = new QPushButton( "Save IDL-File", menuBox, "bt_saveIDL" );
  connect( bt_saveIDL, SIGNAL(clicked()), this, SLOT(saveIDL()) );
  

  /****************************************/
  /* Make a tab widget for the image list */
  /****************************************/
	QTabWidget* tab_main = new QTabWidget(menuBox);
  tab_main->setFocusPolicy(Qt::NoFocus);

  
  /*===============*/
  /* Tab 'Images': */
  /*===============*/
  Q3VBox* tabBox1 = new Q3VBox(tab_main);
  lb_files  = new IDL_EditListBox( tabBox1, "lb_files" );
  connect( lb_files, SIGNAL(currentChanged(Q3ListBoxItem*)), 
           this, SLOT(updateImage(Q3ListBoxItem*)) );

  tab_main->addTab(tabBox1,"Images");

  /*-------------------------------*/
  /* Make a button 'Select Images' */
  /*-------------------------------*/
  bt_selectImgs = new QPushButton( "Select Images...", tabBox1,
                                   "bt_selectImgs" );
  connect( bt_selectImgs, SIGNAL(clicked()), this, SLOT(selectImgs()) );

  /*-------------------------------*/
  /* Make a button 'Remove Images' */
  /*-------------------------------*/
  QPushButton *bt_remove = new QPushButton( "remove Image", tabBox1, 
                                            "bt_remove" );
  connect( bt_remove, SIGNAL(clicked()), this, SLOT(removeImg() ) );
  
  Q3Frame* divider1 = new Q3Frame(tabBox1);
  divider1->setFrameStyle(Q3Frame::HLine);
  divider1->setFrameShadow(Q3Frame::Sunken);
  divider1->setMinimumHeight(16);

  /*-------------------------------------*/
  /* Make a button 'Open Background IDL' */
  /*-------------------------------------*/
  QPushButton *bt_openBackIDL = new QPushButton( "Open Backgrnd-IDL...", 
                                                 tabBox1, "bt_openBackIDL" );
  connect( bt_openBackIDL, SIGNAL(clicked()), this, SLOT(openBackIDL()) );

  /*--------------------------------------*/
  /* Make a button 'Close Background IDL' */
  /*--------------------------------------*/
  QPushButton *bt_closeBackIDL = new QPushButton( "Close Backgrnd-IDL", 
                                                  tabBox1, "bt_closeBackIDL" );
  connect( bt_closeBackIDL, SIGNAL(clicked()), this, SLOT(closeBackIDL()) );
  
  
  /*================*/
  /* Tab 'Utility': */
  /*================*/
  Q3VBox* tabBox2 = new Q3VBox(tab_main);
  tabBox2->setFocusPolicy(Qt::NoFocus);
  tabBox2->setSpacing(10);
  tabBox2->setMargin(20);

  tab_main->addTab(tabBox2,"Utility");
  
  /*---------------------------*/
  /* Make a button 'Save Copy' */
  /*---------------------------*/
  QPushButton *bt_saveCopy = new QPushButton( "Save copy as...", tabBox2, 
                                              "bt_saveCopy" );
  connect( bt_saveCopy, SIGNAL(clicked()), this, SLOT(saveCopy()) );

  /*------------------------------*/
  /* Make a button 'Add IDs from' */
  /*------------------------------*/
  bt_addIDsFrom = new QPushButton( "add IDs from...", tabBox2,
                                   "bt_addIDsFrom" );
  connect( bt_addIDsFrom, SIGNAL(clicked()), this, SLOT(addIDsFrom()) );

  /*---------------------------------*/
  /* Make a button 'Clear All Rects' */
  /*---------------------------------*/
  QPushButton *bt_clearAllRects = new QPushButton( "Clear all  Rectangles",
                                                   tabBox2, "bt_car" );
  connect( bt_clearAllRects, SIGNAL(clicked()), this, SLOT(clearAllRects()) );

  Q3Frame* divider3 = new Q3Frame(tabBox2);
  divider3->setFrameStyle(Q3Frame::HLine);
  divider3->setFrameShadow(Q3Frame::Sunken);
  divider3->setSizePolicy(QSizePolicy(QSizePolicy::Ignored,
                                      QSizePolicy::Ignored,2,0));
  
  /*----------------------------*/
  /* Make a button 'Rect Color' */
  /*----------------------------*/
  QPushButton *bt_normColor = new QPushButton( "Rectangle Color...", tabBox2, 
                                               "bt_openBackIDL" );
  connect( bt_normColor, SIGNAL(clicked()), this, SLOT(setNormColor()) );

  /*----------------------------*/
  /* Make a button 'Sel. Color' */
  /*----------------------------*/
  QPushButton *bt_selColor = new QPushButton( "Selected Rect. Color", tabBox2,
                                              "bt_openBackIDL" );
  connect( bt_selColor, SIGNAL(clicked()), this, SLOT(setSelColor()) );

  /*-------------------------------*/
  /* Make a button 'Backgr. Color' */
  /*-------------------------------*/
  QPushButton *bt_backColor = new QPushButton("Background Rect. Color", 
                                              tabBox2, "bt_openBackIDL" );
  connect( bt_backColor, SIGNAL(clicked()), this, SLOT(setBackColor()) );
  
  
  /*=============*/
  /* Tab 'Info': */
  /*=============*/
  QLabel* lb_help = new QLabel(tab_main);
  lb_help->setText(INFOTEXT.c_str());
  lb_help->setFrameStyle(QFrame::Panel | QFrame::Sunken); 
  lb_help->setAlignment(Qt::AlignTop | Qt::AlignCenter);

  tab_main->addTab(lb_help,"Info");
  
  
  /*************************/
  /* Make an image display */
  /*************************/
  Q3VBox *imgBox  = new Q3VBox( this, "imagebox");
  //Q3VBox *imgBox1  = new Q3VBox( this, "imagebox1");
  /*------------------*/
  /* Add score filter */
  /*------------------*/
  Q3HBox* filterBox = new Q3HBox(imgBox, "filterBox");
  QLabel* tlScoreFilterLow = new QLabel(" Score Threshold (low): ", filterBox);
  leScoreFilterLow = new QLineEdit( "-10000.0", filterBox, 
                                    "score_filter_input1" );
  QLabel* tlScoreFilterHigh = new QLabel(" Score Threshold (high): ", 
                                         filterBox);
  leScoreFilterHigh = new QLineEdit( "10000.0", filterBox, 
                                     "score_filter_input2" );
  connect(leScoreFilterHigh, SIGNAL(textChanged(const QString&)), 
          this, SLOT(thresholdsChanged()));
  connect(leScoreFilterLow, SIGNAL(textChanged(const QString&)), 
          this, SLOT(thresholdsChanged()));

  /*------------------*/
  /* Add save buttons */
  /*------------------*/
  Q3HBox* saveBox = new Q3HBox(imgBox, "saveBox");
  QPushButton* saveDirButton = new QPushButton("Set Save Dir", saveBox, 
                                               "saveDirButton");
  QPushButton* saveCurrent   = new QPushButton("Save Current", saveBox, 
                                               "saveCurrent");
  QPushButton* saveAll       = new QPushButton("Save All", saveBox, 
                                               "saveAll");
  QPushButton* saveCorrect   = new QPushButton("Save Correct", saveBox, 
                                               "saveCorrect");
  QPushButton* saveMissed    = new QPushButton("Save Missed", saveBox, 
                                               "saveMissed");
  connect(saveDirButton, SIGNAL(clicked()), this, SLOT(setSaveDir()));
  connect(saveCurrent, SIGNAL(clicked()), this, SLOT(saveCurrentImage()));
  connect(saveAll, SIGNAL(clicked()), this, SLOT(saveAllImages()));
  connect(saveCorrect, SIGNAL(clicked()), this, SLOT(saveCorrect()));
  connect(saveMissed, SIGNAL(clicked()), this, SLOT(saveMissed()));

  Q3Frame* divider4 = new Q3Frame(imgBox);
  divider4->setFrameStyle(Q3Frame::HLine);
  divider4->setFrameShadow(Q3Frame::Sunken);
  divider4->setMinimumHeight(16);

  /*------------------*/
  /* Add an info line */
  /*------------------*/
  lb_info = new QLabel( "", imgBox, "lb_info" );
  lb_info->setAlignment(Qt::AlignBottom | Qt::AlignCenter);

  /*------------------*/
  /* Add a CoordLabel */
  /*------------------*/
  QtCoordLabel *coordPos = new QtCoordLabel( "Pos: ", imgBox, "coord_pos" );
  coordPos->setAlignment(Qt::AlignBottom | Qt::AlignCenter);
  Q3Frame* divider40 = new Q3Frame(imgBox);
  divider40->setFrameStyle(Q3Frame::HLine);
  divider40->setFrameShadow(Q3Frame::Sunken);
  divider40->setMinimumHeight(16);

  /*----------------------*/
  /* Add an image display */
  /*----------------------*/
  imgDisp = new IDL_EditImgDisp(imgBox, "current image");

  imgDisp->setMouseTracking( TRUE );
  connect( imgDisp, SIGNAL(mouseMovedOnPixel(int, int, QRgb)),
           coordPos, SLOT(setCoordValue( int, int, QRgb )) );
  connect( imgDisp, SIGNAL(changed() ),this, SLOT(updateInfoLabel()) );

  Q3Frame* divider2 = new Q3Frame(imgBox);
  divider2->setFrameStyle(Q3Frame::HLine);
  divider2->setFrameShadow(Q3Frame::Sunken);
  divider2->setMinimumHeight(16);

  /*========================*/
  /* Add navigation buttons */
  /*========================*/
  Q3HBox* navBox = new Q3HBox(imgBox,"navbox");

  /*--------------------------*/
  /* Make a button 'Previous' */
  /*--------------------------*/
  QPushButton *bt_prev = new QPushButton( "previous", navBox, "bt_prev" );
  bt_prev->setMaximumWidth(200);
  connect( bt_prev, SIGNAL(clicked()), this, SLOT(prevImg()) );

  /*------------------------------------------*/
  /* Make buttons 'Select Rect'/'Delete Rect' */
  /*------------------------------------------*/
  Q3VBox* navRectBox = new Q3VBox(navBox,"navbox2");

  navRectBox->setMaximumWidth(150);
  QPushButton *bt_selectRect = new QPushButton( "select rectangles", 
                                                navRectBox, "bt_selectRect" );
  connect( bt_selectRect, SIGNAL(clicked()), imgDisp, SLOT(selectRect() ) );

  QPushButton *bt_delete = new QPushButton( "delete rectangle", navRectBox, 
                                            "bt_delete" );
  connect( bt_delete, SIGNAL(clicked()), imgDisp, SLOT(deleteRect() ) );

  /*----------------------*/
  /* Make a button 'Next' */
  /*----------------------*/
  QPushButton *bt_next = new QPushButton( "next", navBox, "bt_next" );
  bt_next->setMaximumWidth(200);
  connect( bt_next, SIGNAL(clicked()), this, SLOT(nextImg()) );
  

  /***************************/
  /* Set the widget geometry */
  /***************************/
  setGeometry(0,0,800,600);

	
  /****************************/
  /* Initialize the interface */
  /****************************/
	IDL_File = "";
	bt_saveIDL->setEnabled(false);
	bt_selectImgs->setEnabled(false);
	bt_addIDsFrom->setEnabled(false);
	updateCaption();
	updateInfoLabel();
}


/***********************************************************/
/*                          Slots                          */
/***********************************************************/

void IDL_Edit::thresholdsChanged()
{
  threshLow = leScoreFilterLow->text().toDouble();
  threshHigh = leScoreFilterHigh->text().toDouble();
  imgDisp->setThresholdLow(threshLow);
  imgDisp->setThresholdHigh(threshHigh);
  cout << "Setting Thresholds: " << leScoreFilterLow->text().toDouble() << " " << leScoreFilterHigh->text().toDouble() << endl;
}

void IDL_Edit::updateImage( Q3ListBoxItem* item )
{   
	if (item==NULL) {
    imgDisp->setCurImgDescr(NULL); 
	  cerr << "Error in IDL_Edit::updateImage: " 
         << "No item selected!" << endl;
	  return;
	}

  /* prepare the image name */
	QString fullname = item->text();
  
  if( !fullname.isEmpty() && fullname[0] != '/' )
    fullname.prepend(IDL_Dir);

  /* load the image */
  cout << "  Loading image '" << fullname.toStdString() << "'..." << endl;
  loadImage( fullname );
  cout << "  done." << endl;

  /* retrieve the annotations */
	curImgDescr = myIDL.findName(item->text().ascii());
	if (curImgDescr == NULL) 
    cout << "    Couldn't find image decription!" << endl;

  /* retrieve the groundtruth annotations */
	curBackImgDescr = myBackIDL.findName( item->text().ascii() );

  /* display the annotations */
  imgDisp->setCurImgDescr( curImgDescr, curBackImgDescr ); 

  /* select the first rectangle */
  imgDisp->selectRect(true); 
  imgDisp->update();
	return;
}


void IDL_Edit::nextImg( )
{
	lb_files->setCurrentItem( lb_files->currentItem() + 1 );
}

void IDL_Edit::prevImg( )
{
	lb_files->setCurrentItem( lb_files->currentItem() - 1 );
}

void IDL_Edit::removeImg( )
{
  int cur = lb_files->currentItem();
  if (cur > -1) {
    myIDL.findAndRemove( (lb_files->currentText()).ascii() );
	  
	  lb_files->removeItem(cur); 
	}
}




void IDL_Edit::selectImgs( )
{
  /* ask for an image list */
	QStringList sl 
    = Q3FileDialog::getOpenFileNames("Images (*.png *.xpm *.jpg)",
                                    IDL_Dir, 
                                    this,
                                    "file dialog",
                                    "Select the images to analyze" );

  /* process the list of image names */
  QStringList::Iterator it = sl.begin();
	while (it != sl.end()) {
    /* remove the relative paths */
	  if ( (*it).find(IDL_Dir) == 0) {
	    (*it).remove(0,IDL_Dir.length() );
	    //(edi) (*it).remove(0,IDL_Dir.length() );
      myIDL.findOrCreate( (*it).ascii() );
	  }
	  ++it;
	}			 
	
  /* display the image list */
  updateListBox();
}


void IDL_Edit::openIDL()
{
  /* ask for a file name */
	IDL_File 
    = Q3FileDialog::getSaveFileName(IDL_Dir, 
                                   "IDLs (*.idl);;all Files (*)",
                                   this,
                                   "file dialog",
                                   "Open or create an IDL file" );
  
  if( !IDL_File.isEmpty() ) {  
    
    /* extract the path and file name */
    IDL_Dir = IDL_File.left( IDL_File.findRev('/') + 1 );
	  IDL_File.remove( 0, IDL_Dir.length() );
    
    /* append an extension 'idl' */
	  if (IDL_File.find('.') == -1) 
      IDL_File.append(".idl");

	  if( QDir::setCurrent(IDL_Dir) ) {
      
	    ifstream f1(IDL_File);
      if ( f1 ) {
        /* if the file exists => load it */
        myIDL.load(&f1);
        f1.close();

      } else {
        /* if the file doesn't exist yet => create it */
        ofstream f2(IDL_File);
        if ( f2 ) {
          myIDL.clear();
          myIDL.save(f2);
          f2.close();			
        }
      }
      // cout << myIDL.size() << "\n";

      /* update the interface */
    	curImgDescr = NULL;
      updateListBox();	  
      updateCaption();
      bt_saveIDL->setEnabled(true);
 	    bt_selectImgs->setEnabled(true);
 	    bt_addIDsFrom->setEnabled(true);
      
    } else {
	    IDL_Dir = QDir::currentDirPath();
	  }
	}
}



void IDL_Edit::addIDsFrom()
{
  /* ask for a file name */
	QString thisFile 
    = Q3FileDialog::getOpenFileName(IDL_Dir, 
                                   "IDLs (*.idl);;all Files (*)",	
                                   this,
                                   "file dialog",
                                   "Open an Image-Description-List File" );
  
  if( !IDL_File.isEmpty() ) {
    /* extract the path */
    QString thisDir = thisFile.left( thisFile.findRev('/') + 1 );
    
    if( thisDir.find(IDL_Dir) == 0 ) {
      /* remove the relative path */
      thisDir.remove( 0, IDL_Dir.length() );

      /* load the new idl file */
      ifstream f1(thisFile);
      if ( f1 ) {
        ImgDescrList* newIDL = new ImgDescrList(&f1);
        f1.close();
        
        /* and add the new id's */
        myIDL.addIDs( (*newIDL), thisDir.ascii() );
        updateListBox();	  
      } 
    }
  }
}


void IDL_Edit::openBackIDL()
{
  /* ask for a file name */
	QString fname 
    = Q3FileDialog::getOpenFileName(IDL_Dir, 
                                   "IDLs (*.idl);;all Files (*)",
                                   this,
                                   "file dialog",
                                   "Open an Image-Description-List File" );

  if( !IDL_File.isEmpty() ) {								   
    curBackImgDescr = NULL;

    /* open the ground truth idl file */
    ifstream f1(fname);
		if ( f1 ) {
		  myBackIDL.load(&f1);
		  f1.close();
    } 
  }

  if (curImgDescr == NULL) 
    return;
  
  /* display the ground truth annotation for the current image */
  curBackImgDescr = myBackIDL.findName( curImgDescr->name );
  imgDisp->setCurImgDescr( curImgDescr, curBackImgDescr ); 
}


void IDL_Edit::closeBackIDL()
{
  myBackIDL.clear();
  curBackImgDescr = NULL;
  imgDisp->setCurImgDescr(curImgDescr,curBackImgDescr); 
}


void IDL_Edit::saveIDL()
{
  if (QDir::setCurrent(IDL_Dir)) {

    /* save the idl file */
    ofstream f(IDL_File);
   	if ( f ) {
  	  myIDL.save(f);
      f.close();  
    } 
  } 
}


void IDL_Edit::saveCopy()
{
  /* ask for a file name */
  QString CopyFile 
    = Q3FileDialog::getSaveFileName(IDL_Dir, 
                                   "IDLs (*.idl);;all Files (*)",	
                                   this,
                                   "file dialog",
                                   "name of copy" );

  if( !CopyFile.isEmpty() ) {		   
    /* add an extension 'idl' */
    if( CopyFile.findRev('.') <= CopyFile.findRev('/') ) 
      CopyFile.append(".idl");

    /* save the idl file */
  	ofstream f(CopyFile);
   	if ( f ) {
  	  myIDL.save(f);
      f.close();  
    } 
  } 
}


void IDL_Edit::loadImage(const QString& fname)
  /* load an image */
{
  curImg.load( fname );
  imgDisp->loadImage( curImg );
	return;
}


void IDL_Edit::saveCurrentImage()
{
  QString name = saveDir+lb_files->currentText();
  cout << "Saving: "+name.toStdString() << endl;
  
  QPixmap tmp(curImg);
  QPainter pImg(&tmp);
  pImg.setPen( Qt::red );
  for (unsigned int i=0; i<curImgDescr->RectList.size(); i++) {
    Rect& r=curImgDescr->RectList[i];
    pImg.drawRect( r.x1, r.y1, r.x2-r.x1, r.y2-r.y1 );
  }
  QString tmpName = name.left(name.length()-4)+"-rects.png";
  tmp.save(tmpName, "PNG");
  curImg.save(name, "PNG");
}


void IDL_Edit::saveAllImages()
{
  setSaveDir();

  for(int j=0; j<lb_files->numRows(); j++) {
    bool bRecoImg = false;

    QString name = lb_files->text(j);
    cout << "  Processing image '" << name.toStdString() << "'..." << endl;
    QString qsFileName = name;
    if( name.at(0) != '/' )
      qsFileName = IDL_Dir + name;

    QImage img(qsFileName);
    QPixmap tmp(img);
    
    QPainter pImg(&tmp);
    pImg.setPen( Qt::red );
    ImgDescr* idHypo  = myIDL.findName(name.latin1());
    ImgDescr* idAnnot = myBackIDL.findName(name.latin1());
    for (int i=0; i<(int)idHypo->RectList.size(); i++) {
      Rect& r1 = idHypo->RectList[i]; 
      bool bRecoHypo = false;

      if (r1.score>=threshLow && r1.score<threshHigh) {
        for (int k=0; k<(int)idAnnot->RectList.size(); k++) {
          Rect& r2 = idAnnot->RectList[k];
          
          if( ImgDescrList::isMatching( r2, r1,
                                        DIST,COVER,OVERLAP,
                                        ASPECTRATIO, eFixObjDim ) ) {
            bRecoImg  = true;
            bRecoHypo = true;
          }
        }
      
        /* draw the rectangle for this hypothesis */
        if( bRecoHypo )
          pImg.setPen( Qt::green );
        else
          pImg.setPen( Qt::red );
        pImg.drawRect( r1.x1, r1.y1, r1.x2-r1.x1, r1.y2-r1.y1 );
      }
    }
 
    /* save the result image */
    QString qsImgName = qsFileName;
    int pos = qsImgName.findRev('/');
    if( pos > 0 )
      qsImgName = qsImgName.right( qsImgName.length()-pos-1 );
    qsImgName.prepend(saveDir);
    cout << "  => Saving file '" << qsImgName.latin1() << "'..." << endl;
    QString tmpName = qsImgName.left(qsImgName.length()-4)+"-rects.png";
    tmp.save(tmpName, "PNG");
  }  
}


void IDL_Edit::saveCorrect()
{
  setSaveDir();
  
  for(int j=0; j<lb_files->numRows(); j++) {
    bool bRecoImg = false;

    QString name = lb_files->text(j);
    cout << "  Processing image '" << name.toStdString() << "'..." << endl;
    QString qsFileName = name;
    if( name.at(0) != '/' )
      qsFileName = IDL_Dir + name;

    QImage img(qsFileName);
    QPixmap tmp(img);
    
    QPainter pImg(&tmp);
    pImg.setPen( Qt::red );
    ImgDescr* idHypo  = myIDL.findName(name.latin1());
    ImgDescr* idAnnot = myBackIDL.findName(name.latin1());
    for (int i=0; i<(int)idHypo->RectList.size(); i++) {
      Rect& r1 = idHypo->RectList[i]; 
      bool bRecoHypo = false;

      if (r1.score>=threshLow && r1.score<threshHigh) {
        for (int k=0; k<(int)idAnnot->RectList.size(); k++) {
          Rect& r2 = idAnnot->RectList[k];

        if( ImgDescrList::isMatching( r2, r1,
                                      DIST,COVER,OVERLAP,
                                      ASPECTRATIO, eFixObjDim ) ) {
            bRecoImg  = true;
            bRecoHypo = true;
          }
        }

        /* draw the rectangle for this hypothesis */
        if( bRecoHypo )
          pImg.setPen( Qt::green );
        else
          pImg.setPen( Qt::red );
        pImg.drawRect( r1.x1, r1.y1, r1.x2-r1.x1, r1.y2-r1.y1 );        
      }
    }
    
    /* save the result image */
    if (bRecoImg) {
    cout << "  recognized" << endl;
      QString qsImgName = qsFileName;
      int pos = qsImgName.findRev('/');
      if( pos > 0 )
        qsImgName = qsImgName.right( qsImgName.length()-pos-1 );
      qsImgName.prepend(saveDir);
      cout << "  => Saving file '" << qsImgName.latin1() << "'..." << endl;
      QString tmpName = qsImgName.left(qsImgName.length()-4)+"-rects.png";
      tmp.save(tmpName, "PNG");
    }
  }  
}


void IDL_Edit::saveMissed()
{
  setSaveDir();
  
  for(int j=0; j<lb_files->numRows(); j++) {
    bool bRecoImg = false;

    QString name = lb_files->text(j);
    cout << "  Processing image '" << name.toStdString() << "'..." << endl;
    QString qsFileName = name;
    if( name.at(0) != '/' )
      qsFileName = IDL_Dir + name;

    QImage img(qsFileName);
    QPixmap tmp(img);
    
    QPainter pImg(&tmp);
    pImg.setPen( Qt::red );
    ImgDescr* idHypo  = myIDL.findName(name.latin1());
    ImgDescr* idAnnot = myBackIDL.findName(name.latin1());
    for (int i=0; i<(int)idHypo->RectList.size(); i++) {
      Rect& r1 = idHypo->RectList[i]; 
      bool bRecoHypo = false;

      if (r1.score>=threshLow && r1.score<threshHigh) {
        for (int k=0; k<(int)idAnnot->RectList.size(); k++) {
          Rect& r2 = idAnnot->RectList[k];

        if( ImgDescrList::isMatching( r2, r1,
                                      DIST,COVER,OVERLAP,
                                      ASPECTRATIO, eFixObjDim ) ) {
            bRecoImg  = true;
            bRecoHypo = true;
          }
        }

        /* draw the rectangle for this hypothesis */
        if( bRecoHypo )
          pImg.setPen( Qt::green );
        else
          pImg.setPen( Qt::red );
        pImg.drawRect( r1.x1, r1.y1, r1.x2-r1.x1, r1.y2-r1.y1 );
      }
    }
    
    /* save the result image */
    if (!bRecoImg) {
    cout << "  recognized" << endl;
      QString qsImgName = qsFileName;
      int pos = qsImgName.findRev('/');
      if( pos > 0 )
        qsImgName = qsImgName.right( qsImgName.length()-pos-1 );
      qsImgName.prepend(saveDir);
      cout << "  => Saving file '" << qsImgName.latin1() << "'..." << endl;
      QString tmpName = qsImgName.left(qsImgName.length()-4)+"-rects.png";
      tmp.save(tmpName, "PNG");
    }
  }  
}


void IDL_Edit::setSaveDir()
{
  saveDir = Q3FileDialog::getExistingDirectory(".", this, 
                                              "get existing directory",
                                              "Choose a directory",TRUE );
}


void IDL_Edit::updateListBox()
{
  /* construct a string list with all image names */
  QStringList sl;
  for( int i=0; i<(int)myIDL.size(); i++ ) {
    sl.append( myIDL[i]->name.c_str() );
  }

  /* display the names in the list box */
  lb_files->clear();
  lb_files->insertStringList( sl );
  if (lb_files->count() > 0) {
    lb_files->setCurrentItem(0);

  } else {
    imgDisp->setCurImgDescr(NULL); 		
  }
}


void IDL_Edit::updateCaption()
{
  setCaption( "IDL Editor   File: " + IDL_File + " Diriectory: " +IDL_Dir );
}


void IDL_Edit::updateInfoLabel()
{
  /* check if the info text should be empty */
  int curRect = imgDisp->getCurRect();
	if( (curImgDescr == NULL) || (curBackImgDescr == NULL) || 
	    (curRect < 0) || (curBackImgDescr->RectList.size() == 0) ) {
    lb_info->setText("");
	  return;
	}

  /* compute the best match for the current rectangle */
	Rect R      = curImgDescr->RectList[curRect];
  Rect rAnnot = curBackImgDescr->RectList[0];
	double minDist;
  if( !bFixObjDim )
    minDist = ImgDescrList::compRelDist( rAnnot, R );
  else
    minDist = ImgDescrList::compRelDist( rAnnot, R, ASPECTRATIO, eFixObjDim );
	double bestCover   = ImgDescrList::compCover( rAnnot, R);
	double bestOverlap = ImgDescrList::compCover( R, rAnnot);

	double d;
  for( int i=1; i<(int)curBackImgDescr->RectList.size(); i++ ) {
    
    if( !bFixObjDim )
      d = ImgDescrList::compRelDist( curBackImgDescr->RectList[i], R );
    else
      d = ImgDescrList::compRelDist( curBackImgDescr->RectList[i], R,
                                     ASPECTRATIO, eFixObjDim );

	  if (d < minDist) {
      minDist = d;
      bestCover   = ImgDescrList::compCover( curBackImgDescr->RectList[i], R );
	    bestOverlap = ImgDescrList::compCover( R, curBackImgDescr->RectList[i] );
	  }
	}    
  
  /* set the info text */
	QString Text;
	QString Val;
	Text = "Dist: ";	       Text += Val.setNum(minDist,'f',3);
	Text += " | Cover: ";	   Text += Val.setNum(bestCover*100.0,'f',1);
	Text += "% | Overlap: "; Text += Val.setNum(bestOverlap*100.0,'f',1);
	Text += "% | Score: ";   Text += Val.setNum(R.score,'g',3);
  lb_info->setText(Text);
	lb_info->repaint();
}


void IDL_Edit::keyPressEvent(QKeyEvent* e)
{
  switch(e->key()) {
  case Qt::Key_Delete: 
    imgDisp->deleteRect(); 
    break;

  case Qt::Key_Up: 
    prevImg(); 
    break;

  case Qt::Key_Down: 
    nextImg(); 
    break;

  case Qt::Key_Left: 
    imgDisp->selectRect(false); 
    break;

  case Qt::Key_Right: 
    imgDisp->selectRect(true); 
    break;
    
  default: 
    QWidget::keyPressEvent(e);
  } 
}


void IDL_Edit::clearAllRects()
{
  for( int i=0; i<(int)myIDL.size(); i++ )
    myIDL[i]->RectList.clear();

  imgDisp->setCurImgDescr(curImgDescr,curBackImgDescr,-1); 
}


void IDL_Edit::setNormColor()
{
  imgDisp->normColor = QColorDialog::getColor( imgDisp->normColor, this ); 
  imgDisp->repaint();
}


void IDL_Edit::setSelColor()
{
  imgDisp->selColor = QColorDialog::getColor( imgDisp->selColor, this ); 
  imgDisp->repaint();
}


void IDL_Edit::setBackColor()
{
  imgDisp->backColor = QColorDialog::getColor( imgDisp->backColor, this ); 
  imgDisp->repaint();
}
