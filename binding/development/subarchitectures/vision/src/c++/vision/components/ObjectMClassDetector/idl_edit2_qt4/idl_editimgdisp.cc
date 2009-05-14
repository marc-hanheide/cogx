#include <math.h>
#include <qcursor.h>
//Added by qt3to4:
#include <QPaintEvent>
#include "imgdescrlist.hh"
#include "idl_editimgdisp.hh"

const int minDragArea = 100;

IDL_EditImgDisp::IDL_EditImgDisp( QWidget *parent, const char *name)
	: QtResizeImg(parent,name)
{
	connect(this,SIGNAL(leftDragOnPixel(int, int)),
          this,SLOT(onDragLeft(int,int) ));
	connect(this,SIGNAL(leftClickOnPixel(int, int)),
          this,SLOT(onClickLeft(int,int) ));

	connect(this,SIGNAL(rightDragOnPixel(int, int)),
          this,SLOT(onDragRight(int,int) ));
	connect(this,SIGNAL(rightClickOnPixel(int, int)),
          this,SLOT(onClickRight(int,int) ));

	setCursor(QCursor(Qt::CrossCursor));

	curRect = -1;
	curImgDescr=NULL;
	curBackImgDescr=NULL;
	dragRect.score = 0.0;

	threshLow=-10000;
	threshHigh=10000;
	normColor.setNamedColor("red");
	selColor.setNamedColor("green");
	backColor.setNamedColor("blue");
}


void IDL_EditImgDisp::setCurImgDescr(ImgDescr* cID,ImgDescr* bID,int cRect)
{
	curImgDescr = cID;
	curBackImgDescr = bID;
	setCurRect(cRect);
}


void IDL_EditImgDisp::setCurRect(int cRect)
{
	curRect = -1;
	dragRect.x1=0;
	dragRect.y1=0;
	dragRect.x2=0;
	dragRect.y2=0;

	if (curImgDescr == NULL) 
    return;

	if (cRect >= (signed)curImgDescr->RectList.size()) 
    return;

	if (cRect < -1) 
    return;

	curRect = cRect;
	QRect eR(0,0,width(),height());
	QPaintEvent* e = new QPaintEvent(eR);//,true);
	paintEvent(e);
 	emit changed();
}


int IDL_EditImgDisp::getCurRect()
{
	return curRect;
}


void IDL_EditImgDisp::setThresholdLow(double low)
{
  threshLow = low;
  repaint();
}

void IDL_EditImgDisp::setThresholdHigh(double high)
{
  threshHigh = high;
  repaint();
}

void IDL_EditImgDisp::selectRect(bool forward)
{
  int idx, count, size;
  double score;

  if (curImgDescr == NULL)  
    return;

  size = curImgDescr->RectList.size();
  if (size == 0)  
    return;

  count=0;
  int cR = curRect + curImgDescr->RectList.size();
  do {
    count++;
    if( forward ) 
      cR++; 
    else 
      cR--;
    idx = cR % size;
    score = curImgDescr->RectList[idx].score;
  } while( (score<threshLow || score>threshHigh) && count<size);
  
  setCurRect(idx);

}


void IDL_EditImgDisp::deleteRect( )
{
	if (curImgDescr == NULL)  
    return;

  vector<Rect> &v = curImgDescr->RectList;
	int s = v.size();
	if (curRect > s-1)  
    return;
	if (curRect < 0)  
    return;
	
  vector<Rect>::iterator it = v.begin();
  int i = 0;
	while (it != v.end()) {
	  if (i == curRect)  {
      it = v.erase(it);
      setCurRect(min(curRect,s-2));
  		return;
	  } 
	  it++;
	  i++;
	}			 
}


void IDL_EditImgDisp::paintEvent( QPaintEvent * event)
{
  QtResizeImg::paintEvent(event);
  
  Rect R;
	
  if (curBackImgDescr != NULL) {
    for( int i=0; i<(int)curBackImgDescr->RectList.size(); i++ ) {
      R = curBackImgDescr->RectList[i];
      ImgDescrList::sortCoords(R);
      drawRect(R.x1,R.y1,max(R.x2-R.x1,1),max(R.y2-R.y1,1),backColor,false);
    }
  }

  if (curImgDescr == NULL) 
    return;

  for( int i=0; i<(int)curImgDescr->RectList.size(); i++ ) {
    R = curImgDescr->RectList[i];
    ImgDescrList::sortCoords(R);
    if (R.score>threshLow && R.score<threshHigh) {
      if (i == curRect) {
	drawRect(R.x1,R.y1,max(R.x2-R.x1,1),max(R.y2-R.y1,1),selColor,false);
      } else {
	drawRect(R.x1,R.y1,max(R.x2-R.x1,1),max(R.y2-R.y1,1),normColor,false);
      }
    }
  }

}


void IDL_EditImgDisp::onDragLeft(int x, int y)
{
  if (curImgDescr == NULL) 
    return;
  if ((x > imgWidth() ) || (y > imgHeight() )) 
    return;

	if (curRect == -1) {
    dragRect.x2 = x;
    dragRect.y2 = y;
	  if (abs((dragRect.x2-dragRect.x1)*(dragRect.y2-dragRect.y1)) >= 
        minDragArea ) {
      curImgDescr->RectList.push_back(dragRect);
      setCurRect(curImgDescr->RectList.size()-1);
	  }
	  return;	
	}

  Rect R = curImgDescr->RectList[curRect];
	ImgDescrList::sortCoords(R);
	int width = R.x2-R.x1;
	int height = R.y2-R.y1;
	QRect eR((int) imgScaleX()*R.x1-4, (int) imgScaleY()*R.y1-4,
	         (int) imgScaleX()*width+8, (int) imgScaleY()*height+8 );
	QPaintEvent* e = new QPaintEvent(eR);//,true);
  

	curImgDescr->RectList[curRect].x2 = x;
	curImgDescr->RectList[curRect].y2 = y;
 	emit changed();
  
	//(edi) paintEvent(e);
	repaint();
	delete e;
}


void IDL_EditImgDisp::onDragRight(int x, int y)
{
  if ((curImgDescr == NULL) || (curRect == -1))
    return;
  if ((x > imgWidth() ) || (y > imgHeight() )) 
    return;
  
  Rect R = curImgDescr->RectList[curRect];
	ImgDescrList::sortCoords(R);
	int width = R.x2-R.x1;
	int height = R.y2-R.y1;
	QRect eR((int) imgScaleX()*R.x1-4,(int) imgScaleY()*R.y1-4,
	         (int) imgScaleX()*width+8,(int) imgScaleY()*height+8 );
	QPaintEvent* e = new QPaintEvent(eR);//,true);
  
	curImgDescr->RectList[curRect].x1 = x;
	curImgDescr->RectList[curRect].y1 = y;
 	emit changed();
  
	//(edi) paintEvent(e);
	repaint();
	delete e;
}


void IDL_EditImgDisp::onClickLeft(int x, int y)
{

  if (curImgDescr == NULL) 
    return;
  if ((x > imgWidth() ) || (y > imgHeight() )) 
    return;
	setCurRect(-1);
	
	dragRect.x1 = x;
	dragRect.y1 = y;
	dragRect.x2 = x+1;
	dragRect.y2 = y+1;
	setBackgroundMode(Qt::NoBackground);//edi
}

						 
void IDL_EditImgDisp::onClickRight(int x, int y)
{

  if ((curImgDescr == NULL) || (curRect == -1))
    return;
  if ((x > imgWidth() ) || (y > imgHeight() )) 
    return;
  
  Rect R = curImgDescr->RectList[curRect];
	if ( abs(R.x1-x) > abs(R.x2-x)) {
    curImgDescr->RectList[curRect].x1 = R.x2;	
    curImgDescr->RectList[curRect].x2 = R.x1;	
	}
	if ( abs(R.y1-y) > abs(R.y2-y)) {
    curImgDescr->RectList[curRect].y1 = R.y2;	
    curImgDescr->RectList[curRect].y2 = R.y1;	
	}
	onDragRight(x,y);
	setBackgroundMode(Qt::NoBackground);//edi
}

					
