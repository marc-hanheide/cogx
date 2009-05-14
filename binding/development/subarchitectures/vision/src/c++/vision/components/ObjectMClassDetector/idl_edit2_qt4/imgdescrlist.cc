#include <math.h>
#include "idl_parser.hh"
#include "imgdescrlist.hh"

ImgDescrList::ImgDescrList()
{
  curIndx = -1;
}


ImgDescrList::ImgDescrList(ifstream* fIn)
{
  curIndx = -1;
  load(fIn);
}


ImgDescrList::~ImgDescrList()
{
  clear();
}


void ImgDescrList::clear()
{
  curIndx = -1;
  ImgDescrList& v = *this;
  int i = -1;
  while(++i < (signed)v.size() ) delete v[i];
  vector< ImgDescr* >::clear();	
}


void ImgDescrList::load(ifstream* fIn, bool overwrite, bool doSort)
{
    if (overwrite) clear();
    IDL_Parser parser(fIn);
	parser.parse((vector< ImgDescr* >*)this);
	if (doSort) sort();
}


ImgDescr* ImgDescrList::findName(const string &name)
{
  ImgDescrList& v = *this;
  int size = v.size();
  
  //Security checks
  if (size <= 0) return NULL;
  if ((curIndx < -1) || (curIndx >= size)) curIndx = -1;

  int lower = -1;
  int upper = v.size();
  int mid;
  int forward = (curIndx+1)%size;
  int backward = (curIndx+size)%size;
  int step = 0;
  
  do {
    //linear backward search from curIndx
    if (v[backward]->name == name) {
	  curIndx = backward;
	  return v[backward];
	}
    backward = (backward-1+size)%size;
	
    //linear forward search from curIndx+1
    if (v[forward]->name == name) {
      curIndx = forward;
	  return v[forward];
 	}
	forward = (forward+1)%size;
	step +=2;

    //speculative binary search
    if (upper-lower > 1)
    {
      mid = (upper+lower)/2;
      if (v[mid]->name == name) {
  	    curIndx = mid;
	    return v[mid];
	  }
      if (v[mid]->name > name) upper = mid;
      if (v[mid]->name < name) lower = mid;
    }

  } while(step < size);
  
  return NULL;
}


ImgDescr* ImgDescrList::findOrCreate(const string &name)
{
  ImgDescr* descr = findName(name);
  if (descr == NULL) {
    descr = new ImgDescr;
    descr->name = name;
    push_back(descr);
	curIndx = size()-1;
  }
  return descr;
}


void ImgDescrList::findAndRemove(const string &name)
{
	
    ImgDescrList& v = *this;
    ImgDescrList::iterator it = v.begin();
	int i=0;
	while (it != v.end())
	{
	  if ((*it)->name == name) {
		delete (*it);
	    it = v.erase(it);
   	    curIndx = i-1;		 
	  } else {
  	    it++;
		i++;
	  }
	}	
    return;
}


void ImgDescrList::addIDs(ImgDescrList& newIDL, string prefix)
{
	ImgDescr* curID;
	int i = -1;
	while(++i < (signed)newIDL.size()) {
	  string newName = prefix + newIDL[i]->name; 
	  curID = findOrCreate(newName);
	  int i2 = -1;
  	  while(++i2 < (signed)newIDL[i]->RectList.size()) {
	    curID->RectList.push_back(newIDL[i]->RectList[i2]);
	  }
	}
}


void ImgDescrList::save(ofstream& fout,bool sorted)
{
  if (sorted) sort();
  ImgDescrList& v = *this;
  char cRectSep;
  int i2;
  int i = -1;
  while(++i < (signed)v.size() )
  {
    if (i>0) fout << ";\n";
    fout << '"' << v[i]->name << '"';
	i2 = -1;
	cRectSep = ':';
    while(++i2 < (signed)v[i]->RectList.size()) {
	  fout << cRectSep;
	  cRectSep = ',';
	  fout << " (" << v[i]->RectList[i2].x1 << ", ";
	  fout << v[i]->RectList[i2].y1 << ", ";
	  fout << v[i]->RectList[i2].x2 << ", ";
	  fout << v[i]->RectList[i2].y2 << ")";
	  if (v[i]->RectList[i2].score != 0.0)
	     fout << ":" << v[i]->RectList[i2].score;
 
	}
  }
  fout << ".\n";
}


void ImgDescrList::getRectInfo(int& count, double& min, double& max)
{
  ImgDescrList& v = *this;
  char first = true;
  int i2;
  int i = -1;
  double s;
  count = 0;
  while(++i < (signed)v.size() )
  {
    i2=-1;
    while(++i2 < (signed)v[i]->RectList.size()) {
	  s = v[i]->RectList[i2].score;
	  if (first) {
	    min = s;
		max = s;
		first = false;
	  } else {
	    if (s < min) min=s;
	    if (s > max) max=s;
	  }
 	}
	count += i2;
  }
}	


void ImgDescrList::sort()
{
  ImgDescrList& v = *this;
  int size = v.size();
  if (size < 2) return;
  int i2;
  int toBubbleUp;
  ImgDescr* h;

  //Heap aufbauen
  for (int i=size-1; i >= 0; i--) {
    i2 = i;
	while ((i2*2+1) < size) {
	  toBubbleUp =	i2*2+1;	
      if (((i2*2+2) < size)&&(v[i2*2+2]->name > v[toBubbleUp]->name)) toBubbleUp++;
	  if (v[toBubbleUp]->name > v[i2]->name) {
		  h = v[i2]; v[i2] = v[toBubbleUp]; v[toBubbleUp] = h;
	      i2 = toBubbleUp;
	  } else i2 = size;
	}
  }

  //Heap abbauen
  for(size--; size >= 1; size--) {
    h = v[size]; v[size] = v[0]; v[0] = h;
    i2 = 0;
	while ((i2*2+1) < size) {
	  toBubbleUp =	i2*2+1;	
      if (((i2*2+2) < size)&&(v[i2*2+2]->name > v[toBubbleUp]->name)) toBubbleUp++;
	  if (v[toBubbleUp]->name > v[i2]->name) {
		  h = v[i2]; v[i2] = v[toBubbleUp]; v[toBubbleUp] = h;
	      i2 = toBubbleUp;
	  } else i2 = size;
	}
  }
}


/***********************************************************/
/*                   External Functions                    */
/***********************************************************/

void ImgDescrList::sortCoords( Rect& r )
  /*******************************************************************/
  /* Sort the coordinates of an rectangle r such that                */
  /*   (x1,y1) is the upper left corner, and                         */
  /*   (x2,y2) is lower right corner (meaning x2>x1, y2>y1)          */  
  /* Parameters:                                                     */
  /*   r: r is the rectangle to be sorted.                           */
  /*******************************************************************/
{
  int h;
  if( r.x1 > r.x2 ) {
     h    = r.x1; 
     r.x1 = r.x2; 
     r.x2 = h;
  }
  if( r.y1 > r.y2 ) {
     h    = r.y1; 
     r.y1 = r.y2; 
     r.y2 = h;
  }
  return;
}


double ImgDescrList::compCover( Rect r1, Rect r2 )
  /*******************************************************************/
  /* Determine the Fraction of r1 which is covered by r2.            */
  /* Parameters:                                                     */
  /*   r1, r2: The rectangles. Attention: the order does matter!     */
  /* Return Value:                                                   */
  /*   The fraction wich is coverd (always between 0 and 1).         */
  /*******************************************************************/
{
  sortCoords(r1);
  sortCoords(r2);

  int nWidth  = r1.x2 - r1.x1;
  int nHeight = r1.y2 - r1.y1;
  int iWidth  = max(0,min(max(0,r2.x2-r1.x1),nWidth )-max(0,r2.x1-r1.x1));
  int iHeight = max(0,min(max(0,r2.y2-r1.y1),nHeight)-max(0,r2.y1-r1.y1));
  return ((double)iWidth * (double)iHeight)/((double)nWidth * (double)nHeight);
}	


double ImgDescrList::compRelDist( const Rect& r1, const Rect& r2 )
  /*******************************************************************/
  /* Determine the distance between the centers of r1 and r2.        */
  /* How the distance is measured depends on r1:                     */
  /*   If the center of r2 is on the largest inlying ellipse, the    */
  /*   distance is defined to be exactly 1.0. Outside it is greater; */
  /*   inside it is smaller.                                         */
  /* Parameters:                                                     */
  /*   r1, r2: The rectangles. Attention: the order does matter!     */
  /* Return Value:                                                   */
  /*   the relative distance.                                        */
  /*******************************************************************/
{
  double dWidth  = r1.x2 - r1.x1;
  double dHeight = r1.y2 - r1.y1;
  double xdist   = (double)(r1.x1 + r1.x2 - r2.x1 - r2.x2) / dWidth;
  double ydist   = (double)(r1.y1 + r1.y2 - r2.y1 - r2.y2) / dHeight;
  return sqrt(xdist*xdist + ydist*ydist);
}


double ImgDescrList::compRelDist( const Rect& r1, const Rect& r2, 
                                  float dAspectRatio, FixDimType eFixObjDim )
  /*******************************************************************/
  /* Determine the distance between the centers of r1 and r2.        */
  /* How the distance is measured depends on r1:                     */
  /*   If the center of r2 is on the largest inlying ellipse, the    */
  /*   distance is defined to be exactly 1.0. Outside it is greater; */
  /*   inside it is smaller.                                         */
  /* This version of the function uses a ground truth box with fixed */
  /* aspect ratio as reference.                                      */
  /* Parameters:                                                     */
  /*   r1, r2: The rectangles. Attention: the order does matter!     */
  /*   dAspectRatio: the fixed aspect ratio for the first rect.      */
  /*   eFixObjDim  : the information whether the object width or     */
  /*                 height shall be fixed for the computation.      */
  /*                 (allowed values: FIX_OBJWIDTH, FIX_OBJHEIGHT)   */
  /* Return Value:                                                   */
  /*   the relative distance.                                        */
  /*******************************************************************/
{
  double dWidth, dHeight;

  switch( eFixObjDim ) {
  case FIX_OBJWIDTH:
    dWidth  = r1.x2 - r1.x1;
    dHeight = dWidth / dAspectRatio;
    break;

  case FIX_OBJHEIGHT:
    dHeight = r1.y2 - r1.y1;
    dWidth  = dHeight * dAspectRatio;
    break;

  default: 
    cerr << "Error in ImgDescrList::compRelDist(): "
         << "Unknown type for parameter ('which obj dimension to fix?'): "
         << eFixObjDim << "!" << endl;
    return -1.0;
  }

  double xdist = (double)(r1.x1+r1.x2-r2.x1-r2.x2) / dWidth;
  double ydist = (double)(r1.y1+r1.y2-r2.y1-r2.y2) / dHeight;
  return sqrt(xdist*xdist + ydist*ydist);
}


bool ImgDescrList::isMatching( Rect& r1, Rect& r2, 
                               double dTDist, double dTCover, double dTOverlap)
  /*******************************************************************/
  /* Check if r2 matches with r1 according to 3 threshold values.    */
  /* Parameters:                                                     */
  /*   r1, r2: The rectangles. Attention: the order does matter!     */
  /*   TDist   : maximal relative distance r2 is allowed to have     */
  /*             from r1 (always >= 0.0).                            */
  /*             If TDist is > (1 - TCover/2 - TOverlap/2), then     */
  /*             TDist is redundant.                                 */
  /*   TCover  : minimal fraction of r1 that has to be covered by r2 */ 
  /*             (meaningful values are between 0 and 1).            */
  /*   TOverlap: minimal fraction of r2 that has to be covered by r1 */
  /*             (meaningful values are between 0 and 1).            */
  /* Return Value:                                                   */
  /*   true iff r2 passes all 3 threshold tests.                     */
  /*******************************************************************/
{
  return ( (compRelDist(r1,r2) <= dTDist) && 
           (compCover(r1,r2) >= dTCover) && 
           (compCover(r2,r1) >= dTOverlap) );
}


bool ImgDescrList::isMatching( Rect& r1, Rect& r2, 
                               double dTDist, double dTCover, double dTOverlap,
                               float dAspectRatio, FixDimType eFixObjDim )
  /*******************************************************************/
  /* Check if r2 matches with r1 according to 3 threshold values.    */
  /* This version of the function uses a ground truth box with fixed */
  /* aspect ratio as reference.                                      */
  /* Parameters:                                                     */
  /*   r1, r2: The rectangles. Attention: the order does matter!     */
  /*   TDist   : maximal relative distance r2 is allowed to have     */
  /*             from r1 (always >= 0.0).                            */
  /*             If TDist is > (1 - TCover/2 - TOverlap/2), then     */
  /*             TDist is redundant.                                 */
  /*   TCover  : minimal fraction of r1 that has to be covered by r2 */ 
  /*             (meaningful values are between 0 and 1).            */
  /*   TOverlap: minimal fraction of r2 that has to be covered by r1 */
  /*             (meaningful values are between 0 and 1).            */
  /*   dAspectRatio: the fixed aspect ratio for the first rect.      */
  /*   eFixObjDim  : the information whether the object width or     */
  /*                 height shall be fixed for the computation.      */
  /*                 (allowed values: FIX_OBJWIDTH, FIX_OBJHEIGHT)   */
  /* Return Value:                                                   */
  /*   true iff r2 passes all 3 threshold tests.                     */
  /*******************************************************************/
{
  return ( (compRelDist(r1,r2,dAspectRatio,eFixObjDim) <= dTDist) && 
           (compCover(r1,r2) >= dTCover) && 
           (compCover(r2,r1) >= dTOverlap) );
}
