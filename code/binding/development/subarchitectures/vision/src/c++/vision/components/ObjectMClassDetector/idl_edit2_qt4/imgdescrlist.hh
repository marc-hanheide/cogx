#ifndef IMGDESCRLIST_HH
#define IMGDESCRLIST_HH
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "imgdescr.hh"

////////////////////////////////////////////////////////////////////////////////
//
// class ImgDescrList inherits a vector of ImgDescr*
//______________________________________________________________________________
//
// First of all: This class inherits a vector. So don't forget you have 
// all the vector functionality avaible.
// ImgDescrList (Image Description List or short IDL) adds two major
// functionalities:
//   1) It allows you to load, save, edit and examine IDLs comfortably
//	 2) You have static toolkit functions to handle rectangles 
// Something important:
//   The ImgDescrList purposes to own its Image Descriptions.
//   So by executing the destructor or by calling clear() not only the pointers
//   are deleted also the ImgDescr they point to. Be aware of that!
// See further comments after class definition
////////////////////////////////////////////////////////////////////////////////
class ImgDescrList : public vector< ImgDescr* > 
{
public:
  
  ImgDescrList(); 
  ImgDescrList(ifstream* fIn);
  ~ImgDescrList();
  
	void clear();
	void load(ifstream* fIn, bool overwrite = true, bool doSort = false);
	void addIDs(ImgDescrList& newIDL, string prefix = "");
	void save(ofstream& fout, bool sorted = true);
  void sort();
  ImgDescr* findName(const string& name);
  ImgDescr* findOrCreate(const string& name);    
  void findAndRemove(const string& name);
  void getRectInfo(int& count, double& min, double& max);
	
public:
  /**************************/
  /*   External Functions   */
  /**************************/
  enum FixDimType{ FIX_OBJWIDTH = 0, FIX_OBJHEIGHT = 1};

	static void   sortCoords  ( Rect& r );
  static double compCover   ( Rect r1, Rect r2 );
  static double compRelDist ( const Rect& r1, const Rect& r2 );
  static double compRelDist ( const Rect& r1, const Rect& r2, 
                              float dAspectRatio, 
                              FixDimType eFixObjDim=FIX_OBJHEIGHT );
  static bool   isMatching  ( Rect& r1, Rect& r2, 
                              double dTDist, double dTCover, double dTOverlap);
  static bool   isMatching  ( Rect& r1, Rect& r2, 
                              double dTDist, double dTCover, double dTOverlap,
                              float dAspectRatio, 
                              FixDimType eFixObjDim=FIX_OBJHEIGHT );
  
private:
  int curIndx;
};

////////////////////////////////////////////////////////////////////////////////
//
// class ImgDescrList's Constructors and Destructors
//
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// ImgDescrList(); 
//______________________________________________________________________________
// Use: 
//  Creates an empty Image Description List.
//
//
//
// ImgDescrList(ifstream* fIn);
//______________________________________________________________________________
// Use: 
//  Creates an Image Description List out of an IDL-File
// Parameters:
//   fIn: Points to the instream of the IDL_File
// Precond:
//	 fIn must be valid
//
//
//
// ~ImgDescrList(ifstream* fIn);
//______________________________________________________________________________
// Use: 
//  See function clear
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// class ImgDescrList's public functions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// public function clear
//______________________________________________________________________________
// Use: 
//  Frees the memory of all the ImageDescr wich are referenced by this object.
//  Clears the vector itself. 
//  So all the Image Description are presumed to be owned by this object.
//
//
//
// public function load
//______________________________________________________________________________
// Use: 
//   adds the Image Descriptions of an IDL-File to this vector.
// Parameters:
//   ifstream: Points to the instream of the IDL_File
//   overwrite: if true (default)  the existing IDs are deleted.
//   doSort: if true (non-default) the descriptions will be sorted.
//           see sort()
// Precond:
//   ifstream must be valid
//
//
//
// public function addIDs
//______________________________________________________________________________
// Use: 
//   useful to merge two Image Description Lists
//   This Object will contain all image descriptions afterwards 
//   The RectLists of common image descriptions are added.
// Parameters:
//   newIDL: The IDs you want to add. This object remains untouched.
//           All values will be copied.
//   prefix: A string that will be added to the description's names of
//           newIDL before(!) the merging process 
//
//
//
// public function save
//______________________________________________________________________________
// Use: 
//   save this IDL into an IDL-File.
// Parameters:
//   ofstream: the output file stream.
//   sorted: if true (default) this IDL will be sorted before saving. 
// Precond: ofstream must be valid, ready to write and enough space on disk.
//
//
//
// public function sort
//______________________________________________________________________________
// Use: 
//   sorts the Image Descriptions according to their names in asc. order.
//   Heapsort is used. Runtime is O(log(n)).
//
//
//
// public function findName
//______________________________________________________________________________
// Use: 
//   find's an Image Description by its name.
// Parameters:
//   name: the name of the Image Description you're looking for.
// Return Value:
//   if you were looking for an non-existing Description the result is NULL
//   Otherwise you get a valid pointer.
// Runtime Behaviour:
//   n := nuber of Descriptions in this vector. 
//   if you're looking for an non-existing description it will take O(n) always.
//   if you're looking for an element near the last element you searched,
//   it will take constant time.
//   if the vector is sorted it will take O(log(n)).
//   Otherwise it will take O(n).
//
//
//
// public function findOrCreate
//______________________________________________________________________________
// Use: 
//   Closely related to findName.
//   find's an Image Description by its name.
//   If such a Description doesn't exist. One will be created.
// Parameters:
//   name: the name of the Image Description you're looking for.
// Return Value:
//   You get always a valid pointer.
// Runtime Behaviour:
//   see findName()
//
//
//
// public function findAndRemove
//______________________________________________________________________________
// Use: 
//   Closely related to findName.
//   find's an Image Description by its name.
//   If such a Description exists it will be deleted. 
//   The Description itself will be deleted as well as the entry.
// Parameters:
//   name: the name of the Image Description you're looking for.
// Runtime Behaviour:
//   see findName()
//
//
//
// public function getRectInfo
//______________________________________________________________________________
// Use: 
//   give you some Info about the Rectangle in this IDL.
// Return Parameters:
//   count: the number of Rectangles found. 
//   min: the minimum score found.
//   max: the maximum score found.
//   if count == 0, min and max remain unchanged.
///////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
// class ImgDescrList's static public toolkit functions
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// public Toolkit-function SortCoords
//______________________________________________________________________________
// Use: 
//   sorts the coordinates of an rectangle r so that
//   x1,y1 is the upper left corner 
//   x2, y2 is lower right corner (means x2 > x1, y2 > y1)  
// Parameters:
//   r: r is the rectangle wich coordinates yow want to be sorted.
//
//
//
// public Toolkit-function Cover
//______________________________________________________________________________
// Use: 
//   Determines the Fraction of r1 wich is covered by r2.
// Parameters:
//   r1, r2: The rectangles. Attention the order does matter!
// Return Value:
//   The fraction wich is coverd. Always between 0 and 1.
//
//
//
// public Toolkit-function RelativeDist
//______________________________________________________________________________
// Use: 
//   Determines the distance between the centers of r1 and r2. 
//   How the distance is measured depends on r1:
//   If the center of r2 is on the largest inlying ellipse the distance is 
//   defined to be exactly 1.0. Outside it is greater, inside it is smaller.
// Parameters:
//   r1, r2: The rectangles. Attention the order does matter!
// Return Value:
//   the Relative distance.
//
//
//
// public Toolkit-function IsMatching
//______________________________________________________________________________
// Use: 
//   Checks if r2 matches onto r1 according to 3 threshold values.
// Parameters:
//   r1, r2: The rectangles. Attention the order does matter!
//   TDist: maximal relative distance r2 is allowed to be away from r1.
//     always >= 0.0
//     If TDist is > (1- TCover/2 - TOverlap/2) then TDist is redundant.
//   TCover: minimal fraction of r1 that has to be cover by r2. 
//     meaningful values are between 0 and 1.
//   TOverlap: minmal fraction of r2 that has to be covered by r1.
//     meaningful values are between 0 and 1.
// Return Value:
//   true if r2 passes all 3 threshold tests.
///////////////////////////////////////////////////////////////////////////////


#endif
