#ifndef IDL_PARSER_HH
#define IDL_PARSER_HH
#include <iostream>
#include <fstream>
#include <vector>
#include "imgdescr.hh"

///////////////////////////////////////////////////////////////////////////////
//  
// class IDL_Parser
//_____________________________________________________________________________
// IDL_Parser (Image Description List Parser) is made to read in IDL Files.
// IDL Files have an EBNF Definition (see below)
// It is programmed according to the EBNF.
///////////////////////////////////////////////////////////////////////////////
class IDL_Parser 
{
public:
	IDL_Parser(ifstream* fin);
	char parse(vector< ImgDescr* >* );
 private:
    char nextChar();
	char readNumber();
	char readString();
	char readRect(); 
	char readImgDescr();
	char curCh;
	int curNumber;
	string curString;
	ImgDescr* curImgDescr;
	Rect curRect;
	ifstream* fIn;
};

///////////////////////////////////////////////////////////////////////////////
// 
// The EBNF of an IDL File
//_____________________________________________________________________________
//
//  File = '.' | ({ImgDescr ';'} ImgDescr '.')
//  ImgDescr = ImgName | (ImgName ':' RectList)
//  RectList = {Rectangle ',' } Rectangle
//  Rectangle = '(' Number ',' Number ',' Number ',' Number ')' [':' Score] 
//  ImgName = '"' string '"'
//  Score = c++ instream double
//  Number = digit {digit}
//  
//  Eveything after the '.' will be ignored.
//  Spaces and control signs will be ignored. 
//
// 
// Example File:
//   "img001.png":(10,10,30,30):9.1221;
//   "img002.png":(20,30,70,90):0.5361,(18,27,65,93),(18,27,65,93);
//   "subdirectory1/img003.png";
//   "img004.png":(18,27,65,93).
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 	
//  IDL_Parser's constructor
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 	IDL_Parser(ifstream* fin);
//_____________________________________________________________________________
// Use: 
//   Constructs an object of type IDL_Parser.  
// Parameters: 
//    fIn: The input stream you want to parse.
// Precondition:
//   fIn must be valid instream. If it isn't containing an IDL-File
//   you will have no result, but the parser should not crash.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 	
//  IDL_Parser's public function
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 	char parse(vector< ImgDescr* >* );
//_____________________________________________________________________________// Use: 
//  Parses an IDL-File on the instream that was given by the obj's construction
//  It's rarely meaningful to call this routine twice for the same object.
// Paramters
//   data: Where the result should be stored in. 
// Preconditions
//   data must be a valid.
// Return Values and Side-effects:
//   The return value is true if the try was succesful, false otherwise.
//   if succesful, all Image Descriptions are added to the vector.
//   if not succesful all those Image Descr. are added, wich could be parsed.
//   Existing elements of the vector won't be deleted.
//   curCh, curString, curRect and curNumber are updated.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 	
//  IDL_Parser's private functions and members
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// char nextChar() (for internal use)  
//______________________________________________________________________________
// Use:
//  reads in the next character wich isn't a space or a control character.  
// Return Values and Side-effects:
//   the next valid character or EOF
//   the return value is also stored in curCh
//
//
//
// char readNumber() (for internal use)
//______________________________________________________________________________
// Use:
//  Tries to read in a number  
// Return Values and Side-effects:
//   The return value is true if the try was succesful, false otherwise.
//   if succesful the numberis stored in curNumber. curCh is updated as well.
//
//
//
// char readString() (for internal use)
//______________________________________________________________________________
// Use:
//  Tries to read in a string that starts and ends with: ' " ' 
//  WARNING: If the string itself contains an ' " ' this routine won't work.
// Return Values and Side-effects:
//   The return value is true if the try was succesful, false otherwise.
//   if succesful the string is stored in curString. curCh is updated as well.
//
//
//
// char readRect() (for internal use)
//______________________________________________________________________________
// Use: 
//  Tries to read in a rectangle according to the EBNF.
// Return Values and Side-effects:
//   The return value is true if the try was succesful, false otherwise.
//   if succesful the rectangle is stored in curRect. 
//   curCh and curNumber are updated as well.
//
//
//
// char readImgDescr()
//______________________________________________________________________________
// Use: 
//  Tries to read in an image description according to the EBNF.
// Return Values and Side-effects:
//   The return value is true if the try was succesful, false otherwise.
//   if succesful the pointer to the newly created ImgDescr is stored in curImgDescr. 
//   curCh, curString, curRect and curNumber are updated as well.
//
//
//
// private members
//______________________________________________________________________________
//  ifstream* fIn;				input stream to read from
//	char curCh; 				last char currently read
//	int curNumber;				las number currently read
//	string curString;           last string currently read
//	Rect curRect;				las rectangle currently read
//	ImgDescr* curImgDescr;      last image description currently read
///////////////////////////////////////////////////////////////////////////////



#endif
