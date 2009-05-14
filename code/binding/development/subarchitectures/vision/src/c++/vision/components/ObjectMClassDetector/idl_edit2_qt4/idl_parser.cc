///////////////////////////////////////////////////////////////////////////////
//
// File: idl_parser.cc
// Author: Dirk Zimmer
// last change: 13.01.04
//
// Content:
//   Class IDL_Parser. (Image Description List Parser)
//   
//
///////////////////////////////////////////////////////////////////////////////

#include <ctype.h>
#include <vector>
#include "idl_parser.hh"



IDL_Parser::IDL_Parser(ifstream* fin)
{
  curCh = 0;
  fIn = fin;
  curImgDescr = NULL;
  curNumber = 0;
  curString = "";
}


char IDL_Parser::nextChar()
{
  char c;
  (*fIn) >> c;
  while ((c != EOF) && ( (c == ' ') || (iscntrl(c)) )) (*fIn) >> c;
  curCh = c;		
  return c;
}

char IDL_Parser::readNumber()
{
  int nSign = 1;
  char c = curCh;
  if (!isdigit(c) && !(c=='-')) return false;
  
  if( c == '-' ) {
    nSign = -1;
    (*fIn) >> c;
  }
    
  int n = 0;
  while (isdigit(c))  {
    n = 10*n + c - (int)'0';
    (*fIn) >> c;
  }

  while ( (c != EOF) && ((c == ' ') || (iscntrl(c))) ) (*fIn) >> c;
  curCh = c;		
  curNumber = nSign * n;
  return true; 
}

char IDL_Parser::readString()
{
  
  if (curCh != '"') return false;
  string buffer =  "";//new char[limit];
  (*fIn) >> curCh;
  while ( (curCh != EOF) && (curCh != '"') ){
    buffer += curCh;
    (*fIn) >> curCh;
  }	
  if (curCh != '"') {
    return false;
  }
  nextChar();
  curString = buffer;  
  return true;
}


char IDL_Parser::readRect()
{
  Rect R;
  if (curCh != '(') return false;
  nextChar(); 
  if (!readNumber()) return false;
  R.x1 = curNumber;
  
  if (curCh != ',') return false;
  nextChar();
  if (!readNumber()) return false;
  R.y1 = curNumber;

  if (curCh != ',') return false;
  nextChar();
  if (!readNumber()) return false;
  R.x2= curNumber;

  if (curCh != ',') return false;
  nextChar();
  if (!readNumber()) return false;
  R.y2 = curNumber;

  if (curCh != ')') return false;
  nextChar();
  if (curCh == ':') {
    (*fIn) >> R.score;
     nextChar();
  } else R.score = 0.0;
  
  curRect = R;
  return true;
}

char IDL_Parser::readImgDescr()
{
  if (!readString() ) return false;
  ImgDescr* descr = new ImgDescr();
  descr->name = curString;	

  if (curCh == ':') { 
    do { 
      nextChar();
      if (!readRect()) {
		delete descr; 
		descr = NULL;
	    return false;
	  }
	  descr->RectList.push_back(curRect);
    } while (curCh ==',');
  }
  curImgDescr = descr;
  return true;
}

char IDL_Parser::parse(vector< ImgDescr* >* data )
{
  nextChar();
   
  if (curCh == '.') return true;
  
  if (!readImgDescr()) { return false; }
  data->push_back(curImgDescr);
  
  while (curCh == ';') 
  {
    nextChar();
    if (!readImgDescr()) { return false; }
  
    data->push_back(curImgDescr);
  }
  
  if (curCh != '.') { return false; }
  return true;
}
