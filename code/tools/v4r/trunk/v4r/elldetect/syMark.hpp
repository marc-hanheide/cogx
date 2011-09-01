/** \file
  \brief  Klasse zur Markenerkennung

**********************************************************************
  \ingroup RTE
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
#pragma once
#include "multiplatform.hpp"
#include OCV_CV_H
//#include OCV_HIGHGUI_H

#define PI 3.141592653589

#define FM_DIR_CLOCKWISE      -1
#define FM_DIR_ANTICLOCKWISE  1

#define FM_BITCODE12          12
#define FM_BITCODE14          14
#define FM_BITCODE20          20
#define FM_BITCODE_MASK12     0x00000FFF
#define FM_BITCODE_MASK14     0x00003FFF
#define FM_BITCODE_MASK20     0x000FFFFF

#define FM_TYPE_BW            0 // schwarz auf weiss
#define FM_TYPE_WB            1 // weiss auf schwarz

//#define FM_PRECISION          360 // anzahl schritte für einen umlauf
#define FM_PRECISION_MEDIAN   5   // anzahl der werte, die für den medianfilter benutzt werden (in jede richtung, dh # = 2*x+1)
namespace RTE {

class CzMark 
{
public:
  CzMark(int iBit = FM_BITCODE12, int iType = FM_TYPE_WB, int iDirection = FM_DIR_ANTICLOCKWISE);
  ~CzMark();

  // Zu untersuchendes Bild setzen
  // Parameter: pImg:         Zeiger auf das Bild (im OpenCV Format)
  // Rückgabe:  immer true
  bool SetImage(IplImage *pImg);

  // Ermittelt die Marke - muss zuerst aufgerufen werden
  // Parameter: dMidX,dMidY:  Mittelpunkt der Marke in Pixel
  //            dOutRadius:   Äußerer Radius der Marke
  //            dSize:        Breite des Bitcode-Streifens der Marke
  // Rückgabe:  bool - true Ok, false Fehler
  bool FindMark(double dMidX, double dMidY, double dOutRadius, double dSize, double dInnerRadius, double dMidRadius, double dContrastFactor);

  // Gibt die gefundenen Bits zurück
  // Parameter: pBits:        Zeiger auf DWORD. Wenn nicht NULL wird die Bitfolge als DWORD belegt
  // Rückgabe:  CString - Bitfolge mit '0' und '1' als String
//  CString GetMark(DWORD *pBits);
  void GetMark(DWORD *pBits);

  // Identifizieren der Marke anhand der Markentabellen
  // Parameter: dwBits:       zu identifizierende Bitmarke, wenn nicht übergeben, wird die Bitmarke
  //                          des Objekts verwendet (die zuvor von FindMark gefüllt wurde)
  // Rückgabe:  int - Nummer der Marke
  int IdentifyMark(DWORD dwBits = 0);

  // BitCode lesen/setzen - der BitCode sind die Anzahl der Bits der Marke (FM_BITCODE12,FM_BITCODE14,FM_BITCODE20)
  int GetBitCode() const { return m_iBit; }
  void SetBitCode(int val);
  // Typ lesen/setzen - Typ der Marke, schwarz/weiss FM_TYPE_BW oder weiss/schwarz FM_TYPE_WB
  int GetType() const { return m_iType; }
  void SetType(int val) { m_iType = val; }
  // Direction lesen/setzen - Leserichtung der Marke, im Uhrzeigersinn FM_DIR_CLOCKWISE oder gegen den Uhrzeigersinn FM_DIR_ANTICLOCKWISE
  int GetDirection() const { return m_iDirection; }
  void SetDirection(int val) { m_iDirection = val; }

private:
  BYTE GetPixel(int iX, int iY);
  void FilterMedian(int *pSource, int *pDest, int iSize, int iMedianSize);
  double GetDist(int iCirc, int iBitCount);
  int GetBit(int *pEdges, int iCirc, int iPos, int iDist);
  bool GetEdges(int *pThres,int *pEdges, int iCirc);
  int GetStart(int *pEdges, int iCirc, int iBitCount, int *pQuality = NULL);
  int SetColor(int *pEdges, int *pColor, int iCirc);
  bool SetBitsFromColor(int *pColor, int iCirc, BOOL *pBits, int iBitCount, int iStart);
  bool SetBitsFromEdge(int *pEdges, int iCirc, BOOL *pBits, int iBitCount);

private:
  IplImage *m_pImg;
  int m_iType;
  int m_iBit;
  int m_iDirection;
  BOOL *m_pBits;

};

} // namespace RTE