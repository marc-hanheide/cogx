/** \file
  \brief  Klasse zur Markenerkennung

**********************************************************************
  \ingroup RTE
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
//#include "stdafx.h"
#include "syDebug.hpp"

#include "multiplatform.hpp"
#include "syMark.hpp"

namespace RTE {

// globale arrays zur markenidentifizierung sind in externen dateien ausgelagert
// int giCodes_12bit[], int giCodes_14bit[], int giCodes_20bit[]
#include "codes12bit.cpp"
#include "codes14bit.cpp"
#include "codes20bit.cpp"
/**************************** CzMark::CzMark ****************************/
/// Konstruktor
/** 
**********************************************************************
  \param iBit       Codierung der Marke (12Bit: FM_BITCODE12 (default), 14Bit: FM_BITCODE14, 20Bit: FM_BITCODE20)
  \param iType      Typ der Marke (Schwarz auf Weiss: FM_TYPE_BW, Weiss auf Schwarz: FM_TYPE_WB (default))
  \param iDirection Richtung der Marke (Im Uhrzeigersinn: FM_DIR_CLOCKWISE, Gegen den Uhrzeigersinn: FM_DIR_ANTICLOCKWISE (default))
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
CzMark::CzMark(int iBit, int iType, int iDirection)
{
  m_pImg = NULL;
  m_iType = iType;
  m_iBit = iBit;
  m_iDirection = iDirection;
  m_pBits = NULL;
  if(iBit>0)
    m_pBits = new BOOL[iBit];
}

/*************************** CzMark::~CzMark ***************************/
/// Destruktor
/** 
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
CzMark::~CzMark()
{
  if(m_pBits) delete [] m_pBits;
}

/************************* CzMark::SetBitCode *************************/
/// Bit Code setzen
/** 
**********************************************************************
  \param val  FM_BITCODE12, FM_BITCODE14, FM_BITCODE20
**********************************************************************
  \author reindlj
  \date 03.03.2010
***********************************************************************/
void CzMark::SetBitCode(int val)
{
  ASSERT(val == FM_BITCODE12 || val == FM_BITCODE14 || val == FM_BITCODE20);
  m_iBit = val;
  if(m_pBits) { delete [] m_pBits; m_pBits = NULL; };
  if(m_iBit>0)
    m_pBits = new BOOL[m_iBit];
}

/*************************** CzMark::SetImage ***************************/
/// Bilddaten setzen
/** 
**********************************************************************
  \param pImg    Zeiger auf Bilddaten (Graustufenbild, 8-bit) - es wird nur der zeiger gemerkt!
**********************************************************************
  \return true Ok, false Fehler
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
bool CzMark::SetImage(IplImage *pImg)
{
  m_pImg = pImg;
  return true;
}

/******************************* compare *******************************/
/// interne vergleichsfunktion für qsort
/** nur für den MedianFilter
**********************************************************************
  \param pElem1 erstes Element (Zeiger auf int)
  \param pElem2 zweites Element (Zeiger auf int)
**********************************************************************
  \return 0 gleich, -1 kleiner, 1 grösser
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
int compare(const void *pElem1, const void *pElem2)
{
  int iDiff = *(int *)pElem1 - *(int *)pElem2;
  if(iDiff == 0)
    return 0;
  if(iDiff < 0)
    return -1;
  else
    return 1;
}

/************************* CzMark::FilterMedian *************************/
/// Medianfilter zur Entfernung von Ausreissern (Extremwerten)
/** 
**********************************************************************
  \param pSource      Zeiger auf Quelldaten
  \param pDest        Zeiger auf Platz für Ergebnisdaten
  \param iSize        Anzahl Datenpunkte
  \param iMedianSize  Anzahl Filterelemente (*2+1)
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
void CzMark::FilterMedian(int *pSource, int *pDest, int iSize, int iMedianSize)
{
  if(iMedianSize<0) return;
  int *pMedian = new int[2*iMedianSize+1];
  for(int i=0;i<iSize;i++)
  {
    int iM = 0;
    for(int j=-iMedianSize;j<=iMedianSize;j++)
    {
      int k = i+j;
      if(k < 0)
        k = iSize+k;
      if(k > iSize)
        k -= iSize;
      pMedian[iM++] = pSource[k];
    }
    qsort(pMedian,iMedianSize*2+1,sizeof(int),compare);
    pDest[i] = pMedian[iMedianSize];
  }
  delete [] pMedian;
}

/*************************** CzMark::GetPixel ***************************/
/// Bildpunkt zurückgeben
/** 
**********************************************************************
  \param iX X-Koordinate
  \param iY Y-Koordinate
**********************************************************************
  \return Grauwert des Pixels
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
BYTE CzMark::GetPixel(int iX, int iY)
{
  if(!m_pImg) return 0;
  if(iX<0 || iX>m_pImg->width)  return 0;
  if(iY<0 || iY>m_pImg->height) return 0;
  return ((uchar*)(m_pImg->imageData + m_pImg->widthStep*iY))[iX];
}

double CzMark::GetDist(int iCirc, int iBitCount)
{
  return ((double)iCirc/(double)iBitCount);
}

/*************************** CzMark::GetEdges ***************************/
/// Kanten ermitteln und eintragen
/** 1 steigende Kante, -1 fallende Kante
**********************************************************************
  \param pThres Array der Schwellwerte
  \param pEdges Array der Kanten
  \param iCirc  Arraygrösse
**********************************************************************
  \return true ok, false fehler
**********************************************************************
  \author reindlj
  \date 19.01.2010
***********************************************************************/
bool CzMark::GetEdges(int *pThres,int *pEdges, int iCirc)
{
  if(!pThres || !pEdges)  return false;
  int *pFilter = new int[iCirc];
  // extremwerte filtern
  FilterMedian(pThres,pFilter,iCirc,FM_PRECISION_MEDIAN);
  int iMedianMax = 0, iDataMax = 0;
  // maximum der gefilterten und maximum der original werte
  for(int i=0;i<iCirc;i++)
  {
    if(iDataMax<abs(pThres[i]))
      iDataMax = abs(pThres[i]);
    if(iMedianMax<abs(pFilter[i]))
      iMedianMax = abs(pFilter[i]);
  }
#ifdef DEBUG_MARK
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("% 5i "),pFilter[i]);
  }
  TRACE(_T("\n"));
#endif
  delete [] pFilter;
  // schwelle setzen
  int iThreshold = iMedianMax+(iDataMax-iMedianMax)/4;
  //TRACE(_T("Filtermaxwert: %i, Originalmaxwert: %i, Schwellwert: %i\n"),iMedianMax,iDataMax,iThreshold);
  // kanten extrahieren
  for(int i=0;i<iCirc;i++)
  {
    if(pThres[i]>iThreshold)
      pEdges[i] = 1;
    else if(pThres[i]<-iThreshold)
      pEdges[i] = -1;
    else 
      pEdges[i] = 0;
  }
#ifdef DEBUG_MARK
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("% 5i "),pEdges[i]);
  }
  TRACE(_T("\n"));
#endif
  return true;
}

/*************************** CzMark::GetStart ***************************/
/// Startindex ermitteln
/** ermittelt den Startindex für die Bitsuche und gibt ein Qualitätskriterium (0-100) zurück
**********************************************************************
  \param pEdges     Kantenarray
  \param iCirc      Grösse des Kantenarrays
  \param iBitCount  Bit Anzahl
  \param pQuality   Qualitätskriterium 0 schlecht, 100 super
**********************************************************************
  \return Starttindex für Bitbestimmung
**********************************************************************
  \author reindlj
  \date 19.01.2010
***********************************************************************/
int CzMark::GetStart(int *pEdges, int iCirc, int iBitCount, int *pQuality)
{
  int iStart=0;
  int iLastEdge = 0, iLastEdgeIndex = -1;
  double dDist = GetDist(iCirc,iBitCount);
  double dQuality = 100.0;
  while(iStart<iCirc)
  {
    bool bStart = true;
    for(int i=iStart;i<iCirc;i++)
    {
      if(pEdges[i] != 0 && iStart<i && bStart)
      {
        iStart = i;
        bStart = false;
      }
      if(pEdges[i] != 0)
      {
        if(iLastEdge == pEdges[i] && (i-iLastEdgeIndex)>1)  // zwei gleiche kanten??? => abwertung
          dQuality -= (100.0/iBitCount);
        iLastEdge = pEdges[i];
        if(iLastEdgeIndex > -1 && (i-iLastEdgeIndex)>1)
        {
          double dQ = 0;
          if(fabs(dDist-(i-iLastEdgeIndex)) < dDist)
            dQ = fabs(dDist-(i-iLastEdgeIndex));
          else
            dQ = (i-iLastEdgeIndex)%((int)(dDist+0.5));
          dQuality -= dQ*(100.0/iBitCount)/dDist;
        }
        iLastEdgeIndex = i;
      }
    }
    if(bStart) { dQuality = 0.0; break; }
    if(dQuality>70) break;
  }
  if(pQuality)
    *pQuality = (int)dQuality;
  return iStart;
}

/*************************** CzMark::SetColor ***************************/
/// Farbe setzen
/** Je nach kantenänderung Farbe setzen
**********************************************************************
  \param pEdges Kantenarray
  \param pColor Farbarray
  \param iCirc  Grösse der Arrays
**********************************************************************
  \return true ok, false fehler
**********************************************************************
  \author reindlj
  \date 19.01.2010
***********************************************************************/
int CzMark::SetColor(int *pEdges, int *pColor, int iCirc)
{
  if(!pEdges || !pColor) return false;
  int iColor = 0;
  bool bStart = true;
  int iStart = 0;
  for(int i=0;i<iCirc;i++)
  {
    if(pEdges[i] != 0 && bStart)
    {
      bStart = false;
      iStart = i;
    }
    if(pEdges[i] == -1)
      iColor = -1;
    else if(pEdges[i] == 1)
      iColor = 1;
    if(iColor != 0)
      pColor[i] = iColor;
  }
  for(int i=0;i<iStart;i++)
  {
    pColor[i] = iColor;
  }
#ifdef DEBUG_MARK
  TRACE(_T("CzMark::SetColor:\n"));
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("% 5i "),pColor[i]);
  }
  TRACE(_T("\n"));
#endif
  return true;
}
/*********************** CzMark::SetBitsFromEdge ***********************/
/// Bits aus Kantendaten extrahieren
/** 
**********************************************************************
  \param pEdges     Kanteninformation
  \param iCirc      Länge der Kanteninfo
  \param pBits      Bitarray
  \param iBitCount  Länge des Bitarrays
**********************************************************************
  \return true ok, false fehler
**********************************************************************
  \author reindlj
  \date 19.01.2010
***********************************************************************/
bool CzMark::SetBitsFromEdge(int *pEdges, int iCirc, BOOL *pBits, int iBitCount)
{
  if(!pEdges || !pBits) return false;
  int iStart=0;
  while(pEdges[iStart++]==0 && iStart<iCirc);
  if(iStart>0) iStart--;

  double dDist = (double)iCirc/(double)iBitCount;
  int iToleranz = (int)(((double)iCirc/(double)(iBitCount*3))+0.5);
//  TRACE(_T("Circ: %i, Start: %i, Dist: %f, Toleranz: %i\n"),iCirc,iStart,dDist,iToleranz);
  int j=0;
  BOOL bLastBit = -1;
  for(double dI=(double)iStart;dI<(iCirc+dDist) && j<iBitCount;dI+=dDist)
  { // sicherstellen, dass inkl. toleranz die ganze runde geschaut wird (deswegen FM_PRECISION+dDist)
    int i = (int)(dI+0.5);
    pBits[j] = GetBit(pEdges,iCirc,i,iToleranz); // toleranzbereich: je mehr bits, desto kleiner, immer die hälfte des abstands zwischen zwei möglichen kanten
    if(pBits[j] == bLastBit)  // darf nicht passieren, da immer ein 0 -> 1, 1 -> 0 wechsel stattfinden muss
    {
//      TRACE(_T("falsches bit!!!\n"));
      pBits[j] = -1;
    }
    if(pBits[j] != -1)
      bLastBit = pBits[j];
//    TRACE(_T("GetBit(%i - %f)=%i\n"),i,dI,pBits[j]);
    if(pBits[j] == -1)
      pBits[j] = bLastBit;
    j++;
  }
  while(j<iBitCount)
  { // auffüllen, falls zu wenig bits (müssen dann denselben wert haben, sonst wäre der start früher gewesen
    pBits[j++] = bLastBit;
  }

#ifdef DEBUG_MARK
  for(int i=0;i<iBitCount;i++)
  {
    TRACE(_T("% 5i "),pBits[i]);
  }
  TRACE(_T("\n"));
#endif
  return true;
}

/********************** CzMark::SetBitsFromColor **********************/
/// Bits extrahieren
/** 
**********************************************************************
  \param pColor     Farbarray
  \param iCirc      Arraygröße
  \param pBits      Bitarray
  \param iBitCount  Bitanzahl
  \param iStart     Startindex der ersten Kante
**********************************************************************
  \return true ok, false fehler
**********************************************************************
  \author reindlj
  \date 20.01.2010
***********************************************************************/
bool CzMark::SetBitsFromColor(int *pColor, int iCirc, BOOL *pBits, int iBitCount, int iStart)
{
  if(!pColor || !pBits) return false;
  double dDist= GetDist(iCirc,iBitCount);
  int iBitNr = 0;//, iAct = 0;
  for(double dI=(double)iStart+dDist/2.0;dI<(iCirc+iStart+dDist/2.0) && iBitNr<iBitCount;dI+=dDist)
  { 
    int i = (int)(dI+0.5);
    pBits[iBitNr++] = GetBit(pColor,iCirc,i,(int)(dDist/4+0.5));
    if(iBitNr>iBitCount)
      break;
  }
#ifdef DEBUG_MARK
  for(int i=0;i<iBitCount;i++)
  {
    TRACE(_T("% 5i "),pBits[i]);
  }
  TRACE(_T("\n"));
#endif
  return true;
}

/*************************** CzMark::FindMark ***************************/
/// Marke im Bild suchen
/** 
**********************************************************************
  \param dMidX           Mittelpunkt der Marke (X) in Pixel
  \param dMidY           Mittelpunkt der Marke (Y) in Pixel
  \param dOutRadius      Äusserer Radius der Marke in Pixel
  \param dSize           Breite der Marke (des Bitcode-Feldes) in Pixel
  \param dInnerRadius    Innerster Radius der Marke in Pixel
  \param dMidRadius      Mittlerer Radius der Marke in Pixel
  \param dContrastFactor Kontrastfaktor 0-1
**********************************************************************
  \return true Ok, false Fehler
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
bool CzMark::FindMark(double dMidX, double dMidY, double dOutRadius, double dSize, double dInnerRadius, double dMidRadius, double dContrastFactor)
{
  if(m_iBit == 0) return false;
  bool bRet = true;
  double dX, dY, dRStart = dOutRadius-dSize, dR;
  memset(m_pBits,0,sizeof(BOOL)*m_iBit);
  //int iCirc = (int)(2.0*PI*dOutRadius+0.5);
  // zu viele abtastpunkte? mal mit der hälfte probieren...
  int iCirc = (int)(PI*dOutRadius+0.5);
#ifdef DEBUG_MARK
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("%5u "),i);
  }
  TRACE(_T("\n"));
#endif
  int *pThres = new int[iCirc];
  int *pEdges = new int[iCirc];
  memset(pThres,0,sizeof(int)*iCirc);
  memset(pEdges,0,sizeof(int)*iCirc);
  BYTE *pLine = new BYTE[iCirc];
  int *pDiff = new int[iCirc];
  int iMid = 0;
  int *pBright = new int[iCirc];
  memset(pBright,0,sizeof(int)*iCirc);
  for(int k=0;k<(int)dSize;k++)
  {
    // jeden radius durchgehen
    dR = dRStart + (double)k;
    for(int i=0;i<iCirc;i++)
    { // und für jeden radius einmal im kreis und die pixel extrahieren
      double dVal = (double)i*360.0/iCirc/180.0*PI;
      dX = dMidX + cos(dVal)*dR;
      dY = dMidY - m_iDirection*sin(dVal)*dR;
      pLine[i] = GetPixel((int)(dX+0.5),(int)(dY+0.5));
      pBright[i] += pLine[i];
      // pixel der mitte aufaddieren zu kontrastbestimmung
      dX = dMidX + cos(dVal)*(double)k;
      dY = dMidY - m_iDirection*sin(dVal)*(double)k;
      iMid += GetPixel((int)(dX+0.5),(int)(dY+0.5));
    }
    for(int i=0;i<iCirc;i++)
    { // die differenzen der pixel berechnen
      if(i>0)
        pDiff[i] = pLine[i]-pLine[i-1];
      else
        pDiff[0] = pLine[0]-pLine[iCirc-1];
      // und aufaddieren
      pThres[i] += pDiff[i];
    }
  }
  delete [] pDiff;
  delete [] pLine;
#ifdef DEBUG_MARK
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("% 5i "),pThres[i]);
  }
  TRACE(_T("\n"));
  TRACE(_T("Brightness of Pixels: mid: %i, line:\n"),iMid);
  for(int i=0;i<iCirc;i++)
  {
    TRACE(_T("% 5i "),pBright[i]);
  }
  TRACE(_T("\n"));
#endif
  if(!GetEdges(pThres,pEdges,iCirc)) bRet = false;
  int iStart = GetStart(pEdges,iCirc,m_iBit);
  int *pColor = new int[iCirc];
  SetColor(pEdges,pColor,iCirc);
  // helligkeiten ermitteln
  double dMid = iMid, d1 = 0.0, d0 = 0.0;
  int i1Count = 0, i0Count = 0;
  for(int i=0;i<iCirc;i++)
  {
    if(pColor[i] == 1)
    {
      d1 += pBright[i];
      i1Count += (int)dSize;
    }
    else if(pColor[i] == -1)
    {
      d0 += pBright[i];
      i0Count += (int)dSize;
    }
    else
      ASSERT(false);
  }
  
  // 14.02.2011 Vermeidung von Division durch 0, falls keine codierte Marke erkannt wurde
  if(i1Count == 0 || i0Count == 0) return false; 
  
  // aus den helligkeiten den kontrast berechnen
  dMid /= (double)((int)dSize*iCirc);
  d1 /= (double)i1Count;
  d0 /= (double)i0Count;
//  TRACE(_T("dMid: %f, d1: %f, d0: %f\n"),dMid,d1,d0);
  if(m_iType == FM_TYPE_WB)
  {
    double dDiff1 = d1-d0;
    double dDiff2 = dMid-d0;
//    TRACE(_T("FM_TYPE_WB -> %f < %f * %f = %f ?\n"),dDiff1,dDiff2,dContrastFactor,dDiff2*dContrastFactor);
    if(dDiff1 < dDiff2*dContrastFactor) 
      bRet = false; // marke ist zu kontrastarm
  }
  else
  {
/*  double dDiff1 = d0-d1;
    double dDiff2 = d0-dMid;
    14.09.2010 GO: Ermittlung der Differenz war zuvor falsch!! */
    double dDiff1 = d1-d0;
    double dDiff2 = d1-dMid;
//    TRACE(_T("FM_TYPE_BW -> %f < %f * %f = %f ?\n"),dDiff1,dDiff2,dContrastFactor,dDiff2*dContrastFactor);
    if(dDiff1 < dDiff2*dContrastFactor) 
      bRet = false; // marke ist zu kontrastarm
  }
  if(bRet) bRet &= SetBitsFromColor(pColor,iCirc,m_pBits,m_iBit,iStart);
  //if(!SetBitsFromEdge(pEdges,iCirc,m_pBits,m_iBit)) return false;

  delete [] pColor;
  delete [] pThres;
  delete [] pEdges;
  delete [] pBright;
  return bRet;
}

/*************************** CzMark::GetBit ***************************/
/// Bit aus den Kanteninformationen extrahieren
/** Das Bit wird ermittelt anhand des Wertes der Kante (-1 oder 1)
    in einem Toleranzbereich von iDist.
**********************************************************************
  \param *pArray  Kantenarray oder Farbarray
  \param iCirc    Länge des Kantenarrays
  \param iPos     Position (0 - FM_PRECISION)
  \param iDist    Toleranzbereich (*2+1)
**********************************************************************
  \return 0, 1 oder -1 für Fehler
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
int CzMark::GetBit(int *pArray, int iCirc, int iPos, int iDist)
{
  int iBit = 0;
  for(int i=-iDist;i<=iDist;i++)
  {
    int iAct = i+iPos;
    if(iAct<0) iAct += iCirc;
    else if(iAct>=iCirc) iAct -= iCirc;
    if(pArray[iAct]>0) // weiss -> schwarz
      iBit += (m_iType == FM_TYPE_WB) ? 1 : -1; 
    else if(pArray[iAct]<0)  // schwarz -> weiss
      iBit += (m_iType == FM_TYPE_WB) ? -1 : 1; 
  }
  if(iBit <= -1)
    iBit = 0;
  else if(iBit >= 1)
    iBit = 1;
  else
    iBit = -1;
  return iBit;
}

/*************************** CzMark::GetMark ***************************/
/// Erkannte Marke zurückgeben
/** 
**********************************************************************
  \param pBits  Zeiger auf DWORD für die Bits 1.bit = höchstwertiges
**********************************************************************
  \return String mit den Bits
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
/*CString CzMark::GetMark(DWORD *pBits)
{
  CString sBits;
  if(pBits) *pBits = 0;
  for(int i=0;i<m_iBit;i++)
  {
    if(pBits)
      *pBits += m_pBits[i]<<(m_iBit-i-1);
    sBits += m_pBits[i] ? _T("1") : _T("0");
  }
  return sBits;
}*/

void CzMark::GetMark(DWORD *pBits)
{
//  CString sBits;
  if(pBits) *pBits = 0;
  for(int i=0;i<m_iBit;i++)
  {
    if(pBits)
      *pBits += m_pBits[i]<<(m_iBit-i-1);
//    sBits += m_pBits[i] ? _T("1") : _T("0");
  }
//  return sBits;
}

/************************* CzMark::IdentifyMark *************************/
/// Marke identifizieren
/** Wird kein Bitmuster übergeben (dwBits=0), dann wird die Marke des Objekts
    verwendet.
**********************************************************************
  \param dwBits Marke (0 für Marke des Objekts)
**********************************************************************
  \return Nummer der Marke
**********************************************************************
  \author reindlj
  \date 13.10.2009
***********************************************************************/
int CzMark::IdentifyMark(DWORD dwBits)
{
  if(dwBits==0)
    GetMark(&dwBits);
  switch(m_iBit)
  {
  case FM_BITCODE12:  return giCodes_12bit[dwBits];
  case FM_BITCODE14:  return giCodes_14bit[dwBits];
  case FM_BITCODE20:  return giCodes_20bit[dwBits];
  }
  return 0;
}

} // namespace RTE
