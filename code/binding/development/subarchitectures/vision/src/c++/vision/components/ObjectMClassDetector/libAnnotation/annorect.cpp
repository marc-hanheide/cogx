
#include <libAnnotation/annorect.h>
#include <libAnnotation/xmlhelpers.h>
#include <cmath>

using namespace std;

//////////////////////////////////////////////////////////////////////////////
///
///
///   AnnoRect
///
///
//////////////////////////////////////////////////////////////////////////////

// AnnoRect::AnnoRect(const AnnoRect& other)
// {
//   m_x1 = other.m_x1;
//   m_y1 = other.m_y1;
//   m_x2 = other.m_x2;
//   m_y2 = other.m_y2;
//    
//   m_dScore = other.m_dScore;
//   m_nSilhouetteID = other.m_nSilhouetteID;
//   
//   m_vArticulations.clear();
//   for (unsigned i=0; i<other.noArticulations(); i++)
//   {
//     this->addArticulation(other.articulation(i));
//   }
//   m_vViewPoints.clear();
//   for (unsigned i=0; i<other.noViewPoints(); i++)
//   {
//     this->addViewPoint(other.viewPoint(i));
//   }
//   
// }


void AnnoRect::parseXML(const string& rectString)
{
  //cerr << "AnnoRect::parse()"<< endl;
  vector<string> tmp;
  tmp = getElements("x1", rectString);
  if (tmp.size()>0)
    m_x1 = getElementDataInt("x1", tmp[0]);
  tmp = getElements("x2", rectString);
  if (tmp.size()>0)
    m_x2 = getElementDataInt("x2", tmp[0]);
  tmp = getElements("y1", rectString);
  if (tmp.size()>0)
    m_y1 = getElementDataInt("y1", tmp[0]);
  tmp = getElements("y2", rectString);
  if (tmp.size()>0)
    m_y2 = getElementDataInt("y2", tmp[0]);

  tmp = getElements("score", rectString);
  if (tmp.size()>0)
    m_dScore = getElementDataFloat("score", tmp[0]);

  tmp = getElements("silhouette", rectString);
  if (tmp.size()>0)
  {
    tmp = getElements("id", tmp[0]);
    for(unsigned i=0; i<tmp.size(); i++)
      m_nSilhouetteID = getElementDataInt("id", tmp[0]);
  }

  tmp = getElements("articulation", rectString);
  if (tmp.size()>0)
  {
    tmp = getElements("id", tmp[0]);
    for(unsigned i=0; i<tmp.size(); i++)
      m_vArticulations.push_back(getElementDataInt("id", tmp[i]));
  }

  tmp = getElements("viewpoint", rectString);
  if (tmp.size()>0)
  {
    tmp = getElements("id", tmp[0]);
    for(unsigned i=0; i<tmp.size(); i++)
      m_vViewPoints.push_back(getElementDataInt("id", tmp[i]));
  }

  //printXML();

}

void AnnoRect::parseIDL(const string& rectString)
{
  string::size_type start=1, end;
  end = rectString.find(",", start);
  m_x1 = atoi(rectString.substr(start, end-start).c_str());
  //cout << m_x1 << endl;
  start=end+1;
  end = rectString.find(",", start);
  m_y1 = atoi(rectString.substr(start, end-start).c_str());
  //cout << m_y1 << endl;
  start=end+1;
  end = rectString.find(",", start);
  m_x2 = atoi(rectString.substr(start, end-start).c_str());
  //cout << m_x2 << endl;
  start=end+1;
  end = rectString.find("):", start);
  m_y2 = atoi(rectString.substr(start, end-start).c_str());
  //cout << m_y2 << endl;

  start = end+2;
  end = rectString.find("/", start);
  m_dScore = atof(rectString.substr(start, end-start).c_str());
  //cout << m_dScore << endl;

  if (end==string::npos)
    m_nSilhouetteID=-1;
  else
  {
    start = end+1;
    m_nSilhouetteID = atoi(rectString.substr(start,string::npos).c_str());
  }
  //cout << m_nSilhouetteID << endl;

  //printXML();
}

void AnnoRect::writeXML(ofstream& out) const
{
  out << "    <annorect>\n";
  out << "        <x1>" << m_x1 << "</x1>\n";
  out << "        <y1>" << m_y1 << "</y1>\n";
  out << "        <x2>" << m_x2 << "</x2>\n";
  out << "        <y2>" << m_y2 << "</y2>\n";
  if (m_dScore!=-1)
    out << "        <score>" << m_dScore << "</score>\n";
  if (m_nSilhouetteID!=-1)
  {
    out << "        <silhouette>\n";
    out << "            <id>"<< m_nSilhouetteID << "</id>\n";
    out << "        </silhouette>\n";
  }
  if (m_vArticulations.size()>0)
  {
    out << "        <articulation>\n";
    vector<int>::const_iterator it;
    for(it=m_vArticulations.begin(); it!=m_vArticulations.end(); it++)
      out << "            <id>"<< *it << "</id>\n";
    out << "        </articulation>\n";
  }
  if (m_vViewPoints.size()>0)
  {
    out << "        <viewpoint>\n";
    vector<int>::const_iterator it;
    for(it=m_vViewPoints.begin(); it!=m_vViewPoints.end(); it++)
      out << "            <id>"<< *it << "</id>\n";
    out << "        </viewpoint>\n";
  }
  out << "    </annorect>\n";
}


void AnnoRect::printXML() const
{
  cout << "    <annorect>\n";
  cout << "        <x1>" << m_x1 << "</x1>\n";
  cout << "        <y1>" << m_y1 << "</y1>\n";
  cout << "        <x2>" << m_x2 << "</x2>\n";
  cout << "        <y2>" << m_y2 << "</y2>\n";
  if (m_dScore!=-1)
    cout << "        <score>" << m_dScore << "</score>\n";
  if (m_nSilhouetteID!=-1)
  {
    cout << "        <silhouette>\n";
    cout << "            <id>"<< m_nSilhouetteID << "</id>\n";
    cout << "        </silhouette>\n";
  }
  if (m_vArticulations.size()>0)
  {
    cout << "        <articulation>\n";
    vector<int>::const_iterator it;
    for(it=m_vArticulations.begin(); it!=m_vArticulations.end(); it++)
      cout << "            <id>"<< *it << "</id>\n";
    cout << "        </articulation>\n";
  }
  if (m_vViewPoints.size()>0)
  {
    cout << "        <viewpoint>\n";
    vector<int>::const_iterator it;
    for(it=m_vViewPoints.begin(); it!=m_vViewPoints.end(); it++)
      cout << "            <id>"<< *it << "</id>\n";
    cout << "        </viewpoint>\n";
  }
  cout << "    </annorect>\n";
}

void AnnoRect::writeIDL(ofstream& out) const
{
  if (m_nSilhouetteID==-1)
    out << "(" << m_x1 <<", " << m_y1 << ", " << m_x2 << ", " << m_y2 << "):" << m_dScore;
  else
    out << "(" << m_x1 <<", " << m_y1 << ", " << m_x2 << ", " << m_y2 << "):" << m_dScore <<"/" << m_nSilhouetteID;
}

void AnnoRect::printIDL() const
{
  if (m_nSilhouetteID==-1)
    cout << "(" << m_x1 <<", " << m_y1 << ", " << m_x2 << ", " << m_y2 << "):" << m_dScore;
  else
    cout << "(" << m_x1 <<", " << m_y1 << ", " << m_x2 << ", " << m_y2 << "):" << m_dScore <<"/" << m_nSilhouetteID;
}

void AnnoRect::sortCoords()
{
  int tmp;
  if (m_x1>m_x2)
  {
    tmp = m_x1;
    m_x1= m_x2;
    m_x2= tmp;
  }
  if (m_y1>m_y2)
  {
    tmp = m_y1;
    m_y1= m_y2;
    m_y2= tmp;
  }
}

double AnnoRect::compCover( const AnnoRect& other ) const
{
  AnnoRect r1 = *this;
  AnnoRect r2 = other;
  r1.sortCoords();
  r2.sortCoords();

  int nWidth  = r1.x2() - r1.x1();
  int nHeight = r1.y2() - r1.y1();
  int iWidth  = max(0,min(max(0,r2.x2()-r1.x1()),nWidth )-max(0,r2.x1()-r1.x1()));
  int iHeight = max(0,min(max(0,r2.y2()-r1.y1()),nHeight)-max(0,r2.y1()-r1.y1()));
  return ((double)iWidth * (double)iHeight)/((double)nWidth * (double)nHeight);
}

double AnnoRect::compRelDist( const AnnoRect& other, float dAspectRatio, FixDimType eFixObjDim ) const
{
  double dWidth, dHeight;

  switch( eFixObjDim )
  {
  case FIX_OBJWIDTH:
    dWidth  = m_x2 - m_x1;
    dHeight = dWidth / dAspectRatio;
    break;

  case FIX_OBJHEIGHT:
    dHeight = m_y2 - m_y1;
    dWidth  = dHeight * dAspectRatio;
    break;

  default:
    cerr << "Error in ImgDescrList::compRelDist(): "
    << "Unknown type for parameter ('which obj dimension to fix?'): "
    << eFixObjDim << "!" << endl;
    return -1.0;
  }

  double xdist = (double)(m_x1+m_x2-other.x1()-other.x2()) / dWidth;
  double ydist = (double)(m_y1+m_y2-other.y1()-other.y2()) / dHeight;
  return sqrt(xdist*xdist + ydist*ydist);
}


double AnnoRect::compRelDist( const AnnoRect& other ) const
{
  double dWidth  = m_x2 - m_x1;
  double dHeight = m_y2 - m_y1;
  double xdist   = (double)(m_x1 + m_x2 - other.x1() - other.x2()) / dWidth;
  double ydist   = (double)(m_y1 + m_y2 - other.y1() - other.y2()) / dHeight;
  return sqrt(xdist*xdist + ydist*ydist);
}


// bool AnnoRect::isMatching( const AnnoRect& other, double dTDist, double dTCover, double dTOverlap, float dAspectRatio, FixDimType eFixObjDim )
// {
//   double dWidth, dHeight;
//
//   switch( eFixObjDim ) {
//   case FIX_OBJWIDTH:
//     dWidth  = m_x2 - m_x1;
//     dHeight = dWidth / dAspectRatio;
//     break;
//
//   case FIX_OBJHEIGHT:
//     dHeight = m_y2 - m_y1;
//     dWidth  = dHeight * dAspectRatio;
//     break;
//
//   default:
//     cerr << "Error in ImgDescrList::compRelDist(): "
//          << "Unknown type for parameter ('which obj dimension to fix?'): "
//          << eFixObjDim << "!" << endl;
//     return -1.0;
//   }
//
//   double xdist = (double)(m_x1+m_x2-other.x1()-other.x2()) / dWidth;
//   double ydist = (double)(m_y1+m_y2-other.y1()-other.y2()) / dHeight;
//   return sqrt(xdist*xdist + ydist*ydist);
// }


bool AnnoRect::isMatching( const AnnoRect& other, double dTDist, double dTCover, double dTOverlap) const
{
  return ( (compRelDist(other) <= dTDist) &&
           (compCover(other) >= dTCover) &&
           (other.compCover(*this) >= dTOverlap) );
}

float AnnoRect::compOverlap(const AnnoRect& other) const {
	int nWidth = this->w();  
	int nHeight = this->h();
	
	int iWidth  = std::max(0, std::min(std::max(0, other.right()- this->left() ), nWidth ) - std::max(0, other.left() - this->left()));
	int iHeight = std::max(0, std::min(std::max(0, other.bottom() - this->top() ), nHeight ) - std::max(0, other.top() - this->top()));
	
	int interSection = iWidth * iHeight;
	int union12 = this->w() * this->h() + other.w() * other.h() - interSection;
	float overlap = interSection * 1.0 / union12;
	
	return overlap;
}

bool AnnoRect::isMatchingPascal( const AnnoRect& other, double minOverlap) const {	
	if (compOverlap(other) >= minOverlap)
		return true;
	else
		return false;
}


