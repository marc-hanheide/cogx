/**
 * $Id$
 */


#include "EllPatternDetector.hh"

#define DEBUG


namespace P 
{

static const bool CmpNbPatternPts(const cv::Ptr<EllPattern>& a, const cv::Ptr<EllPattern>& b)
{
  return (a->imgPoints.size() > b->imgPoints.size());
}



/************************************************************************************
 * Constructor/Destructor
 */

EllPatternDetector::EllPatternDetector(Parameter p)
 : param(p)
{ 

  elldetector = new RTE::EDWrapper(RTE::EDWrapper::Parameter(true,true,true,false,true));
}

EllPatternDetector::~EllPatternDetector()
{
}

/**
 * Insert an ellipse to the pattern
 */
void EllPatternDetector::Insert(EllPattern &pattern, int xIdx, int yIdx, RTE::CzEllipse &ell)
{
  pattern.imgPoints.push_back(cv::Point2f(ell.dX,ell.dY));    //x1
  pattern.objPoints.push_back(cv::Point3f(param.xDist*xIdx,param.yDist*yIdx,0.));
  pattern.location.push_back(make_pair(xIdx,yIdx));
}


/**
 *  Detect coordinate system(s)  
 */
void EllPatternDetector::DetectCoordSys(vector<RTE::CzEllipse> &whiteElls, vector<RTE::CzEllipse> &blackElls, vector<pair<unsigned, unsigned> > &rings, vector<cv::Ptr<EllPattern> > &patterns)
{
  patterns.clear();

  cv::Point2f center;
  unsigned idx1, idx2;
  double min1, min2, dist, sa, sb;
  cv::Ptr<EllPattern> pat;

  for (unsigned i=0; i<rings.size(); i++)
  {
    RTE::CzEllipse &we = whiteElls[rings[i].first];
    RTE::CzEllipse &be = blackElls[rings[i].second];
    center.x = (we.dX+be.dX) / 2.;
    center.y = (we.dY+be.dY) / 2.;

    pat = new EllPattern(center, (we.dA/be.dA+we.dB/be.dB)/2.);
    
    idx1 = idx2 = UINT_MAX;
    min1 = min2 = DBL_MAX;
    for (unsigned j=0; j<whiteElls.size(); j++)
    {
      dist = PVec::DistanceSqr2(&center.x, &whiteElls[j].dX); 
      if (dist > 5.)         // to avoid finding the origin ellipse
      {
        if (dist<min1)
        {
          min2 = min1;
          min1 = dist;
          idx2 = idx1;
          idx1 = j;
        }
        else if (dist<min2)
        {
          min2 = dist;
          idx2 = j;
        }
      }
    }

    if (idx1!=UINT_MAX && idx2!=UINT_MAX)
    {
      cv::Point2d d, x_to_c;
      PVec::Sub2(&whiteElls[idx1].dX, &whiteElls[idx2].dX, &d.x);
      PVec::Sub2(&center.x,&whiteElls[idx1].dX, &x_to_c.x);
      PVec::Normalise2(&d.x,&d.x);

      if (PVec::Cross2(&x_to_c.x, &d.x) < 0.)    // correct orientation
      {
        Insert(*pat, 1,0,whiteElls[idx1]);
        Insert(*pat, 0,1,whiteElls[idx2]); 
        pat->ids[0] = whiteElls[idx1].iMarkerID;
        pat->ids[1] = whiteElls[idx2].iMarkerID;        
      }
      else                                            // wrong orientation
      {
        Insert(*pat, 1,0,whiteElls[idx2]);
        Insert(*pat, 0,1,whiteElls[idx1]); 
        pat->ids[0] = whiteElls[idx2].iMarkerID;
        pat->ids[1] = whiteElls[idx1].iMarkerID;        
      }
      patterns.push_back(pat);
    }
  }
}

/**
 * InitGraph()
 */
void EllPatternDetector::InitGraph(EllPattern &pat)
{
  int xSize=abs(param.xMin)+abs(param.xMax)+1;
  int ySize=abs(param.yMin)+abs(param.yMax)+1;
  
  graph.clear();
  graph.resize(ySize, vector<EPNode>(xSize) );

  for (int v=param.yMin; v<=param.yMax; v++)
  {
    for (int u=param.xMin; u<=param.xMax; u++)
    {
      EPNode &n = GetNode(u,v);
      n.pos = cv::Point3f(u*param.xDist, v*param.yDist, 0.);
    }
  }

  GetNode(0,0).pt = pat.imgPoints[0];
  GetNode(1,0).pt = pat.imgPoints[1]; 
  GetNode(0,1).pt = pat.imgPoints[2];
}

/**
 * Mark location in graph
 */
void EllPatternDetector::Mark(int x, int y)
{
  GetNode(x,y).nb = 1;
}

/**
 * IsLine2
 */
bool EllPatternDetector::IsLine2(float p1[2], float p2[2], float p3[2])
{
  float d[2], p_to_q[2];
  PVec::Sub2(p2,p3,d);
  PVec::Normalise2(d,d);
  PVec::Sub2(p1, p2, p_to_q);
  return (fabs(PVec::Cross2(p_to_q, d)) < 1.);
}

/**
 * Get the model to predict next location 
 */
bool EllPatternDetector::GetModel(int x, int y, float H[9])
{
  vector<cv::Point2f> pts;
  vector<cv::Point3f> pos;
  for (int v=y-1; v<=y+1; v++)
  {
    for (int u=x-1; u<=x+1; u++)
    {
      if (u>=param.xMin && u<=param.xMax && v>=param.yMin && v<=param.yMax && GetNode(u,v).nb==1)
      {
        bool ok=true;
        if (pos.size()==2)
        {
          if(IsLine2(&pos[0].x, &pos[1].x, &GetNode(u,v).pos.x))
            ok=false;
        }
        if (ok)
        {
          pos.push_back(GetNode(u,v).pos);
          pts.push_back(GetNode(u,v).pt);
          if (pts.size()>=3)
            break;
        }
      }
    }
    if (pts.size()>=3)
      break;
  }
  if (pts.size()>=3)
  {
    if (PHom::ComputeAff(&pos[0].x, &pos[1].x, &pos[2].x, &pts[0].x, &pts[1].x, &pts[2].x, H))
      return true;
  }

  return false;
}

/**
 * InsertEllipseAt
 */
bool EllPatternDetector::InsertEllipseAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, float H[9])
{
  unsigned idx=UINT_MAX;
  double dist, minDist=DBL_MAX;
  cv::Point2f pt;
  EPNode &p = GetNode(x,y);
  PHom::MapPoint(&p.pos.x, H, &pt.x);
  #ifdef DEBUG
  cv::circle(dbg, pt, 2, CV_RGB(255,0,0), 2);
  #endif

  for (unsigned i=0; i<ells.size(); i++)
  {
    dist = PVec::DistanceSqr2(&ells[i].dX, &pt.x);
    if (dist < minDist)
    {
      idx = i;
      minDist = dist;
    }
  }

  if (idx!=UINT_MAX && minDist < sqrDist)
  {
    p.pt = cv::Point2f(ells[idx].dX,ells[idx].dY);
    Insert(pat, x,y, ells[idx]);
    Mark(x,y);
    return true;
  }

  return false;
}

/**
 * Detect ellipses at position
 */
void EllPatternDetector::Look4EllsAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat)
{
  float H[9];
  if(GetModel(x, y, H))
  {
    for (int v=y-1; v<=y+1; v++)
    {
      for (int u=x-1; u<=x+1; u++)
      {
        if (u>=param.xMin && u<=param.xMax && v>=param.yMin && v<=param.yMax && GetNode(u,v).nb==0)
          InsertEllipseAt(u, v, ells, pat, H);
      }
    }
  }
}

/**
 * AddSupportingEllipses
 */
void EllPatternDetector::AddSupportingEllipses(EllPattern &pat, vector<RTE::CzEllipse> &ells)
{
  // init the pattern and and 3d ellipse locations
  InitGraph(pat);
  sqrDist = param.dist*param.dist;

  // mark inserted locations
  Mark(0,0);
  Mark(1,0);
  Mark(0,1);

  // search for ellipses at location x,y
  bool ok=true;
  int u,v,xMin=0, xMax=0, yMin=0, yMax=0;
  while(ok)
  {
    for (u=xMin,v=yMin; u<=xMax; u++)
      Look4EllsAt(u,v, ells, pat);
    for (u=xMax,v=yMin; v<=yMax; v++)
      Look4EllsAt(u,v, ells, pat);
    for (u=xMax,v=yMax; u>=xMin; u--)
      Look4EllsAt(u,v, ells, pat);
    for (u=xMin,v=yMax; v>=yMin; v--)
      Look4EllsAt(u,v, ells, pat);

    ok = false;
    if (xMin>param.xMin) {ok=true;xMin--;}
    if (xMax<param.xMax) {ok=true;xMax++;}
    if (yMin>param.yMin) {ok=true;yMin--;}
    if (yMax<param.yMax) {ok=true;yMax++;}
  }
}



/******************************* PUBLIC **************************************
 * Detect calibration pattern with ellipses
 */
void EllPatternDetector::Detect(const cv::Mat &img, vector<cv::Ptr<EllPattern> > &patterns)
{
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  #ifdef DEBUG
  elldetector->dbg = dbg;
  #endif

  // detect the ellipses
  elldetector->Detect(grayImage,whiteElls, blackElls, rings);

  // detect coordinate system hypotheses
  DetectCoordSys(whiteElls, blackElls, rings, patterns);
  
  // insert supporting ellipses
  for (unsigned i=0; i<patterns.size(); i++)
  {
    AddSupportingEllipses(*patterns[i], blackElls);
  }

  sort(patterns.begin(),patterns.end(),CmpNbPatternPts);

  #ifdef DEBUG
  cout<<"Num. found patterns: "<<patterns.size()<<" ( ";
  for (unsigned i=0; i<patterns.size(); i++)
  {
    EllPattern &p = *patterns[i];
    if (!dbg.empty())
    {
      for (unsigned i=0; i<p.imgPoints.size(); i++)
        cv::circle(dbg,p.imgPoints[i], 2, CV_RGB(0,255,0), 2);
      cv::line(dbg, p.imgPoints[0], p.imgPoints[1], CV_RGB(255,0,0), 2);
      cv::line(dbg, p.imgPoints[0], p.imgPoints[2], CV_RGB(0,255,0), 2);
    }
    cout<<p.imgPoints.size()<<" ";
  }
  cout<<")"<<endl;
  #endif
}

/** 
 * Draw a pattern
 */
void EllPatternDetector::Draw(cv::Mat &img, EllPattern &p)
{
  if (p.imgPoints.size()<3)
    return;

  for (unsigned i=0; i<p.imgPoints.size(); i++)
    cv::circle(img,p.imgPoints[i], 2, CV_RGB(0,0,255), 2);

  cv::line(img, p.imgPoints[0], p.imgPoints[1], CV_RGB(255,0,0), 2);
  cv::line(img, p.imgPoints[0], p.imgPoints[2], CV_RGB(0,255,0), 2);

  if (p.ids[0] != -1)
  {
      std::ostringstream sin;
      sin << p.ids[0];
      std::string val = sin.str();
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[1].x + 18,p.imgPoints[1].y+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[1].x + 22,p.imgPoints[1].y+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[1].x + 22,p.imgPoints[1].y+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[1].x + 18,p.imgPoints[1].y+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[1].x + 20,p.imgPoints[1].y+20), cv::FONT_HERSHEY_DUPLEX, .8, CV_RGB(255,255,255));
  }
  if (p.ids[1] != -1) 
  {
      std::ostringstream sin;
      sin << p.ids[1];
      std::string val = sin.str();
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[2].x + 18,p.imgPoints[2].y+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[2].x + 22,p.imgPoints[2].y+18), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[2].x + 22,p.imgPoints[2].y+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[2].x + 18,p.imgPoints[2].y+22), cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(0.0));
      cv::putText(img, val.c_str(), cvPoint(p.imgPoints[2].x + 20,p.imgPoints[2].y+20), cv::FONT_HERSHEY_DUPLEX, .8, CV_RGB(255,255,255));
  }
}




}











