/**
 * $Id$
 */


#include "arGTPatternDetector.hh"

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
arGTPatternDetector::arGTPatternDetector(const string &_pat1, const string &_pat2, Parameter p1, Parameter p2)
{ 
  param[0] = p1, param[1] = p2;
  pat[0] = _pat1, pat[1] = _pat2; 

  ids[0] = ardetector.loadMarker(_pat1);
  ids[1] = ardetector.loadMarker(_pat2);
  
  if (ids[0] < 0 || ids[1] < 0)
    throw runtime_error ("arGTPatternDetector: pattern 1/2 not found!");

  elldetector = new RTE::EDWrapper(RTE::EDWrapper::Parameter(false,true,false,false,false));
}

arGTPatternDetector::~arGTPatternDetector()
{
}

/**
 * Insert an ellipse to the pattern
 */
void arGTPatternDetector::Insert(EllPattern &pattern, int xIdx, int yIdx, RTE::CzEllipse &ell, Parameter &param)
{
  pattern.imgPoints.push_back(cv::Point2f(ell.dX,ell.dY));    //x1
  pattern.objPoints.push_back(cv::Point3f(param.xDist*(xIdx+param.xOffs),param.yDist*(yIdx+param.yOffs),0.));
  pattern.location.push_back(make_pair(xIdx,yIdx));
}

/**
 *  Detect coordinate system(s)  
 */
void arGTPatternDetector::DetectCoordSys(const cv::Mat &img, vector<cv::Ptr<EllPattern> > &patterns)
{
  patterns.clear();
  cv::Ptr<EllPattern> pat;
  cv::Point3f c3;
  float xDist, yDist;

  std::vector<V4R::Marker> marker = ardetector.detectMarker(img);

  for (unsigned i=0; i<marker.size(); i++)
  {
    V4R::Marker &m = marker[i];
    pat = new EllPattern();
    Parameter *p=0;

    #ifdef DEBUG
    if (!dbg.empty())
    {
      V4R::MarkerHdl markerHdl(&marker[i]);
      IplImage oImg(dbg);
      markerHdl.set(&oImg);
      markerHdl.drawCenter(cvScalar(0,0,255,0), 10);
      markerHdl.drawVertex(cvScalar(255,0,0,0));
      markerHdl.drawId(cvScalar(255,255,255,0));
      if ( marker[i].id < 0) {
         markerHdl.drawLines(cvScalar(255,255,255,0));
      } else {
         markerHdl.drawLines(cvScalar(0,255,0,0));
      }
    }
    #endif


    if (m.id==ids[0])
      { p = &param[0]; pat->ids[0]=0; }
    else if (m.id==ids[1])
      { p = &param[1]; pat->ids[0]=1; }

    if (p!=0)
    {
      xDist = p->xDist*2.;
      yDist = p->yDist*2.;
      c3 = cv::Point3f(p->xDist*p->xOffs-(p->xDist/2.), p->yDist*p->yOffs-(p->yDist/2.), 0.);

      pat->imgPoints.push_back(cv::Point2f(m.vertex[2][0], m.vertex[2][1]));    // 0 0
      pat->objPoints.push_back(c3);
      pat->location.push_back(make_pair(0,0));

      pat->imgPoints.push_back(cv::Point2f(m.vertex[1][0], m.vertex[1][1]));    // 1 0
      pat->objPoints.push_back(cv::Point3f(c3.x+xDist,c3.y,c3.z));
      pat->location.push_back(make_pair(1,0));
 
      pat->imgPoints.push_back(cv::Point2f(m.vertex[3][0], m.vertex[3][1]));    // 0 1
      pat->objPoints.push_back(cv::Point3f(c3.x,c3.y+yDist,c3.z));
      pat->location.push_back(make_pair(0,1));


      pat->imgPoints.push_back(cv::Point2f(m.vertex[0][0], m.vertex[0][1]));    // 1 1
      pat->objPoints.push_back(cv::Point3f(c3.x+xDist,c3.y+yDist,c3.z));
      pat->location.push_back(make_pair(1,1)); 
  
      patterns.push_back(pat);
    }
  }
}

/**
 * InitGraph()
 */
void arGTPatternDetector::InitGraph(EllPattern &pat, Parameter &param)
{
  int xSize=abs(param.xMin)+abs(param.xMax)+1;
  int ySize=abs(param.yMin)+abs(param.yMax)+1;
  
  graph.clear();
  graph.resize(ySize, vector<EPNode>(xSize) );

  for (int v=param.yMin; v<=param.yMax; v++)
  {
    for (int u=param.xMin; u<=param.xMax; u++)
    {
      EPNode &n = GetNode(u,v, param);
      n.pos = cv::Point3f((u+param.xOffs)*param.xDist, (v+param.yOffs)*param.yDist, 0.);
    }
  }

  GetNode(0,0,param).pt = pat.imgPoints[0];
  GetNode(0,0,param).pos = pat.objPoints[0];  
  GetNode(1,0,param).pt = pat.imgPoints[1]; 
  GetNode(1,0,param).pos = pat.objPoints[1];
  GetNode(0,1,param).pt = pat.imgPoints[2];
  GetNode(0,1,param).pos = pat.objPoints[2];
  GetNode(1,1,param).pt = pat.imgPoints[3];
  GetNode(1,1,param).pos = pat.objPoints[3];
}

/**
 * Mark location in graph
 */
void arGTPatternDetector::Mark(int x, int y, Parameter &param)
{
  GetNode(x,y,param).nb = 1;
}

/**
 * IsLine2
 */
bool arGTPatternDetector::IsLine2(float p1[2], float p2[2], float p3[2])
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
bool arGTPatternDetector::GetModel(int x, int y, float H[9], Parameter &param)
{
  vector<cv::Point2f> pts;
  vector<cv::Point3f> pos;
  for (int v=y-1; v<=y+1; v++)
  {
    for (int u=x-1; u<=x+1; u++)
    {
      if (u>=param.xMin && u<=param.xMax && v>=param.yMin && v<=param.yMax && GetNode(u,v,param).nb==1)
      {
        bool ok=true;
        if (pos.size()==2)
        {
          if(IsLine2(&pos[0].x, &pos[1].x, &GetNode(u,v,param).pos.x))
            ok=false;
        }
        if (ok)
        {
          pos.push_back(GetNode(u,v,param).pos);
          pts.push_back(GetNode(u,v,param).pt);
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
bool arGTPatternDetector::InsertEllipseAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, float H[9],Parameter &param)
{
  unsigned idx=UINT_MAX;
  double dist, minDist=DBL_MAX;
  cv::Point2f pt;
  EPNode &p = GetNode(x,y,param);
  PHom::MapPoint(&p.pos.x, H, &pt.x);
  #ifdef DEBUG
  //cv::circle(dbg, pt, 2, CV_RGB(255,0,0), 2);
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
    Insert(pat, x,y, ells[idx],param);
    Mark(x,y, param);
    return true;
  }

  return false;
}

/**
 * Detect ellipses at position
 */
void arGTPatternDetector::Look4EllsAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, Parameter &param)
{
  float H[9];
  haveModel |= GetModel(x, y, H, param);
  //if(GetModel(x, y, H, param))
  if (haveModel);
  {
    for (int v=y-1; v<=y+1; v++)
    {
      for (int u=x-1; u<=x+1; u++)
      {
        if (u>=param.xMin && u<=param.xMax && v>=param.yMin && v<=param.yMax && GetNode(u,v,param).nb==0)
          InsertEllipseAt(u, v, ells, pat, H, param);
      }
    }
  }
}

/**
 * AddSupportingEllipses
 */
void arGTPatternDetector::AddSupportingEllipses(EllPattern &pat, vector<RTE::CzEllipse> &ells, Parameter &param)
{
  // init the pattern and and 3d ellipse locations
  InitGraph(pat, param);
  sqrDist = param.dist*param.dist;

  // mark inserted locations
  Mark(0,0,param);
  Mark(1,0,param);
  Mark(0,1,param);
  Mark(1,1,param);


  // search for ellipses at location x,y
  bool ok=true;
  int u,v,xMin=0, xMax=0, yMin=0, yMax=0;
  haveModel=false;
  while(ok)
  {
    for (u=xMin,v=yMin; u<=xMax; u++)
      Look4EllsAt(u,v, ells, pat, param);
    for (u=xMax,v=yMin; v<=yMax; v++)
      Look4EllsAt(u,v, ells, pat, param);
    for (u=xMax,v=yMax; u>=xMin; u--)
      Look4EllsAt(u,v, ells, pat, param);
    for (u=xMin,v=yMax; v>=yMin; v--)
      Look4EllsAt(u,v, ells, pat, param);

    ok = false;
    if (xMin>param.xMin) {ok=true;xMin--;}
    if (xMax<param.xMax) {ok=true;xMax++;}
    if (yMin>param.yMin) {ok=true;yMin--;}
    if (yMax<param.yMax) {ok=true;yMax++;}
  }
}

/**
 * Use homography to filter outliers
 */
void arGTPatternDetector::FilterHom(const EllPattern &src, const Parameter &param, EllPattern &dst)
{
  vector<uchar> mask;
  vector<cv::Point2f> objPts;
  dst = EllPattern();

  for (unsigned i=0; i<src.objPoints.size(); i++)
    objPts.push_back(cv::Point2f(src.objPoints[i].x, src.objPoints[i].y));

  cv::Mat H = findHomography(cv::Mat(objPts), cv::Mat(src.imgPoints), mask, CV_LMEDS, param.distH);

  unsigned cnt=0;
  for (unsigned i=0; i<mask.size(); i++)
    if (mask[i]>0 || i<3)
    {
      dst.objPoints.push_back(src.objPoints[i]);
      dst.imgPoints.push_back(src.imgPoints[i]);
      dst.location.push_back(src.location[i]);
      cnt++;
    }

  dst.ids[0] = src.ids[0];
  dst.ids[1] = src.ids[1];
  dst.quotientRing = src.quotientRing;
  
  #ifdef DEBUG
  //cout<<"src.objPoints.size()="<<src.objPoints.size()<<"->"<<cnt<<", outl="<<src.objPoints.size()-cnt<<endl;
  #endif
}


/******************************* PUBLIC **************************************
 * Detect calibration pattern with ellipses
 */
bool arGTPatternDetector::Detect(const cv::Mat &img, EllPattern &pattern)
{
  vector<cv::Ptr<EllPattern> > patterns;

  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  #ifdef DEBUG
  elldetector->dbg = dbg;
  #endif

  // detect the ellipses
  elldetector->Detect(grayImage,whiteElls, blackElls, rings);

  // detect coordinate system hypotheses
  DetectCoordSys(img, patterns);
 
  // insert supporting ellipses
  for (unsigned i=0; i<patterns.size(); i++)
  {
    AddSupportingEllipses(*patterns[i], blackElls, param[patterns[i]->ids[0]]);
  }

  if (patterns.size() > 0)
  {
    sort(patterns.begin(),patterns.end(),CmpNbPatternPts);

    if (param[patterns[0]->ids[0]].filterHom)
      FilterHom(*patterns[0], param[patterns[0]->ids[0]], pattern);
    else
      pattern = *patterns[0];

    #ifdef DEBUG
    if (!dbg.empty())
    {
      for (unsigned i=0; i<pattern.imgPoints.size(); i++)
        cv::circle(dbg,pattern.imgPoints[i], 2, CV_RGB(0,255,0), 2);
      cv::line(dbg, pattern.imgPoints[0], pattern.imgPoints[1], CV_RGB(255,0,0), 2);
      cv::line(dbg, pattern.imgPoints[0], pattern.imgPoints[2], CV_RGB(0,255,0), 2);
    }
    #endif

    return true;
  }
  else
  {
    pattern = EllPattern();
  }

  return false;
}

/** 
 * Draw a pattern
 */
void arGTPatternDetector::Draw(cv::Mat &img, EllPattern &p)
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












