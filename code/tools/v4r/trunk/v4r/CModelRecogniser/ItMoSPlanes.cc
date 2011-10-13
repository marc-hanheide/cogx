/**
 * $Id$
 * Iterative model selection for plane detection 
 *
 * TODO:
 * - create graph 
 *   + horizontal (detect between images), 
 *   + vertical (merge planes depending on homography for boundary points OR fundamental matrix)
 * - graph cut
 * - reasoning, occlusion?
 * ? reconstruct and bundle
 *
 */


#include "ItMoSPlanes.hh"

#define DEBUG


namespace P 
{



/********************** ItMoSPlanes ************************
 * Constructor/Destructor
 */
ItMoSPlanes::ItMoSPlanes(cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                         Parameter _param)
 : param(_param)
{
  matcher = descMatcher;
  sqrInlDist = param.inlDist*param.inlDist;
  selmat = new SelectMatchesMRF(SelectMatchesMRF::Parameter(param.width, param.height));

  CONST_ERR1 = 1. / (param.sigmaError*sqrt(2*M_PI));
  CONST_ERR2 = 1. / (2 * param.sigmaError*param.sigmaError);
}

ItMoSPlanes::~ItMoSPlanes()
{
}




/************************** PRIVATE ************************/

/**
 * get median motion of moving keypoints
 */
double ItMoSPlanes::GetMedianMot(const vector<cv::Ptr<P::PKeypoint> > &keys, const vector<cv::Ptr<P::PKeypoint> > &svKeys, vector<vector<cv::DMatch> > &matches, vector<int> &selected)
{
  double dist;
  vector<float> dists;
 
  for(unsigned i=0; i<matches.size(); i++)
  {
    if (selected[i]>=0)
    {
      dist = PVec::Distance2(&keys[matches[i][selected[i]].queryIdx]->pt.x, &svKeys[matches[i][selected[i]].trainIdx]->pt.x);

      if (dist > 1)
      {
        dists.push_back(dist);
      }
    }
  }

  if (dists.size()>2)
  {
    sort(dists.begin(), dists.end());
    return dists[dists.size()/2];
  }

  return DBL_MAX;
}


/**
 * Create delaunay neighbourhood graph
 */
void ItMoSPlanes::CreateGraph(vector<cv::Ptr<P::PKeypoint> > &keys)
{
  // init
  CvPoint2D32f pt;
  CvMemStorage* storage;
  CvSubdiv2D* subdiv;

  storage = cvCreateMemStorage(0);
  subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv), sizeof(CvSubdiv2DPoint),
                             sizeof(CvQuadEdge2D), storage );
  cvInitSubdivDelaunay2D( subdiv, cvRect(0,0,param.width+10, param.height+10));

  // insert points
  for (unsigned i=0; i<keys.size(); i++)
  {
    keys[i]->ReleaseLinks();
    pt = cvPoint2D32f((float)(keys[i]->pt.x), (float)(keys[i]->pt.y));
    cvSubdivDelaunay2DInsert( subdiv, pt )->id = i;
  }

  // create graph
  CvSeqReader  reader;
  CvSubdiv2DPoint* ptOrg;
  CvSubdiv2DPoint* ptDst;
  int i, total = subdiv->edges->total;
  int elem_size = subdiv->edges->elem_size;

  //cvCalcSubdivVoronoi2D( subdiv );
  cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

  for( i = 0; i < total; i++ )
  {
      CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

      if( CV_IS_SET_ELEM( edge ))
      {
        //draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );
        ptOrg = cvSubdiv2DEdgeOrg((CvSubdiv2DEdge)edge);
        ptDst = cvSubdiv2DEdgeDst((CvSubdiv2DEdge)edge); 
        if (ptOrg && ptDst && ptOrg->id!=-1 && ptDst->id!=-1)
        {
          keys[ptOrg->id]->InsertLink(*keys[ptDst->id]);
          #ifdef DEBUG
          //if (!dbg.empty()) cv::line(dbg, keys[ptOrg->id]->pt,keys[ptDst->id]->pt, CV_RGB(0,0,0));
          #endif
        }
      }

      CV_NEXT_SEQ_ELEM( elem_size, reader );
  }

  cvReleaseMemStorage( &storage );
}


/**
 * CountExplainedKeys
 */
int ItMoSPlanes::CountExplainedKeys(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes, int &numKeys)
{
  int cnt=0;
  numKeys = 0;
  PKeypoint::nbcnt++;

  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    for (unsigned j=0; j<plane.keys.size(); j++)
      plane.keys[j]->nb = PKeypoint::nbcnt;
  }

  for (unsigned i=0; i<keys.size(); i++)
  {
    if (keys[i]->bw !=0)
    {
      numKeys++;
      if (keys[i]->nb == PKeypoint::nbcnt)
        cnt++;
    }
  }

  return cnt;
}

/**
 * SetTrackingLinks
 */
void ItMoSPlanes::SetTrackingLinks(vector<cv::Ptr<P::PKeypoint> > &queryKeys, vector<cv::Ptr<P::PKeypoint> > &trainKeys, vector<vector<cv::DMatch> > &matches, vector<int> &selected)
{
  for (unsigned i=0; i<matches.size(); i++)
  {
    if (selected[i]>=0)
    {
      cv::DMatch &ma = matches[i][selected[i]];
      if (queryKeys[ma.queryIdx]->bw==0)
      {
        queryKeys[ma.queryIdx]->InsertBW(*trainKeys[ma.trainIdx]);
        queryKeys[ma.queryIdx]->response = ma.distance;
      }
      else if (queryKeys[ma.queryIdx]->response > ma.distance)
      {
        queryKeys[ma.queryIdx]->InsertBW(*trainKeys[ma.trainIdx]);
        queryKeys[ma.queryIdx]->response = ma.distance;
      }
      else selected[i]=false;
    }
  }
}

/**
 * ClearFWBW
 */
void ItMoSPlanes::ClearFWBW(vector<cv::Ptr<P::PKeypoint> > &keys)
{
  for (unsigned i=0; i<keys.size(); i++)
  {
    keys[i]->ReleaseFW();
    keys[i]->ReleaseBW();
  }
}

/**
 * GetUnusedMatches
 */
void ItMoSPlanes::GetUnusedMatches(vector< cv::Ptr<Plane> > &planes, vector<cv::Ptr<P::PKeypoint> > &keys, vector<cv::Ptr<P::PKeypoint> > &unKeys)
{
  PKeypoint::nbcnt++;

  unKeys.clear();  
  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    for (unsigned j=0; j<plane.keys.size(); j++)
      plane.keys[j]->nb=PKeypoint::nbcnt;
  }

  for (unsigned i=0; i<keys.size(); i++)
    if (keys[i]->bw!=0 && keys[i]->nb!=PKeypoint::nbcnt)
      unKeys.push_back(keys[i]);
}

/**
 * GetNapKeys
 */
void ItMoSPlanes::GetNapIdx(vector<cv::Ptr<P::PKeypoint> > &keys, int num, vector<int> &idx)
{
  unsigned temp;
  vector<DistIdx> dist;
  idx.clear();

  if (keys.size()<num)
    return;

  do{
    temp = rand()%keys.size();
  }while(keys[temp]->bw==0);
  idx.push_back(temp);
  cv::Point2d &pt = keys[idx[0]]->pt;

  for (unsigned i=0; i<keys.size(); i++)
    if (i!=idx[0])
      dist.push_back(DistIdx(PVec::DistanceSqr2(&pt.x, &keys[i]->pt.x),i));

  sort(dist.begin(), dist.end(), CmpDistIdxAsc);

  for (unsigned i=0; i<num-1; i++)
  {
    do{
      temp = PMath::ExpSelect(dist.size()-1);
    }while(keys[dist[temp].idx]->bw==0 || Contains(idx, dist[temp].idx));
    idx.push_back(dist[temp].idx);
  }
}

/**
 * AccumulateKeys
 */
void ItMoSPlanes::AccumulateKeys(vector<cv::Ptr<P::PKeypoint> > &keys, PKeypoint &key, cv::Mat_<double> &H, vector<cv::Ptr<P::PKeypoint> > &planeKeys)
{
  cv::Point2d pt;
  set<PKeypoint*>::iterator it;

  for (it = key.links.begin(); it!= key.links.end(); it++)
  {
    if ((*it)->nb != key.nb && (*it)->bw!=0)
    {
      PHom::MapPoint(&(*it)->bw->pt.x, &H(0,0), &pt.x);
      if (PVec::DistanceSqr2(&(*it)->pt.x, &pt.x) < sqrInlDist)
      {
        (*it)->nb = key.nb;
        planeKeys.push_back(keys[(*it)->id]);
        AccumulateKeys(keys, (**it), H, planeKeys);
      }
    }
  }
}

/**
 * AddRandHypotheses
 */
#define NUM_RAND_POINTS 3
void ItMoSPlanes::AddRandHypotheses(vector<cv::Ptr<P::PKeypoint> > &keys, vector<cv::Ptr<P::PKeypoint> > &unKeys, vector< cv::Ptr<Plane> > &planes)
{
  if (unKeys.size()<NUM_RAND_POINTS)
    return;

  cv::Ptr<Plane> plane;
  vector<int> idx;
  int z;
  bool ok;

  for (int i=0, cnt=0; i<1000 && cnt<param.numRandHypotheses; i++)
  {
    plane = new Plane();
    plane->H = cv::Mat_<double>(3,3);
    z=0;

    do{
      GetNapIdx(unKeys, NUM_RAND_POINTS, idx);

      ok = PHom::ComputeAff(&unKeys[idx[0]]->bw->pt.x, &unKeys[idx[1]]->bw->pt.x, 
                            &unKeys[idx[2]]->bw->pt.x,
                            &unKeys[idx[0]]->pt.x, &unKeys[idx[1]]->pt.x, 
                            &unKeys[idx[2]]->pt.x, 
                            &plane->H(0,0));
      
      z++;
    }while((!ok) && z<1000);
   
    plane->keys.push_back(unKeys[idx[0]]);
    PKeypoint::nbcnt++;
    plane->keys[0]->nb = PKeypoint::nbcnt;

    AccumulateKeys(keys, *plane->keys[0], plane->H, plane->keys);

    if (plane->keys.size() > 4 && unKeys[idx[1]]->nb==PKeypoint::nbcnt && unKeys[idx[2]]->nb==PKeypoint::nbcnt)
    {
      planes.push_back(plane);
      cnt++;
    }
  }
}
#undef NUM_RAND_POINTS

/**
 * AddRefineHypotheses
 */
#define NUM_RAND_POINTS 3
void ItMoSPlanes::AddRefineHypotheses(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes)
{
  cv::Ptr<Plane> plane;
  vector<int> idx;
  int z;
  bool ok;
  unsigned size = planes.size();

  for (int i=0, cnt=0; i<1000 && cnt<param.numRandHypotheses; i++)
  {
    for (unsigned j=0; j<size; j++)
    {
      plane = new Plane();
      plane->H = cv::Mat_<double>(3,3);
      vector<cv::Ptr<P::PKeypoint> > &plKeys = planes[j]->keys;
      z=0;

      do{
        GetNapIdx(plKeys, NUM_RAND_POINTS, idx);

        ok = PHom::ComputeAff(&plKeys[idx[0]]->bw->pt.x, &plKeys[idx[1]]->bw->pt.x, 
                              &plKeys[idx[2]]->bw->pt.x,
                              &plKeys[idx[0]]->pt.x, &plKeys[idx[1]]->pt.x, 
                              &plKeys[idx[2]]->pt.x, 
                              &plane->H(0,0));
        
        z++;
      }while((!ok) && z<1000);
     
      plane->keys.push_back(plKeys[idx[0]]);
      PKeypoint::nbcnt++;
      plane->keys[0]->nb = PKeypoint::nbcnt;

      AccumulateKeys(keys, *plane->keys[0], plane->H, plane->keys);

      if (plane->keys.size() > 4 && plKeys[idx[1]]->nb==PKeypoint::nbcnt && plKeys[idx[2]]->nb==PKeypoint::nbcnt)
      {
        planes.push_back(plane);
        cnt++;
      }
    }
  }
}
#undef NUM_RAND_POINTS

/**
 * SetIds
 */
void ItMoSPlanes::SetIds(vector<cv::Ptr<P::PKeypoint> > &keys)
{
  for (unsigned i=0; i<keys.size(); i++)
    keys[i]->id = i;
}


/**
 * CalcLSHomography
 */
void ItMoSPlanes::CalcLSHomography(vector< cv::Ptr<Plane> > &planes, int method, bool filter)
{
  vector<uchar> mask;
  vector<cv::Point2f> pts1, pts2;
  vector<cv::Ptr<P::PKeypoint> > tmpKeys;
  
  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    pts1.clear(), pts2.clear();
    
    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      PKeypoint &key = *plane.keys[j];
      if (key.bw!=0)
      {
        pts1.push_back(cv::Point2f(key.bw->pt.x, key.bw->pt.y));
        pts2.push_back(cv::Point2f(key.pt.x, key.pt.y));
      }
    }

    plane.H = cv::findHomography(cv::Mat(pts1), cv::Mat(pts2), mask, method, param.inlDist);

    if (filter)
    {
      tmpKeys.clear();

      for (unsigned j=0; j<mask.size(); j++)
      {
        if (mask[j]>0)
          tmpKeys.push_back(plane.keys[j]);
      }

      if (tmpKeys.size()>4)
      {
        plane.keys = tmpKeys;
      }
      else
      {
        planes.erase(planes.begin()+i);
        i--;
      }
    }
  }
}

/**
 * AccumulateKeys
 */
void ItMoSPlanes::AccumulateKeys(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes)
{
  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    if (plane.keys.size()>0)
    {
      PKeypoint::nbcnt++;
      plane.keys[0]->nb = PKeypoint::nbcnt;
      plane.keys.erase(plane.keys.begin()+1,plane.keys.end());

      AccumulateKeys(keys, *plane.keys[0], plane.H, plane.keys);

      if (keys.size()<=4)
      {
        planes.erase(planes.begin()+i);
        i--;
      }
    }
  }
}

/**
 * Compute Gaussian error probability using Euclidean distance
 */
void ItMoSPlanes::GetErrProbEuc(double m1[2],double m2[2],double H[9], double err[1])
{
  cv::Point2d pt;

  PHom::MapPoint(m1, H, &pt.x);
  *err =  sqrt(PMath::Sqr(pt.x-m2[0]) + PMath::Sqr(pt.y-m2[1]));       // euc. distance
  *err = CONST_ERR1 * exp(-(*err * *err * CONST_ERR2));    // gaussian
}


/**
 * individual values
 */
void ItMoSPlanes::ComputeQii(Plane &pl, double &weight)
{
  double err;

  for (unsigned i=0; i<pl.keys.size(); i++)
  {
    PKeypoint &key = *pl.keys[i];
    if (key.bw!=0)
    {
      GetErrProbEuc(&key.bw->pt.x, &key.pt.x, &pl.H(0,0), &err);
      err = (1.-param.kappa2) + param.kappa2*err;

      weight += (err>0.?err:0.);
    }
  }

  if (weight > param.kappa1)
    weight -= param.kappa1;
  else
    weight = 0.;
}

/**
 * interaction value
 */
void ItMoSPlanes::ComputeQij(Plane &pi, Plane &pj, double &weight)
{
  double temp=0, wi, wj;
  PKeypoint::nbcnt++;
  double err;

  for (unsigned i=0; i<pi.keys.size(); i++)
    pi.keys[i]->nb = PKeypoint::nbcnt;

  for (unsigned i=0; i<pj.keys.size(); i++)
  {
    if (pj.keys[i]->nb == PKeypoint::nbcnt)
    {
      GetErrProbEuc(&pj.keys[i]->bw->pt.x, &pj.keys[i]->pt.x, &pi.H(0,0), &err);
      err = (1.-param.kappa2) + param.kappa2*err;
      wi = (err>0.?err:0.);

      GetErrProbEuc(&pj.keys[i]->bw->pt.x, &pj.keys[i]->pt.x, &pj.H(0,0), &err);
      err = (1.-param.kappa2) + param.kappa2*err;
      wj = (err>0.?err:0.);

      temp -= (wi<wj?wj:wi);
    }
  }

  weight+=(temp/2.);
}



/**
 * CreateSegmentationMatrixQ
 */
void ItMoSPlanes::CreateSegmentationMatrixQ(vector< cv::Ptr<Plane> > &planes, cv::Mat_<double> &Q)
{
  for (unsigned i=0; i<planes.size(); i++)
  {
    for (unsigned j=0; j<planes.size(); j++)
    {
      if (i==j)
        ComputeQii(*planes[i],Q(i,j));
      else
        ComputeQij(*planes[j], *planes[i], Q(i,j));
    }
  }

}

/**
 * GreedyQBP
 */
void ItMoSPlanes::GreedyQBP(cv::Mat_<double> &Q, cv::Mat_<double> &m)
{
  cv::Mat_<double> mt;
  m.copyTo(mt);
  double max;
  unsigned idx;
  bool finished = false;
  vector<double> s(Q.rows);
  cv::Mat_<double> v(1,Q.rows);
  cv::Mat_<double> tmp1(1,1);
  cv::Mat_<double> tmp2(1,1);

  do{
    for (int i=0; i<m.rows; i++)
      if (PMath::IsZero(m(i,0)))
      {
        m.copyTo(mt);
        mt(i,0) = 1.;
        cv::gemm(mt, Q, 1, cv::Mat(), 1, v, cv::GEMM_1_T);
        cv::gemm(v,mt, 1, cv::Mat(), 1, tmp1, 0 );
        cv::gemm( m, Q, 1, cv::Mat(), 1, v, cv::GEMM_1_T);
        cv::gemm(v, m, 1, cv::Mat(), 1, tmp2, 0);
        s[i] = tmp1(0,0) - tmp2(0,0);
      }

    max = -DBL_MAX;
    for (int i=0; i<m.rows; i++)
    {
      if (PMath::IsZero(m(i,0)))
      {
        if (s[i] > max)
        {
          max = s[i];
          idx = i;
        }
      }
    }
   if (max>0)
      m(idx,0) = 1.;
    else
      finished=true;
  }while(!finished);
}


/**
 * SelectPlanes
 */
void ItMoSPlanes::SelectPlanes(vector< cv::Ptr<Plane> > &in, vector< cv::Ptr<Plane> > &out)
{
  if (in.size()==0)
    return;

  vector< cv::Ptr<Plane> > tmpPlanes;

  cv::Mat_<double> Q = cv::Mat::zeros(in.size(), in.size(), CV_64F);
  cv::Mat_<double> m = cv::Mat::zeros(in.size(),1, CV_64F);

  CreateSegmentationMatrixQ(in, Q);
  GreedyQBP(Q,m);

  for (unsigned i=0; i<in.size(); i++)
  {
    if ( m(i,0) > .5 )
      tmpPlanes.push_back(in[i]);
  }

  out = tmpPlanes;
}


/**
 * CopyDescriptors
 */
void ItMoSPlanes::CopyDescriptors(const cv::Mat_<float> &descs, vector< cv::Ptr<Plane> > &planes)
{
  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    plane.descriptors = cv::Mat_<float>(plane.keys.size(),descs.cols);

    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      for (unsigned k=0; k<descs.cols; k++)
        plane.descriptors(j,k) = descs(plane.keys[j]->id,k);

      //plane.descriptors(j) = descs(plane.keys[j]->id)+0;
      //descs.row(plane.keys[j]->id).copyTo( plane.descriptors.row(j) );
    }
  }
}

/**
 * TrackPriorPlanes
 */
void ItMoSPlanes::TrackPriorPlanes(vector<cv::Ptr<P::PKeypoint> > &keys,
      const vector< cv::Ptr<Plane> > &priorPlanes, vector< cv::Ptr<Plane> > &tentPlanes)
{
  cv::Ptr<Plane> trPlane;
  tentPlanes.clear();

  for (unsigned i=0; i<priorPlanes.size(); i++)
  {
    const Plane &plane = *priorPlanes[i];
    trPlane = new Plane(plane.id);

    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      if (plane.keys[j]->fw != 0)
        trPlane->keys.push_back( keys[ plane.keys[j]->fw->id ] );
    }
    
    if (trPlane->keys.size()>4)
      tentPlanes.push_back(trPlane);
  }

  CalcLSHomography(tentPlanes, CV_RANSAC, true);
}

/**
 * Match keypoints and filter on descriptor distance and NNR
 */
void ItMoSPlanes::MatchKeysNNR(const cv::Mat_<float> &queryDescs, 
      const cv::Mat_<float> &trainDescs, vector<vector<cv::DMatch> > &matches, vector<int> &selectedMatches)
{
  matcher->knnMatch(queryDescs, trainDescs, matches, param.kMatches);
  selectedMatches.resize(matches.size());

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (matches[i].size()>=2)
    {
      if (matches[i][0].distance/matches[i][1].distance < param.nnRatio && 
          matches[i][0].distance<param.thrDesc) 
        selectedMatches[i]=0;
      else selectedMatches[i]=-1;  
    }
    else if (matches[i].size()==1 && matches[i][0].distance<param.thrDesc) 
        selectedMatches[i]=0;
      else selectedMatches[i]=-1;
  }
}

/**
 * SetPriorPlanes
 */
void ItMoSPlanes::SetPriorPlanes(vector< cv::Ptr<Plane> > &planes, vector< cv::Ptr<Plane> > &priorPlanes)
{
  cv::Ptr<Plane> pr;
  priorPlanes.clear();

  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &pl = *planes[i];
    pr = new Plane();

    if (pl.id==UINT_MAX)
      pr->id = pl.id = Plane::idcnt++;
    else
      pr->id = pl.id;

    pr->keys = pl.keys;

    priorPlanes.push_back(pr);
  }
}

/**
 * If points are assigned to more than one group assign it to the cluster
 * which minimizes the error
 */
void ItMoSPlanes::ReleaseInteractions(vector< cv::Ptr<Plane> > &planes)
{
  double err;
  cv::Point2d pt;
  vector<cv::Ptr<PKeypoint> > keys;

  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    for (unsigned j=0; j<plane.keys.size(); j++)
      plane.keys[j]->response = DBL_MAX;
  }

  for (unsigned i=0; i<planes.size(); i++)
  {
      Plane &plane = *planes[i];
      for (unsigned j=0; j<plane.keys.size(); j++)
      {
        PHom::MapPoint(&plane.keys[j]->bw->pt.x, &plane.H(0,0), &pt.x);
        err = PVec::DistanceSqr2(&pt.x, &plane.keys[j]->pt.x);
        if (err < plane.keys[j]->response)
        {
          plane.keys[j]->response = err;
          plane.keys[j]->nb2 = i;
        }
      }
  }
  for (unsigned i=0; i<planes.size(); i++)
  {
    keys.clear();
    Plane &plane = *planes[i];
    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      if (plane.keys[j]->nb2 == i)
        keys.push_back(plane.keys[j]);
    }
    plane.keys = keys;
  }
}

/**
 * GetSelectedKeys
 */
void ItMoSPlanes::GetSelectedQueryKeys(const vector<cv::Ptr<P::PKeypoint> > &queryKeys, 
      vector<vector<cv::DMatch> > &matches, vector<int> &selected, 
      vector<cv::Ptr<P::PKeypoint> > &selectedKeys)
{
  selectedKeys.clear();

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (selected[i]>=0)
    {
      if (queryKeys[matches[i][selected[i]].queryIdx]->bw!=0) 
        selectedKeys.push_back(queryKeys[matches[i][selected[i]].queryIdx]);
      else
        selected[i]=-1;
    }
  }
}

/**
 * ConstrainedMatching
 * (if testAll==true than outliers are removed depending on plane constraint 
 *  otherwise only not selected matches are tested)
 */
void ItMoSPlanes::ConstrainedMatching(vector<cv::Ptr<Plane> > &planes, 
      vector<cv::Ptr<P::PKeypoint> > &queryKeys, 
      vector<cv::Ptr<P::PKeypoint> > &trainKeys, vector<vector<cv::DMatch> > &matches, 
      vector<int> &selectedMatches, double inlDist, bool markOutlier)
{
  unsigned idx;
  double dist, minDist;
  cv::Point2d pt;
  cv::DMatch ma;
  inlDist = PMath::Sqr(inlDist);

  for (unsigned i=0; i<matches.size(); i++)
  {
    //if (!selectedMatches[i] || testAll)
    {
      idx = UINT_MAX;
      minDist = DBL_MAX;
      for (unsigned j=0; j<planes.size(); j++)
      {
        Plane &plane = *planes[j];
        for (unsigned k=0; k<matches[i].size(); k++)
        {
          PHom::MapPoint(&trainKeys[matches[i][k].trainIdx]->pt.x, &plane.H(0,0), &pt.x);
          dist = PVec::DistanceSqr2(&pt.x, &queryKeys[matches[i][k].queryIdx]->pt.x);
          if (dist<minDist)
          {
            idx=k;
            minDist=dist;
          }
        }
      }
      if (minDist<=inlDist && matches[i][idx].distance<param.thrDesc)
      {
        ma = matches[i][idx];
        queryKeys[ma.queryIdx]->InsertBW(*trainKeys[ma.trainIdx]);
        selectedMatches[i]=idx;
      }
      else if(idx!=UINT_MAX && markOutlier) selectedMatches[i]=-1;
    }
  }
}

/**
 * SetLastKeypoints
 */
void ItMoSPlanes::SetLastKeypoints(vector<cv::Ptr<P::PKeypoint> > &svKeys, vector< cv::Ptr<Plane> > &planes)
{
  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];
    plane.lastKeys.resize(plane.keys.size(),0);

    for (unsigned j=0; j<plane.keys.size(); j++)
      plane.lastKeys[j] = svKeys[plane.keys[j]->bw->id];

    MeanKeys(plane.keys, plane.center);
  }
}

/**
 * MeanKeys
 */
void ItMoSPlanes::MeanKeys(vector< cv::Ptr<P::PKeypoint> > &keys, cv::Point2d &ptMean)
{
  ptMean = cv::Point2d(0.,0.);
  for (unsigned i=0; i<keys.size(); i++)
  {
    ptMean += keys[i]->pt;
  }
  ptMean.x /= (double)keys.size();
  ptMean.y /= (double)keys.size();
}






/************************** PUBLIC *************************/

/**
 * Delete the models and clear the vocabulary tree
 */
void ItMoSPlanes::Clear()
{
  svKeys.clear();
  svDescs.release();
}


/**
 * Track interest points and detect planes (homographies)
 * based on iterative model selection
 */
bool ItMoSPlanes::Operate(const vector<cv::Ptr<P::PKeypoint> > &ks, const cv::Mat_<float> &descs, 
                          vector< cv::Ptr<Plane> > &planes)
{
  vector< cv::Ptr<Plane> > tentPlanes;
  PKeypoint::Copy(ks, keys);
  SetIds(keys);

  matches.clear();
  if (!descs.empty() && !svDescs.empty())
  {
    if (param.mrfMatchFilter)
    {
      matcher->knnMatch(descs, svDescs, matches, param.kMatches);
      selmat->Operate(keys, svKeys, matches, selectedMatches);
    }
    else MatchKeysNNR(descs, svDescs, matches, selectedMatches);
  }

  double mot = GetMedianMot(keys, svKeys, matches, selectedMatches);
  SetTrackingLinks(keys, svKeys, matches, selectedMatches);
  TrackPriorPlanes(keys, svPlanes, tentPlanes); 
  if (param.planeConstrainedMatching) 
    ConstrainedMatching(tentPlanes, keys, svKeys, matches, selectedMatches, param.maxMotion, true);
  GetSelectedQueryKeys(keys, matches, selectedMatches, selectedKeys);

  #ifdef DEBUG
  /*cout<<"selectedKeys.size()="<<selectedKeys.size()<<endl;
  if (!dbg.empty())
  {
    P::KeypointDetector::Draw(dbg, selectedKeys, CV_RGB(0,0,255), 2);
    for (unsigned i=0; i<selectedKeys.size();i++)
      cv::line(dbg, selectedKeys[i]->bw->pt, selectedKeys[i]->pt, CV_RGB(255,255,255));
  }*/
  cout<<"motion="<<mot<<" "<<(mot>param.minMotion)<<endl;
  #endif

  // create graph and compute planes
  if (mot > param.minMotion)
  {

    if (!PMath::IsEqual(mot,DBL_MAX))
    {
      #ifdef DEBUG
      cout<<"******************** DETECT PLANES *********************"<<endl;
      #endif
      double eps;
      int numKeys, k=0, inl, inls=0;
      vector< cv::Ptr<PKeypoint> > tmpKeys;
      srand(time(NULL));

      CreateGraph(selectedKeys);
    
      inls = inl = CountExplainedKeys(keys, tentPlanes, numKeys);
      eps = ((double)inl)/(double)numKeys;

      while(pow(1. - pow(eps,(4)),k) >= param.etaRansac && k<param.maxRansacIter)
      {
        GetUnusedMatches(tentPlanes, selectedKeys, tmpKeys);
        AddRefineHypotheses(keys, tentPlanes);
        AddRandHypotheses(keys, tmpKeys, tentPlanes);

        CalcLSHomography(tentPlanes, 0, false);
        AccumulateKeys(keys, tentPlanes);

        SelectPlanes(tentPlanes, tentPlanes);
       
        inl = CountExplainedKeys(selectedKeys, tentPlanes, numKeys);
        if (inl > inls)
        {
          inls = inl;
          eps = ((double)inls) / (double)numKeys;
        }
        k++;
      }

      //CalcLSHomography(tentPlanes, CV_LMEDS, false);

      /*ConstrainedMatching(tentPlanes, keys, svKeys, matches, selectedMatches, 1., true);
      SetTrackingLinks(keys, svKeys, matches, selectedMatches);
      GetSelectedQueryKeys(keys, matches, selectedMatches, selectedKeys);
      CreateGraph(selectedKeys);
      AccumulateKeys(keys, tentPlanes);
      SelectPlanes(tentPlanes, tentPlanes);
      for (unsigned i=0; i<selectedKeys.size();i++)
        cv::line(dbg, selectedKeys[i]->bw->pt, selectedKeys[i]->pt, CV_RGB(255,255,255));*/

      ReleaseInteractions(tentPlanes);
      CopyDescriptors(descs, tentPlanes);

      #ifdef DEBUG
      cout<<"k="<<k<<", inls="<<inls<<"/"<<numKeys<<endl;
      #endif
    }

    SetLastKeypoints(svKeys, tentPlanes);
    svKeys = keys;
    descs.copyTo(svDescs);
    SetPriorPlanes(tentPlanes, svPlanes);
  }

  //return planes
  planes = tentPlanes;
  ClearFWBW(svKeys);

  #ifdef DEBUG
  cout<<"Number of detected planes: "<<planes.size()<<endl;
  for (unsigned i=0; i<planes.size(); i++)
    cout<<"id="<<planes[i]->id<<": numOfKeys="<<planes[i]->keys.size()<<endl;
  cout<<"--"<<endl;
  #endif

  if (mot > param.minMotion)
    return true;
  return false;
}






/**
 * Draw
 */
static map<unsigned, CvScalar> cols;
void ItMoSPlanes::Draw(cv::Mat &image, vector< cv::Ptr<Plane> > &planes, unsigned detail)
{
  cv::Scalar col;
  /*cols[0] = CV_RGB(255,0,0);
  cols[1] = CV_RGB(0,255,0);
  cols[2] = CV_RGB(0,0,255);
  cols[3] = CV_RGB(255,255,0);
  cols[4] = CV_RGB(0,255,255);
  cols[5] = CV_RGB(255,0,255);
  cols[6] = CV_RGB(128,0,0);
  cols[7] = CV_RGB(0,128,0);
  cols[8] = CV_RGB(0,0,128);
  cols[9] = CV_RGB(128,128,0);
  cols[9] = CV_RGB(0,128,128);
  cols[9] = CV_RGB(128,0,128);*/

  string tag("id=");

  for (unsigned i=0; i<planes.size(); i++)
  {
    Plane &plane = *planes[i];

    /*map<unsigned,CvScalar>::iterator it = cols.find(plane.id);
    if( it == cols.end() )
    {
      col = CV_RGB(rand()%255,rand()%255,rand()%255);
      cols[plane.id] = col;
    }
    else col = cols[plane.id];*/

    col = CV_RGB(rand()%255,rand()%255,rand()%255);

    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      cv::circle(image, plane.keys[j]->pt, 2, col, 2);
      if (detail>1 && plane.lastKeys.size()==plane.keys.size())
        cv::line(image, plane.lastKeys[j]->pt, plane.keys[j]->pt, CV_RGB(255,255,255));
    }

    if (detail>0)
    {
      cv::putText(image, tag+toString(plane.id), plane.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,255,255), 2);
      cv::putText(image, tag+toString(plane.id), plane.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 1);
    }

    
    //cv::imshow("Image",image);
    //cv::waitKey(0);
  }
}


} //-- THE END --






