/**
 * $Id$
 */


#include "SelectMatchesMRF.hh"




namespace P 
{



/********************** SelectMatchesMRF ************************
 * Constructor/Destructor
 */
SelectMatchesMRF::SelectMatchesMRF(Parameter p)
 : param(p)
{
  INV_SQR_SIGMA2 = 1./(2*PMath::Sqr(param.sigmaMotDiff));
}

SelectMatchesMRF::~SelectMatchesMRF()
{
}


/**
 * Create delaunay neighbourhood graph
 */
void SelectMatchesMRF::CreateGraph(std::vector<PKeypointMRF> &keys)
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
    keys[i].ReleaseLinks(keys);
    pt = cvPoint2D32f((float)(keys[i].pt.x), (float)(keys[i].pt.y));
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
          keys[ptOrg->id].InsertLink(keys, ptDst->id);
          #ifdef DEBUG
          //if (!dbg.empty()) cv::line(dbg, keys[ptOrg->id].pt,keys[ptDst->id].pt, CV_RGB(0,0,0));
          #endif
        }
      }

      CV_NEXT_SEQ_ELEM( elem_size, reader );
  }

  cvReleaseMemStorage( &storage );
}

/**
 * InitKeypoints
 */
void SelectMatchesMRF::InitKeypoints(const std::vector<cv::KeyPoint> &queryKeys, std::vector<PKeypointMRF> &mrfKeys)
{
  mrfKeys.resize(queryKeys.size());

  for (unsigned i=0; i<mrfKeys.size(); i++)
  {
    mrfKeys[i] = queryKeys[i];
    mrfKeys[i].id = i;
  }
}

/**
 * InitKeypoints
 */
void SelectMatchesMRF::InitKeypoints(const std::vector<cv::Ptr<PKeypoint> > &queryKeys, std::vector<PKeypointMRF> &mrfKeys)
{
  mrfKeys.resize(queryKeys.size());

  for (unsigned i=0; i<mrfKeys.size(); i++)
  {
    mrfKeys[i] = *queryKeys[i];
    mrfKeys[i].id = i;
  }
}

/**
 * InitCosts
 */
void SelectMatchesMRF::InitCosts(const std::vector<std::vector<cv::DMatch> > &matches, std::vector<PKeypointMRF> &mrfKeys)
{
  std::set<unsigned>::iterator it;

  CreateGraph(mrfKeys);

  for (unsigned i=0; i<mrfKeys.size(); i++)
  {
    PKeypointMRF &key = mrfKeys[i];

    key.cost.resize(matches[i].size(),1);
//    for (unsigned j=0; j<key.cost.size(); j++)     //TODO: base cost function -> desc. dist.?
//      key.cost[j] = 1./matches[i][j].distance;
    key.tmpCost = key.cost;
  }  
}

/**
 * Msg
 */
void SelectMatchesMRF::Msg(PKeypointMRF &key, std::vector<PKeypointMRF> &keys, const std::vector<cv::Ptr<PKeypoint> > &trainKeys, const std::vector<std::vector<cv::DMatch> > &matches)
{
  unsigned idx;
  double prob, cost, minCost, sqrMin, mot1[2], mot2[2];
  std::set<unsigned>::iterator it;

  for (unsigned i=0; i<key.tmpCost.size(); i++)
  {
    key.tmpCost[i] = 0.;
    PKeypointMRF &k1 = keys[matches[key.id][i].queryIdx];
    PVec::Sub2(&key.pt.x, &trainKeys[matches[key.id][i].trainIdx]->pt.x, mot1);

    for (it = key.links.begin(); it!=key.links.end(); it++)
    {
      minCost = 0;
      
      for (unsigned j=0; j<matches[*it].size(); j++)
      {
        PVec::Sub2(&keys[matches[*it][j].queryIdx].pt.x,
                   &trainKeys[matches[*it][j].trainIdx]->pt.x, mot2);
        prob =  exp( -(PVec::DistanceSqr2(mot1,mot2)*INV_SQR_SIGMA2) );
        cost = prob * keys[*it].cost[j];

        if (cost > minCost)
        {
          minCost = cost;
          idx = j;
        }
      }
      key.tmpCost[i] += minCost;
    }
  }
}



/******************************* PUBLIC ******************************/
/**
 * Operate
 */
void SelectMatchesMRF::Operate(const std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
        const std::vector<cv::Ptr<PKeypoint> > &trainKeys, const std::vector<std::vector<cv::DMatch> > &matches, 
        std::vector<int> &selected)
{
  selected.clear();
  if (queryKeys.size()!=matches.size())
    return;

  std::vector<PKeypointMRF> mrfKeys;
  InitKeypoints(queryKeys, mrfKeys);
  InitCosts(matches, mrfKeys);

  // belief propagation
  for (unsigned i=0; i<param.numIter; i++)
  {
    for (unsigned j=0; j<mrfKeys.size(); j++)
      Msg(mrfKeys[j], mrfKeys, trainKeys, matches);
    for (unsigned j=0; j<mrfKeys.size(); j++)
      mrfKeys[j].cost = mrfKeys[j].tmpCost;
  }

  // copy back result
  selected.resize(matches.size());
  float min;
  unsigned idx;
  for (unsigned i=0; i<selected.size(); i++)
  {
    min=-FLT_MAX;
    PKeypointMRF &key = mrfKeys[i];
    for (unsigned j=0; j<key.cost.size(); j++)
    {
      if (key.cost[j] > min)
      {
        min = key.cost[j];
        idx = j;
      }
    }
    if (key.cost[idx] > param.baseCost)
      selected[i] = idx;
    else selected[i] = -1;
  }
}

/**
 * Operate
 */
void SelectMatchesMRF::Operate(const std::vector<cv::KeyPoint> &queryKeys, 
        const std::vector<cv::KeyPoint> &trainKeys, 
        const std::vector<std::vector<cv::DMatch> > &matches, std::vector<int> &selected)
{
  selected.clear();
  if (queryKeys.size()!=matches.size())
    return;

  std::vector<PKeypointMRF> mrfKeys;
  std::vector<cv::Ptr<PKeypoint> > pTrainKeys(trainKeys.size());

  InitKeypoints(queryKeys, mrfKeys);
  InitCosts(matches, mrfKeys);

  for (unsigned i=0; i<trainKeys.size(); i++)
    pTrainKeys[i] = new PKeypoint(trainKeys[i]);

  // belief propagation
  for (unsigned i=0; i<param.numIter; i++)
  {
    for (unsigned j=0; j<mrfKeys.size(); j++)
      Msg(mrfKeys[j], mrfKeys, pTrainKeys, matches);
    for (unsigned j=0; j<mrfKeys.size(); j++)
      mrfKeys[j].cost = mrfKeys[j].tmpCost;
  }

  // copy back result
  selected.resize(matches.size());
  float min;
  unsigned idx;
  for (unsigned i=0; i<selected.size(); i++)
  {
    min=-FLT_MAX;
    PKeypointMRF &key = mrfKeys[i];
    for (unsigned j=0; j<key.cost.size(); j++)
    {
      if (key.cost[j] > min)
      {
        min = key.cost[j];
        idx = j;
      }
    }
    if (key.cost[idx] > param.baseCost)
      selected[i] = idx;
    else selected[i] = -1;
  }
}



}





