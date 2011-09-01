/**
 * $Id$
 * Johann Prankl, 20091118
 */


#include "LPSegment.hh"

#define DEBUG_SEG

#define LP_COL_SEG 128
#define LP_COL_FG 255
#define LP_COL_BG 0

#define LP_HEAP_THR 1000



namespace P 
{



/**
 * for sorting an array decreasing cost
 */
static int CmpCost(const void *a, const void *b)
{
  if ( (*(LPNode**)a)->T > (*(LPNode**)b)->T)
    return -1;  // a is first
  else
    return 1 ;  // b is first
}



/********************** LPSegment ************************
 * Constructor/Destructor
 */
LPSegment::LPSegment()
 : dbg(0), edge(0), dx(0), dy(0), cannyImg(0), nodes(0), expTab(0), expSize(0),
   prob(0), fg(0), bg(0), uvprob(0), uvfg(0), uvbg(0)

{
  cfg.useColourEdges = LP_USE_COLOUR_EDGES;
  cfg.useColourPrior = LP_USE_COLOUR_PRIOR;
  cfg.useWeightPrior = LP_USE_WEIGHT_PRIOR;
  cfg.useMask = LP_USE_MASK;
  cfg.magnitudeScale = LP_MAGNITUDE_SCALE;
  cfg.weightColour = LP_WEIGHT_COLOUR;
  cfg.weightPrior = LP_WEIGHT_PRIOR;
  cfg.sigmaMask = LP_SIGMA_MASK;
}

LPSegment::~LPSegment()
{
  if (edge!=0) cvReleaseImage(&edge);
  if (dx!=0) cvReleaseImage(&dx);
  if (dy!=0) cvReleaseImage(&dy);
  if (cannyImg!=0) cvReleaseImage(&cannyImg);
  if (nodes!=0) delete[] nodes;
  if (prob!=0) delete prob;
  if (fg!=0) delete fg;
  if (bg!=0) delete bg;
  if (uvprob!=0) delete uvprob;
  if (uvfg!=0) delete uvfg;
  if (uvbg!=0) delete uvbg;
}


/**
 * Create the graph
 */
void LPSegment::CreateGraph(IplImage *lp)
{
  if (nodes!=0)
    if (width!=lp->width || height!=lp->height)
    {
      delete[] nodes;
      nodes=0;
    }
  if (nodes==0) 
  {
    width = lp->width;
    height = lp->height;
    nodes = new LPNode[lp->width*lp->height];
  }

  //set body...
  float *d;
  LPNode *n;
  double max, min;
  cvMinMaxLoc( lp, &min, &max);
  #ifdef DEBUG_SEG
  cout << "max cost = "<<max<<endl;
  #endif
  for (int v=1; v<height-1; v++)
  {
    n = nodes + v*width;
    d = (float*)(lp->imageData + lp->widthStep*v);
    *n = LPNode(0,v);
    n->N.PushBack(n+1);
    n->N.PushBack(n-width);
    n->N.PushBack(n+width);
    n->cost = CostFkt(*d,max);
    n++, d++;
    for (int u=1; u<width-1; u++,n++, d++)
    {
      *n = LPNode(u,v);
      n->N.PushBack(n-1);
      n->N.PushBack(n+1);
      n->N.PushBack(n-width);
      n->N.PushBack(n+width);
      n->cost = CostFkt(*d,max);
    }
    *n = LPNode(width-1,v);
    n->N.PushBack(n-1);
    n->N.PushBack(n-width);
    n->N.PushBack(n+width);
    n->cost = CostFkt(*d,max);
  }
 
  //set start nodes... 
  s = LPNode(INT_MAX, INT_MAX, 0.);
  n = nodes;
  d = (float*)(lp->imageData);
  *n = LPNode(0,0);
  n->N.PushBack(n+1);
  n->N.PushBack(n+width);
  n->cost = CostFkt(*d,max);

  n++, d++;
  for (int i=1; i<width-1; i++, n++, d++)
  {
    *n = LPNode(i,0);
    n->N.PushBack(n-1);
    n->N.PushBack(n+1);
    n->N.PushBack(n+width);
    n->cost = CostFkt(*d,max);
    s.N.PushBack(n);
  }
  *n = LPNode(width-1,0);
  n->N.PushBack(n-1);
  n->N.PushBack(n+width);
  n->cost = CostFkt(*d,max);
  s.N.PushBack(n);

  //set target nodes...
  t = LPNode(INT_MAX, INT_MAX, 0.);
  n = nodes + (height-1)*width;
  d = (float*)(lp->imageData + lp->widthStep*(height-1));
  *n = LPNode(0,0);
  n->N.PushBack(n+1);
  n->N.PushBack(n-width);
  n->cost = CostFkt(*d,max);
  n++, d++;
  for (int i=1; i<width-1; i++, n++, d++)
  {
    *n = LPNode(i,width-1);
    n->N.PushBack(n-1);
    n->N.PushBack(n+1);
    n->N.PushBack(n-width);
    n->cost = CostFkt(*d,max);
    n->N.PushBack(&t);
  }
  *n = LPNode(width-1,height-1);
  n->N.PushBack(n-1);
  n->N.PushBack(n-width);
  n->cost = CostFkt(*d,max);
  n->N.PushBack(&t);
}

/**
 * Find path with minimal cost using Dijkstra algorithm
 */
void LPSegment::FindPath(LPNode *nodes, LPNode *s, LPNode *t)
{
  //Array - version
  s->T=0;
  s->e=true;
  LPNode *u = s;
  unsigned idx;
  P::Array<LPNode*> Q;
  Q.PushBack(s);
  double minT;
  double tmpCost;

  while (Q.Size()>0 && u!=t)
  {
    minT=DBL_MAX;
    for (unsigned i=0; i<Q.Size(); i++)
    {
      if(Q[i]->T < minT)
      {
        minT = Q[i]->T;
        idx = i;
      }
    }
    u = Q[idx];
    Q.Erase(idx);

    for (unsigned i=0; i<u->N.Size(); i++)
    {
      if (!u->N[i]->e)
      {
        u->N[i]->e = true;
        Q.PushBack(u->N[i]);
        #ifdef LP_HEAP_THR
        if (Q.Size()>LP_HEAP_THR) Q.EraseFirst();
        #endif
      }
      tmpCost = u->T + u->N[i]->cost;
      if (tmpCost < u->N[i]->T)
      {
        u->N[i]->T = tmpCost;
        u->N[i]->B = u;
      }
    }
  }
}


/**
 * Recursive 
 */
double LPSegment::GetLPContour(LPNode *s, LPNode *t, P::Array<P::Vector2> &contour)
{
  double totalCost=0;
  LPNode *u;
  contour.Clear();

  if (t->B!=0)
  {
    u = t->B;
    while (u != s)
    {
      totalCost+=u->T;
      contour.PushBack(P::Vector2(u->x,u->y));
      u = u->B;
    }
  }

  return totalCost;
}

/**
 * Map contour to cartesian space
 */
void LPSegment::LPContour2Cart(P::Array<P::Vector2> &contour, Vector2 center, Vector2 size, double M)
{
  for (unsigned i=0; i<contour.Size(); i++)
  {
    LogPolar2Cart(contour[i], center, size, cfg.magnitudeScale, contour[i]);
  }
}

/**
 * remove edges to source and to target node for testing two specific solutions
 */
void LPSegment::RemoveSTEdges(LPNode *nodes, int width, int height, LPNode *s, LPNode *t)
{
  //reset start nodes
  LPNode *n, *n_end;
  s->N.Clear();

  //set target nodes...
  n = nodes + (height-1)*width;
  n->N.Clear();
  n->N.PushBack(n+1);
  n->N.PushBack(n-width);
  n++;
  for (int i=1; i<width-1; i++, n++)
  {
    n->N.Clear();
    n->N.PushBack(n-1);
    n->N.PushBack(n+1);
    n->N.PushBack(n-width);
  }
  n->N.Clear();
  n->N.PushBack(n-1);
  n->N.PushBack(n-width);

  s->e=false; s->T=DBL_MAX; s->B=0;
  t->e=false; t->T=DBL_MAX; t->B=0; 
  n = nodes;
  n_end = nodes + width*height;
  for (;n!=n_end; n++)
  {
    n->e=false; n->T=DBL_MAX; n->B=0;
  }
}

/**
 * set one source and target edge
 */
void LPSegment::SetSTEdges(LPNode *nodes, int width, int height, LPNode *s, LPNode *t, int r)
{
  LPNode *n;

  s->N.PushBack(nodes+r);
  n = nodes + (height-1)*width + r;
 
  n->N.PushBack(t);
}

/**
 * Compute integral image
 */
void LPSegment::Integral(IplImage *img, IplImage *sum)
{
  if (img->depth == IPL_DEPTH_8U && sum->depth == IPL_DEPTH_32F)
  {
    uchar* d;
    float* s;
    
    for (int v=0; v<img->height; v++)
    {
      d = (uchar*)(img->imageData + img->widthStep*v);
      s = (float*)(sum->imageData + sum->widthStep*v);
      *s = *d;
      for (int u=1; u<img->width; u++, d++, s++)
      {
        *s = *(s-1) + *d;
      }
    }
  }
  else
  {
    cout<<"Integral image: Wrong image format!"<<endl;
  }
}



/**
 * Create the graph and search for optimal path
 */
double LPSegment::ProcessGraph(IplImage *costLP, P::Array<P::Vector2> &contour)
{
  double tCost1, tCost2;
  P::Array<P::Vector2> c1,c2;
  CreateGraph(costLP);  
  FindPath(nodes, &s, &t);
  tCost1 = GetLPContour(&s,&t, contour);
  #ifdef DEBUG_SEG
  SaveGraphVisited("log/graphVisitedRun0.jpg",nodes,costLP->width, costLP->height);
  #endif

  if (contour.Size()>0)
  {
    if (fabs(contour.First().x-contour.Last().x) > 1.)
    {
      RemoveSTEdges(nodes, width, height, &s, &t);
      SetSTEdges(nodes, width, height, &s, &t, contour.First().x);
      FindPath(nodes, &s, &t);
      tCost1 = GetLPContour(&s,&t, c1);
      #ifdef DEBUG_SEG
      SaveGraphVisited("log/graphVisitedRun0.jpg",nodes,costLP->width, costLP->height);
      #endif

      RemoveSTEdges(nodes, width, height, &s, &t);
      SetSTEdges(nodes, width, height, &s, &t, contour.Last().x);
      FindPath(nodes, &s, &t);
      tCost2 = GetLPContour(&s,&t, c2);
      #ifdef DEBUG_SEG
      SaveGraphVisited("log/graphVisitedRun0.jpg",nodes,costLP->width, costLP->height);
      #endif

      if (tCost1 < tCost2)
      {
        contour = c1;
        return tCost1;
      }
      else
      {
        contour = c2;
        return tCost2;
      }
    }
  }
  return tCost1;
}


/**
 * Compute bayes colour prior
 */
void LPSegment::ComputeRGBColourPrior(IplImage *img, IplImage *mask, IplImage *prior)
{
  IplImage *tmp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3 );
  cvSmooth(img, tmp, CV_BLUR, 7, 0, 0, 0 );

  uchar *m, *m_end, *d;
  
  if (bg==0) bg = new RGBHistogram();
  if (fg==0) fg = new RGBHistogram();
  if (prob==0) prob = new RGBColourProb();
  bg->Clear(); fg->Clear();

  for (int v=0; v<mask->height; v++)
  {
    m = (uchar*)(mask->imageData + mask->widthStep*v);
    d = (uchar*)(tmp->imageData + tmp->widthStep*v);
    m_end = m+mask->width;
    for (;m!=m_end; m++,d+=3)
    {
      if (*m == LP_COL_BG)
        bg->Add(d[0],d[1],d[2]);
      else if (*m == LP_COL_FG)
        fg->Add(d[0],d[1],d[2]);
    }
  }

  prob->ComputeProb(*bg, *fg);

  uchar *p;
  uchar *p_end;

  for (register int v=0; v<prior->height; v++)
  {
    d = (uchar*)(tmp->imageData + tmp->widthStep*v);
    p = (uchar*)(prior->imageData + prior->widthStep*v);
    p_end = p+prior->width;
    for (;p!=p_end; p++,d+=3)
    {
      *p = (uchar)(prob->Prob(d[0], d[1], d[2])*255);
    }
  }

  cvReleaseImage(&tmp);
}

/**
 * Compute bayes colour prior
 */
void LPSegment::ComputeUVColourPrior(IplImage *img, IplImage *mask, IplImage *prior)
{
  IplImage *tmp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3 );
  cvCvtColor( img,tmp, CV_BGR2YCrCb);
  cvSmooth(tmp, tmp, CV_BLUR, 7, 0, 0, 0 );

  if (uvbg==0) uvbg = new UVHistogram();
  if (uvfg==0) uvfg = new UVHistogram();
  if (uvprob==0) uvprob = new UVColourProb();
  uvbg->Clear(); uvfg->Clear();

  uchar *m, *m_end, *d;

  for (int v=0; v<mask->height; v++)
  {
    m = (uchar*)(mask->imageData + mask->widthStep*v);
    d = (uchar*)(tmp->imageData + tmp->widthStep*v);
    m_end = m+mask->width;
    for (;m!=m_end; m++,d+=3)
    {
      if (*m == LP_COL_BG)
        uvbg->Add(d[1],d[2]);
      else if (*m == LP_COL_FG)
        uvfg->Add(d[1],d[2]);
    }
  }

  uvprob->ComputeProb(*uvbg, *uvfg);

  uchar *p;
  uchar *p_end;

  for (register int v=0; v<prior->height; v++)
  {
    d = (uchar*)(tmp->imageData + tmp->widthStep*v);
    p = (uchar*)(prior->imageData + prior->widthStep*v);
    p_end = p+prior->width;
    for (;p!=p_end; p++,d+=3)
    {
      *p = (uchar)(uvprob->Prob(d[1], d[2])*255);
    }
  }

  cvReleaseImage(&tmp);
}

/**
 * SpanRectangle
 * spans a rectangle arround the polygon
 */
void LPSegment::SpanRectangle(Array<Vector2> &vs, Vector2 &ul, Vector2 &lr)
{
  ul.x=DBL_MAX;
  lr.x=DBL_MIN;
  ul.y=DBL_MAX;
  lr.y=DBL_MIN;

  for (unsigned i=0; i<vs.Size(); i++)
  {
    if (vs[i].x < ul.x) ul.x = vs[i].x;
    if (vs[i].x > lr.x) lr.x = vs[i].x;
    if (vs[i].y < ul.y) ul.y = vs[i].y;
    if (vs[i].y > lr.y) lr.y = vs[i].y;
  }
;
}

/**
 * add colour prior
 */
void LPSegment::AddColourPrior(IplImage *img, Array<Vector2> &contour, IplImage *edgeWeighted)
{
  Vector2 ul, lr;

  IplImage *mask = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1 );
  IplImage *prior = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1 );

  cvSetZero(mask);
  SpanRectangle(contour,ul,lr);
  SDraw::DrawRectangle(mask, ul.x,ul.y, lr.x,lr.y, cvScalar(LP_COL_SEG), CV_FILLED);
  SDraw:: DrawFillPoly(mask, contour, cvScalar(LP_COL_FG));
 
  if (LP_USE_RGB)
    ComputeRGBColourPrior(img, mask, prior);
  else
    ComputeUVColourPrior(img, mask, prior);

  #ifdef DEBUG_SEG
  cvSaveImage("log/colourPrior.jpg",prior);
  #endif

  AddPrior(edgeWeighted, prior, cfg.weightColour);

  cvReleaseImage(&mask);
  cvReleaseImage(&prior);
}

/**
 * add colour prior
 */
void LPSegment::AddPrior(IplImage *edgeWeighted, IplImage *prior, double weight)
{
  double e, norm, min,max;
  Line *l;

  cvSmooth(prior, prior, CV_BLUR, 3, 0, 0, 0 );
  cvMinMaxLoc(prior, &min, &max);
  norm = max-min;

  // use more accurate line orientation...
  for (unsigned i=0; i<lines.Size(); i++)
  {
    l = lines[i];
    l->CalculateSignificance(prior, norm);
    for (unsigned j=l->idx[START]; j<l->idx[END]; j++)
    {
      e = GetPx32FC1(edgeWeighted,l->seg->edgels[j].p.x, l->seg->edgels[j].p.y);
      SetPx32FC1(edgeWeighted, l->seg->edgels[j].p.x, l->seg->edgels[j].p.y,
                 e + l->sig * weight);
    }
  }
}

/**
 * Set exp. lookup table 
 */
void LPSegment::SetExpTab(float *tab, unsigned size, float sigma)
{
  for (unsigned i=0; i<size; i++)
    tab[i] = exp(-.5*Sqr(i/sigma));
}


/**
 * use a gaussian mask
 */
void LPSegment::MulGaussMask(Array<Vector2> &contour, IplImage *edgeWeighted)
{
  IplImage *cImg = cvCreateImage(cvGetSize(edgeWeighted), IPL_DEPTH_8U, 1 );
  IplImage *distImg = cvCreateImage(cvGetSize(edgeWeighted), IPL_DEPTH_32F, 1 );

  cvSet(cImg,cvScalar(255));
  SDraw::DrawPoly(cImg,contour, cvScalar(0),1);

  SetExpTab(expTab, expSize, cfg.sigmaMask);
  cvDistTransform(cImg, distImg, CV_DIST_L2, 3);

  float *s, *d, *d_end;

  for (int v=0; v<edgeWeighted->height; v++)
  {
    s = (float*)(distImg->imageData + distImg->widthStep*v);
    d = (float*)(edgeWeighted->imageData + edgeWeighted->widthStep*v);
    d_end = d + edgeWeighted->width;

    for (;d!=d_end; d++, s++)
      *d *= expTab[(int)(*s)];
  }

  cvReleaseImage(&cImg);
  cvReleaseImage(&distImg);
}

/**
 * use a gaussian mask
 */
void LPSegment::MulGaussMask(Vector2 &center, double dist, double sigma, IplImage *edgeWeighted)
{
  IplImage *cImg = cvCreateImage(cvGetSize(edgeWeighted), IPL_DEPTH_8U, 1 );
  IplImage *distImg = cvCreateImage(cvGetSize(edgeWeighted), IPL_DEPTH_32F, 1 );

  cvSet(cImg,cvScalar(255));
  cvCircle(cImg, cvPoint(center.x,center.y), dist, cvScalar(0), 2);

  SetExpTab(expTab, expSize, sigma);
  cvDistTransform(cImg, distImg, CV_DIST_L2, 3);

  float *s, *d, *d_end;

  for (int v=0; v<edgeWeighted->height; v++)
  {
    s = (float*)(distImg->imageData + distImg->widthStep*v);
    d = (float*)(edgeWeighted->imageData + edgeWeighted->widthStep*v);
    d_end = d + edgeWeighted->width;

    for (;d!=d_end; d++, s++)
      *d *= expTab[(int)(*s)];
  }

  cvReleaseImage(&cImg);
  cvReleaseImage(&distImg);
}








/*********************** PUBLIC ******************************************/
/**
 * compute edge images
 */
void LPSegment::ComputeEdges(IplImage *img)
{
  struct timespec start1, end1;
  clock_gettime(CLOCK_REALTIME, &start1);

  if (edge!=0)
  {
    if (!IsImageSizeEqual(img, edge))
    {
      cvReleaseImage(&edge); 
      cvReleaseImage(&cannyImg);
      cvReleaseImage(&dx);
      cvReleaseImage(&dy);
      edge=0; cannyImg=0; dx=0; dy=0;
      
    }
  }
  if (edge==0)
  {
    edge = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1 );
    cannyImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1 );
    dx = cvCreateImage(cvGetSize(img), IPL_DEPTH_16S, 1 );
    dy = cvCreateImage(cvGetSize(img), IPL_DEPTH_16S, 1 );
    expSize = (unsigned)sqrt(Sqr(img->width) + Sqr(img->height)) + 1;
    expTab = new float[expSize];
  }

  if (cfg.useColourEdges)
  {
    computeEdge.Set(cfg.useColourEdges, 3);   //apperture for sobel
    computeEdge.Sobel(img,dx,dy);
  }
  else
  {
    IplImage *grey = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1 );
    cvConvertImage(img,grey);
    computeEdge.Set(cfg.useColourEdges, 3);   //apperture for sobel
    computeEdge.Sobel(grey,dx,dy);
    cvReleaseImage(&grey);
  }
  computeEdge.Norm(dx,dy,edge);
  computeEdge.Canny(dx,dy,cannyImg,LP_CANNY_HIGH/2.,LP_CANNY_HIGH);
  DeleteSegments(segments);
  DeleteLines(lines);
  computeEdge.LinkEdge(cannyImg, segments);
  formLines.Operate(segments,lines);

  cvNormalize(edge,edge, 0.,1., CV_MINMAX);

  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time colour sobel[s]: "<<P::timespec_diff(&end1, &start1)<<endl;

  #ifdef DEBUG_SEG
  SaveImage("log/edge.jpg",edge);
  SaveImage("log/cannyImg.jpg",cannyImg);
  #endif
}

/**
 * CenterOfGravity
 */
void LPSegment::CenterOfGravity(P::Array<Vector2> &contour, Vector2 &center)
{
  center=Vector2(0.,0.);

  if (contour.Size()==0)
    return;

  for (unsigned i=0; i<contour.Size(); i++)
  {
    center+=contour[i];
  }
  center/=contour.Size();
}

/**
 * Segment the image using log polar space
 */
void LPSegment::DoSegmentation(IplImage *img, P::Array<Vector2> &contour, P::Array<Vector2> &segContour, IplImage *weight)
{
  if (contour.Size()==0 || edge==0)
    throw Except(__HERE__,"Did you specify a contour/edges?");
  if (!IsImageSizeEqual(img,edge))
    throw Except(__HERE__,"Did you compute edges?");

  LogPolar logPolar;
  Vector2 center;
  CenterOfGravity(contour,center);

  IplImage *edgeWeighted = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1);
  IplImage *edgeLP = cvCreateImage( cvSize(512,512), IPL_DEPTH_32F, 1 );

  //compute priors...
  struct timespec start1, end1, start2, end2, start3, end3;
  clock_gettime(CLOCK_REALTIME, &start1);

  cvCopy(edge,edgeWeighted);   //we have normalized edges ...

  if (cfg.useColourPrior)      //add priors
    AddColourPrior(img, contour, edgeWeighted);

  #ifdef DEBUG_SEG
  SaveImage("log/edgeWeightedCol.jpg",edgeWeighted);
  #endif

  if (cfg.useWeightPrior && weight!=0)
    AddPrior(edgeWeighted, weight, cfg.weightPrior);

  #ifdef DEBUG_SEG
  SaveImage("log/edgeWeightedPrior.jpg",edgeWeighted);
  #endif

  if (cfg.useMask)
    MulGaussMask(contour, edgeWeighted);

  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time compute colour/weight priors [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

  //start map to log polar space...
  clock_gettime(CLOCK_REALTIME, &start2);
  logPolar.ComputeMaps(cvGetSize(edgeWeighted), cvGetSize(edgeLP), 
                       cvPoint2D32f(center.x, center.y),cfg.magnitudeScale,
                       CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
  logPolar.MapImage(edgeWeighted,edgeLP,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,0);
  clock_gettime(CLOCK_REALTIME, &end2);
  cout<<"Time log polar mapping [s]: "<<P::timespec_diff(&end2, &start2)<<endl;

  //create/segment graph...
  clock_gettime(CLOCK_REALTIME, &start3); //create graph...
  ProcessGraph(edgeLP, segContour);
  LPContour2Cart(segContour, center, Vector2(edgeLP->width,edgeLP->height), cfg.magnitudeScale);
  clock_gettime(CLOCK_REALTIME, &end3);
  cout<<"Time create/segment graph [s]: "<<P::timespec_diff(&end3, &start3)<<endl;

  #ifdef DEBUG_SEG
  SaveImage("log/edgeLP.jpg",edgeLP);
  SaveImage("log/edgeWeighted.jpg",edgeWeighted);
  #endif

  cvReleaseImage(&edgeWeighted);
  cvReleaseImage(&edgeLP);
}

/**
 * DoSegmentation 
 */
void LPSegment::DoSegmentation(IplImage *img, Vector2 center, double dist, double sigma, P::Array<Vector2> &segContour, IplImage *weight)
{
  if (edge==0)
    throw Except(__HERE__,"Did you specify a contour/edges?");
  if (!IsImageSizeEqual(img,edge))
    throw Except(__HERE__,"Did you compute edges?");

  LogPolar logPolar;

  IplImage *edgeWeighted = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1);
  IplImage *edgeLP = cvCreateImage( cvSize(512,512), IPL_DEPTH_32F, 1 );

  //compute priors...
  struct timespec start1, end1, start2, end2, start3, end3;
  clock_gettime(CLOCK_REALTIME, &start1);

  cvCopy(edge,edgeWeighted);   //we have normalized edges ...

  //if (cfg.useColourPrior)      //add priors
  //  AddColourPrior(img, contour, edgeWeighted);

  #ifdef DEBUG_SEG
  SaveImage("log/edgeWeightedCol.jpg",edgeWeighted);
  #endif

  if (cfg.useWeightPrior && weight!=0)
    AddPrior(edgeWeighted, weight, cfg.weightPrior);

  #ifdef DEBUG_SEG
  SaveImage("log/edgeWeightedPrior.jpg",edgeWeighted);
  #endif

  if (cfg.useMask)
    MulGaussMask(center, dist, sigma, edgeWeighted);

  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time compute colour/weight priors [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

  //start map to log polar space...
  clock_gettime(CLOCK_REALTIME, &start2);
  logPolar.ComputeMaps(cvGetSize(edgeWeighted), cvGetSize(edgeLP), 
                       cvPoint2D32f(center.x, center.y),cfg.magnitudeScale,
                       CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
  logPolar.MapImage(edgeWeighted,edgeLP,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,0);
  clock_gettime(CLOCK_REALTIME, &end2);
  cout<<"Time log polar mapping [s]: "<<P::timespec_diff(&end2, &start2)<<endl;

  //create/segment graph...
  clock_gettime(CLOCK_REALTIME, &start3); //create graph...
  ProcessGraph(edgeLP, segContour);
  LPContour2Cart(segContour, center, Vector2(edgeLP->width,edgeLP->height), cfg.magnitudeScale);
  clock_gettime(CLOCK_REALTIME, &end3);
  cout<<"Time create/segment graph [s]: "<<P::timespec_diff(&end3, &start3)<<endl;

  #ifdef DEBUG_SEG
  SaveImage("log/edgeLP.jpg",edgeLP);
  SaveImage("log/edgeWeighted.jpg",edgeWeighted);
  #endif

  cvReleaseImage(&edgeWeighted);
  cvReleaseImage(&edgeLP);

}


/**
 * computes edges and segments the images
 */
void LPSegment::Operate(IplImage *img, P::Array<Vector2> &contour, P::Array<Vector2> &segContour, IplImage *weight)
{
  ComputeEdges(img);
  DoSegmentation(img,contour,segContour, weight);
}

/**
 * computes edges and segments the images
 */
void LPSegment::Operate(cv::Mat &image, vector<cv::Point2d> &contour, vector<cv::Point2d> &segContour, cv::Mat weight)
{
  IplImage img=image;
  P::Array<Vector2> arContour, arSegContour;

  for (unsigned i=0; i<contour.size(); i++)
    arContour.PushBack(Vector2(contour[i].x,contour[i].y));

  Operate(&img,arContour,arSegContour);

  segContour.clear();
  for (unsigned i=0; i<arSegContour.Size(); i++)
    segContour.push_back(cv::Point2d(arSegContour[i].x, arSegContour[i].y));
}

/**
 * computes edges and segments the images
 */
void LPSegment::Operate(cv::Mat &image, vector<cv::Point2d> &segContour, cv::Point2d &center, double dist, double sigma, cv::Mat weight)
{
  IplImage img=image;
  IplImage cvWeight = weight;
  P::Array<Vector2> arSegContour;

  ComputeEdges(&img);
  if (weight.empty())
    DoSegmentation(&img, Vector2(center.x,center.y), dist, sigma, arSegContour, 0);
  else
    DoSegmentation(&img, Vector2(center.x,center.y), dist, sigma, arSegContour, &cvWeight);


  segContour.clear();
  for (unsigned i=0; i<arSegContour.Size(); i++)
    segContour.push_back(cv::Point2d(arSegContour[i].x, arSegContour[i].y));
}

/**
 * Draw the polygon
 */
void LPSegment::Draw(cv::Mat &image, vector<cv::Point2d> &contour)
{
  IplImage img = image;

  Array<Vector2> poly;
  for (unsigned i=0; i<contour.size(); i++)
    poly.PushBack(Vector2(contour[i].x, contour[i].y));

  SDraw::DrawPoly(&img, poly, CV_RGB(0,0,255), 2);
}



/****************************
 * just a debug save image
 */
void LPSegment::SaveImage(const char *file, IplImage *img, float scale)
{
  IplImage *img8U = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, img->nChannels );
  double min,max;
  cvMinMaxLoc( img, &min, &max);
  if (scale==FLT_MAX)
    cvConvertScale( img, img8U, 255./(max-min), -min*(255./(max-min)));
  else
    cvConvertScale( img, img8U, 255./scale, 0);
  cvSaveImage(file,img8U);
  cvReleaseImage(&img8U);
}

void LPSegment::Graph2Img(LPNode *n, int width, int height, IplImage *img)
{
  if (img->width==width && img->height==height && img->depth==IPL_DEPTH_32F)
  {
    float *d, *d_end;
    LPNode *dn;
    for (int v=0; v<img->height; v++)
    {
      d = ((float*)(img->imageData + img->widthStep*v));
      d_end = d+img->width;
      dn = n + width*v;

      for (;d!=d_end; d++, dn++)
      {
        *d= dn->cost;
      }
    }
  }
}

void LPSegment::SaveGraphVisited(const char *file, LPNode *graph, int width, int height)
{
  uchar *d, *d_end;
  LPNode *n;
  IplImage *img = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1 );

  cvZero(img);
  for (int v=0; v<img->height; v++)
  {
    d = ((uchar*)(img->imageData + img->widthStep*v));
    d_end = d+img->width;
    n = graph + width*v;

    for (;d!=d_end; d++, n++)
    {
      if(n->e) *d = 255;
    }
  }

  cvSaveImage(file,img);

  cvReleaseImage(&img);
}





}

