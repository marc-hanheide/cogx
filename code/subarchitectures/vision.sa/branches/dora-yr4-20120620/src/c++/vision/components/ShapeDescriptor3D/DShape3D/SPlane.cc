/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "SPlane.hh"

//#define DEBUG

#define SP_INL_DIST 1. //1.3 //1.5  // TUNING maybe use 0.8
#define SP_SELECT_POINTS 2       //method: 1=random, 2=NAP
#define SP_NUM_RAND_HYPOTHESES 10
#define SP_DET_THR 10
#define SP_KAPPA2 .3 
#define SP_KAPPA1 5. 
#define SP_RANSAC_ETA0 0.01
#define SP_STD_MUL 1. 
#define SP_MIN_CLUSTER_SIZE 5 //10 
#define SP_MAX_RANSAC_ITER 10000

//#define SP_USE_AFFINE_MODEL

#define SP_SIGMA_ERR .4 

namespace P 
{

double SPlane::CONST_ERR1 = 1. / (SP_SIGMA_ERR*sqrt(2*M_PI));
double SPlane::CONST_ERR2 = 1. / (2 * SP_SIGMA_ERR*SP_SIGMA_ERR);
       

static int CmpPlaneKps(const void *a, const void *b)
{
  if ( ((Plane*)a)->keys.Size() > ((Plane*)b)->keys.Size())
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/********************** SPlane ************************
 * Constructor/Destructor
 */
SPlane::SPlane()
 : dbg(0), dt(0)
{
}

SPlane::~SPlane()
{
  if (dt!=0) delete dt;
}



/**
 * GetRandIdx
 */
void SPlane::GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx)
{
  unsigned temp;
  idx.Clear();
  for (unsigned i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(idx.Contains(temp));
    idx.PushBack(temp);
  }
}

/**
 * get n adjacent points
 * - select random point
 * - sort points depending on the distance to first point
 * - select near points (gauss weighted)
 */
void SPlane::GetNapIdx(P::Array<KeypointDescriptor*> &kps, unsigned num, P::Array<unsigned> &idx)
{
  unsigned temp;
  P::Array<DistIdx> dist;
  idx.Clear();

  if (kps.Size()<num)
    return;

  idx.PushBack(rand()%kps.Size());
  for (unsigned i=0; i<kps.Size(); i++)
    if (i!=idx[0])
      dist.PushBack(DistIdx(Distance(kps[idx[0]]->p, kps[i]->p),i));

  if (dist.Size()<num)
    return;

  dist.Sort(CmpDistIdxAsc);

  for (unsigned i=0; i<num-1; i++)
  {
    do{
      temp = ExpSelect(dist.Size()-1);
    }while(idx.Contains(dist[temp].idx));
    idx.PushBack(dist[temp].idx);
  }
}

/**
 * Count inlier
 */
void SPlane::GetInlier(Array<KeypointDescriptor*> &kps, CvMat *H, int &inl)
{
  inl=0;
  Vector2 p;

  for (unsigned i=0; i<kps.Size(); i++)
  {
    MapPoint(&kps[i]->bw->p.x, H->data.db, &p.x);

    if (Distance(p,kps[i]->p) < SP_INL_DIST)
      inl++;
  }
}

/**
 * ComputeHomography
 */
bool SPlane::ComputeHomography(P::Array<KeypointDescriptor*> &kps, CvMat *H, double inl_pcent, bool nonl, bool outl)
{
  if (kps.Size()<5)
    return false;

  int nbOutl;
  
  hg.Resize(kps.Size());

  for (unsigned i=0; i<kps.Size(); i++)
    hg.InsertNext(&kps[i]->bw->p.x, &kps[i]->p.x);

  #ifdef SP_USE_AFFINE_MODEL
  Array<KeypointDescriptor*> temp;
  bool ok = hg.EstimateAff(H->data.db, nbOutl, inl_pcent, true);

  for (unsigned i=0; i<kps.Size(); i++)
    {
      if (hg.outl[i] != 1)
        temp.PushBack(kps[i]);
    }

  kps = temp;
  return ok;
  #else
  if (nonl)
  {
    Array<KeypointDescriptor*> temp;
    bool ok = hg.EstimateHom(H->data.db, nbOutl, inl_pcent, true);

    for (unsigned i=0; i<kps.Size(); i++)
    {
      if (hg.outl[i] != 1)
        temp.PushBack(kps[i]);
    }

    kps = temp;

    return ok;
  }
  else
  {
    if (outl)
      return hg.EstimateHom(H->data.db, nbOutl, inl_pcent,false, 1, HOMEST_NO_NLN_REFINE);
    else
      return hg.ComputeHomLS(H->data.db);
  }
  #endif
}

/**
 * Accumulate all matched keypoints which are coplanar to our detected plane 
 */
void SPlane::AccumulateKeys(Array<KeypointDescriptor*> &kps, Plane &plane)
{
  Vector2 p;

  plane.keys.Clear();

  for (unsigned i=0; i<kps.Size(); i++)
  {
    MapPoint(&kps[i]->bw->p.x, plane.H->data.db, &p.x);

    if (Distance(p,kps[i]->p) < SP_INL_DIST)
      plane.keys.PushBack(kps[i]);
  }
}

/**
 * test points for insertion
 */
bool SPlane::NearPoint(Array<KeypointDescriptor*> &kps, Keypoint* k, double &thr)
{
  for (unsigned i=0; i<kps.Size(); i++)
    if (Distance(k->p, kps[i]->p)<thr)
      return true;

  return false;
}

/**
 * Accumulate keys whthin a certain distance
 */
void SPlane::AccumulateKeys(Array<KeypointDescriptor*> &kps, Plane &plane, double thrDist)
{
  Vector2 p;

  //mark keys (avoid to insert twice)
  for (unsigned i=0; i<kps.Size(); i++)   
    kps[i]->nb=0;
  for (unsigned i=0; i<plane.keys.Size(); i++)
    plane.keys[i]->nb=1;

  //now insert points
  for (unsigned i=0; i<kps.Size(); i++)
  {
    if (kps[i]->nb==0)
    {
      MapPoint(&kps[i]->bw->p.x, plane.H->data.db, &p.x);

      if (Distance(p,kps[i]->p) < SP_INL_DIST)
      {
        if (NearPoint(plane.keys, kps[i], thrDist))
        {
          plane.keys.PushBack(kps[i]);
        }
      }
    }
  }

}

/**
 * randomly select corners and compute homography
 */
#ifdef SP_USE_AFFINE_MODEL
#define SP_RAND_POINTS 3 
void SPlane::GetRandHypotheses(P::Array<KeypointDescriptor*> &all, P::Array<KeypointDescriptor*> &kps, P::Array<Plane*> &tentPlanes)
{
  if (kps.Size()<SP_RAND_POINTS)
    return;

  //srand(time(NULL));
  srand(1);
  P::Array<unsigned> idx;
  Plane *plane;
  double det;
  static double INV_DET_THR = 1./SP_DET_THR;
  bool ok;
  unsigned z;

  for (unsigned i=0; i<SP_NUM_RAND_HYPOTHESES; i++)
  { 
    plane = new Plane();
    z=0;

    do{
      if (SP_SELECT_POINTS==1)
        GetRandIdx(kps.Size(), SP_RAND_POINTS, idx);
      else if (SP_SELECT_POINTS==2)
        GetNapIdx(kps, SP_RAND_POINTS, idx);

      z++;
      if (idx.Size()==SP_RAND_POINTS)
      {
        ok = hg.ComputeAff(&kps[idx[0]]->bw->p.x, &kps[idx[1]]->bw->p.x, 
                           &kps[idx[2]]->bw->p.x, 
                           &kps[idx[0]]->p.x, &kps[idx[1]]->p.x, 
                           &kps[idx[2]]->p.x,
                           plane->H->data.db);

        if (ok) det = Det33(plane->H->data.db);
      }
      else ok=false;

    }while((!ok || det < INV_DET_THR || det > SP_DET_THR) && z<1000);

    if (z<1000)
      tentPlanes.PushBack(plane);
  }

  for (unsigned i=0; i<tentPlanes.Size(); i++)
    if (tentPlanes[i]->id==UINT_MAX)
      AccumulateKeys(all, *tentPlanes[i]);   
}
#else
#define SP_RAND_POINTS 4 
void SPlane::GetRandHypotheses(P::Array<KeypointDescriptor*> &all, P::Array<KeypointDescriptor*> &kps, P::Array<Plane*> &tentPlanes)
{
  if (kps.Size()<SP_RAND_POINTS)
    return;

  //srand(time(NULL));
  srand(1);
  P::Array<unsigned> idx;
  Plane *plane;
  double det;
  static double INV_DET_THR = 1./SP_DET_THR;
  bool ok;
  unsigned z;

  for (unsigned i=0; i<SP_NUM_RAND_HYPOTHESES; i++)
  { 
    plane = new Plane();
    z=0;

    do{
      if (SP_SELECT_POINTS==1)
        GetRandIdx(kps.Size(), SP_RAND_POINTS, idx);
      else if (SP_SELECT_POINTS==2)
        GetNapIdx(kps, SP_RAND_POINTS, idx);

      z++;
      if (idx.Size()==SP_RAND_POINTS)
      {
        ok = hg.ComputeHom(&kps[idx[0]]->bw->p.x, &kps[idx[1]]->bw->p.x, 
                           &kps[idx[2]]->bw->p.x, &kps[idx[3]]->bw->p.x,
                           &kps[idx[0]]->p.x, &kps[idx[1]]->p.x, 
                           &kps[idx[2]]->p.x, &kps[idx[3]]->p.x,
                           plane->H->data.db);

        if (ok) det = Det33(plane->H->data.db);
      }
      else ok=false;

    }while((!ok || det < INV_DET_THR || det > SP_DET_THR) && z<1000);

    if (z<1000)
      tentPlanes.PushBack(plane);
  }

  for (unsigned i=0; i<tentPlanes.Size(); i++)
    if (tentPlanes[i]->id==UINT_MAX)
      AccumulateKeys(all, *tentPlanes[i]);   
}
#endif

/**
 * Compute Gaussian error probability using Euclidean distance
 */
void SPlane::GetErrProbEuc(double m1[2],double m2[2],double H[9], double err[1])
{
  Vector2 p;

  MapPoint(m1, H, &p.x);
  *err =  sqrt(Sqr(p.x-m2[0]) + Sqr(p.y-m2[1]));       // euc. distance
  *err = CONST_ERR1 * exp(-(*err * *err * CONST_ERR2));    // gaussian
}


/**
 * individual values
 */
void SPlane::ComputeQii(Plane* kg, double &weight)
{
  double err;

  for (unsigned i=0; i<kg->keys.Size(); i++)
  {
        GetErrProbEuc(&kg->keys[i]->bw->p.x, &kg->keys[i]->p.x, kg->H->data.db, &err);
        err = (1.-SP_KAPPA2) + SP_KAPPA2*err;

        weight += (err>0.?err:0.);
  }

  if (weight > SP_KAPPA1)
    weight -= SP_KAPPA1;
  else
    weight = 0.;
}

/**
 * interaction value
 */
void SPlane::ComputeQij(Plane* ki, Plane* kj, double &weight)
{
  double temp=0, wi, wj;
  Keypoint::nbcnt++;
  double err;

  for (unsigned i=0; i<ki->keys.Size(); i++)
    ki->keys[i]->nb = Keypoint::nbcnt;

  for (unsigned i=0; i<kj->keys.Size(); i++)
  {
    if (kj->keys[i]->nb == Keypoint::nbcnt)
    {
      GetErrProbEuc(&kj->keys[i]->bw->p.x, &kj->keys[i]->p.x, ki->H->data.db, &err);
      err = (1.-SP_KAPPA2) + SP_KAPPA2*err;
      wi = (err>0.?err:0.);

      GetErrProbEuc(&kj->keys[i]->bw->p.x, &kj->keys[i]->p.x, kj->H->data.db, &err);
      err = (1.-SP_KAPPA2) + SP_KAPPA2*err;
      wj = (err>0.?err:0.);
     
      temp -= (wi<wj?wj:wi);
    }
  }

  weight+=(temp/2.);
}


/**
 * create mdl segmentation matrix
 */
void SPlane::CreateSegmentationMatrixQ(P::Array<Plane*> &tempPlanes, CvMat *Q)
{
  for (unsigned i=0; i<tempPlanes.Size(); i++)
  {
    for (unsigned j=0; j<tempPlanes.Size(); j++)
    {
      if (i==j)
        ComputeQii(tempPlanes[i],Q->data.db[i*Q->cols+j]);
      else
        ComputeQij(tempPlanes[j], tempPlanes[i], Q->data.db[i*Q->cols+j]);
    }
  }
}

/**
 * solve QBP
 */
void SPlane::GreedyQBP(CvMat *Q, CvMat *m)
{
  CvMat *mt = cvCloneMat(m);
  double max;
  unsigned idx;
  bool finished = false;
  P::Array<double> s(Q->rows);
  CvMat *v = cvCreateMat(1,Q->rows,CV_64F);
  CvMat *tmp1 = cvCreateMat(1,1,CV_64F);
  CvMat *tmp2 = cvCreateMat(1,1,CV_64F);

  do{
    for (int i=0; i<m->rows; i++)
      if (IsZero(cvmGet(m,i,0)))
      {
        cvCopy(m, mt);
        cvmSet(mt,i,0,1.);
        cvGEMM( mt, Q, 1, 0, 1, v, CV_GEMM_A_T);
        cvGEMM( v, mt, 1, 0, 1, tmp1, 0);
        cvGEMM( m, Q, 1, 0, 1, v, CV_GEMM_A_T);
        cvGEMM( v, m, 1, 0, 1, tmp2, 0);
        s[i] = cvmGet(tmp1,0,0) - cvmGet(tmp2,0,0);
      }

    max = -DBL_MAX;
    for (int i=0; i<m->rows; i++)
    {
      if (IsZero(cvmGet(m,i,0)))
      {
        if (s[i] > max)
        {
          max = s[i];
          idx = i;
        }
      }
    }

    if (max>0)
      cvmSet(m,idx,0,1.);
    else
      finished=true;
  }while(!finished);

  cvReleaseMat(&mt);
  cvReleaseMat(&v);
  cvReleaseMat(&tmp1);
  cvReleaseMat(&tmp2);
}

/**
 * select plane surfaces
 */
void SPlane::SelectHypotheses(Array<Plane*> &randPlanes, P::Array<Plane*> &planes)
{
  if (randPlanes.Size()==0)
    return;

  CvMat *Q = cvCreateMat( randPlanes.Size(), randPlanes.Size(), CV_64F );
  CvMat *m = cvCreateMat(randPlanes.Size(),1,CV_64F);
  cvZero(Q);
  cvZero(m);

  CreateSegmentationMatrixQ(randPlanes, Q);
  GreedyQBP(Q,m);

  for (unsigned i=0; i<randPlanes.Size(); i++)
    if (cvmGet(m,i,0)>0.5)
      planes.PushBack(randPlanes[i]);
    else
      delete randPlanes[i];

  cvReleaseMat(&Q);
  cvReleaseMat(&m);

  randPlanes.Clear();
}

/**
 * If points are assigned to more than one group assign it to the cluster
 * which minimizes the error
 */
void SPlane::ReleaseInteractions(Array<Plane*> &planes)
{
  float err;
  Vector2 p;
  Plane *kg;

  for (unsigned i=0; i<planes.Size(); i++)
    for (unsigned j=0; j<planes[i]->keys.Size(); j++)
      planes[i]->keys[j]->error = DBL_MAX;

  for (unsigned i=0; i<planes.Size(); i++)
  {
      kg=planes[i];
      for (unsigned j=0; j<kg->keys.Size(); j++)
      {
        MapPoint(&kg->keys[j]->bw->p.x, kg->H->data.db, &p.x);
        err = (float)Distance(p, kg->keys[j]->p);
        if (err < kg->keys[j]->error)
        {
          kg->keys[j]->error=err;
          kg->keys[j]->id = i;
        }
      }
  }
  for (unsigned i=0; i<planes.Size(); i++)
  {
    kg=planes[i];
    for (unsigned j=0; j<kg->keys.Size(); j++)
    {
      if (kg->keys[j]->id!=i)
      {
        kg->keys.Erase(j);
        j--;
      }
    }
  }
}

/**
 * Split intersection planes
 */
void SPlane::SplitPlanes(Array<Plane*> &planes)
{
  Vector2 p;
  P::Array<KDVertex*> kdvs;
  Array<Array<KeypointDescriptor*> > groups;
  Array<Plane*> splitted;
  Plane *plane;
  
  
  planes.Sort(CmpPlaneKps);

  for (int i=(int)planes.Size()-1; i>=0; i--)
  {
    if (dt!=0) delete dt;
    dt = new KPTree();
    for (unsigned j=0; j<kdvs.Size(); j++)
      delete kdvs[j];
    kdvs.Clear();

    for (unsigned j=0; (int)j<i; j++)
      for (unsigned k=0; k<planes[j]->keys.Size(); k++)
      {
        kdvs.PushBack(new KDVertex(planes[j]->keys[k],KDVertex::CL_DP));
        *dt += kdvs.Last();
      }

    for (unsigned k=0; k<planes[i]->keys.Size(); k++)
    {
      kdvs.PushBack(new KDVertex(planes[i]->keys[k],KDVertex::CL_NOT_SET));
      *dt += kdvs.Last();
    }

    for (unsigned j=0; j<splitted.Size(); j++)
      for (unsigned k=0; k<splitted[j]->keys.Size(); k++)
      {
        kdvs.PushBack(new KDVertex(splitted[j]->keys[k],KDVertex::CL_DP));
        *dt += kdvs.Last();
      }
    
    groups.Clear();
    dt->GetGroups(groups);
    for (unsigned j=0; j<groups.Size(); j++)
    {
      plane = new Plane();
      plane->keys = groups[j];
      ComputeHomography(plane->keys, plane->H, 0.8,false);
      splitted.PushBack(plane);
    }
  }

  DeletePlanes(planes);
  planes = splitted;

  //clean up
  delete dt;
  dt=0;

  for (unsigned i=0; i<kdvs.Size(); i++)
    delete kdvs[i];
}

/**
 * compute sigma
 */
#ifdef DEBUG
//IplImage *img;
#endif
void SPlane::GetSigma(P::Array<KDVertex*> &kdvs, double &mean, double &sigma)
{
  sigma = 0;
  mean=0.;
  unsigned num=0;

  for (unsigned i=0; i<kdvs.Size(); i++)
  {
    for (unsigned j=0; j<kdvs[i]->links.Size(); j++)
    {
      kdvs[i]->dist.PushBack(Distance(kdvs[i]->kp->p, kdvs[i]->links[j]->kp->p));
      mean += kdvs[i]->dist.Last();
      #ifdef DEBUG
      //SDraw::DrawLine(img,kdvs[i]->kp->p.x,kdvs[i]->kp->p.y,
      //kdvs[i]->links[j]->kp->p.x, kdvs[i]->links[j]->kp->p.y, CV_RGB(255,255,255));
      #endif
      num++;
    }
  }

  mean /= (double)num;

  for (unsigned i=0; i<kdvs.Size(); i++)
    for (unsigned j=0; j<kdvs[i]->links.Size(); j++)
      sigma += Sqr(kdvs[i]->dist[j]-mean);

  sigma = sqrt(sigma/(((double)num)-1.));
}

/**
 * look for unclustered node
 */
KDVertex* SPlane::GetNextNode(Array<KDVertex*> &kdvs)
{
  for (unsigned i=0; i<kdvs.Size(); i++)
  {
    if (kdvs[i]->ncl == KDVertex::CL_NOT_SET)
      return kdvs[i];
  }
  return 0;
}

/**
 * Split intersection planes
 */
void SPlane::SplitPlanes2(Array<Plane*> &planes)
{
  KDVertex *node;
  double mean, sigma;
  Vector2 p;
  P::Array<KDVertex*> kdvs;
  Array<Plane*> splitted;
  Plane *plane;

  for (unsigned j=0; j<planes.Size(); j++)
  {
    if (planes[j]->id!=UINT_MAX)
    {
      splitted.PushBack(planes[j]);
      planes.Erase(j);
      j--;
      continue;
    }

    if (dt!=0) delete dt;
    dt = new KPTree();

    for (unsigned i=0; i<kdvs.Size(); i++)
      delete kdvs[i];
    kdvs.Clear();

    #ifdef DEBUG
    //img = cvCloneImage(dbg);
    #endif
    for (unsigned i=0; i<planes[j]->keys.Size(); i++)
    {
      kdvs.PushBack(new KDVertex(planes[j]->keys[i]));
      kdvs.Last()->ncl = KDVertex::CL_NOT_SET;
      *dt += kdvs.Last();
      #ifdef DEBUG
      //SDraw::DrawCross(img,planes[j]->keys[i]->p.x,planes[j]->keys[i]->p.y,CV_RGB(255,255,255),3);
      #endif
    }

    dt->SetVertexLinks();
    GetSigma(kdvs, mean, sigma);
    double thr = mean+sigma*SP_STD_MUL;

    //cout<<j<<"/"<<planes[j]->keys.Size()<<": mean+sigma*SP_STD_MUL="<<mean+sigma*SP_STD_MUL<<endl;
    do{
      node = GetNextNode(kdvs);

      if (node!=0)
      {
        plane = new Plane();
        plane->thrDist = thr;
        node->ncl++;
        node->Cluster(plane->keys, thr);

        if (plane->keys.Size() > SP_MIN_CLUSTER_SIZE)
        {
          Plane::idcnt++;               //id's are now handled by CModelCore!
          plane->id = Plane::idcnt;
          ComputeHomography(plane->keys, plane->H, 0.8, false);
          splitted.PushBack(plane);
          #ifdef DEBUG
          //CvScalar col = CV_RGB(rand()%255, rand()%255, rand()%255);
          //for (unsigned i=0; i<plane->keys.Size(); i++)
          //  SDraw::DrawCross(img,plane->keys[i]->p.x,plane->keys[i]->p.y,col,2);
          #endif
        }
        else
          delete plane;
      }
    }while (node != 0);

    #ifdef DEBUG
    //char file[1024];
    //snprintf(file, 1024, "log/sets-%03d.jpg",j);
    //cvSaveImage(file, img);
    //cvReleaseImage(&img);
    #endif
  }

  DeletePlanes(planes);
  planes = splitted;

  //clean up
  delete dt;
  dt=0;

  for (unsigned i=0; i<kdvs.Size(); i++)
    delete kdvs[i];
}

/**
 * copy unexplaned keypoints
 */
void SPlane::RenewKps(Array<Plane*> &planes, Array<KeypointDescriptor*> &ks, Array<KeypointDescriptor*> &kps)
{
  kps.Clear();

  for (unsigned i=0; i<ks.Size();  i++)
    ks[i]->nb = 0;

  for (unsigned i=0; i<planes.Size(); i++)
    for (unsigned j=0; j<planes[i]->keys.Size(); j++)
      planes[i]->keys[j]->nb = 1;

   for (unsigned i=0; i<ks.Size();  i++)
    if (ks[i]->nb == 0)
      kps.PushBack(ks[i]);
}

/**
 * count explaned keypoints
 */
void SPlane::CountExplainedKps(Array<KeypointDescriptor*> &kps, Array<Plane*> &planes, int &numEx)
{
  int numNotEx=0;

  for (unsigned i=0; i<kps.Size(); i++)
    kps[i]->nb=0;

  for (unsigned i=0; i<planes.Size(); i++)
  {
    for (unsigned j=0; j<planes[i]->keys.Size(); j++)
      planes[i]->keys[j]->nb=1;
  }

  for (unsigned i=0; i<kps.Size(); i++)
    if(kps[i]->nb==0) numNotEx++;

  numEx = kps.Size()-numNotEx;

}

/**
 * detect and track planes
 */
void SPlane::AccumulateKeys(Array<KeypointDescriptor*> &kps, Array<Plane*> &planes)
{
  for (unsigned i=0; i<planes.Size(); i++)
    AccumulateKeys(kps, *planes[i], planes[i]->thrDist);
}


/**
 * Get keypoints with a match
 */
void SPlane::GetMatches(Array<KeypointDescriptor*> &keys, Array<KeypointDescriptor*> &matches)
{
  matches.Clear();
  for (unsigned i=0; i<keys.Size(); i++)
    if (keys[i]->bw!=0) 
      matches.PushBack(keys[i]);
}






/******************************** PUBLIC **************************/

/**
 * Plane detection and tracking
 * Planes are tracked and new planes are detected.
 * We do not care about framerate!
 */
void SPlane::DetectPlanes(Array<KeypointDescriptor*> &keys, Array<Plane*> &planes)
{
  GetMatches(keys, matches);

  if (matches.Size()>=4)
  {
    #ifdef DEBUG
    struct timespec start1, end1;
    clock_gettime(CLOCK_REALTIME, &start1);
    #endif
    #ifdef SP_USE_AFFINE_MODEL
    int MODEL_SIZE = 3;
    #else
    int MODEL_SIZE = 4;
    #endif

    Array<Plane*> randPl;
    Array<Plane*> tentPl = planes;;
    planes.Clear();

    AccumulateKeys(matches, tentPl);

    int k=0;
    int inl, inls = 0;
    CountExplainedKps(matches, tentPl, inl);
    double eps = ((double)inl)/(double)matches.Size();
 
    while(pow(1. - pow(eps,(tentPl.Size()*MODEL_SIZE)),k) >= SP_RANSAC_ETA0 && k<SP_MAX_RANSAC_ITER)
    {
      randPl = tentPl;
      tentPl.Clear();

      RenewKps(randPl, matches, ks);

      GetRandHypotheses(matches, ks, randPl); 

      SelectHypotheses(randPl, tentPl); 

      for (unsigned i=0; i<tentPl.Size(); i++)
        if (tentPl[i]->id==UINT_MAX)
          ComputeHomography(tentPl[i]->keys, tentPl[i]->H, 0.8, false, true);

      CountExplainedKps(matches, tentPl, inl);
    
      if (inl>inls)
      {
        inls = inl;
        eps = (double)inls / (double)matches.Size();
      }
      
      k++;
    }
    #ifdef DEBUG
    cout<<"Number of iterations: "<<k;
    if (k>=SP_MAX_RANSAC_ITER) cout<<" convergence failed!";
    cout<<endl;
    #endif

    SplitPlanes2(tentPl); 
    ReleaseInteractions(tentPl);
    SelectHypotheses(tentPl, planes);

    for (unsigned i=0; i<planes.Size(); i++)  //nonl homography refinement
      ComputeHomography(planes[i]->keys, planes[i]->H, 0.8, true, true);

    #ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &end1);
    cout<<"Time for model selection based plane detection [s]: "<<P::timespec_diff(&end1, &start1)<<endl;
    #endif
  }
}

/**
 * Track the planes to the current frame
 */
void SPlane::TrackPlanes(Array<Plane*> &in,  Array<Plane*> &out)
{
  DeletePlanes(out);
  Plane *plane, *tplane;

  for (unsigned i=0; i<in.Size(); i++)
  {
    plane = in[i];

    tplane = new Plane();
    for (unsigned j=0; j<plane->keys.Size(); j++)
    {
      if (plane->keys[j]->fw!=0)
        tplane->keys.PushBack((KeypointDescriptor*)plane->keys[j]->fw);
    }

    if (ComputeHomography(tplane->keys, tplane->H, .7, true))
    {
      CopyParameter(plane, tplane);

      if ( tplane->keys.Size() > SP_MIN_CLUSTER_SIZE)
        out.PushBack(tplane);
      else
        delete tplane;
    }
    else
      delete tplane;
  }
}


/************************************ DEBUG METHODES ********************************/

void SPlane::DrawPlane(IplImage *img, Plane &plane, CvScalar col)
{
  for (unsigned j=0; j<plane.keys.Size(); j++)
  {
    SDraw::DrawCross(img, plane.keys[j]->X(), plane.keys[j]->Y(), col,2);
  }
  
}

static map<unsigned, CvScalar> cols;
void SPlane::DrawPlanes(IplImage *img, Array<Plane*> &planes)
{
  char txt[256];
  CvScalar col;
  //srand(time(NULL));
  srand(1);

  for (unsigned i=0; i<planes.Size(); i++)
  {
    if (planes[i]->id!=UINT_MAX)
    {
      map<unsigned,CvScalar>::iterator it = cols.find(planes[i]->id);
      if( it == cols.end() )
      {
        col = CV_RGB(rand()%255,rand()%255,rand()%255);
        cols[planes[i]->id] = col;
      }
      else
        col = cols[planes[i]->id];
    }
    else
      col = CV_RGB(rand()%255,rand()%255,rand()%255);

    DrawPlane(img, *planes[i], col);

    int n=0;
    n+=snprintf(txt+n,256-n,"Id: %u(%u)", planes[i]->id, planes[i]->oid );
    if (planes[i]->keys.Size()!=0)
      SDraw::WriteText(img,(const char*)(&txt), planes[i]->keys[0]->X(), planes[i]->keys[0]->Y(), col);
  }
}



}

