/**
 * $Id$
 */


#include "SBAWrapper.hh"
#include "eucsba_driver.ic"

#define DEBUG


namespace P 
{

static const bool CmpProjs(const pair<unsigned,cv::Point2d> &a, const pair<unsigned,cv::Point2d>& b) 
{
  return (a.first < b.first);
}


/********************** SBAWrapper ************************
 * Constructor/Destructor
 */
SBAWrapper::SBAWrapper(unsigned _sbaMaxIter, cv::Mat intrinsic)
{
  SetCameraParameter(intrinsic);
  sbaMaxIter = _sbaMaxIter;
}

SBAWrapper::~SBAWrapper()
{
}


/********************** PRIVATE *************************/



/**
 * Convert data to sba layout
 */
bool SBAWrapper::Convert2SBA(vector<cv::Ptr<Pose> > &cams, vector<cv::Ptr<SBAData> > &data, int &n2Dprojs, int cnp, int pnp, int mnp, double **motstruct, double **initrot, double **imgpts, char **vmask)
{
  char *v;
  double *p, *d, *r;
  int ptno = 0;
  int ncams = cams.size();
  int n3Dpts = data.size();

  //count projs
  n2Dprojs=0;
  for (unsigned i=0; i<data.size(); i++)
    n2Dprojs+=data[i]->projs.size();

  #ifdef DEBUG
  cout<<"ncams="<<ncams<<", n3Dpts="<<n3Dpts<<", n2Dprojs="<<n2Dprojs<<endl;
  #endif

  if (ncams<2 || n3Dpts<3 || n2Dprojs<6)
    return false;

  //allocate memory  
  *motstruct=(double *)malloc((ncams*cnp + n3Dpts*pnp)*sizeof(double));
  if(*motstruct==NULL) throw runtime_error ("SBAWrapper::Convert2SBA: Failed to allocate memory for SBA!");

  *initrot=(double *)malloc((ncams*FULLQUATSZ)*sizeof(double));
  if(*initrot==NULL) throw runtime_error ("SBAWrapper::Convert2SBA: Failed to allocate memory for SBA!");

  *imgpts=(double *)malloc(n2Dprojs*mnp*sizeof(double));
  if(*imgpts==NULL) throw runtime_error ("SBAWrapper::Convert2SBA: Failed to allocate memory for SBA!");

  *vmask=(char *)malloc(n3Dpts * ncams * sizeof(char));
  if(*vmask==NULL) throw runtime_error ("SBAWrapper::Convert2SBA: Failed to allocate memory for SBA!");
  memset(*vmask, 0, n3Dpts * ncams * sizeof(char));

  // set global cameras
  d = *motstruct;
  r = *initrot;

  // set cameras to data structure
  double *t;
  for (unsigned i=0; i<cams.size(); i++)
  {
    Rot2Vec3(cams[i]->R.ptr<double>(),d);
    t = cams[i]->t.ptr<double>();

    r[1] = *d++;
    r[2] = *d++;
    r[3] = *d++;
    *d++ = t[0];
    *d++ = t[1];
    *d++ = t[2];
    r[0] = sqrt(1.0 - r[1]*r[1] - r[2]*r[2] - r[3]*r[3]);
    r+=FULLQUATSZ;
  }

  // set points to data structure
  SBAData *sba;  
  p = *imgpts;
  v = *vmask;

  for (unsigned i=0; i<data.size(); i++)
  {
    sba = &(*data[i]);

    *d++ = sba->pos.x;
    *d++ = sba->pos.y;
    *d++ = sba->pos.z;

    for (unsigned j=0; j<sba->projs.size(); j++)
    {
      *p++ = sba->projs[j].second.x;
      *p++ = sba->projs[j].second.y;

      v[ptno * ncams + sba->projs[j].first] = 1;
    }

    ptno++;
  }

  return true;
}

/**
 * Convert2Obj
 */
void SBAWrapper::Convert2Obj(vector<cv::Ptr<Pose> > &cams, vector<cv::Ptr<SBAData> > &data, int n2Dprojs, double **motstruct, double **initrot, double **imgpts, char **vmask)
{
  double *t, *d = *motstruct;

  // set cameras to data structure
  for (unsigned i=0; i<cams.size(); i++)
  {
    Vec32Rot(d,cams[i]->R.ptr<double>());
    d+=3;
    t = cams[i]->t.ptr<double>();
    t[0] = *d++;
    t[1] = *d++;
    t[2] = *d++;
  }

  // set 3d points
  for (unsigned i=0; i<data.size(); i++)
  {
    data[i]->pos = cv::Point3d(d[0],d[1],d[2]);
    d+=3;
  }

  //free memory  
  free (*motstruct);
  free (*initrot);
  free (*imgpts);
  free (*vmask);
}



/**
 * sort cameras and projections
 */
void SBAWrapper::SortData(vector<cv::Ptr<SBAData> > &data)
{
  for (unsigned i=0; i<data.size(); i++)
  {
    sort(data[i]->projs.begin(), data[i]->projs.end(), CmpProjs);
  }
}


void SBAWrapper::EvaluateData(vector<cv::Ptr<SBAData> > &data)
{

  cout<<"DEBUG SBA DATA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

  double cpos[3], pt[2];
  double sumDist=0;
  unsigned cnt=0;
  
  for (unsigned i=0; i<data.size(); i++)
  {
    cv::Point3d &pos0 = data[i]->pos;
    for (unsigned j=0; j<data[i]->projs.size(); j++)
    {
      PMat::MulAdd3( cameras[data[i]->projs[j].first]->R.ptr<double>(), &pos0.x, 
               cameras[data[i]->projs[j].first]->t.ptr<double>(), cpos);
      ProjectPoint2Image(cpos, cam.ptr<double>(), pt);

      double dist = PVec::Distance2(pt,&data[i]->projs[j].second.x);
      sumDist+= dist;
      cnt++;

      cout<<dist<<"("<<data[i]->projs[j].first<<") ";
    }
    cout<<endl;
  }

  cout<<"---> sumDist="<<sumDist<<", mean="<<(sumDist/(double)cnt)<<endl;
}




/*********************** PUBLIC *************************/

/**
 * Backtace object locations, compute feature locations and bundle structure and cameras
 */
int SBAWrapper::BundleMotStruct(int nconstframes)
{
  if (cam.empty()) throw runtime_error ("SBAWrapper::BundleMotStruct: Camera parameter initialized!");

  #ifdef DEBUG
  struct timespec start0, end0;
  clock_gettime(CLOCK_REALTIME, &start0);
  #endif

  unsigned n = 1;
  double *motstruct, *imgpts, *initrot;
  double ical[5];       // intrinsic calibration matrix & temp. storage for its params
  char *vmask;
  double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
  int numprojs, numpts3D, numFrames;
  int cnp=6;           // 3 rot params + 3 trans params 
  int pnp=3;           // euclidean 3D points 
  int mnp=2;           // image points are 2D 

  int z=0;
  
  SortData(points);

  if (!Convert2SBA(cameras, points, numprojs, cnp, pnp, mnp, &motstruct, &initrot, &imgpts, &vmask))
    return 0;

  numFrames = cameras.size();
  numpts3D = points.size();
  if (nconstframes > cameras.size()) nconstframes=cameras.size();

  for(int i=0; i<numFrames; ++i)
  {
      register int j;
      j=(i+1)*cnp;
      motstruct[j-4]=motstruct[j-5]=motstruct[j-6]=0.0; // clear rotation
  }

  // set up globs structure 
  double *c = cam.ptr<double>();
  globs.cnp=cnp; globs.pnp=pnp; globs.mnp=mnp;
  globs.rot0params=initrot;
  ical[0] = c[0];      // fu
  ical[1] = c[2];      // u0
  ical[2] = c[5];      // v0
  ical[3] = c[4]/c[0]; // ar
  ical[4] = c[1];
  globs.intrcalib=ical;

  globs.ptparams=NULL;
  globs.camparams=NULL;

  opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH; opts[2]=SBA_STOP_THRESH;
  opts[3]=SBA_STOP_THRESH;  
  opts[4]=0.0;

  n = sba_motstr_levmar_x(numpts3D, 0, numFrames, nconstframes, vmask, motstruct, cnp, pnp, 
                      imgpts,  NULL, mnp, img_projsRTS_x, img_projsRTS_jac_x ,
                      (void *)(&globs), sbaMaxIter, 0, opts, info);

  for(int i=0; i<numFrames; ++i)
  {
      double *v, qs[FULLQUATSZ], *q0, prd[FULLQUATSZ];

      v=motstruct + (i+1)*cnp - 6;
      _MK_QUAT_FRM_VEC(qs, v);

      q0=initrot+i*FULLQUATSZ;
      quatMultFast(qs, q0, prd); // prd=qs*q0

      if(prd[0]>=0.0)
      {
        v[0]=prd[1];
        v[1]=prd[2];
        v[2]=prd[3];
      }
      else
      {
        v[0]=-prd[1];
        v[1]=-prd[2];
        v[2]=-prd[3];
      }
  }

  Convert2Obj(cameras,points, numprojs, &motstruct, &initrot, &imgpts, &vmask);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end0);
  cout<<"--------- bundle ---------"<<endl;
  fprintf(stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n", n,
                    info[5], info[6], info[1]/numprojs, info[0]/numprojs, (int)info[7], (int)info[8], (int)info[9]);
  cout<<"Time for bundle adjustment (motion and structure) [s]: "<<PMath::timespec_diff(&end0, &start0)<<endl;
  cout<<"--------------------------"<<endl;
  #endif

  return n;
}


/**
 * Backtace object locations, compute feature locations and bundle cameras
 */
int SBAWrapper::BundleMot(int nconstframes)
{
  if (cam.empty()) throw runtime_error ("SBAWrapper::BundleMot: Camera parameter not set!");

  #ifdef DEBUG
  struct timespec start0, end0;
  clock_gettime(CLOCK_REALTIME, &start0);
  #endif

  unsigned n = 1;
  double *motstruct, *imgpts, *initrot;
  double ical[5];       // intrinsic calibration matrix & temp. storage for its params
  char *vmask;
  double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
  int numprojs, numpts3D, numFrames;
  int cnp=6; // 3 rot params + 3 trans params 
  int pnp=3; // euclidean 3D points 
  int mnp=2; // image points are 2D 

  SortData(points);

  #ifdef DEBUG
  //EvaluateData(points); // DEBUG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //cout<<"cams.size()="<<cams.size()<<endl;
  #endif

  if (!Convert2SBA(cameras, points, numprojs, cnp, pnp, mnp, &motstruct, &initrot, &imgpts, &vmask))
    return 0;

  numFrames = cameras.size();
  numpts3D = points.size();
  if (nconstframes>cameras.size()) nconstframes=cameras.size();

  for(int i=0; i<numFrames; ++i)
  {
      register int j;
      j=(i+1)*cnp;
      motstruct[j-4]=motstruct[j-5]=motstruct[j-6]=0.0; // clear rotation
  }
 
  // set up globs structure 
  double *c = cam.ptr<double>();
  globs.cnp=cnp; globs.pnp=pnp; globs.mnp=mnp;
  globs.rot0params=initrot;
  ical[0] = c[0];      // fu
  ical[1] = c[2];      // u0
  ical[2] = c[5];      // v0
  ical[3] = c[4]/c[0]; // ar
  ical[4] = c[1];
  globs.intrcalib=ical;

  globs.ptparams=NULL;
  globs.camparams=NULL;

  opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH; opts[2]=SBA_STOP_THRESH;
  opts[3]=SBA_STOP_THRESH;  
  opts[4]=0.0;

  globs.ptparams=motstruct+numFrames*cnp; 
  n = sba_mot_levmar_x(numpts3D, numFrames, nconstframes, vmask, motstruct, cnp, imgpts, 0, mnp,
                       img_projsRT_x, img_projsRT_jac_x,
                       (void *)(&globs), sbaMaxIter, 0, opts, info);

  for(int i=0; i<numFrames; ++i)
  {
      double *v, qs[FULLQUATSZ], *q0, prd[FULLQUATSZ];

      v=motstruct + (i+1)*cnp - 6;
      _MK_QUAT_FRM_VEC(qs, v);

      q0=initrot+i*FULLQUATSZ;
      quatMultFast(qs, q0, prd); // prd=qs*q0

      if(prd[0]>=0.0)
      {
        v[0]=prd[1];
        v[1]=prd[2];
        v[2]=prd[3];
      }
      else
      {
        v[0]=-prd[1];
        v[1]=-prd[2];
        v[2]=-prd[3];
      }
  }

  Convert2Obj(cameras, points, numprojs, &motstruct, &initrot, &imgpts, &vmask);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end0);
  fprintf(stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n", n,
                    info[5], info[6], info[1]/numprojs, info[0]/numprojs, (int)info[7], (int)info[8], (int)info[9]);
  cout<<"Time for bundle adjustment (motion) [s]: "<<PMath::timespec_diff(&end0, &start0)<<endl;
  #endif

  return n;
}


/**
 * Set cameras locations, 3d points and corresponding projections
 * Attention!
 * Projection ids must correpsond to the camera index (_camera)
 * We use shared pointer, the data is not copied!
 * After optimization this data structure contains the refined cameras and points!!!!!!!
 */
void SBAWrapper::SetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Ptr<SBAData> > &_points)
{
  cameras = _cameras;
  points = _points;
}

/**
 * clear data structure
 */
void SBAWrapper::Clear()
{
  cameras.clear();
  points.clear();
}

/**
 * Insert a camera pose
 * Attention: Order of cameras must correspond to the id of the 2d projections
 * @param R 3x3 rotation matrix (double)
 * @param t 3x1 translation vector
 */
void SBAWrapper::InsertCamera(const cv::Mat &R, const cv::Mat &t)
{
  cameras.push_back(new Pose(R,t));
}

/**
 * Insert a 3d point and the projections to the data structure
 * Attention: The id of the projections must correspond to the camera index
 * @param pos 3d point
 * @param projs vector of projections <index_of_the_camera,2d_projection>
 */
void SBAWrapper::InsertPoints(const cv::Point3d &pos, const vector<pair<unsigned,cv::Point2d> > &projs)
{
  points.push_back(new SBAData(pos));
  
  SBAData &dat = *points.back();
  
  for (unsigned i=0; i<projs.size(); i++)
  {
    dat.InsertProjection(projs[i]);
  } 
}

/**
 * Insert a 3d point and the projections to the data structure
 * Attention: The id of the projections must correspond to the camera index
 * @param pos 3d point
 * @param projs vector of projections <index_of_the_camera,2d_projection>
 */
void SBAWrapper::InsertPoints(const cv::Point3f &pos, const vector<pair<unsigned,cv::Point2f> > &projs)
{
  points.push_back(new SBAData(pos));
  
  SBAData &dat = *points.back();
  
  for (unsigned i=0; i<projs.size(); i++)
  {
    dat.InsertProjection(projs[i]);
  } 
}

/**
 * Return the optimized cameras (and 3d points)
 */
void SBAWrapper::GetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Ptr<SBAData> > &_points) const
{
  _cameras = cameras;
  _points = points;
}

/**
 * Return the optimized cameras (and 3d points)
 */
void SBAWrapper::GetData(vector<cv::Ptr<Pose> > &_cameras, vector<cv::Point3d> &_points) const
{
  _cameras = cameras;

  _points.resize(points.size());  
  for (unsigned i=0; i<points.size(); i++)
  {
    _points[i] = points[i]->pos;
  }
}

/**
 * set camera parameter 
 */
void SBAWrapper::SetCameraParameter(const cv::Mat &intrinsic)
{
  cam = intrinsic;

  if (intrinsic.type() != CV_64F)
    intrinsic.convertTo(cam, CV_64F);
}





}





