/**
 * $Id$
 */


#include "SphereHistogram.hh"


namespace P 
{






/********************** SphereHistogram ************************
 * Constructor/Destructor
 */
SphereHistogram::SphereHistogram(unsigned subdevisions)
 : sumWeight(0.)
{
  Init(subdevisions);
}

SphereHistogram::~SphereHistogram()
{
  Release();
}

/**
 * Init icosahedron
 */
void SphereHistogram::InitIcosahedron()
{
  double t = (1.+sqrt(5.))/2.;
  double tau = t/sqrt(1.+t*t);
  double one = 1./sqrt(1.+t*t);

  vertices.resize(12);

  vertices[0] = cv::Point3d(tau, one, 0.0);
  vertices[1] = cv::Point3d(-tau, one, 0.0);
  vertices[2] = cv::Point3d(-tau, -one, 0.0);
  vertices[3] = cv::Point3d(tau, -one, 0.0);
  vertices[4] = cv::Point3d(one, 0.0 ,  tau);
  vertices[5] = cv::Point3d(one, 0.0 , -tau);
  vertices[6] = cv::Point3d(-one, 0.0 , -tau);
  vertices[7] = cv::Point3d(-one, 0.0 , tau);
  vertices[8] = cv::Point3d(0.0 , tau, one);
  vertices[9] = cv::Point3d(0.0 , -tau, one);
  vertices[10] = cv::Point3d(0.0 , -tau, -one);
  vertices[11] = cv::Point3d(0.0 , tau, -one);

  unsigned icosahedron_faces[] = {
              4, 8, 7,
              4, 7, 9,
              5, 6, 11,
              5, 10, 6,
              0, 4, 3,
              0, 3, 5,
              2, 7, 1,
              2, 1, 6,
              8, 0, 11,
              8, 11, 1,
              9, 10, 3,
              9, 2, 10,
              8, 4, 0,
              11, 0, 5,
              4, 9, 3,
              5, 3, 10,
              7, 8, 1,
              6, 1, 11,
              7, 2, 9,
              6, 10, 2};

  for (unsigned i=0; i<20; i++)
  {
    startFaces.push_back(new SFace());
    memcpy ((void*)&startFaces[i]->vs[0], (void*)&icosahedron_faces[i*3], 3*sizeof(unsigned));
  }

  subdivFaces=startFaces;
}


/**
 * SearchMidpoint
 */
unsigned SphereHistogram::SearchMidpoint (unsigned idxStart, unsigned idxEnd)
{
  for (unsigned i=0; i<edgeWalk; i++)
    if ((start[i] == idxStart && end[i] == idxEnd) ||
        (start[i] == idxEnd && end[i] == idxStart))
    {
      unsigned res = midpoint[i];

      // update the arrays
      start[i]    = start[edgeWalk-1];
      end[i]      = end[edgeWalk-1];
      midpoint[i] = midpoint[edgeWalk-1];
      edgeWalk--;

      return res;
    }

  // vertex not in the list, so we add it
  start[edgeWalk] = idxStart;
  end[edgeWalk] = idxEnd;
  midpoint[edgeWalk] = vertices.size();

  // create new vertex
  vertices.push_back(cv::Point3d(
    (vertices[idxStart].x + vertices[idxEnd].x) / 2.,
    (vertices[idxStart].y + vertices[idxEnd].y) / 2.,
    (vertices[idxStart].z + vertices[idxEnd].z) / 2.
  ));

  // normalize the new vertex
  PVec::Normalise3(&vertices.back().x, &vertices.back().x);
  edgeWalk++;
  return midpoint[edgeWalk-1];
}


/**
 * Subdevide face
 */
void SphereHistogram::Subdevide()
{
  edgeWalk = 0;
  unsigned numEdges = 2*vertices.size() + 3*subdivFaces.size();

  start.resize(numEdges);
  end.resize(numEdges);
  midpoint.resize(numEdges);

  vector<SFace*> oldFaces = subdivFaces;
  subdivFaces.resize(4*oldFaces.size());

  unsigned z=0;
  for (unsigned i=0; i<oldFaces.size(); i++)
  {
    unsigned a = oldFaces[i]->vs[0];
    unsigned b = oldFaces[i]->vs[1];
    unsigned c = oldFaces[i]->vs[2];

    unsigned ab_midpoint = SearchMidpoint(b, a);
    unsigned bc_midpoint = SearchMidpoint(c, b);
    unsigned ca_midpoint = SearchMidpoint(a, c);

    subdivFaces[z] = new SFace();
    subdivFaces[z]->vs[0] = a;
    subdivFaces[z]->vs[1] = ab_midpoint;
    subdivFaces[z]->vs[2] = ca_midpoint;
    oldFaces[i]->subFaces.push_back(subdivFaces[z]);
    z++;
    subdivFaces[z] = new SFace();
    subdivFaces[z]->vs[0] = ca_midpoint;
    subdivFaces[z]->vs[1] = ab_midpoint;
    subdivFaces[z]->vs[2] = bc_midpoint;
    oldFaces[i]->subFaces.push_back(subdivFaces[z]);
    z++;
    subdivFaces[z] = new SFace();
    subdivFaces[z]->vs[0] = ca_midpoint;
    subdivFaces[z]->vs[1] = bc_midpoint;
    subdivFaces[z]->vs[2] = c;
    oldFaces[i]->subFaces.push_back(subdivFaces[z]);
    z++;
    subdivFaces[z] = new SFace();
    subdivFaces[z]->vs[0] = ab_midpoint;
    subdivFaces[z]->vs[1] = b;
    subdivFaces[z]->vs[2] = bc_midpoint;
    oldFaces[i]->subFaces.push_back(subdivFaces[z]);
    z++;
  }

  start.clear();
  end.clear();
  midpoint.clear();
}

/**
 * Compute normals of the current subdevided faces
 */
void SphereHistogram::ComputeNormals()
{
  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    Plane3dExpNormal(&vertices[subdivFaces[i]->vs[0]].x,
                     &vertices[subdivFaces[i]->vs[1]].x,
                     &vertices[subdivFaces[i]->vs[2]].x,
                     &subdivFaces[i]->n.x);
  }
}

/**
 * Copy a face
 */
void SphereHistogram::CopyFace(SFace *src, SFace *dst, vector<SFace*> &subdiv)
{
  dst->vs[0]=src->vs[0], dst->vs[1]=src->vs[1], dst->vs[2]=src->vs[2];
  dst->n = src->n;
  dst->weight = src->weight;
  dst->views = src->views;

  if (src->subFaces.size()==0)
  {
    subdiv.push_back(dst);
  }
  else
  {
    for (unsigned i=0; i<src->subFaces.size(); i++)
    {
      dst->subFaces.push_back(new SFace());
      CopyFace(src->subFaces[i], dst->subFaces.back(), subdiv);
    }
  }
}

/**
 * deep copy of a SphereHistogram
 */
void SphereHistogram::DeepCopy(const SphereHistogram *src, SphereHistogram *dst)
{
  dst->Release();

  dst->vertices = src->vertices;
  dst->sumWeight = src->sumWeight;
  for (unsigned i=0; i<src->startFaces.size(); i++)
  {
    dst->startFaces.push_back(new SFace());
    CopyFace(src->startFaces[i], dst->startFaces.back(), dst->subdivFaces);
  }
}

/**
 * find best match
 */
bool SphereHistogram::FindMatch(cv::Point3d &n, vector<SFace*> *in, vector<SFace*> **out, SFace **match)
{
  *match=0;
  double tmp, min=DBL_MIN;

  for (unsigned i=0; i<(*in).size(); i++)
  {
    tmp = PVec::Dot3(&n.x, &(*in)[i]->n.x);
    if (tmp > min)
    {
      min = tmp;
      (*match) = (*in)[i];
    }
  }

  if ((*match)->subFaces.size() > 0)
  {
    (*out) = &((*match)->subFaces);
    return false;
  }

  return true;
}

/**
 * SetNeighbours
 */
void SphereHistogram::SetNeighbours()
{
  SFace *face;
  double dist, dist1, dist2, dist3;

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    face = subdivFaces[i];
    dist1 = dist2 = dist3 = 0.;
    for (unsigned j=0; j<subdivFaces.size(); j++)
    {
      if (i!=j)
      {
        dist = PVec::Dot3(&subdivFaces[i]->n.x,&subdivFaces[j]->n.x);
        if (dist > dist1)
        {
          dist3 = dist2;
          dist2 = dist1;
          dist1 = dist;
          face->neighbours[2] = face->neighbours[1];
          face->neighbours[1] = face->neighbours[0];
          face->neighbours[0] = subdivFaces[j];
        }
        else
        {
          if (dist > dist2)
          {
            dist3 = dist2;
            dist2 = dist;
            face->neighbours[2] = face->neighbours[1];
            face->neighbours[1] = subdivFaces[j];
          }
          else
          {
            if (dist > dist3)
            {
              dist3 = dist;
              face->neighbours[2] = subdivFaces[j];
            }
          }
        }
      }
    }
  }
}






/************************* PUBLIC **********************************/

/**
 * clear()
 */
void SphereHistogram::Release()
{
  for (unsigned i=0; i<startFaces.size(); i++)
    delete startFaces[i];
  startFaces.clear();
  subdivFaces.clear();
  vertices.clear();

  sumWeight=0.;
}

/**
 * Init indexing sphere
 */
void SphereHistogram::Init(unsigned subdevisions)
{
  Release();
  InitIcosahedron();  
  ComputeNormals();
  
  for (unsigned i=0; i<subdevisions; i++)
  {
    Subdevide();
    ComputeNormals();
  }

  SetNeighbours();
}

/**
 * Let a normal vector index to the sphere
 */
void SphereHistogram::Insert(cv::Point3d n, double weight, unsigned id)
{
  if (startFaces.size()==0)
    throw runtime_error("SphereHistogram::Insert : Please init indexing sphere!");

  bool terminated;

  SFace *match;
  vector<SFace*> *faces = &startFaces;

  do{
    terminated = FindMatch(n, faces, &faces, &match);

    if (match != 0)
    {
      match->views.insert(id);
      match->weight += weight;
    }
  }while(!terminated);

  sumWeight += weight;
}

/**
 * Get face for an vector
 */
SFace* SphereHistogram::GetFace(cv::Point3d n)
{
  if (startFaces.size()==0)
    throw runtime_error("SphereHistogram::GetFace : Please init indexing sphere!");;

  bool terminated;
  
  SFace *match, *saved=0;
  vector<SFace*> *faces = &startFaces;

  do{
    terminated = FindMatch(n, faces, &faces, &match);

    if (match != 0)
      saved=match;

  }while(!terminated);

  return saved;
}

/**
 * insert weighted depending on weightfunc
 */
void SphereHistogram::InsertMax(unsigned idx, cv::Point3d vr, ProbModel &pred)
{
  if (startFaces.size()==0)
    throw runtime_error("SphereHistogram::Insert Please init indexing sphere!");

  double weight, da;

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    da = PVec::Dot3(&vr.x,&subdivFaces[i]->n.x);
    if (da > 0)
    {
      weight = pred.PredictProbFromAngle(acos(da));
      if (weight > subdivFaces[i]->weight)
      {
        sumWeight -= subdivFaces[i]->weight;
        sumWeight += weight;
        subdivFaces[i]->weight = weight;
        subdivFaces[i]->idx = idx;
      }
    }
  }
}

/**
 * GetViewRays
 */
void SphereHistogram::GetViewRays(vector<cv::Point3d> &vr, double minScore, double maxScore)
{
  vr.clear();
  SFace *face;

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    face = subdivFaces[i];
    if (face->weight>minScore && face->weight<maxScore)
    {
      vr.push_back(face->n);
    }
  }

}

/**
 * searches for the maximum weight of the most detailed faces
 */
double SphereHistogram::GetMax()
{
  double max=0.;
  
  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    if (subdivFaces[i]->weight > max)
      max = subdivFaces[i]->weight;
  }

  return max;
}

/**
 * search for global minimum
 */
void SphereHistogram::GetMin(cv::Point3d vr, double &min)
{
  min=DBL_MAX;

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    if (subdivFaces[i]->weight < min)
    {
      min = subdivFaces[i]->weight; 
      vr = subdivFaces[i]->n;
    }
  }
}

/**
 * normalise the the most detailed sphere to ..
 */
void SphereHistogram::Normalise(double norm)
{
  if (PMath::IsZero(norm))
    return;

  sumWeight /= norm;

  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    subdivFaces[i]->weight /= norm;
  }
}

/**
 * normalise the the most detailed sphere to ..
 */
void SphereHistogram::Normalise()
{
  Normalise(sumWeight);
}

/**
 * Copy operator
 */
SphereHistogram& SphereHistogram::operator=(const SphereHistogram &sphere)
{   
  DeepCopy((SphereHistogram*)&sphere, this);
  this->SetNeighbours();
  return *this;
}


/**
 * Clear weights and links
 */
void SphereHistogram::Clear()
{ 
  for (unsigned i=0; i<subdivFaces.size(); i++)
  {
    subdivFaces[i]->weight=0.;
    subdivFaces[i]->views.clear();
  }  
}

/**
 * deep copy...
 */
void SphereHistogram::copyTo(SphereHistogram &dst)
{
  DeepCopy(this, &dst);
}


}

