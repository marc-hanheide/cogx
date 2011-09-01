/**
 * $Id$
 */


#include "MeanShift.hh"

#define DEBUG

#define MS_MAX_ITER 100 //500 //100     // max. number of iterations
#define MS_LAMBDA 200 //40 //50 //100       // threshold  (flat part of the kernel)
#define MS_SIGMA 30 //40 //30 //10 //10 //20 //50         // sigma (gaussian part of the kernel)
#define MS_THR_PRUNING 5 //1 //5 //20   // threshold for early pruning of clusters
#define MS_INIT_PCENT 1.    // [0..1] defines initialisation points



namespace P 
{



/********************** MeanShift ************************
 * Constructor/Destructor
 */
MeanShift::MeanShift()
 : sqrLambda(MS_LAMBDA*MS_LAMBDA), sqrThrPruning(MS_THR_PRUNING*MS_THR_PRUNING),
   initPcent(MS_INIT_PCENT), maxIter(MS_MAX_ITER), maxDist(PMath::eps)
{
  invSqrSigma = 1. / (MS_SIGMA*MS_SIGMA);
}

/**
 * Mean shift clustering
 * @param lambda       Threshold of flat kernel
 * @param sigma        Sigma of the gaussian part
 * @param thrPrune     Threshold for early pruning of clusters
 * @param initP        Number of points to init = initP * number of samples [0..1]
 * @param thrIter      Max. number of iterations
 * @param epsConverge  Final convergence eps
 */
MeanShift::MeanShift(double lambda, double sigma, double thrPrune, double initP, unsigned thrIter, double epsConverge)
 : sqrLambda(lambda*lambda), sqrThrPruning(thrPrune*thrPrune),
   initPcent(initP), maxIter(thrIter), maxDist(epsConverge)
{
  invSqrSigma = 1. / (sigma*sigma);
}

MeanShift::~MeanShift()
{
}

/**
 * Add a sample to the nearest cluster center
 */
void MeanShift::AddSampesToCluster(vector<ClusterSample *> &samples, IMCluster* cluster)
{
  for (unsigned i=0; i<samples.size(); i++)
  {
    if (cluster->DistSqr(samples[i]->GetVec()) < sqrLambda)
    {
      cluster->samples.push_back(samples[i]);
    }
  }
}

/**
 * Compute sample mean
 */
void MeanShift::ComputeMean(IMCluster *cluster)
{
  cluster->SetZero();
  for (unsigned i=0; i<cluster->samples.size(); i++)
    cluster->Add(cluster->samples[i]->GetVec());
  cluster->Div(cluster->ClusterSize());
}

/**
 * Compute new sample mean
 * return true if converged
 */
int MeanShift::UpdateMean(vector<ClusterSample *> &samples, IMCluster* cluster)
{
  IMCluster mean(cluster->GetVec(), cluster->GetSize());
  double weight, sumWeight = 0.;

  cluster->SetZero();

  for (unsigned i=0; i<samples.size(); i++)
  {
    weight = TruncatedGaussian( mean.DistSqr(samples[i]->GetVec()) );
    cluster->AddMul(samples[i]->GetVec(),weight);
    sumWeight += weight;
  }


  if (sumWeight > PMath::eps) 
  {
    cluster->Mul(1./sumWeight);
  }
  else
    return -1;

  if (IsConverge(mean.vec,cluster->vec, cluster->GetSize()))
    return 1;

  return 0;
}

void MeanShift::InitClusters(vector<ClusterSample *> &samples, vector<IMCluster* > &clusters)
{
  if (samples.size()<1)
    return;

  unsigned vecSize = samples[0]->GetSize();
  
  if (fabs(initPcent-1.) < PMath::eps)
  {
    for (unsigned i=0; i<samples.size(); i++)
      clusters.push_back(new IMCluster(samples[i]->GetVec(), vecSize));
  }
  else
  {
    unsigned num = (unsigned)(initPcent*(double)samples.size());
    num = (num < 1 ? 1 : (num > samples.size() ? samples.size() : num) );

    for (unsigned i=0; i<num; i++)
    {
      clusters.push_back(new IMCluster(samples[rand()%samples.size()]->GetVec(), vecSize));
    }
  }
}






/****************************** PUBLIC *************************/
/**
 * Mean shift clustering
 */
void MeanShift::Cluster(vector<ClusterSample *> &samples, vector<IMCluster* > &clusters)
{
  if (samples.size()==0)
    return;

  srand(time(NULL));
  DeleteIMClusters(clusters);
  vector<bool> converged(samples.size());
  for (unsigned i=0; i<converged.size(); i++) converged[i]=false;

  InitClusters(samples, clusters);

  unsigned it=0;
  bool  changed = true;
  int status;
  while(changed && it<maxIter)
  {
    changed = false;

    for (unsigned i=0; i<clusters.size(); i++)
    {
      if (!converged[i])
      {
        status = UpdateMean(samples, clusters[i]);

        if (status == 1)
        {
          converged[i]=true;
        }
        else 
        {
          if (status == 0)
          {
            changed |= true;
          }
          else
          {
            delete clusters[i];
            clusters.erase(clusters.begin()+i);
            converged.erase(converged.begin()+i);
            i--;
          }
        }
      }
    }

    //pruning of clusters
    for (unsigned i=0; i<clusters.size(); i++)
    {
      for (unsigned j=i+1; j<clusters.size(); j++)
      {
        if (clusters[i]->DistSqr(clusters[j]->GetVec()) < sqrThrPruning)
        {
          delete clusters[j];
          clusters.erase(clusters.begin()+j);
          converged.erase(converged.begin()+j);
          j--;
        }
      }
    }
    it++;
  }

  if (clusters.size()==0) return;

  //Assign samples to nearest cluster
  unsigned idx;
  double dist, minDist;
  for (unsigned i=0; i<samples.size(); i++)
  {
    idx=UINT_MAX, minDist=DBL_MAX;
    for (unsigned j=0; j<clusters.size(); j++)
    {
      dist = clusters[j]->DistSqr(samples[i]->GetVec());
      if (dist < minDist)
      {
        minDist = dist;
        idx=j;
      }
    }

    if (idx!=UINT_MAX) clusters[idx]->samples.push_back(samples[i]);
  }
}

/**
 * Mean shift clustering
 */
void MeanShift::Cluster(cv::Mat_<double> &samples, vector<vector<unsigned> > &clusters)
{
  vector<IMCluster* > cls;
  vector<ClusterSample*> sam(samples.rows);

  //prepare data
  for (unsigned i=0; i<sam.size(); i++)
    sam[i] = new ClusterSample(&samples(i,0),samples.cols,i);

  //cluster
  Cluster(sam, cls);

  //return result
  clusters.resize(cls.size());
  for (unsigned i=0; i<cls.size(); i++)
  {
    clusters[i].clear();
    for (unsigned j=0; j<cls[i]->samples.size(); j++)
      clusters[i].push_back(cls[i]->samples[j]->id);
  }

  DeleteIMClusters(cls);
  DeleteClusterSamples(sam);
}


}

