/**
 * $Id$
 * Johann Prankl, 2011-02-24 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_VOCABULARY_TREE_HH
#define P_VOCABULARY_TREE_HH

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "CModel.hh"
#include "View.hh"
#include "Codebook.hh"



namespace P
{

/**
 * VTNode
 */
class VTNode
{
public:
  cv::Mat_<float> descriptors;
  vector< cv::Ptr<VTNode> > nodes;

  unsigned nb;
  const unsigned &N;
  unsigned ni;                    // query keypoint counter
  float wi;                       // weight of this node ( ln(N/Ni) )
  map<unsigned,vector<unsigned> > links;   // links to object views and keypoints <idxObject,viewKey> 

  VTNode(unsigned &_N) : nb(0), N(_N), wi(0.) {};
  inline unsigned Ni() { return links.size(); }       // number of objects through this node
  inline void Insert(unsigned vidx, unsigned kidx)
  {
    links[vidx].push_back(kidx); 
    wi = log((float)N/(float)Ni());
  }
};


/**
 * VocabularyTree
 */
class VocabularyTree : public Codebook
{
public:
  class Parameter
  {
  public:
    unsigned k;     // branch factor (10)
    unsigned l;     // levels (4)
    unsigned numRandCenters; // number of random centers 5
    int flags;
    Parameter(unsigned _k=50, unsigned _l=2, unsigned nbr=5, int _flags=cv::KMEANS_PP_CENTERS) 
     : k(_k), l(_l), numRandCenters(nbr), flags(_flags) {};
  };

private:
  cv::Mat_<float> descs;              // temp descriptor for clustering
  cv::Mat_<int> labels;               // temp labels for clustering
  vector< cv::Mat_<float> > descMats; // collection of mats to cluster

  static unsigned nbcnt;
  unsigned N;                           // number of objects in the database
  vector< cv::Ptr<View> > views;             // object views
  bool haveNorm;
  vector<float> norm_m;                 // norms for object models
  vector<pair<unsigned, float> > diff;  // diff object_model/query
  cv::Ptr<VTNode> root;                 // root node of vocabulary tree

  cv::Ptr<cv::DescriptorMatcher> matcher;

  void SumDiff(VTNode &node, vector<short> &path, int &z, vector<float> &norm_m, float norm_n, vector<pair<unsigned,float> > &diff);
  void QueryDescriptor(cv::Ptr<VTNode> &node, cv::Mat_<float> &desc, vector<short> &path, cv::Ptr<VTNode> &matchNode, vector<cv::DMatch> &tmp);
  void Reset_ni(VTNode &node);
  void Norm2_m(VTNode &node, vector<float> &norm_m);
  void Norm2_n(VTNode &node, float &norm_n);
  void AssignDescriptor(VTNode &node, cv::Mat_<float> &desc, unsigned vidx, unsigned kidx, vector<cv::DMatch> &tmp);
  void Clear(VTNode &node);

  void CreateNodes(VTNode &node, vector<float*> &ptDescs, unsigned &sizeDescs, unsigned &l);
  void Write(ofstream &os, VTNode &node);
  void Read(ifstream &is, VTNode &node);


 
public:
  Parameter param;

  VocabularyTree(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param = Parameter());
  ~VocabularyTree();

  bool HaveTree(){return root->nodes.size();};
  void PrecomputeNorm2_m();

  void AddForClustering(const cv::Ptr<CModel> &model);
  void AddForClustering(const cv::Mat_<float> &descriptors);
  void CreateTree();

  virtual void clear();
  virtual void InsertView(unsigned oidx, unsigned vidx, cv::Ptr<View> &view); 
  virtual void QueryObjects(cv::Mat_<float> &descriptors, vector<cv::Ptr<OVMatches> > &matches);
  virtual void Load(const string &filename);
  virtual void Save(const string &filename);
};





/************************** INLINE METHODES ******************************/



}

#endif

