/**
 * $Id$
 * Johann Prankl, 2011-03-31 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_MS_CODEBOOK_HH
#define P_MS_CODEBOOK_HH

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "CModel.hh"
#include "View.hh"
#include "CBEntry.hh"
#include "Codebook.hh"




namespace P
{

/**
 * MSCodebook
 */
class MSCodebook : public Codebook
{
public:
  class Parameter
  {
  public:
    float thrDesc;                    // threshold for matching descriptors (0.3)
    float sigmaDesc;                  // sigma for mean shift update (0.1)
    bool useFlann;
    int numTrees;
    int searchDepth; 

    Parameter(float _thrDesc=0.3, float _sigmaDesc=0.1, bool flann=false, int _numTrees=4, int depth=64)
     : thrDesc(_thrDesc), sigmaDesc(_sigmaDesc), useFlann(flann), numTrees(_numTrees), searchDepth(depth) {};
  };

private:
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<cv::DescriptorMatcher> flannMatcher;

  vector< cv::Ptr<CBEntry> > cbEntries;      // the codebook
  cv::Mat_<float> descriptors;               // a copy of the descriptors of the codebook
  vector< cv::Ptr<View> > views;             // object views

  bool optimized;

  unsigned N;
  bool haveNorm;
  vector<float> norm_m;                      // norms for object views
  vector<pair<unsigned, float> > diff;       // diff object_model / query

  void Reset_ni();
  float Norm2_n();
  void PrecomputeNorm2_m();

 
public:
  Parameter param;

  MSCodebook(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param = Parameter());
  ~MSCodebook();

  virtual void clear();
  virtual void Optimize();
  virtual void InsertView(unsigned oidx, unsigned vidx, cv::Ptr<View> &view); 
  virtual void QueryObjects(cv::Mat_<float> &queryDescriptors, vector<cv::Ptr<OVMatches> > &matches);

  //virtual void Load(const string &filename);
  //virtual void Save(const string &filename);
};





/************************** INLINE METHODES ******************************/



}

#endif

