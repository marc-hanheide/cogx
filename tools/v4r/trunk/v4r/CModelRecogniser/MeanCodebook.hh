/**
 * $Id$
 * Johann Prankl, 2011-03-31 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_MEAN_CODEBOOK_HH
#define P_MEAN_CODEBOOK_HH

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
 * MeanCodebook
 */
class MeanCodebook : public Codebook
{
public:
  class Parameter
  {
  public:
    float thrDesc;                    // 0.3
    float sigmaDesc;                  // sigma for mean shift update (0.2)
    float nnRatio;
    bool useFlann;
    int numTrees;
    int searchDepth; 

    Parameter(float _thrDesc=0.3, float _sigmaDesc=0.2, float nnr=0.8, bool flann=false, int _numTrees=4, int depth=64)
     : thrDesc(_thrDesc), sigmaDesc(_sigmaDesc), nnRatio(nnr), useFlann(flann), numTrees(_numTrees), searchDepth(depth) {};
  };

private:
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<cv::DescriptorMatcher> flannMatcher;

  vector< cv::Ptr<CBEntry> > cbEntries;      // the codebook
  cv::Mat_<float> descriptors;               // a copy of the descriptors of the codebook

  bool optimized;

  unsigned N;

 
public:
  Parameter param;

  MeanCodebook(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param = Parameter());
  ~MeanCodebook();

  virtual void clear();
  virtual void Optimize();
  virtual void InsertView(unsigned idxObject,unsigned idxView,std::vector<cv::Ptr<CModel> > &objs);
  virtual void QueryObjects(cv::Mat_<float> &queryDescriptors, map<unsigned, vector<cv::DMatch> > &matches);

  //virtual void Load(const string &filename);
  //virtual void Save(const string &filename);
};





/************************** INLINE METHODES ******************************/



}

#endif

