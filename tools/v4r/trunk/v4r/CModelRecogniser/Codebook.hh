/**
 * $Id$
 * Johann Prankl, 2011-03-31 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_CODEBOOK_HH
#define P_CODEBOOK_HH

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include "PKeypoint.hh"
#include "View.hh"
#include "CModel.hh"




namespace P
{

class ObjectMatches;

/**
 * Codebook
 */
class Codebook
{
private:
 
public:
  Codebook() {}
  ~Codebook() {}

  virtual void clear() {}
  virtual void Optimize() {}
  virtual void InsertView(unsigned idxObject,unsigned idxView,std::vector<cv::Ptr<CModel> > &objs) = 0;
  virtual void QueryObjects(cv::Mat_<float> &queryDescriptors, map<unsigned, vector<cv::DMatch> > &matches) = 0;

  virtual void Load(const string &filename) {};
  virtual void Save(const string &filename) {};
};


/**
 * matches with object
 */
class ObjectMatches
{     
public: 
  unsigned idxObject;
  double conf;

  vector<cv::DMatch> matches;    
  
  ObjectMatches(unsigned &oidx, double c) : idxObject(oidx), conf(c) {}
};    



/************************** INLINE METHODES ******************************/



}

#endif

