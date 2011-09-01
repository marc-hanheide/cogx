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




namespace P
{

class OVMatches;

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
  virtual void InsertView(unsigned oidx, unsigned vidx, cv::Ptr<View> &view) = 0;
  virtual void QueryObjects(cv::Mat_<float> &queryDescriptors, vector<cv::Ptr<OVMatches> > &matches) = 0;

  virtual void Load(const string &filename) {};
  virtual void Save(const string &filename) {};
};


/**
 * matches with object view
 */
class OVMatches
{     
public: 
  unsigned oid;
  unsigned vid;
  double conf;
  vector<cv::DMatch> matches;    
  
  OVMatches(unsigned &_oid, unsigned &_vid, double c) : oid(_oid), vid(_vid), conf(c) {}
};    



/************************** INLINE METHODES ******************************/



}

#endif

