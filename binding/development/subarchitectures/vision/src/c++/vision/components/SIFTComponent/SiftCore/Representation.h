#ifndef _REPRESENTATION_H_
#define _REPRESENTATION_H_

#include "Recognizer.h"

class Representation {
 public:
  Representation();
  ~Representation();

  void push_back(FeatureVector *pfeatures);
  int save(ofstream &file);  
  /** Reads a FeatureVector structure from an input stream. */
  int load(ifstream &file);
  
  int get_numFeatureVectors(){return v_pFeatures.size();};
  bool has_singleFeatureVector(FeatureVector *&pfeatureVector);
  int get_pfeatureVector(int iVec, FeatureVector *&pfeatureVector);

 public:
  vector< FeatureVector *>  v_pFeatures;
};

#endif
