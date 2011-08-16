/**
 * @file SVMPredictor.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#ifndef Z_SVM_PREDICTOR_H
#define Z_SVM_PREDICTOR_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "SVMPredictor.h"
#include "svm.h"

namespace Z
{
  
/**
 * @brief Class SVMPredictor: 
 */
class SVMPredictor
{
 
public:
  
private:

  struct svm_node *x;             ///< node of svm
  int max_nr_attr;                ///< Maximum attributes = maximum size of feature vector

  struct svm_model* model;        ///< SVM-model
  bool predict_probability;       ///< Predict with probability values

  bool Predict(const std::vector<double> &vec, std::vector<double> &prob);
  
  
public:
  SVMPredictor(const char* filename = "svm.model");
  ~SVMPredictor();
  
  bool GetResult(const std::vector<double> &val, std::vector<double> &prob);
  void GetResults(std::vector< std::vector<double> > vals);
//   void GetResults(std::vector<Relation> &rel);
};

}

#endif

