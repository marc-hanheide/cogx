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

  struct svm_node *node;                        ///< node of svm
  int max_nr_attr;                              ///< Maximum attributes = maximum size of feature vector

  int max_type_svm;                             ///< Maximum types of SVM's
  std::vector<struct svm_model*> models;        ///< SVM-models
  bool predict_probability;                     ///< Predict with probability values

  bool Predict(int type, const std::vector<double> &vec, std::vector<double> &prob);
  
  
public:
  SVMPredictor(int max_svm, std::vector<const char*> filenames);// const char* filename = "svm.model");
  ~SVMPredictor();
  
  bool GetResult(int type, const std::vector<double> &val, std::vector<double> &prob);
};

}

#endif

