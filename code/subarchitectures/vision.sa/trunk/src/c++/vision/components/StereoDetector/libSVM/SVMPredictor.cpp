/**
 * @file SVMPredictor.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */


#include "SVMPredictor.h"
#include "highgui.h"

namespace Z
{

/**
 * @brief Constructor of SVMPredictor
 */
SVMPredictor::SVMPredictor(int max_svm, std::vector<const char*> filenames)       /// TODO no more filename
{
printf("SVMPredictor::SVMPredictor: constructor.\n");

  if(filenames.size() != max_svm+1);

  models.clear();
  max_type_svm = max_svm;
  max_nr_attr = 64;
  predict_probability = true;
    
  struct svm_model* pp_model;
  struct svm_model* pl_model;
  struct svm_model* ll_model;

// printf("SVMPredictor::SVMPredictor: constructor 2.\n");
  // Load svm models
  if((pp_model = svm_load_model("./instantiations/11-05-11/1027/PP-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open patch-patch model file.\n");
    exit(1);
  }
  cvWaitKey(250);
// printf("SVMPredictor::SVMPredictor: constructor 3.\n");
  if((pl_model = svm_load_model("./instantiations/11-05-11/PL-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open patch-line model file.\n");
    exit(1);
  }
  cvWaitKey(250);
// printf("SVMPredictor::SVMPredictor: constructor 4.\n");
  if((ll_model = svm_load_model("./instantiations/11-05-11/LL-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open line-line model file.\n");
    exit(1);
  }
// printf("SVMPredictor::SVMPredictor: constructor 5.\n");

  models.push_back(pp_model);
  models.push_back(pl_model);
  models.push_back(ll_model);

  /// TODO Wieso kann ich das so nicht laden?
//   cvWaitKey(250);     // TODO Wait a while
//   for(unsigned i=0; i<filenames.size(); i++)
//   {
//     struct svm_model* model;
// printf("SVMPredictor: Try to load model[%u]: %s!\n", i, filenames[i]);
//     cvWaitKey(250);    // TODO Wait a while
//     if((model = svm_load_model(filenames[i])) == 0)
//     {
//       models.push_back(model);
//     }
// if (svm_check_probability_model(models[i]) == 0) printf("Double check: nicht bestanden!\n");
//   }


printf("SVMPredictor::SVMPredictor: Allocate memory.\n");
  // allocate memory for model (and nodes)
  node = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
printf("SVMPredictor::SVMPredictor: Allocate memory done.\n");
  if(predict_probability)
  {
printf("Check probability!\n");
    if(svm_check_probability_model(models[0]) == 0 ||                         /// TODO dynamisch
       svm_check_probability_model(models[1]) == 0 || 
       svm_check_probability_model(models[2]) == 0)
    {
      printf("SVMPredictor::SVMPredictor: Error: Model does not support probabiliy estimates\n");
      exit(1);
    }
  }
  else
  {
    if(svm_check_probability_model(models[0]) == 0 || 
       svm_check_probability_model(models[1]) == 0 || 
       svm_check_probability_model(models[2]) == 0)
      printf("SVMPredictor::SVMPredictor: Warning: Model supports probability estimates, but disabled in prediction.\n");
  }
printf("SVMPredictor::SVMPredictor: done!\n");
}

/**
 * @brief Destructor of SVMPredictor
 */
SVMPredictor::~SVMPredictor()
{
  for(size_t i=0; i< models.size(); i++)
    svm_free_and_destroy_model(&models[i]);
  free(node);
}

/**
 * @brief Predict with SVM for a given vector.
 * @param type Type of feature vector: Type of svm-model
 * @param vec Feature vector for the SVM.
 * @param prob Probability of correct prediction for each class.
 * @return Returns the prediction label
 */
bool SVMPredictor::Predict(int type, const std::vector<double> &vec, std::vector<double> &prob)
{
  int svm_type = svm_get_svm_type(models[type-1]);
  int nr_class = svm_get_nr_class(models[type-1]);
  double *prob_estimates = NULL;
  int j;

  if(predict_probability)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(models[type]));
    else
      prob_estimates = (double *) malloc(nr_class*sizeof(double));
  }

  // we copy now the feature vector
  double predict_label;
  for(unsigned idx = 0; idx < vec.size(); idx++)
  {
    node[idx].index = idx;
    node[idx].value = vec[idx];
    node[idx+1].index = -1;
  }

  if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
  {
    predict_label = svm_predict_probability(models[type-1], node, prob_estimates);                 /// TODO How long takes one prediction???
    for(j=0;j<nr_class;j++)
      prob.push_back(prob_estimates[j]);
  }
  else
    predict_label = svm_predict(models[type-1], node);

  if(predict_probability)
    free(prob_estimates);
  
  return predict_label;
}


/**
 * @brief Process the relation extraction algorithm
 * @param type Type of SVM relation
 * @param val Feature vector of relation
 * @param prob Probability for correct predicton
 */
bool SVMPredictor::GetResult(int type,
                             const std::vector<double> &val,
                             std::vector<double> &prob)
{
  return Predict(type, val, prob);
}

} 











