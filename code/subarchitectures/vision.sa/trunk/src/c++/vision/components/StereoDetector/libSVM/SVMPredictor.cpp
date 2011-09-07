/**
 * @file SVMPredictor.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */


#include "SVMPredictor.h"

namespace Z
{

/**
 * @brief Constructor of SVMPredictor
 */
SVMPredictor::SVMPredictor(int max_svm, const char* filename)       /// TODO no more filename
{
  max_type_svm = max_svm;
  max_nr_attr = 64;
  predict_probability = true;
    
  struct svm_model* pp_model;
  struct svm_model* pl_model;
  struct svm_model* ll_model;

  // Load svm models
  if((pp_model = svm_load_model("./instantiations/11-05-11/PP-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open patch-patch model file.\n");
    exit(1);
  }
  if((pl_model = svm_load_model("./instantiations/11-05-11/PL-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open patch-line model file.\n");
    exit(1);
  }
  if((ll_model = svm_load_model("./instantiations/11-05-11/LL-Trainingsset.txt.model")) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open line-line model file.\n");
    exit(1);
  }
  models.push_back(pp_model);
  models.push_back(pl_model);
  models.push_back(ll_model);

  // allocate memory for model (and nodes)
  x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
  if(predict_probability)
  {
    if(svm_check_probability_model(models[0]) == 0 || 
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
       svm_check_probability_model(models[1]) == 0)
      printf("SVMPredictor::SVMPredictor: Warning: Model supports probability estimates, but disabled in prediction.\n");
  }
}

/**
 * @brief Destructor of SVMPredictor
 */
SVMPredictor::~SVMPredictor()
{
//   svm_free_and_destroy_model(&model);
  free(x);
}

/**
 * @brief Predict with SVM for a given vector.
 * @param vec Feature vector for the SVM.
 * @param prob Probability of correct prediction for each class.
 */
bool SVMPredictor::Predict(int type, const std::vector<double> &vec, std::vector<double> &prob)
{
  int svm_type = svm_get_svm_type(models[type-1]);
  int nr_class = svm_get_nr_class(models[type-1]);
  double *prob_estimates=NULL;
  int j;

  if(predict_probability)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(models[type]));
    else
    {
      int *labels=(int *) malloc(nr_class*sizeof(int));
      svm_get_labels(models[type-1], labels);
      prob_estimates = (double *) malloc(nr_class*sizeof(double));
      free(labels);
    }
  }

// printf(" SVMPredictor::Predict: after init\n");

  // we calculate only one feature vector
  double predict_label;
  for(unsigned idx = 0; idx < vec.size(); idx++)
  {
    x[idx].index = idx;
    x[idx].value = vec[idx];
    x[idx+1].index = -1;
// printf("SVMPredictor: Check index: x[idx]: %u und x[idx+1]: %i\n", x[idx].index, x[idx+1].index);
  }

// printf(" SVMPredictor::Predict: ready to predict\n");

  if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
  {
    predict_label = svm_predict_probability(models[type-1], x, prob_estimates);
//     printf("%4.3f ", predict_label);
    for(j=0;j<nr_class;j++)
    {
      prob.push_back(prob_estimates[j]);
//       printf("prob[%u]: %4.3f ", j, prob_estimates[j]);
    }
//     printf("\n");
  }
  else
  {
    predict_label = svm_predict(models[type-1], x);
//     printf("%4.3f\n", predict_label);
  }

  /// TODO check if predict_label is also target label => We need ground truth!
//     if(predict_label == target_label)
//       ++correct;
//     error += (predict_label-target_label)*(predict_label-target_label);
//   sump += predict_label;
//   sumt += target_label;
//   sumpp += predict_label*predict_label;
//   sumtt += target_label*target_label;
//   sumpt += predict_label*target_label;
//   ++total;

//   if (svm_type==NU_SVR || svm_type==EPSILON_SVR)
//   {
//     printf("Squared correlation coefficient = %g (regression)\n",
//            ((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
//            ((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
//            );
//   }
//   else
//     printf("Accuracy = %g%% (%d/%d) (classification)\n",
//            (double)correct/total*100,correct,total);
           
  if(predict_probability)
    free(prob_estimates);
  
// printf(" SVMPredictor::Predict: end!\n");
  
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











