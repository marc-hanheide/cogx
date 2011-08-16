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
SVMPredictor::SVMPredictor(const char* filename)
{
  max_nr_attr = 64;           // TODO maximum relations ???
  predict_probability=true;   // use predictor with probabilities
  
  // Load svm model
  if((model=svm_load_model(filename)) == 0)
  {
    printf("SVMPredictor::SVMPredictor: Error: Can't open model file %s\n", filename);
    exit(1);
  }

  // allocate memory for model (and nodes)
  x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
  if(predict_probability)
  {
    if(svm_check_probability_model(model) == 0)
    {
      printf("SVMPredictor::SVMPredictor: Error: Model does not support probabiliy estimates\n");
      exit(1);
    }
  }
  else
  {
    if(svm_check_probability_model(model) != 0)
      printf("SVMPredictor::SVMPredictor: Warning: Model supports probability estimates, but disabled in prediction.\n");
  }
}

/**
 * @brief Destructor of SVMPredictor
 */
SVMPredictor::~SVMPredictor()
{
  svm_free_and_destroy_model(&model);
  free(x);
}

/**
 * @brief Predict with SVM for a given vector.
 * @param vec Feature vector for the SVM.
 */
bool SVMPredictor::Predict(const std::vector<double> &vec, std::vector<double> &prob)
{
//   int correct = 0;
//   int total = 0;
//   double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

  int svm_type = svm_get_svm_type(model);
  int nr_class = svm_get_nr_class(model);
  double *prob_estimates=NULL;
  int j;

  if(predict_probability)
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(model));
    else
    {
      int *labels=(int *) malloc(nr_class*sizeof(int));
      svm_get_labels(model,labels);
      prob_estimates = (double *) malloc(nr_class*sizeof(double));
      free(labels);
    }
  }

  // we calculate only one feature vector
  double target_label, predict_label;
  for(unsigned idx = 0; idx < vec.size(); idx++)
  {
    x[idx].index = idx;
    x[idx].value = vec[idx];
    x[idx+1].index = -1;
// printf("SVMPredictor: Check index: x[idx]: %u und x[idx+1]: %i\n", x[idx].index, x[idx+1].index);
  }

  if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC))
  {
    predict_label = svm_predict_probability(model, x, prob_estimates);
//     printf("%4.3f ", predict_label);
    for(j=0;j<nr_class;j++)
    {
      prob.push_back(prob_estimates[j]);
//       printf("%4.3f ", prob_estimates[j]);
    }
//     printf("\n");
  }
  else
  {
    predict_label = svm_predict(model,x);
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
  return predict_label;
}


/**
 * @brief Process the relation extraction algorithm
 */
bool SVMPredictor::GetResult(const std::vector<double> &val,
                             std::vector<double> &prob)
{
  return Predict(val, prob);
}

/**
 * @brief Process the relation extraction algorithm
 */
void SVMPredictor::GetResults(std::vector< std::vector<double> > vals)
{
  /// TODO Antiquated!
  printf("SVMPredictor::GetResults: Antiquated: Time to reimplement!\n");
//   for(unsigned i=0; i<vals.size(); i++)
//     GetResult(vals[i]);
}

// void SVMPredictor::GetResults(std::vector<Relation> &rel)
// {
//   printf("SVMPredictor::GetResult: Time to implement!\n"); 
// }



} 











