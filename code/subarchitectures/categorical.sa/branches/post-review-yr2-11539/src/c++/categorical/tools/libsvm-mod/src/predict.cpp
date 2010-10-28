/**
 * The modified and extended svm_predict function and some tools.
 *
 * Author:
 *   Andrzej Pronobis
 *   pronobis@nada.kth.se
 */

#include "predict.h"
#include "confidence.h"
#include "svm.h"


predict_parameter::~predict_parameter()
{
  if (dasA)
  {
    for(int i=0; i<cues; ++i)
      if (dasA[i])
        free(dasA[i]);
    free(dasA);
  }
}


/**
 * Calculates f(x) for all cues and hyperplanes for the OaA multiclass method.
 */
void calculateF_OaA(const svm_model * const *models, const svm_node * const *xes,
                    double **fs, const predict_parameter *pred_param)
{
  int nr_cues = pred_param->cues; // Number of cues
  int nr_hyp = models[0]->nr_class; // Number of hyperplanes

  for (int c=0; c<nr_cues; ++c)
  {
    int l = models[c]->l; // Number of SVs
    // Add rho
    for(int i=0; i<nr_hyp; ++i)
    {
      fs[c][i] = -models[c]->rho[i];
    }
    // Calculate K(xsv,x)
    for (int i=0; i<l; ++i)
    {
      double kv = Kernel::k_function(xes[c], models[c]->SV[i], models[c]->param);
      for (int j=0; j<nr_hyp; ++j)
        fs[c][j] += models[c]->sv_coef[j][i]*kv;
    }
  }
}



/**
 * Performs prediction for the One-Against-All multiclass method.
 */
bool predictOneAgainstAll(const svm_model * const *models, const svm_node * const *xes,
                          double **fs, const predict_parameter *pred_param, double target,
                          double *v, int *classes, double *confidence, double *outputs, 
                          int* outputLabels1, int *outputLabels2, int *outputCount)
{
  // Initialize
  int nr_class = models[0]->nr_class; // Number of classes
  int nr_cues = pred_param->cues; // Number of cues
  int nr_hyp = nr_class; // Number of hyperplanes
  bool correct; // True if the classification result is correct for K matches
  int winnerIdx; // Index of the winning class

  // Allocate memory
  int freeClasses=false; // If true, the classes table has been created here
  int freeConfidence=false; // If true, the confidence table has been created here
  int freeOutputs=false; // If true, the scores table has been created here
  int freeFValues=false; // If true, the fvalues table has been created here
  double *eval = Malloc(double,nr_class); // Some kind of evaluation of each class
  if (!classes)
  {
    freeClasses=true;
    classes = Malloc(int, nr_class);
  }
  if (!confidence)
  {
    freeConfidence=true;
    confidence = Malloc(double, nr_class);
  }
  if (!outputs)
  {
    freeOutputs=true;
    outputs = Malloc(double, nr_hyp);
    outputLabels1 = Malloc(int, nr_hyp);
    outputLabels2 = Malloc(int, nr_hyp);
  }
  if (!fs) // f(x)
  {
    freeFValues=true;
    fs = Malloc(double*, nr_cues);
    for (int i=0; i<nr_cues; ++i)
      fs[i]=Malloc(double, nr_hyp);
  }

  // Calculate f_i(x) if feature vectors are available
  if (xes)
    calculateF_OaA(models, xes, fs, pred_param);

  // Choose the algorithm
  switch(pred_param->oaaAlg)
  {
  case 2: // Alg. 2
    computeEvalAlg2_OaA(fs, models, pred_param, eval);
    getOutputsAlg2_OaA(models[0], eval, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_hyp;
    winnerIdx=getWinnerIdxAlg2_OaA(models[0], eval);
    getConfidenceAlg2_OaA(models[0], eval, winnerIdx, classes, confidence);
    break;
  case 3: // Alg. 3
    computeEvalAlg3_OaA(fs, models, pred_param, eval);
    getOutputsAlg3_OaA(models[0], eval, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_hyp;
    winnerIdx=getWinnerIdxAlg3_OaA(models[0], eval);
    getConfidenceAlg3_OaA(models[0], eval, winnerIdx, classes, confidence);
    break;
  default: // Alg. 1
    computeEvalAlg1_OaA(fs, models, pred_param, eval);
    getOutputsAlg1_OaA(models[0], eval, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_hyp;
    winnerIdx=getWinnerIdxAlg1_OaA(models[0], eval);
    getConfidenceAlg1_OaA(models[0], eval, winnerIdx, classes, confidence);
    break;
  };

  // Check if one of the pred_param->matches best matches is the target class
  correct=checkMatches(pred_param, target, classes);

  // Clean-up
  free(eval);
  if (freeConfidence) free(confidence);
  if (freeOutputs)
  {
    free(outputs);
    free(outputLabels1);
    free(outputLabels2);
  }
  if (freeClasses) free(classes);
  if (freeFValues)
  {
    for (int i=0; i<nr_cues; ++i)
      free(fs[i]);
    free(fs);
  }

  // Return result
  //(*v)=model->label[winnerIdx];
  (*v)=classes[0];
  return correct;
}



/**
 * Calculates f(x) for all cues and hyperplanes for the OaA multiclass method.
 */
void calculateF_OaO(const svm_model * const *models, const svm_node * const *xes,
                    double **fs, const predict_parameter *pred_param)
{
  int nr_cues = pred_param->cues; // Number of cues
  int nr_class = models[0]->nr_class; // Number of hyperplanes

  for (int c=0; c<nr_cues; ++c)
  {
    int l = models[c]->l; // Number of SVs
    // Calculate K(x_sv, x)
    double *kvalue = Malloc(double,l);
    for(int i=0; i<l; ++i)
      kvalue[i] = Kernel::k_function(xes[c],models[c]->SV[i],models[c]->param);
    // Prepare the start table indicating where SVs for which class start
    int *start = Malloc(int,nr_class);
    start[0] = 0;
    for(int i=1; i<nr_class; ++i)
      start[i] = start[i-1]+models[c]->nSV[i-1];
    // Calculate f(x)
    int p=0;
    for(int i=0; i<nr_class; ++i)
      for(int j=i+1; j<nr_class; ++j)
      {
        // Initialize
        double sum = 0;
        int si = start[i];
        int sj = start[j];
        int ci = models[c]->nSV[i];
        int cj = models[c]->nSV[j];
        // Calculate f(x)
        int k;
        double *coef1 = models[c]->sv_coef[j-1];
        double *coef2 = models[c]->sv_coef[i];
        for(k=0;k<ci;k++)
          sum += coef1[si+k] * kvalue[si+k];
        for(k=0;k<cj;k++)
          sum += coef2[sj+k] * kvalue[sj+k];
        sum -= models[c]->rho[p];
        // Save computed value to array f
        fs[c][p]=sum;
        ++p;
      }
    //
    // Free tables that are not used anymore
    free(kvalue);
    free(start);
  }
}


/**
 * Performs prediction for the One-Against-One multiclass method.
 */
bool predictOneAgainstOne(const svm_model * const *models, const svm_node * const *xes,
                          double **fs, const predict_parameter *pred_param, double target,
                          double *v, int *classes, double *confidence, double *outputs, 
                          int* outputLabels1, int *outputLabels2, int *outputCount)
{
  // Initialize
  int nr_class = models[0]->nr_class; // Number of classes
  int nr_cues = pred_param->cues; // Number of cues
  int nr_hyp = nr_class*(nr_class-1)/2; // Number of hyperplanes
  bool correct; // True if the classification result is correct for K matches
  int winnerIdx=0; // Index of the winning class

  // Allocate memory
  int freeClasses=false; // If true, the classes table has been created here
  int freeConfidence=false; // If true, the confidence table has been created here
  int freeOutputs=false; // If true, the scores table has been created here
  int freeFValues=false; // If true, the fvalues table has been created here
  double *eval = Malloc(double,nr_class); // Some kind of evaluation of each class
  if (!classes)
  {
    freeClasses=true;
    classes = Malloc(int,nr_class);
  }
  if (!confidence)
  {
    freeConfidence=true;
    confidence = Malloc(double,nr_class);
  }
  if (!outputs)
  {
    freeOutputs=true;
    outputs = Malloc(double, nr_hyp);
    outputLabels1 = Malloc(int, nr_hyp);
    outputLabels2 = Malloc(int, nr_hyp);
  }
  if (!outputs)
  {
    freeOutputs=true;
    outputs = Malloc(double, nr_hyp);
  }
  if (!fs) // f(x)
  {
    freeFValues=true;
    fs = Malloc(double*, nr_cues);
    for (int i=0; i<nr_cues; ++i)
      fs[i]=Malloc(double, nr_hyp);
  }

  // Calculate f_i(x) if feature vectors are available
  if (xes)
    calculateF_OaO(models, xes, fs, pred_param);

  // Choose the algorithm
  switch(pred_param->oaoAlg)
  {
  case 2:
    computeEvalAlg2_OaO(fs, models, pred_param, eval);
    getOutputsAlg2_OaO(models[0], eval, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_class;
    winnerIdx=getWinnerIdxAlg2_OaO(models[0], eval);
    getConfidenceAlg2_OaO(models[0], eval, winnerIdx, classes, confidence);
    break;
  case 3: // Alg. 3
    computeEvalAlg3_OaO(fs, models, pred_param, eval);
    getOutputsAlg3_OaO(models[0], eval, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_class;
    winnerIdx=getWinnerIdxAlg3_OaO(models[0], eval);
    getConfidenceAlg3_OaO(models[0], eval, winnerIdx, classes, confidence);
    break;
  case 4: // Alg. 4
    {
      // This is not compatible with DAS
      if (pred_param->multiCueAlg==MULTICUEALG_DAS)
      {
        fprintf(stderr,"WARNING: DAS NOT SUPPORTED FOR THIS ALGORITHM!\n");
        fprintf(stderr,"         ONLY THE FIRST MODEL WILL BE USED!\n");
      }
      // Table indicating which classes are to be evaluated at the current
      // level of decision tree. At the beginning set to all ones.
      int *classesToEvaluate = Malloc(int, nr_class);
      int maxIdx;
      int minIdx;
      for(int i=0; i<nr_class; ++i)
        classesToEvaluate[i]=1;
      // Go through all the levels of the tree (n-1 levels, but it will break anyway)
      for(int i=0; i<nr_class; ++i)
      {
        // Get distances to hyperplanes considering only classesToEvaluate.
        getDistToHypAlg4OaO(fs[0], models[0], pred_param, classesToEvaluate, eval);
        // Get index of the class with the maximum absolute distance
        findMinMaxIdxAlg4OaO(models[0], classesToEvaluate, eval, &minIdx, &maxIdx);
        // Is that the result?
        if (fabs(maxIdx)>=fabs(minIdx))
        { // Yes!
          winnerIdx=maxIdx;
          break;
        }
        else // fabs(minIdx)>fabs(maxIdx) - this is rather the rest than the class
        { // No, exclude the minIdx class
          classesToEvaluate[minIdx]=0;
        }
      }
      free(classesToEvaluate);
      // Save info to the classes tab - TEMPORARY SOLUTION
      classes[0]=models[0]->label[winnerIdx];
      for (int i=0; i<nr_hyp; ++i)
      {
        outputs[i]=0; // Clear scores
        outputLabels1[i]=-1;
        outputLabels2[i]=-1;
      }
      (*outputCount)=nr_hyp;
      break;
    }
  default: // Alg. 1
    winnerIdx=getWinnerIdxAlg1_OaO(fs, models, pred_param);
    getOutputsAlg1_OaO(fs, models, pred_param, outputs, outputLabels1, outputLabels2);
    (*outputCount)=nr_hyp;
    computeEvalAlg1_OaO(fs, models, pred_param, winnerIdx, eval);
    getConfidenceAlg1_OaO(models[0], eval, winnerIdx, classes, confidence);
    break;
  };

  // Check if one of the pred_param->matches best matches is the target class
  correct=checkMatches(pred_param, target, classes);

  // Clean-up
  free(eval);
  if (freeConfidence) free(confidence);
  if (freeOutputs)
  {
    free(outputs);
    free(outputLabels1);
    free(outputLabels2);
  }
  if (freeClasses) free(classes);
  if (freeFValues)
  {
    for (int i=0; i<nr_cues; ++i)
      free(fs[i]);
    free(fs);
  }

  // Return result
  (*v)=classes[0];
  return correct;
}


/**
 * Performs prediction for multiple cues.
 * Arguments:
 * models - [in] Models for all cues
 * xes - [in] Feature vectors for all cues. If equal to 0, the function will
 *       try to use the precomputed values of f(x) in the f_values and these
 *       must be given instead.
 * f_values - [in/out] If non-zero, will be filled with the values of f(x)
 *            for all cues and hyperplanes. If xes==0, it must provide
 *            precomputed values of f(x).
 * pred_param - [in] Prediction parameters
 * target - [in] Correct class label for the test sample
 * v - [out] Class label for the best match
 * classes - [out] Ordered class labels
 * confidence - [out] Ordered confidence estimations
 *
 * Returns:
 *  True if one of the K predicted best matches was correct, otherwise false.
 */
bool svm_predict(const svm_model* const * models, const svm_node * const *xes,
                 double **f_values, const predict_parameter *pred_param, double target,
                 double *v, int *classes, double *confidence, double *outputs, 
                 int* outputLabels1, int *outputLabels2, int *outputCount)
{
  int svm_type = models[0]->param.svm_type;

  if(svm_type == ONE_CLASS ||
     svm_type == EPSILON_SVR ||
     svm_type == NU_SVR)
  {
    // ------------------------------------
    // One-class, nu/epsilon SVR
    // ------------------------------------
    // Check if we tried to provide fvalues directly
    if (!xes)
    {
      fprintf(stderr, "This option is not implemented for this svm-type!\n");
      exit(1);
    }
    // Convert the values for multiple cues to the standard ones.
    const svm_model *model = models[0];
    const svm_node *x = xes[0];
    // Perform prediction
    double *sv_coef = model->sv_coef[0];
    double sum = 0;
    for(int i=0;i<model->l;i++)
      sum += sv_coef[i] * Kernel::k_function(x,model->SV[i],model->param);
    sum -= model->rho[0];
    if(model->param.svm_type == ONE_CLASS)
      *v = (sum>0)?1:-1;
    else
      *v = sum;
    return true;
  }
  else if(svm_type == ONE_AGAINST_ALL)
  {
    // ------------------------------------
    // One Against All C-SVC
    // ------------------------------------
    return predictOneAgainstAll(models, xes, f_values, pred_param, target, v, classes, confidence, outputs, outputLabels1, outputLabels2, outputCount);
  }
  else
  {
    // ------------------------------------
    // One Against One C-SVC
    // ------------------------------------
    return predictOneAgainstOne(models, xes, f_values, pred_param, target, v, classes, confidence, outputs, outputLabels1, outputLabels2, outputCount);
  }
}



/**
 * Performs prediction, accepts only one model for one cue.
 */
bool svm_predict(const svm_model* model, const svm_node *x,
                 const predict_parameter *pred_param, double target,
                 double *v )
{
  // Predict
  bool correct;
  int outputCount;
  correct=svm_predict(&model, &x, 0, pred_param, target, v, 0, 0, 0, 0, 0, &outputCount);

  // Return result
  return correct;
}
