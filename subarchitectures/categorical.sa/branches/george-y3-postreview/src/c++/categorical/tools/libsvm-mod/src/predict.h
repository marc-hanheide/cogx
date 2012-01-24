/**
 * The modified and extended svm_predict function and some tools.
 *
 * Author:
 *   Andrzej Pronobis
 *   pronobis@nada.kth.se
 */

#ifndef _PREDICT_H
#define _PREDICT_H

/** Multi-cue algorithms. */
#define MULTICUEALG_NONE 0
#define MULTICUEALG_DAS 1


struct svm_model;
struct svm_node;

/**
 * Parameters used for prediction only.
 */
struct predict_parameter
{
  /** Number of considered best matches when calculating the class. rate. */
  int matches;
  /** Algorithm used to find the best matches and confidence for the one-against-all multi-class method. */
  int oaaAlg;
  /** Algorithm used to find the best matches and confidence for the one-against-one multi-class method. */
  int oaoAlg;
  /** Algorithm used for combining multiple cues. */
  int multiCueAlg;
  /** Measure used while estimating the best matching class. */
  int measure;
  /** Number of cues available. */
  int cues;

  /** Matrix of DAS a parameters (dasA[cues][1 or more if the alg. allows]).*/
  double **dasA;

  /** Constructor, sets default values */
  predict_parameter() : matches(1), oaaAlg(1), oaoAlg(1),
                        multiCueAlg(MULTICUEALG_NONE), measure(1), cues(1),
                        dasA(0)
    { }

  /** Destructor, frees memory if needed. */
  ~predict_parameter();

};

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
 * scoresV - [out] Scores assigned by hyperplanes to the test sample
 *
 * Returns:
 *  True if one of the K predicted best matches was correct, otherwise false.
 */
bool svm_predict(const svm_model* const * models, const svm_node * const *xes,
                 double **f_values, const predict_parameter *pred_param, double target,
                 double *v, int *classes, double *confidence, double *outputs, 
                 int* outputLabels1, int *outputLabels2, int *outputCount );


/**
 * Performs prediction, accepts only one model for one cue.
 */
bool svm_predict(const svm_model* model, const svm_node *x,
                 const predict_parameter *pred_param, double target,
                 double *v );


#endif
