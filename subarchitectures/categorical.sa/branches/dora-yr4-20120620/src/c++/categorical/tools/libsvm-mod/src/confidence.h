/*
* Classifier's confidence estimation and retrieving N top matches.
*
* Author:
*   Andrzej Pronobis
*   pronobis@nada.kth.se
*/

#ifndef _CONFIDENCE_H
#define _CONFIDENCE_H

#include "svm.h"


class Kernel;
struct predict_parameter;

/** Calculates the L2 Norm of omega (||w||=sqrt(wTw)). */
double calcOmegaL2Norm(const Kernel &Q, const double *alpha, int l);

/**
 * Calculates the L2 Norm of omega (||w||=sqrt(wTw)) and
 * average distances of both positve and negative samples .
 * Params:
 * Q - [in] Q matrix
 * alpha - [in] alpha coeffitients
 * y - [in] class labels
 * l - [in] total number of samples in this subproblem
 * omegaNorm - [out] L2 Norm of omega (||w||=sqrt(wTw))
 * avgDistP - [out] Avg distance between the positive samples and the hyperplane
 * avgDistN - [out] Avg distance between the negative samples and the hyperplane
 * avgFP - [out] Avg value of the decision function for positive samples
 * avgFN - [out] Avg value of the decision function for negative samples
 */
void calcAvgDists(const Kernel &Q, const double *alpha, double rho, signed char *y, int l,
                    Solver::SolutionInfo *si);


/**
 * Evaluates all classes using the algorithm no. 1 for the once-against-all method.
 * The algorithm no. 1 evaluates the classes based on the distance of the test
 * sample from the hyperplanes.
 */
void computeEvalAlg1_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval);


/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg1_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2);


/**
 * Returns the number of best matching class.
 */
int getWinnerIdxAlg1_OaA(const svm_model *model, const double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg1_OaA(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence);


/**
 * Evaluates all classes using the algorithm no. 2 for the once-against-all method.
 * In case of the alg. 2, each hyperplane votes with some dist. measure either to
 * the class associated with the hyperplane or to all other classes.
 */
void computeEvalAlg2_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval);


/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg2_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2);


/**
 * Returns the number of best matching class.
 */
int getWinnerIdxAlg2_OaA(const svm_model *model, const double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg2_OaA(const svm_model *model, const double *eval, int winnerIdx,
                           int *classes, double *confidence);


/**
 * Evaluates all classes using the algorithm no. 3 for the once-against-all method.
 * The evaluation in this case is based on the distance between the distance of the
 * sample to the hyperplane and the average distance of the training samples to the
 * hyperplane.
 */
void computeEvalAlg3_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval);


/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg3_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2);


/**
 * Returns the number of best matching class.
 */
int getWinnerIdxAlg3_OaA(const svm_model *model, const double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg3_OaA(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence);


/**
 * Returns the number of best matching class. This is done using voting.
 * Each hyper plane has one vote and votes for one of the classes it distinguished
 * between. If two classes get the same amount of votes the conflict must be resolved.
 * In such case, only those hyperplanes that distinguish between the conflicting
 * classes vote. If at the end, all classes get the same amount of votes, we use
 * distance to the hyperplane to resolve this conflict.
 */
int getWinnerIdxAlg1_OaO(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param);


/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg1_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *outputs, int* outputLabels1, int *outputLabels2);


/**
 * Evaluates all classes using the algorithm no. 1 for the once-against-one method.
 * The value assigned to each class is the distance of the sample to the hyperplane
 * distinguishing between the winner and the class
 */
void computeEvalAlg1_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, int winnerIdx, double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg1_OaO(const svm_model *model, const double *eval, int winnerIdx,
                           int *classes, double *confidence);



/**
 * Evaluates all classes using the algorithm no. 2 for the once-against-one method.
 * The algorithm no. 2 performs voting using a distance to each hyperplane.
 */
void computeEvalAlg2_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *eval);


/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg2_OaO(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2);


/**
 * Returns the number of best matching class.
 */
int getWinnerIdxAlg2_OaO(const svm_model *model, const double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg2_OaO(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence);




/**
 * Evaluates all classes using the algorithm no. 3 for the once-against-one method.
 * The algorithm no. 3 converts the OaO case to OaA and uses the Alg.1 for OaA.
 */
void computeEvalAlg3_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *eval);

/**
 * Saves classifier's outputs into the outputs array. The outputs are sorted according to the labels
 * so the order will be always the same if the labels are the same.
 */
void getOutputsAlg3_OaO(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2);

/**
 * Returns the number of best matching class.
 */
int getWinnerIdxAlg3_OaO(const svm_model *model, const double *eval);

/**
 * Returns the labels of classes orderd according to how good the matching is and
 * confidence for each class. The winner is of course first.
 */
void getConfidenceAlg3_OaO(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence);




/**
 * Returns distances to the hyperplanes created as in case of Alg. 3. Only
 * the hyperplanes for classes indicated in classesToEval will be consiered.
 */
void getDistToHypAlg4OaO(const double *f, const svm_model *model,
                         const predict_parameter *pred_param,
                         const int *classesToEval, double *eval);

/**
 * Finds min and max in an array
 */
void findMinMaxIdxAlg4OaO(const svm_model *model, const int *classesToEval,
                          const double *eval, int *minIdx, int *maxIdx);









/** Checks whether one of the pred_param->matches best matches is the target class. */
bool checkMatches(const predict_parameter *pred_param, const double target, const int *classes);


#endif
