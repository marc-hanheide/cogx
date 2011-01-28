/*
* This file implements the algorithm decreasing the number of support
* vectors by reducing those that are linearly depentent in feature space.
*
* References:    (TO BE UPDATED)
*   - T. Downs, K. E. Gates and A. Masters. Exact Simplification of
*     Support Vector Solutions. Journal of Machine Learning Research 2,
*     2001, 293-297
*   - C. D. Mayer. Matrix Analysis and Applied Linear Algebra. SIAM, 2000
*   - B. Noble, J. W. Daniel. Applied Linear Algebra. Third Edition. Prentice-Hall, 1988
*
* Author:
*   Andrzej Pronobis
*   pronobis@nada.kth.se
*   pronobis@o2.pl
*/

#ifndef _SVREDUCTION_H
#define _SVREDUCTION_H

struct decision_function;
struct svm_problem;


/*
* Reduces the support vectors that are linearly dependent
* in the feature space. Should be called after training.
*/
void reduceLinDepSV(decision_function *df, const svm_problem *prob, const svm_parameter* param);


/*
* Reduces the support vectors from given model that are
* linearly dependent in the feature space.
* Returns false if error occured.
*/
bool reduceModel(char *training_set, char *input_model, char *output_model,
                 double threshold, char algorithm, char rankdef, char normalize);


#endif
