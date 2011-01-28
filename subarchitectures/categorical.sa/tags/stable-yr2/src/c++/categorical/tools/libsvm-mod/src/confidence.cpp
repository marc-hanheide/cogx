/**
 * Classifier's confidence estimation and retrieving N top matches.
 *
 * Author:
 *   Andrzej Pronobis
 *   pronobis@nada.kth.se
 */

#include "confidence.h"
#include "svm.h"
#include <limits.h>
#include <math.h>
#include <string.h>


/** Calculates the L2 Norm of omega (||w||=sqrt(wTw)). */
double calcOmegaL2Norm(const Kernel &Q, const double *alpha, int l)
{
  double wTw = 0.0;

  // Iterate through columns of Q matrix
  for(int i=0; i<l; ++i)
  {
    // If alpha[i]==0 it doesn't make much sense to calculate this column anyway
    if (alpha[i]!=0)
    {
      // Get column i from the Q matrix
      Qfloat *Qi=Q.get_Q(i, l);

      // Iterate through rows and compute Ai
      double Ai=0.0;
      for (int j=0; j<l; ++j)
        Ai+=alpha[j]*Qi[j]; // Qij=col[i];

      wTw+=alpha[i]*Ai;
    }
  }

  return sqrt(wTw);
}



void calcAvgDists(const Kernel &Q, const double *alpha, double rho, signed char *y, int l,
                    Solver::SolutionInfo *si)
{
  double wTw = 0.0;
  double Fp = 0.0;
  double Fn = 0.0;
  int Mp = 0;
  int Mn = 0;

  // Iterate through columns of Q matrix
  for(int i=0; i<l; ++i)
  {
    // Get column i from the Q matrix
    Qfloat *Qi=Q.get_Q(i, l);

    // Iterate through rows and compute Ai
    double Ai=0.0;
    for (int j=0; j<l; ++j)
      Ai+=alpha[j]*Qi[j]; // Qij=col[i];

    // Update wTw
    wTw+=alpha[i]*Ai;

    // Update Fp and Fn
    if (y[i]>0)
    {
      Fp+=Ai-rho;
      ++Mp;
    }
    else
    {
      Fn+=Ai+rho;
      ++Mn;
    }
  }

  // Final computations
  si->omegaNorm=sqrt(wTw);
  si->avgFP=Fp/Mp;
  si->avgFN=Fn/Mn;
  si->avgDistP=Fp/(Mp*(si->omegaNorm));
  si->avgDistN=Fn/(Mn*(si->omegaNorm));
}



void computeEvalAlg1_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval)
{
  int nr_class=models[0]->nr_class;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  {
    int nr_cues=pred_param->cues;
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFP[i]);
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->omegaNorm[i]);
      }
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*(fs[c][i]);
      }
      break;
    }
  }
  else
  { // No DAS, use the first model only
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
        eval[i]=fs[0][i]/models[0]->avgFP[i];
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
        eval[i]=fs[0][i]/models[0]->omegaNorm[i];
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
        eval[i]=fs[0][i];
      break;
    }
  }

}


void getOutputsAlg1_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2)
{
  // The eval contains the scores in this case
  // It just must be sorted according to labels
  int nr_class = model->nr_class;

  int lastMin=INT_MIN;
  for(int i=0; i<nr_class; ++i)
  {
    // Find next min label
    int minLabelIdx=0;
    int minLabel=INT_MAX;
    for (int j=0; j<nr_class; ++j)
    {
      if ((model->label[j]>lastMin) && (model->label[j]<minLabel))
      {
        minLabel=model->label[j];
        minLabelIdx=j;
      }
    }
    lastMin=minLabel;
    // Save the score
    outputs[i]=eval[minLabelIdx];
    outputLabels1[i]=minLabel;
    outputLabels2[i]=-1;
  }
}


int getWinnerIdxAlg1_OaA(const svm_model *model, const double *eval)
{
  int max_idx=0;

  for(int i=1; i<model->nr_class; ++i)
    if(eval[i] > eval[max_idx])
      max_idx = i;

  return max_idx;
}


void getConfidenceAlg1_OaA(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  // Create copy of eval
  int nr_class=model->nr_class;
  double *ev=Malloc(double, nr_class);
  memcpy(ev, eval, sizeof(double)*nr_class);

  // Get evaluation for the winner
  double winnerEval=ev[winnerIdx];

  // Set winner to be the first
  classes[0]=model->label[winnerIdx];
  ev[winnerIdx]=-INF;

  // Get the order of the other matches
  // and evaluate how strong they are
  double maxEval=0;
  for(int i=1; i<nr_class; ++i)
  {
    int max_idx=0;
    for(int j=1; j<nr_class; ++j)
      if (ev[j]>ev[max_idx])
        max_idx=j;
    if (i==1)
      maxEval=ev[max_idx];
    classes[i]=model->label[max_idx];
    confidence[i]=winnerEval-ev[max_idx];
    ev[max_idx]=-INF;
  }

  // Set confidence of the decision
  confidence[0]=winnerEval-maxEval;

  // Clean up
  free(ev);
}


bool checkMatches(const predict_parameter *pred_param, const double target, const int *classes)
{
  for(int i=0; i<pred_param->matches; ++i)
    if (classes[i]==target)
      return true;

  return false;
}


void computeEvalAlg2_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval)
{
  int nr_class=models[0]->nr_class;
  // Clear eval
  for(int i=0; i<nr_class; ++i)
    eval[i]=0;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  {
    int nr_cues=pred_param->cues;
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
        for(int c=0; c<nr_cues; ++c)
        {
          if (fs[c][i]>0)
          {
            eval[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFP[i]);
          }
          else
          {
            for(int j=0; j<nr_class; ++j)
            {
              if (i!=j)
                eval[j]+=pred_param->dasA[c][0]*(-fs[c][i]/models[c]->avgFN[i]);
            }
          }
        }
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
        for(int c=0; c<nr_cues; ++c)
        {
          if (fs[c][i]>0)
          {
            eval[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->omegaNorm[i]);
          }
          else
          {
            for(int j=0; j<nr_class; ++j)
            {
              if (i!=j)
                eval[j]+=pred_param->dasA[c][0]*(-fs[c][i]/models[c]->omegaNorm[i]);
            }
          }
        }
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
        for(int c=0; c<nr_cues; ++c)
        {
          if (fs[c][i]>0)
          {
            eval[i]+=pred_param->dasA[c][0]*(fs[c][i]);
          }
          else
          {
            for(int j=0; j<nr_class; ++j)
            {
              if (i!=j)
                eval[j]+=pred_param->dasA[c][0]*(-fs[c][i]);
            }
          }
        }
      break;
    }
  }
  else
  { // No DAS, use the first model only
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
      {
        if (fs[0][i]>0)
        {
          eval[i]+=fs[0][i]/models[0]->avgFP[i];
        }
        else
        {
          for(int j=0; j<nr_class; ++j)
          {
            if (i!=j)
              eval[j]+=-fs[0][i]/models[0]->avgFN[i];
          }
        }
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
      {
        if (fs[0][i]>0)
        {
          eval[i]+=fs[0][i]/models[0]->omegaNorm[i];
        }
        else
        {
          for(int j=0; j<nr_class; ++j)
          {
            if (i!=j)
              eval[j]+=-fs[0][i]/models[0]->omegaNorm[i];
          }
        }
      }
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
      {
        if (fs[0][i]>0)
        {
          eval[i]+=fs[0][i];
        }
        else
        {
          for(int j=0; j<nr_class; ++j)
          {
            if (i!=j)
              eval[j]+=-fs[0][i];
          }
        }
      }
      break;
    }
  }
}


void getOutputsAlg2_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2)
{
  getOutputsAlg1_OaA(model, eval, outputs, outputLabels1, outputLabels2);
}



int getWinnerIdxAlg2_OaA(const svm_model *model, const double *eval)
{
  return getWinnerIdxAlg1_OaA(model, eval);
}


void getConfidenceAlg2_OaA(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  getConfidenceAlg1_OaA(model, eval, winnerIdx, classes, confidence);
}


void computeEvalAlg3_OaA(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param, double *eval)
{
  int nr_class=models[0]->nr_class;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  { // DAS enabled, use all models
    int nr_cues=pred_param->cues;

    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*fabs(1-fs[c][i]/models[c]->avgFP[i]);
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*fabs(models[c]->avgDistP[i]-fs[c][i]/models[c]->omegaNorm[i]);
      }
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
      {
        eval[i]=0;
        for(int c=0; c<nr_cues; ++c)
          eval[i]+=pred_param->dasA[c][0]*fabs(models[c]->avgFP[i]-fs[c][i]);
      }
      break;
    }
  }
  else
  { // No DAS, use the first model only
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_class; ++i)
        eval[i]=fabs(1-fs[0][i]/models[0]->avgFP[i]);
      break;
    case 3: // std distance
      for(int i=0; i<nr_class; ++i)
        eval[i]=fabs(models[0]->avgDistP[i]-fs[0][i]/models[0]->omegaNorm[i]);
      break;
    default: // f(x)
      for(int i=0; i<nr_class; ++i)
        eval[i]=fabs(models[0]->avgFP[i]-fs[0][i]);
      break;
    }
  }
}


void getOutputsAlg3_OaA(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2)
{
  getOutputsAlg1_OaA(model, eval, outputs, outputLabels1, outputLabels2);
}


int getWinnerIdxAlg3_OaA(const svm_model *model, const double *eval)
{
  int min_idx=0;

  for(int i=1; i<model->nr_class; ++i)
    if(eval[i] < eval[min_idx])
      min_idx = i;

  return min_idx;
}


void getConfidenceAlg3_OaA(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  // Create copy of eval
  int nr_class=model->nr_class;
  double *ev=Malloc(double, nr_class);
  memcpy(ev, eval, sizeof(double)*nr_class);

  // Get evaluation for the winner
  double winnerEval=ev[winnerIdx];

  // Set winner to be the first
  classes[0]=model->label[winnerIdx];
  ev[winnerIdx]=INF;

  // Get the order of the other matches
  // and evaluate how strong they are
  double minEval=0;
  for(int i=1; i<nr_class; ++i)
  {
    int min_idx=0;
    for(int j=1; j<nr_class; ++j)
      if (ev[j]<ev[min_idx])
        min_idx=j;
    if (i==1)
      minEval=ev[min_idx];
    classes[i]=model->label[min_idx];
    confidence[i]=ev[min_idx]-winnerEval;
    ev[min_idx]=INF;
  }

  // Set confidence of the decision
  confidence[0]=minEval-winnerEval;

  // Clean up
  free(ev);
}



int getWinnerIdxAlg1_OaO(const double * const *fs, const svm_model * const *models,
                         const predict_parameter *pred_param)
{
  // Allocate memory and initialize
  int nr_class=models[0]->nr_class;
  double *votes = Malloc(double, nr_class);
  int *toEvaluate = Malloc(int, nr_class);
  // Set initial values
  for(int i=0; i<nr_class; ++i)
  {
    votes[i] = 0;
    toEvaluate[i]=1;
  }

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  {
    // -----------------------------------------------
    // DAS enabled, use all models
    // -----------------------------------------------
    int nr_cues = pred_param->cues; // Number of cues
    int nr_hyp = (nr_class*(nr_class-1))/2; // Number of hyperplanes
    double *tmpf = Malloc(double, nr_hyp); // Temporary array to hold mixture of fs

    // Perform DAS on fs
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_hyp; ++i)
      {
        tmpf[i]=0;
        for(int c=0; c<nr_cues; ++c)
        {
          if (fs[c][i]>0)
            tmpf[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFP[i]);
          else
            tmpf[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFN[i]);
        }
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_hyp; ++i)
      {
        tmpf[i]=0;
        for(int c=0; c<nr_cues; ++c)
          tmpf[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->omegaNorm[i]);
      }
      break;
    default: // f(x)
      for(int i=0; i<nr_hyp; ++i)
      {
        tmpf[i]=0;
        for(int c=0; c<nr_cues; ++c)
          tmpf[i]+=pred_param->dasA[c][0]*(fs[c][i]);
      }
      break;
    }

    // Here a while goes until either only one wins,
    // or all evaluated still have the same number of votes
    bool cont=true;
    int max_votes_count=0;
    int evaluatedClasses=0;
    while(cont)
    {
      // Perform voting
      int p=0;
      double max_votes=0;
      evaluatedClasses=0;
      for(int i=0; i<nr_class; ++i)
        for(int j=i+1; j<nr_class; ++j)
        {
          if ((toEvaluate[i]==1) && (toEvaluate[j]==1))
          {
            // Perform voting
            if(tmpf[p] > 0)
            {
              ++votes[i];
              if (votes[i]>max_votes)
                max_votes=votes[i];
            }
            else
            {
              ++votes[j];
              if (votes[j]>max_votes)
                max_votes=votes[j];
            }
            ++evaluatedClasses;
          }
          // Increase p
          ++p;
        }
      // Check whether to continue
      max_votes_count=0;
      for(int i=0; i<nr_class; ++i)
        if (votes[i]==max_votes)
          ++max_votes_count;
        else
          toEvaluate[i]=0;
      //
      if ((max_votes_count==1) or ((max_votes_count*(max_votes_count-1)/2)==evaluatedClasses))
        cont=false;
    }

    // Check if there is still a conflict to resolve - use distances in case
    if (max_votes_count!=1)
    {
      // Set votes to -INF so that finding max works
      for(int i=0; i<nr_class; ++i)
        votes[i] = -INF;
      // Check distances
      int p=0;
      for(int i=0; i<nr_class; ++i)
      {
        for(int j=i+1; j<nr_class; ++j)
        {
          if ((toEvaluate[i]==1) && (toEvaluate[j]==1))
          { // We consider only classes that conflict with each other
            // Now lets measure the distance to each hyperplane and use it to
            // evaluate the class the hyperplane doesn't vote for
            if (tmpf[p]>0) // It votes for i, we should reverse the distance and put it for the class j
            {
              if ((-tmpf[p])>votes[j]) // This is a bit different from the one-cue case since
                votes[j]=-tmpf[p];     // to create tmpf using avg. dist we divide by the other avg dist
            }
            else // It votes for j, We should take the distance as it is and put it for the class i
            {
              if ((tmpf[p])>votes[i])
                votes[i]=tmpf[p];
            }
          }
          // Increase p
          ++p;
        }
      }
    }

    // Free temporary memory
    free(tmpf);
  }
  else
  {
    // -----------------------------------------------
    // No DAS, use the first model only
    // -----------------------------------------------

    // Here a while goes until either only one wins,
    // or all evaluated still have the same number of votes
    bool cont=true;
    int max_votes_count=0;
    int evaluatedClasses=0;
    while(cont)
    {
      // Perform voting
      int p=0;
      double max_votes=0;
      evaluatedClasses=0;
      for(int i=0; i<nr_class; ++i)
        for(int j=i+1; j<nr_class; ++j)
        {
          if ((toEvaluate[i]==1) && (toEvaluate[j]==1))
          {
            // Perform voting
            if(fs[0][p] > 0)
            {
              ++votes[i];
              if (votes[i]>max_votes)
                max_votes=votes[i];
            }
            else
            {
              ++votes[j];
              if (votes[j]>max_votes)
                max_votes=votes[j];
            }
            ++evaluatedClasses;
          }
          // Increase p
          ++p;
        }
      // Check whether to continue
      max_votes_count=0;
      for(int i=0; i<nr_class; ++i)
        if (votes[i]==max_votes)
          ++max_votes_count;
        else
          toEvaluate[i]=0;
      //
      if ((max_votes_count==1) or ((max_votes_count*(max_votes_count-1)/2)==evaluatedClasses))
        cont=false;
    }

    // Check if there is still a conflict to resolve - use distances in case
    if (max_votes_count!=1)
    {
      // Set votes to -INF so that finding max works
      for(int i=0; i<nr_class; ++i)
        votes[i] = -INF;
      // Check distances
      int p=0;
      for(int i=0; i<nr_class; ++i)
      {
        for(int j=i+1; j<nr_class; ++j)
        {
          if ((toEvaluate[i]==1) && (toEvaluate[j]==1))
          { // We consider only classes that conflict with each other
            // Now lets measure the distance to each hyperplane and use it to
            // evaluate the class the hyperplane doesn't vote for
            if (fs[0][p]>0) // It votes for i, we should reverse the distance and put it for the class j
            {
              switch(pred_param->measure)
              {
              case 2: // normalized distance
                if ((-fs[0][p]/models[0]->avgFN[p])>votes[j])
                  votes[j]=-fs[0][p]/models[0]->avgFN[p];
                break;
              case 3: // std distance
                if ((-fs[0][p]/models[0]->omegaNorm[p])>votes[j])
                  votes[j]=-fs[0][p]/models[0]->omegaNorm[p];
                break;
              default: // f(x)
                if ((-fs[0][p])>votes[j])
                  votes[j]=-fs[0][p];
                break;
              }
            }
            else // It votes for j, We should take the distance as it is and put it for the class i
            {
              switch(pred_param->measure)
              {
              case 2: // normalized distance
                if ((fs[0][p]/models[0]->avgFP[p])>votes[i])
                  votes[i]=fs[0][p]/models[0]->avgFP[p];
                break;
              case 3: // std distance
                if ((fs[0][p]/models[0]->omegaNorm[p])>votes[i])
                  votes[i]=fs[0][p]/models[0]->omegaNorm[p];
                break;
              default: // f(x)
                if ((fs[0][p])>votes[i])
                  votes[i]=fs[0][p];
                break;
              }
            }
          }
          // Increase p
          ++p;
        }
      }
    }
  } // END DAS vs NO-DAS

  // Return the winner - find max
  int vote_max_idx;
  vote_max_idx = 0;
  for(int i=1; i<nr_class; ++i)
  {
    if(votes[i] > votes[vote_max_idx])
      vote_max_idx = i;
  }

  // Free memory
  free(votes);
  free(toEvaluate);

  // Return winner
  return vote_max_idx;
}



void getOutputsAlg1_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *outputs, int* outputLabels1, int *outputLabels2)
{
  int nr_class=models[0]->nr_class;
  int nr_hyp = (nr_class*(nr_class-1))/2; // Number of hyperplanes
  double *unsortedOutputs = Malloc(double, nr_hyp); // Temporary array to hold mixture of fs

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  { // DAS enabled, use all models
    int nr_cues = pred_param->cues; // Number of cues
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_hyp; ++i)
      {
        unsortedOutputs[i]=0;
        for(int c=0; c<nr_cues; ++c)
        {
          if (fs[c][i]>0)
            unsortedOutputs[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFP[i]);
          else
            unsortedOutputs[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->avgFN[i]);
        }
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_hyp; ++i)
      {
        unsortedOutputs[i]=0;
        for(int c=0; c<nr_cues; ++c)
          unsortedOutputs[i]+=pred_param->dasA[c][0]*(fs[c][i]/models[c]->omegaNorm[i]);
      }
      break;
    default: // f(x)
      for(int i=0; i<nr_hyp; ++i)
      {
        unsortedOutputs[i]=0;
        for(int c=0; c<nr_cues; ++c)
          unsortedOutputs[i]+=pred_param->dasA[c][0]*(fs[c][i]);
      }
      break;
    }
  }
  else // NO DAS, just single model used
  {
    switch(pred_param->measure)
    {
    case 2: // normalized distance
      for(int i=0; i<nr_hyp; ++i)
      {
        if (fs[0][i]>0)
          unsortedOutputs[i]=(fs[0][i]/models[0]->avgFP[i]);
        else
          unsortedOutputs[i]=(fs[0][i]/models[0]->avgFN[i]);
      }
      break;
    case 3: // std distance
      for(int i=0; i<nr_hyp; ++i)
        unsortedOutputs[i]=fs[0][i]/models[0]->omegaNorm[i];
      break;
    default: // f(x)
      for(int i=0; i<nr_hyp; ++i)
        unsortedOutputs[i]=fs[0][i];
      break;
    }
  } // Was there multi-cue alg used?

  // Prepare list of labels for each hyperplane
  int *label1 = Malloc(int, nr_hyp);
  int *label2 = Malloc(int, nr_hyp);
  int p=0;
  for(int i=0; i<nr_class; ++i)
    for(int j=i+1; j<nr_class; ++j)
    {
      label1[p]=models[0]->label[i];
      label2[p]=models[0]->label[j];
      ++p;
    }

  // Invert the classifiers if label1[i]>label2[i]
  for(int i=0; i<nr_hyp; ++i)
  {
    if (label1[i]>label2[i])
    {
      unsortedOutputs[i]=-unsortedOutputs[i];
      int tmp=label1[i];
      label1[i]=label2[i];
      label2[i]=tmp;
    }
  }

  // Sort outputs
  for(int i=0; i<nr_hyp; ++i)
  {
    int minLabelIdx=0;
    // Find next min label pair
    for (int j=1; j<nr_hyp; ++j)
    {
      if ((label1[j]<label1[minLabelIdx]) || ((label1[j]==label1[minLabelIdx]) && (label2[j]<label2[minLabelIdx])))
        minLabelIdx=j;
    }
    // Set the outputs
    //printf("%d [%d %d] -> %d   ", minLabelIdx, label1[minLabelIdx], label2[minLabelIdx], i);
    outputs[i]=unsortedOutputs[minLabelIdx];
    outputLabels1[i]=label1[minLabelIdx];
    outputLabels2[i]=label2[minLabelIdx];
    // Mark this pair as done
    label1[minLabelIdx]=INT_MAX;
    label2[minLabelIdx]=INT_MAX;
  }
  //printf("\n");

  // Free memory
  free(label1);
  free(label2);
  free(unsortedOutputs);
}



void computeEvalAlg1_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, int winnerIdx, double *eval)
{
  int nr_class=models[0]->nr_class;

  // Initialize eval for the winner to 0
  eval[winnerIdx]=0;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  { // DAS enabled
    int nr_cues=pred_param->cues;

    // Get distances to the hyperplanes
    int p=0;
    for(int i=0; i<nr_class; ++i)
      for(int j=i+1; j<nr_class; ++j)
      {
        if (i==winnerIdx)
        {
          switch(pred_param->measure)
          {
          case 2: // normalized distance
            eval[j]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[j]+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->avgFP[p]);
            break;
          case 3: // std distance
            eval[j]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[j]+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->omegaNorm[p]);
            break;
          default: // f(x)
            eval[j]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[j]+=pred_param->dasA[c][0]*(fs[c][p]);
            break;
          }
        }
        else if (j==winnerIdx)
        {
          switch(pred_param->measure)
          {
          case 2: // normalized distance
            eval[i]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[i]+=-pred_param->dasA[c][0]*(fs[c][p]/models[c]->avgFN[p]);
            break;
          case 3: // std distance
            eval[i]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[i]+=-pred_param->dasA[c][0]*(fs[c][p]/models[c]->omegaNorm[p]);
            break;
          default: // f(x)
            eval[i]=0;
            for(int c=0; c<nr_cues; ++c)
              eval[i]+=-pred_param->dasA[c][0]*(fs[c][p]);
            break;
          }
        }
        // Increase p
        ++p;
      }
  }
  else
  { // No DAS, use the first model only
    // Get distances to the hyperplanes
    int p=0;
    for(int i=0; i<nr_class; ++i)
      for(int j=i+1; j<nr_class; ++j)
      {
        if (i==winnerIdx)
        {
          switch(pred_param->measure)
          {
          case 2: // normalized distance
            eval[j]=fs[0][p]/models[0]->avgFP[p];
            break;
          case 3: // std distance
            eval[j]=fs[0][p]/models[0]->omegaNorm[p];
            break;
          default: // f(x)
            eval[j]=fs[0][p];
            break;
          }
        }
        else if (j==winnerIdx)
        {
          switch(pred_param->measure)
          {
          case 2: // normalized distance
            eval[i]=-fs[0][p]/models[0]->avgFN[p];
            break;
          case 3: // std distance
            eval[i]=-fs[0][p]/models[0]->omegaNorm[p];
            break;
          default: // f(x)
            eval[i]=-fs[0][p];
            break;
          }
        }
        // Increase p
        ++p;
      }
  }
}


void getConfidenceAlg1_OaO(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  // Create copy of eval
  int nr_class=model->nr_class;
  double *ev=Malloc(double, nr_class);
  memcpy(ev, eval, sizeof(double)*nr_class);

  // Set winner to be the first
  classes[0]=model->label[winnerIdx];
  ev[winnerIdx]=INF;

  // Get the order of the other matches
  // and evaluate how strong they are
  double minEval=0;
  for(int i=1; i<nr_class; ++i)
  {
    int min_idx=0;
    for(int j=1; j<nr_class; ++j)
      if (ev[j]<ev[min_idx])
        min_idx=j;
    if (i==1)
      minEval=ev[min_idx];
    classes[i]=model->label[min_idx];
    confidence[i]=ev[min_idx];
    ev[min_idx]=INF;
  }

  // Set confidence of the decision
  confidence[0]=minEval;

  // Clean up
  free(ev);
}




void computeEvalAlg2_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *eval)
{
  // Initialize
  int nr_class=models[0]->nr_class;
  for(int i=0; i<nr_class; ++i)
    eval[i]=0;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  { // DAS used
    int nr_cues=pred_param->cues;
    // Perform voting
    int p=0;
    for(int i=0; i<nr_class; ++i)
      for(int j=i+1; j<nr_class; ++j)
      {
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          for(int c=0; c<nr_cues; ++c)
          {
            if(fs[c][p] > 0)
              eval[i]+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->avgFP[p]);
            else
              eval[j]+=-pred_param->dasA[c][0]*(fs[c][p]/models[c]->avgFN[p]);
          }
          break;
        case 3: // std distance
          for(int c=0; c<nr_cues; ++c)
          {
            if(fs[c][p] > 0)
              eval[i]+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->omegaNorm[p]);
            else
              eval[j]+=-pred_param->dasA[c][0]*(fs[c][p]/models[c]->omegaNorm[p]);
          }
          break;
        default: // f(x)
          for(int c=0; c<nr_cues; ++c)
          {
            if(fs[c][p] > 0)
              eval[i]+=pred_param->dasA[c][0]*(fs[c][p]);
            else
              eval[j]+=-pred_param->dasA[c][0]*(fs[c][p]);
          }
          break;
        }
        // Increase p
        ++p;
      }
  }
  else
  { // No DAS, use the first model only
    // Perform voting
    int p=0;
    for(int i=0; i<nr_class; ++i)
      for(int j=i+1; j<nr_class; ++j)
      {
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          if(fs[0][p] > 0)
            eval[i]+=fs[0][p]/models[0]->avgFP[p];
          else
            eval[j]+=-fs[0][p]/models[0]->avgFN[p];
          break;
        case 3: // std distance
          if(fs[0][p] > 0)
            eval[i]+=fs[0][p]/models[0]->omegaNorm[p];
          else
            eval[j]+=-fs[0][p]/models[0]->omegaNorm[p];
          break;
        default: // f(x)
          if(fs[0][p] > 0)
            eval[i]+=fs[0][p];
          else
            eval[j]+=-fs[0][p];
          break;
        }
        // Increase p
        ++p;
      }
  }
}


void getOutputsAlg2_OaO(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2)
{
  getOutputsAlg1_OaA(model, eval, outputs, outputLabels1, outputLabels2);
}


int getWinnerIdxAlg2_OaO(const svm_model *model, const double *eval)
{
  // Works like Alg1. for OaA
  return getWinnerIdxAlg1_OaA(model, eval);
}


void getConfidenceAlg2_OaO(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  // Works like Alg1. for OaA
  getConfidenceAlg1_OaA(model, eval, winnerIdx, classes, confidence);
}






void computeEvalAlg3_OaO(const double * const *fs, const svm_model * const *models,
                        const predict_parameter *pred_param, double *eval)
{
  // Initialize eval to INF so that we can find minumum later
  int nr_class=models[0]->nr_class;
  for(int i=0; i<nr_class; ++i)
    eval[i]=INF;

  // Check if there is DAS enabled
  if (pred_param->multiCueAlg==MULTICUEALG_DAS)
  { // Use DAS
    int nr_cues=pred_param->cues;

    // Iterate through all hyperplanes
    int p=0;
    for(int i=0; i<nr_class; ++i)
    {
      for(int j=i+1; j<nr_class; ++j)
      {
        // Let's take the positive side first
        // the hyperplane is a part of combined-hyperplane 'class i vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          {
            // Compute distance for all cues
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->avgFP[p]);
            if (dist<eval[i])
              eval[i]=dist;
            break;
          }
        case 3: // std distance
          {
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(fs[c][p]/models[c]->omegaNorm[p]);
            if (dist<eval[i])
              eval[i]=dist;
            break;
          }
        default: // f(x)
          {
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(fs[c][p]);
            if (dist<eval[i])
              eval[i]=dist;
            break;
          }
        }
        // Let's take the negative side now
        // the hyperplane is a part of combined-hyperplane 'class j vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          {
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(-fs[c][p]/models[c]->avgFN[p]);
            if (dist<eval[j])
              eval[j]=dist;
            break;
          }
        case 3: // std distance
          {
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(-fs[c][p]/models[c]->omegaNorm[p]);
            if (dist<eval[j])
              eval[j]=dist;
            break;
          }
        default: // f(x)
          {
            double dist=0;
            for(int c=0; c<nr_cues; ++c)
              dist+=pred_param->dasA[c][0]*(-fs[c][p]);
            if (dist<eval[j])
              eval[j]=dist;
            break;
          }
        }
        // Increment p
        ++p;
      }
    }
  }
  else
  { // No DAS
    // Iterate through all hyperplanes
    int p=0;
    for(int i=0; i<nr_class; ++i)
    {
      for(int j=i+1; j<nr_class; ++j)
      {
        // Let's take the positive side first
        // the hyperplane is a part of combined-hyperplane 'class i vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          if ((fs[0][p]/models[0]->avgFP[p])<eval[i])
          eval[i]=fs[0][p]/models[0]->avgFP[p];
          break;
        case 3: // std distance
          if ((fs[0][p]/models[0]->omegaNorm[p])<eval[i])
          eval[i]=fs[0][p]/models[0]->omegaNorm[p];
          break;
        default: // f(x)
          if ((fs[0][p])<eval[i])
            eval[i]=fs[0][p];
          break;
        }
        // Let's take the negative side now
        // the hyperplane is a part of combined-hyperplane 'class j vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          if ((-fs[0][p]/models[0]->avgFN[p])<eval[j])
          eval[j]=-fs[0][p]/models[0]->avgFN[p];
          break;
        case 3: // std distance
          if ((-fs[0][p]/models[0]->omegaNorm[p])<eval[j])
          eval[j]=-fs[0][p]/models[0]->omegaNorm[p];
          break;
        default: // f(x)
          if ((-fs[0][p])<eval[j])
            eval[j]=-fs[0][p];
          break;
        }
        // Increment p
        ++p;
      }
    }
  } // End DAS vs no DAS
}


void getOutputsAlg3_OaO(const svm_model *model, const double *eval, double *outputs, int* outputLabels1, int *outputLabels2)
{
  getOutputsAlg1_OaA(model, eval, outputs, outputLabels1, outputLabels2);
}


int getWinnerIdxAlg3_OaO(const svm_model *model, const double *eval)
{
  // Works like Alg1. for OaA
  return getWinnerIdxAlg1_OaA(model, eval);
}


void getConfidenceAlg3_OaO(const svm_model *model, const double *eval, int winnerIdx,
                          int *classes, double *confidence)
{
  // Works like Alg1. for OaA
  getConfidenceAlg1_OaA(model, eval, winnerIdx, classes, confidence);
}





void getDistToHypAlg4OaO(const double *f, const svm_model *model,
                         const predict_parameter *pred_param,
                         const int *classesToEval, double *eval)
{
  // Initialize eval to INF so that we can find minumum later
  int nr_class=model->nr_class;
  for(int i=0; i<nr_class; ++i)
    eval[i]=INF;

  // Iterate through hyperplanes between classes in classesToEval
  int p=0;
  for(int i=0; i<nr_class; ++i)
  {
    for(int j=i+1; j<nr_class; ++j)
    {
      if ((classesToEval[i]==1) && (classesToEval[j]==1))
      {
        // Let's take the positive side first
        // the hyperplane is a part of combined-hyperplane 'class i vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          if ((f[p]/model->avgFP[p])<eval[i])
          eval[i]=f[p]/model->avgFP[p];
          break;
        case 3: // std distance
          if ((f[p]/model->omegaNorm[p])<eval[i])
          eval[i]=f[p]/model->omegaNorm[p];
          break;
        default: // f(x)
          if ((f[p])<eval[i])
            eval[i]=f[p];
          break;
        }
        // Let's take the negative side now
        // the hyperplane is a part of combined-hyperplane 'class j vs. all'
        switch(pred_param->measure)
        {
        case 2: // normalized distance
          if ((-f[p]/model->avgFN[p])<eval[j])
          eval[j]=-f[p]/model->avgFN[p];
          break;
        case 3: // std distance
          if ((-f[p]/model->omegaNorm[p])<eval[j])
          eval[j]=-f[p]/model->omegaNorm[p];
          break;
        default: // f(x)
          if ((-f[p])<eval[j])
            eval[j]=-f[p];
          break;
        }
      }
      // Increment p
      ++p;
    }
  }
}


void findMinMaxIdxAlg4OaO(const svm_model *model, const int *classesToEval,
                          const double *eval, int *minIdx, int *maxIdx)
{
  // Find first class to evaluate
  int firstToEval=-1;
  for(int i=0; i<model->nr_class; ++i)
    if (classesToEval[i]==1)
    {
      firstToEval=i;
      break;
    }
  // Sanity check
  if (firstToEval<0)
  {
    printf("STH WRONG IN findMinMaxIdxAlg2OaO!!!!\n");
    exit(-1);
  }
  // Set initial minmax
  (*maxIdx)=firstToEval;
  (*minIdx)=firstToEval;

  // Find minmax
  for(int i=firstToEval+1; i<model->nr_class; ++i)
    if (classesToEval[i]==1)
    {
      if (eval[i] > eval[*maxIdx])
        (*maxIdx) = i;
      if (eval[i] < eval[*minIdx])
        (*minIdx) = i;
    }
}
















/*

    for(i=0;i<nr_class;i++)
    {
      //dist[i]/=model->omegaNorm[i];
      printf("%g %g %g ", dist[i], dist[i]/model->avgDistP[i], dist[i]/model->avgDistN[i]);
      if (dist[i]>=0)
      {
        vote[i]+=dist[i]; // /model->avgDistP[i];
        printf("P:%g |", dist[i]/model->avgDistP[i]);
      }
      else
      {
        printf("N:%g |",-dist[i]/model->avgDistN[i]);
        for(int j=0; j<nr_class; ++j)
          if (i!=j)
            vote[j]+=-dist[i]; // /model->avgDistN[i];
      }
    }
    printf("\n");

lub


//     for(i=0;i<nr_class;i++)
//     {
//       dist[i]/=model->omegaNorm[i];
//       vote[i]=dist[i]/model->avgDistP[i];
//     }
//     printf("\n");


*/









// Pieces of original code from svm.cpp
// svm-predict for OaA
/*
    // Initialize
    int i;
    int nr_class = model->nr_class;
    int l = model->l;

    // Allocate vector storing votes
    double *vote = Malloc(double,nr_class);

    // Calculate vote[i]=f_i(x)
    for(i=0;i<nr_class;i++)
      vote[i] = -model->rho[i];
    for (i=0;i<l;i++)
    {
      double kv = Kernel::k_function(x,model->SV[i],model->param);
      for (int j=0;j<nr_class;j++)
        vote[j] += model->sv_coef[j][i]*kv;
    }

    printf("\n");

    // Save votes to vote matrix
    for(i=0;i<nr_class;i++)
      voteMat[act_sample][i]= vote[i];
    act_sample++;

    // Check results
    int vote_max_idx;
    for(int j=0; j<votes; ++j)
    {
      // Find max
      vote_max_idx = 0;
      for(i=1; i<nr_class; ++i)
        if(vote[i] > vote[vote_max_idx])
          vote_max_idx = i;
      if (j==0)
        *v = model->label[vote_max_idx];
      if (model->label[vote_max_idx]==target)
      { // Correct
        free(vote);
        return true;
      }
      vote[vote_max_idx]=0.0;
    }

    // Not correct
    free(vote);
    return false;
*/

// One against one
/*
    int i;
    int nr_class = model->nr_class;
    int l = model->l;

    double *kvalue = Malloc(double,l);
    for(i=0;i<l;i++)
      kvalue[i] = Kernel::k_function(x,model->SV[i],model->param);

    int *start = Malloc(int,nr_class);
    start[0] = 0;
    for(i=1;i<nr_class;i++)
      start[i] = start[i-1]+model->nSV[i-1];

    int *vote = Malloc(int,nr_class);


    for(i=0;i<nr_class;i++)
      vote[i] = 0;
    int p=0;
    for(i=0;i<nr_class;i++)
      for(int j=i+1;j<nr_class;j++)
      {
        double sum = 0;
        int si = start[i];
        int sj = start[j];
        int ci = model->nSV[i];
        int cj = model->nSV[j];

        int k;
        double *coef1 = model->sv_coef[j-1];
        double *coef2 = model->sv_coef[i];
        for(k=0;k<ci;k++)
          sum += coef1[si+k] * kvalue[si+k];
        for(k=0;k<cj;k++)
          sum += coef2[sj+k] * kvalue[sj+k];
        sum -= model->rho[p++];


        if(sum > 0)
          ++vote[i];
        else
          ++vote[j];
      }


    // Free tables
    free(kvalue);
    free(start);

    // Save votes to vote matrix
    for(i=0;i<nr_class;i++)
      voteMat[act_sample][i]= vote[i];
    act_sample++;

    // Check results
    int vote_max_idx;
    for(int j=0; j<votes; ++j)
    {
      // Find max
      vote_max_idx = 0;
      for(i=1; i<nr_class; ++i)
        if(vote[i] > vote[vote_max_idx])
          vote_max_idx = i;
      if (j==0)
        *v = model->label[vote_max_idx];
      if (model->label[vote_max_idx]==target)
      { // Correct
        free(vote);
        return true;
      }
      vote[vote_max_idx]=0;
    }

    // Not correct
    free(vote);
    return false;
  }
*/
