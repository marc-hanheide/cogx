/*
* This file runs the algorithm decreasing the number of support
* vectors by reducing those that are linearly depentent in feature space.
* Model obtained using svm-train should be used as an input file.
*
* Author:
*   Andrzej Pronobis
*   pronobis@nada.kth.se
*   pronobis@o2.pl
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "svm.h"
#include "svreduction.h"

#define MAX_PATH 1024


void printHelp()
{
  printf(
    "Usage: svm-reduce [options] training_set_file input_model_file [output_model_file]\n"
    "Options:\n"
    "\t-t threshold\t- Threshold (Default 0.01)\n"
    "\t-a alg_type\t- Algorithm: 1 - QR Factorization (Default)\n"
    "\t           \t             2 - Gauss-Jordan Elimination\n"
    "\t-n value\t- Normalize (-a 1 only): 1 - No\n"
    "\t        \t                         2 - Yes (Default)\n"
    "\t-r rank_def\t- Numerical rank definition(-a 1 only): 1 - ||R22||2 (Default)\n"
    "\t           \t                                        2 - rii\n"
    "\t           \t                                        3 - inf(R11)\n"
    "\n"
  );
}


int parseParams(int argc, char **argv, double *threshold, char *algorithm, char *rankdef, char *normalize,
                char *training_set, char *input_model, char *output_model)
{
  // Default values
  (*threshold) = 0.01;
  (*algorithm) = 1;
  (*rankdef) = 1;
  (*normalize) = 2;

  // Parse options
  int i;
  for (i=1; (i<argc)&&(argv[i][0]=='-'); ++i)
  {
    switch (argv[i][1])
    {
      case 't':
          ++i;
          if (i>=argc) return 0;
          (*threshold) = atof(argv[i]);
          if ((*threshold)<=0)
          {
            printf("Error: The threshold must be greater than 0!\n\n");
            return 0;
          }
        break;
      case 'a':
          ++i;
          if (i>=argc) return 0;
          (*algorithm) = atoi(argv[i]);
          if (((*algorithm)!=1) && ((*algorithm)!=2))
          {
            printf("Error: Incorrect value for -a switch!\n\n");
            return 0;
          }
        break;
      case 'n':
          ++i;
          if (i>=argc) return 0;
          (*normalize) = atoi(argv[i]);
          if (((*normalize)!=1) && ((*normalize)!=2))
          {
            printf("Error: Incorrect value for -n switch!\n\n");
            return 0;
          }
        break;
      case 'r':
          ++i;
          if (i>=argc) return 0;
          (*rankdef) = atoi(argv[i]);
          if (((*rankdef)<1) && ((*rankdef)>3))
          {
            printf("Error: Incorrect value for -r switch!\n\n");
            return 0;
          }
        break;


      default:
        return 0;
        break;
    }
  }

  // Parse training set's filename
  if (i>=argc) return 0;
  strcpy(training_set, argv[i]);
  ++i;

  // Parse input model's filename
  if (i>=argc) return 0;
  strcpy(input_model, argv[i]);
  ++i;


  // Parse output model's filename
  if (i<argc)
    strcpy(output_model, argv[i]);
  else
    sprintf(output_model, "%s_reduced", argv[i-1]);

  return 1;
}

int main(int argc, char **argv)
{
  // Parse command line parameters
  char training_set[MAX_PATH];
  char input_model[MAX_PATH];
  char output_model[MAX_PATH];
  double threshold;
  char algorithm;
  char rankdef;
  char normalize;

  if (!parseParams(argc, argv, &threshold, &algorithm, &rankdef, &normalize, training_set, input_model, output_model))
  {
    printHelp();
    return(1);
  }


  // Run algorithm
  if (reduceModel(training_set, input_model, output_model, threshold, algorithm, rankdef, normalize))
    return 0;
  else
    return 1;
}




