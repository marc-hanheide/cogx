#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "svm.h"
#include "time.h"
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

#define MAX_TRAINING_SETS 50


void exit_with_help()
{
  printf(
    "Usage: svm-train [options] training_set_files [model_file]\n"
    "\n"
    "training_set_files - training set files separated by '+'\n"
    "\n"
    "Standard options:\n"
    " -s svm_type : set type of SVM (default 0)\n"
    " \t0 -- C-SVC\n"
    " \t1 -- nu-SVC\n"
    " \t2 -- one-class SVM\n"
    " \t3 -- epsilon-SVR\n"
    " \t4 -- nu-SVR\n"
    " \t5 -- one-against-all C-SVC\n"
    " -t kernel_type : set type of kernel function (default 2)\n"
    " \t0 -- linear: u'*v\n"
    " \t1 -- polynomial: (gamma*u'*v + coef0)^degree\n"
    " \t2 -- radial basis function: exp(-gamma*|u-v|^2)\n"
    " \t3 -- sigmoid: tanh(gamma*u'*v + coef0)\n"
    " \t4 -- local kernel\n"
    " \t5 -- chi squared kernel\n"
    " \t6 -- generalized gaussian kernel\n"
    " \t     use paramters d and r to specify\n"
    " \t     the degree and the coefficient\n"
    " \t7 -- histograms intersection kernel\n"
    " \t8 -- fast matching local kernel for SIFT features\n"
    " -d degree : set degree in kernel function (default 3)\n"
    " -g gamma : set gamma in kernel function (default 1/k)\n"
    " -r coef0 : set coef0 in kernel function (default 0)\n"
    " -c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)\n"
    " -n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)\n"
    " -p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)\n"
    "\n"
    "Advanced options:\n"
    " -m cachesize : set cache memory size in MB (default 40)\n"
    " -e epsilon : set tolerance of termination criterion (default 0.001)\n"
    " -h shrinking: whether to use the shrinking heuristics, 0 or 1 (default 1)\n"
    " -wi weight: set the parameter C of class i to weight*C, for C-SVC (default 1)\n"
    " -v n: n-fold cross validation mode\n"
    " -V n: reversed n-fold cross validation mode (small training set, large test set)\n"
    "   -z step: Use only every n-th set for cross-validation (Default 1)\n"
    "   -x alg: Algorithm used to find the best matches and confidence during\n"
    "           cross-validation. Works as -o and -a parameters in svm-predict (Default 1)\n"
    "   -y measure: Measure used while estimating the best matching class during\n"
    "               cross-validation. Works as -m parameter in svm-predict (Default 1)\n"
    "\n"
    "Paralelized training:\n"
    " -o only train class o to parallelize the training\n"
    " \t-1 normal training = disable feature (default)\n"
    " \t-2 gather trained models\n"
    " \tor the index of the class to train (starting from 0)\n"
    " -q temporary file to store the model (for parallelized training);\n"
    "    use %%d or similar in the filename, so that files can be collected later on\n"
    "\n"
    "Local kernel params:\n"
    " -N #localfeatures: maximum number of local features per sample\n"
    " -M #localfeatures: number of considered local feature matches\n"
    " -D local feature dimension\n"
    " -S threshold below which a local feature match is not considered\n"
    " -T local feature kernel type\n"
    " \t0 : correlation match\n"
    " \t1 : chi-squared match\n"
    " \t2 : squarred error match\n"
    " -Q type of position information which should be used in local kernel\n"
    " \t0 -- feature distances\n"
    " \t1 -- distance histograms\n"
    " \t2 -- distance profiles\n"
    " -P sigma for position information in local kernel;\n"
    "    set to zero for not considering position information (default)\n"
    " -A size of the neighbourhood considered by the distance histogram\n"
    "    extension of the local kernel\n"
    " -B number of bins for the distance histogram extension of the local kernel\n"
    " -L lap: If lap=1, the feature vectors contain signs of laplacians. Default:off.\n"
    "         This does not mean that the signs will be used, just that they are there.\n"
    " -U enh: Enable (enh=1) or disable (enh=0) algorithm enhancements for SURF features.\n"
    "         Requires that the laplacian sign was in the feature vectors. Default:off.\n"
    "\n"
    "Fast local kernel params:\n"
    " -N #localfeatures: maximum number of local features per sample\n"
    " -M #localfeatures: number of considered local feature matches\n"
    " -D local feature dimension\n"
    " -A coefficient: A coefficient by which the calculated distance is multiplied\n"
    "                 in order to compensate for the vector elements that were omitted.\n"
    "                 If equals 0, no compensation is used and the results are exact.\n"
    " -L lap: If lap=1, the feature vectors contain signs of laplacians. Default:off.\n"
    "         This does not mean that the signs will be used, just that they are there.\n"
    " -U enh: Enable (enh=1) or disable (enh=0) algorithm enhancements for SURF features.\n"
    "         Requires that the laplacian sign was in the feature vectors. Default:off.\n"
    "\n"
    "Reduction of linearly dependent SVs (C-SVC only):\n"
    " -l threshold alg_type: reduce linearly dependent vectors in feature space. Params:\n"
    "                        threshold: Threshold value\n"
    "                        alg_type: Algorithm type (1-QR factorization 2-Gauss-Jordan elimination)\n"
    " (It is recommended to use the external svm-reduce application instead (more options))\n"
    "\n"
//     "Confidence estimation (C-SVC only, not compatible with SV reduction):\n"
//     " -C n: compute average distances between training samples and hyperplanes:\n"
//     " \t0 - No (Default)\n"
//     " \t1 - Yes\n"
);
  exit(1);
}

void parse_command_line(int argc, char **argv, char input_file_names[][1024], int*inputFileCount, char *model_file_name);
void read_problem(const char *filename, int l, int *firstVector, int *firstElement);
void do_cross_validation();
void estimateProblemSize(const char *filename, int *elements, int *l);


struct svm_parameter param;   // set by parse_command_line
struct svm_problem prob;    // set by read_problem
struct svm_model *model;
struct svm_node *x_space;
int cross_validation = 0;
int nr_fold;
int OaAOaOAlg = 1;
int DistMeasure = 1;
int reversedCrossValidation=0;
int everyNthCrossValidation=1;


int main(int argc, char **argv)
{
  char inputFileNames[MAX_TRAINING_SETS][1024];
  char model_file_name[1024];
  const char *error_msg;
  int inputFileCount=0;
  int elementsTab[MAX_TRAINING_SETS];
  int lTab[MAX_TRAINING_SETS];

  // Parse cmd line args
  parse_command_line(argc, argv, inputFileNames, &inputFileCount, model_file_name);

  // Get problem size
  int totalElements=0;
  int totalL=0;
  printf("-- Estimating problem size... \n");
  fflush(stdout);
  for (int i=0; i<inputFileCount; ++i)
  {
    estimateProblemSize(inputFileNames[i], &(elementsTab[i]), &(lTab[i]));
    totalElements+=elementsTab[i];
    totalL+=lTab[i];
  }
  printf("Done!\n");
  fflush(stdout);

  // Allocate space for the problem
  prob.l = totalL;
  prob.y = Malloc(double, totalL);
  prob.x = Malloc(struct svm_node *, totalL);
  x_space = Malloc(struct svm_node, totalElements);

  // Read the problem
  printf("-- Reading problem... \n");
  fflush(stdout);
  int firstVector=0;
  int firstElement=0;
  for (int i=0; i<inputFileCount; ++i)
  {
    read_problem(inputFileNames[i], lTab[i], &firstVector, &firstElement);
    printf("%d%%\n", (firstVector*100)/totalL);
    fflush(stdout);
  }
  printf("Done!\n");
  fflush(stdout);

  // Check parameters
  error_msg = svm_check_parameter(&prob,&param);


  if(error_msg)
  {
    fprintf(stderr,"Error: %s\n",error_msg);
    exit(1);
  }

  if(cross_validation)
  {
    do_cross_validation();
  }
  else
  {
    fprintf(stdout, "-- Training...\n");
    fflush(stdout);

    // Start measuring time
    timerReset();

    // Train
    model = svm_train(&prob,&param);

    // Stop measuring time
    TimeInterval timeI = timerQuery();

    fprintf(stdout, "Trained.\n");
    fprintf(stdout, "-- Training time: real=%gs, virt=%gs, prof=%gs\n", timeI.real, timeI.virt, timeI.prof);
    fflush(stdout);

    // Save model
    fprintf(stdout, "-- Saving... ");
    fflush(stdout);
    svm_save_model(model_file_name,model);
    fprintf(stdout, "Done!\n");
    fflush(stdout);

    // Destroy model
    fprintf(stdout, "-- Destroying... ");
    fflush(stdout);
    svm_destroy_model(model);
    fprintf(stdout, "Done!\n");
    fflush(stdout);
  }

  free(prob.y);
  free(prob.x);
  free(x_space);

  return 0;
}

void do_cross_validation()
{
  int i;
  int total_correct = 0;
  int total_predicted = 0;
  double total_error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;

  // random shuffle
  for(i=0;i<prob.l;i++)
  {
    int j = i+rand()%(prob.l-i);
    struct svm_node *tx;
    double ty;

    tx = prob.x[i];
    prob.x[i] = prob.x[j];
    prob.x[j] = tx;

    ty = prob.y[i];
    prob.y[i] = prob.y[j];
    prob.y[j] = ty;
  }

  for(i=0;i<nr_fold;i+=everyNthCrossValidation)
  {
    int begin = i*prob.l/nr_fold;
    int end = (i+1)*prob.l/nr_fold;
    int j,k;
    struct svm_problem subprob;

    if (reversedCrossValidation)
    {
      subprob.l = end-begin;
      subprob.x = Malloc(struct svm_node*,subprob.l);
      subprob.y = Malloc(double,subprob.l);
      k=0;
      for(j=begin;j<end;j++)
      {
        subprob.x[k] = prob.x[j];
        subprob.y[k] = prob.y[j];
        ++k;
      }
    }
    else
    {
      subprob.l = prob.l-(end-begin);
      subprob.x = Malloc(struct svm_node*,subprob.l);
      subprob.y = Malloc(double,subprob.l);

      k=0;
      for(j=0;j<begin;j++)
      {
        subprob.x[k] = prob.x[j];
        subprob.y[k] = prob.y[j];
        ++k;
      }
      for(j=end;j<prob.l;j++)
      {
        subprob.x[k] = prob.x[j];
        subprob.y[k] = prob.y[j];
        ++k;
      }
    }

    if(param.svm_type == EPSILON_SVR ||
       param.svm_type == NU_SVR)
    { // Regression
      struct svm_model *submodel = svm_train(&subprob,&param);
      double error = 0;
      for(j=begin;j<end;j++)
      {
        double v;
        predict_parameter pred_param;

        svm_predict(submodel,prob.x[j], &pred_param, 1, &v);
        double y = prob.y[j];
        error += (v-y)*(v-y);
        sumv += v;
        sumy += y;
        sumvv += v*v;
        sumyy += y*y;
        sumvy += v*y;
      }
      svm_destroy_model(submodel);
      printf("Mean squared error = %g\n", error/(end-begin));
      total_error += error;
    }
    else
    { // Classification
      struct svm_model *submodel = svm_train(&subprob,&param);
      int correct = 0;
      double v;
      predict_parameter pred_param;
      pred_param.oaaAlg=OaAOaOAlg;
      pred_param.oaoAlg=OaAOaOAlg;
      pred_param.measure=DistMeasure;

      if (reversedCrossValidation)
      {
        for(j=0;j<begin;j++)
        {
          if (svm_predict(submodel, prob.x[j], &pred_param, prob.y[j], &v))
            ++correct;
        }
        for(j=end;j<prob.l;j++)
        {
          if (svm_predict(submodel, prob.x[j], &pred_param, prob.y[j], &v))
            ++correct;
        }
      }
      else
      {
        for(j=begin;j<end;j++)
        {
          if (svm_predict(submodel, prob.x[j], &pred_param, prob.y[j], &v))
            ++correct;
        }
      }

      svm_destroy_model(submodel);

      if (reversedCrossValidation)
      {
        printf("Accuracy = %g%% (%d/%d)\n", 100.0*correct/(prob.l-(end-begin)),correct,(prob.l-(end-begin)));
        total_predicted+=prob.l-(end-begin);
      }
      else
        printf("Accuracy = %g%% (%d/%d)\n", 100.0*correct/(end-begin),correct,(end-begin));

      total_correct += correct;
    }

    free(subprob.x);
    free(subprob.y);
  } // main for

  // Print final results
  if(param.svm_type == EPSILON_SVR || param.svm_type == NU_SVR)
  {
    printf("Cross Validation Mean squared error = %g\n",total_error/prob.l);
    printf("Cross Validation Squared correlation coefficient = %g\n",
      ((prob.l*sumvy-sumv*sumy)*(prob.l*sumvy-sumv*sumy))/
      ((prob.l*sumvv-sumv*sumv)*(prob.l*sumyy-sumy*sumy))
      );
  }
  else
  { // Classification
    if (reversedCrossValidation)
      printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/total_predicted);
    else
      printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/prob.l);
  }
}

void parse_command_line(int argc, char **argv, char input_file_names[][1024], int *inputFileCount, char *model_file_name)
{
  int i;

  // default values
  param.svm_type = C_SVC;
  param.kernel_type = RBF;
  param.degree = 3;
  param.gamma = 0;  // 1/k
  param.coef0 = 0;
  param.nu = 0.5;
  param.cache_size = 40;
  param.C = 1;
  param.eps = 1e-3;
  param.p = 0.1;
  param.shrinking = 1;
  param.nr_weight = 0;
  param.weight_label = NULL;
  param.weight = NULL;
  param.n_features= 89;
  param.featuredim= 9;
  param.nummax= 50;
  param.simthresh= -999999;
  param.posType= -1;
  param.sigmaPos= 0;
  param.distHistNeighbourhood= 30;
  param.distHistBins= 10;
  param.trainOnlyClass= -1;
  param.tmpModelFilename= "tmpModelFile.%d.dat";
  // Fast local kernel
  param.distCompensationCoeff=0.0;
  param.surfEnhancements=0;
  param.surfLaplacians=0;
  // RLDSV
  param.RLDSV = 0;
  param.RLDSVthreshold = 0.01;
  param.RLDSValgorithm = 1;
  param.RLDSVrankdef = 1;
  param.RLDSVnormalize = 2;


  // parse options
  for(i=1;i<argc;i++)
  {
    if(argv[i][0] != '-') break;
    ++i;
    switch(argv[i-1][1])
    {
      case 's':
        param.svm_type = atoi(argv[i]);
        break;
      case 't':
        param.kernel_type = atoi(argv[i]);
        break;
      case 'd':
        param.degree = atof(argv[i]);
        break;
      case 'g':
        param.gamma = atof(argv[i]);
        break;
      case 'r':
        param.coef0 = atof(argv[i]);
        break;
      case 'n':
        param.nu = atof(argv[i]);
        break;
      case 'm':
        param.cache_size = atof(argv[i]);
        break;
      case 'c':
        param.C = atof(argv[i]);
        break;
      case 'e':
        param.eps = atof(argv[i]);
        break;
      case 'p':
        param.p = atof(argv[i]);
        break;
      case 'h':
        param.shrinking = atoi(argv[i]);
        break;
      case 'v':
        reversedCrossValidation=0;
        cross_validation = 1;
        nr_fold = atoi(argv[i]);
        if(nr_fold < 2)
        {
          fprintf(stderr,"n-fold cross validation: n must >= 2\n");
          exit_with_help();
        }
        break;
      case 'V':
        reversedCrossValidation=1;
        cross_validation = 1;
        nr_fold = atoi(argv[i]);
        if(nr_fold < 2)
        {
          fprintf(stderr,"n-fold cross validation: n must >= 2\n");
          exit_with_help();
        }
        break;
      case 'z':
        everyNthCrossValidation = atoi(argv[i]);
        break;
      case 'x':
        OaAOaOAlg = atoi(argv[i]);
        break;
      case 'y':
        DistMeasure = atoi(argv[i]);
        break;
      case 'w':
        ++param.nr_weight;
        param.weight_label = (int *)realloc(param.weight_label,sizeof(int)*param.nr_weight);
        param.weight = (double *)realloc(param.weight,sizeof(double)*param.nr_weight);
        param.weight_label[param.nr_weight-1] = atoi(&argv[i-1][2]);
        param.weight[param.nr_weight-1] = atof(argv[i]);
        break;
      /* parameters for parallelization */
      case 'o':
        param.trainOnlyClass= atoi(argv[i]);
        break;
      case 'q':
        param.tmpModelFilename= argv[i];
        break;

      /* take local kernel options */
      case 'N':
        param.n_features= atoi(argv[i]);
        break;
      case 'D':
        param.featuredim= atoi(argv[i]);
        break;
      case 'M':
        param.nummax= atoi(argv[i]);
        break;
      case 'S':
        param.simthresh= atof(argv[i]);
        break;
      case 'T':
        param.localKernelType= atoi(argv[i]);
        break;
      case 'Q':
        param.posType= atoi(argv[i]);
        break;
      case 'P':
        param.sigmaPos= atof(argv[i]);
        break;
      case 'U':
        param.surfEnhancements=atoi(argv[i]);
        break;
      case 'L':
        param.surfLaplacians=atoi(argv[i]);
        break;
      case 'A':
        param.distHistNeighbourhood = atoi(argv[i]);
        param.distCompensationCoeff = atof(argv[i]);
        break;
      case 'B':
        param.distHistBins= atoi(argv[i]);
        break;

      // Reduction of SVs that are linearly dependent in feature space
      case 'l':
        if(i>=argc)
        {
          exit_with_help();
        }
        param.RLDSV = 1;
        param.RLDSVthreshold = atof(argv[i]);
        if(param.RLDSVthreshold <= 0)
        {
          fprintf(stderr,"Error: The threshold must be greater than 0!\n\n");
          exit_with_help();
        }
        ++i;
        if(i>=argc)
        {
          fprintf(stderr,"Error: Incorrect algorithm type!\n\n");
          exit_with_help();
        }
        param.RLDSValgorithm = atoi(argv[i]);
        if ((param.RLDSValgorithm != 1) && (param.RLDSValgorithm != 2))
        {
          fprintf(stderr,"Error: Incorrect algorithm type!\n\n");
          exit_with_help();
        }

        break;

//      Turned on by default
//     // Confidence estimation
//     case 'C':
//       param.calcAvgDist = atoi(argv[i]);
//       if ((param.calcAvgDist < 0) || (param.calcAvgDist > 1))
//       {
//         fprintf(stderr,"Error: incorrect value of the -C option!\n\n");
//         exit_with_help();
//       }
//       break;

    default:
        fprintf(stderr,"unknown option %s\n",argv[i-1]);
        exit_with_help();
    }
  }

  // Do some safety checks
  if (param.n_features<param.nummax)
  {
    fprintf(stderr,"Incorrect local kernel parameters. M cannot be greater than N!\n");
    exit_with_help();
  }

  // Determine filenames
  if(i>=argc)
    exit_with_help();

  // Decode input file names
  char *inputFileName = strtok(argv[i],"+");
  (*inputFileCount)=0;
  while (inputFileName != NULL)
  {
    strcpy(input_file_names[(*inputFileCount)], inputFileName);
    (*inputFileCount)++;
    inputFileName = strtok(NULL,"+");
  }

  if(i<argc-1)
    strcpy(model_file_name,argv[i+1]);
  else
  {
    char *p = strrchr(argv[i],'/');
    if(p==NULL)
      p = argv[i];
    else
      ++p;
    sprintf(model_file_name,"%s.model",p);
  }
}


void estimateProblemSize(const char *filename, int *elements, int *l)
{
  FILE *fp = fopen(filename,"r");

  if(fp == NULL)
  {
    fprintf(stderr,"Can't open input file %s\n",filename);
    exit(1);
  }

  (*l) = 0;
  (*elements) = 0;
  while(1)
  {
    int c = fgetc(fp);
    switch(c)
    {
      case '\n':
        ++(*l);
        // fall through,
        // count the '-1' element   (no break here!)
      case ':':
        ++(*elements);
        break;
      case EOF:
        goto out;
      default:
        ;
    }
  }
out:
  printf("Found %d elements in %d samples.\n", *elements, *l);
  fflush(stdout);

  // Close file
  fclose(fp);
}


// read in a problem (in svmlight format)
void read_problem(const char *filename, int l, int *firstVector, int *firstElement)
{
  FILE *fp = fopen(filename,"r");
  if(fp == NULL)
  {
    fprintf(stderr,"Can't open input file %s\n", filename);
    exit(1);
  }

  int max_index = 0;
  int j=(*firstElement);
  int pastLastVector=(*firstVector)+l;
  for(int i=(*firstVector); i<pastLastVector; ++i)
  {
    double label;
    prob.x[i] = &x_space[j];
    fscanf(fp,"%lf",&label);
    prob.y[i] = label;
    while(1)
    {
      int c;
      do
      {
        c = getc(fp);
        if (c=='\n') 
          goto out2;
      }
      while (isspace(c));
      ungetc(c,fp);
      fscanf(fp,"%d:%lf",&(x_space[j].index),&(x_space[j].value));
      ++j;
    }
out2:
    if (j>=1 && x_space[j-1].index > max_index)
      max_index = x_space[j-1].index;
    x_space[j++].index = -1;
    //fflush(stdout);
  }

  // Close file
  fclose(fp);

  // Update indexes for the next file
  (*firstVector)=pastLastVector;
  (*firstElement)=j;

  // Default gamma
  if(param.gamma == 0)
    param.gamma = 1.0/max_index;
}
