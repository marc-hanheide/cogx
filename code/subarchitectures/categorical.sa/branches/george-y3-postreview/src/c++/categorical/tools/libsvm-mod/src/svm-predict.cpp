#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "svm.h"
#include "time.h"

/**
 * Initial values
 */
int max_line_len = 1024;
const int max_nr_attr = 64;


/**
 * Structure storing the command line options
 * and some other variables
 * used by the svm-predict.cpp only and not by
 * the svm_predict function.
 */
struct Options
{
  /** List of input file names. */
  char **inputFileNames;
  /** List of test file handles. */
  FILE **inputFilePtrs;
  /** List of model file names. */
  char **modelFileNames;
  /** Output/confidence file name. */
  char *outputFileName;
  /** Outpout/confidence file handle. */
  FILE *outputFilePtr;
  /** Save plane winner class label or full confidence info?. */
  bool saveConfidence;
  /** Target file name. */
  char *targetFileName;
  /** Target file handle. */
  FILE *targetFilePtr;
  /** Number of cues. */
  int cues;
  /** If 1, we should load precalculated values of f(x) instead of test vectors.*/
  bool loadF;
  /** Name of a file to which we should save values of f(x). */
  char *saveFFileName;
  /** File handle. */
  FILE *saveFFilePtr;
  /** Name of a file to which we should save the scores V(x). */
  char *saveOutputsFileName;
  /** File handle. */
  FILE *saveOutputsFilePtr;
  /** Modify order of classes in the models so that the models match. */
  bool matchModels;
  /** Add labels to the outputs. */
  bool labelOutputs;
  /**
   * Arrays used to reorganize classes in the models (if matchModels=true).
   * There is one array per model. If the pointer to the array is 0, it means
   * the model does not have to be reorganized. If it's non zero, the array
   * contains the proper indexes of the classes for the model e.g.
   * classOrders[1][0]==3 means that for model 1, the class 0 corresponds to
   * class 3 in the first model (used as the proper one).
   */
  int **classOrders;

  /** Constructor, sets default values. */
  Options() : inputFileNames(0), inputFilePtrs(0), modelFileNames(0),
              outputFileName(0), outputFilePtr(0), saveConfidence(false),
              targetFileName(0), targetFilePtr(0), cues(1), loadF(false),
              saveFFileName(0), saveFFilePtr(0), saveOutputsFileName(0),
              saveOutputsFilePtr(0), matchModels(false), labelOutputs(false),
              classOrders(0)
    { }
};



/**
 * Display command line help and exit.
 */
void exit_with_help()
{
  printf("usage: svm-predict <options> (<test_file> <model_file>)* <output_file>\n"
         "General options:\n"
         "-t <file_name>: Create target file\n"
         "-s <file_name>: Save the calculated values of the decision function for the\n"
         "                test samples together with class label to a file. If multiple\n"
         "                test sets are given, only values for the first one are saved.\n"
         "-l <load>: Load precalculated values of the decision function instead of test\n"
         "           vectors from the test files (<test_file>):\n"
         "           0 - No, load stdandard test vectors and compute f(x) (Default)\n"
         "           1 - Yes, load f(x) from files directly\n"
         "-v <file_name>: Saves the outpus of the multi-class classifier (usually the\n"
         "                distance-based scores V(x)) for each test sample. If G-DAS is\n"
         "                used, these will be the outputs obtained after appplying G-DAS.\n"
         "                The outputs are sorted according to the class labels.\n"
         "\n"
         "Options for C-SVC:\n"
         "-b <nr_matches>: How many best matches should be taken into account when\n"
         "                 calculating the classification rate. Default: 1\n"
         "-m <measure>: Measure used while estimating the best matching class:\n"
         "              1 - Value of the decision function f(x) (default)\n"
         "              2 - Normalized distance to the hyperplane\n"
         "              3 - Standard distance to the hyperplane\n"
         "-o <alg>: Algorithm used to find the best matches and confidence for\n"
         "          the one-against-one multi-class method:\n"
         "          1 - Standard voting (with single votes) (default)\n"
         "          2 - Voting with distances from the hyperplanes\n"
         "          3 - Convert OaO to OaA and use alg. 1 for OaA\n"
         "          4 - Dynamically created decision tree (beta)\n"
         "-a <alg>: Algorithm used to find the best matches and confidence for\n"
         "          the once-against-all multi-class method:\n"
         "          1 - Max distance from the hyperplanes (default)\n"
         "          2 - Voting with distances from the hyperplane \n"
         "              (each dec. func. votes eather for one class or the rest)\n"
         "          3 - Min distance from the average distance of the training\n"
         "              samples to the hyperplane\n"
         "-u <alg>: Perform mult-cue combination using the given algorithm:\n"
         "          0 - No multi-cue combination, only the first cue is used (Default)\n"
         "          1 - G-DAS (Generalized Discriminative Accumulation Scheme)\n"
         "-c <conf>: Save information about the order of matches and confidence\n"
         "           into the output file. The confidence estimation method depends\n"
         "           on the value of -o and -a switches. Default: 0\n"
         "-x <x>: Modify order of classes in the models (in case of cue integration)\n"
         "        so that the models match: 0 - No, 1 - Yes (Default: 0)\n"
         "-p <p>: Add labels to the saved outputs.\n"
         "        Values: 0 - No, 1 - Yes (Default: 0)\n"
         "\n"
         "Options for DAS:\n"
         "-Ax <a>: Set the a coefficient for the cue no. x. Default 1/nr_cues.\n"
         "\n"
        );
  exit(1);
}



/**
 * Parses the command line arguments.
 */
void parse_command_line(int argc, char** argv, Options *options,
                        predict_parameter *pred_param)
{
  // Temporary memory for DAS coeffitients
  // So far it will just be tmpDasA[i]=[cueNr, A]
  // but in future we might add separate As for various hyperplanes
  double **tmpDasA=0;
  int tmpDasACount=0;

  // Parse options
  int i;
  for(i=1;i<argc;i++)
  {
    if(argv[i][0] != '-') break;
    ++i;
    switch(argv[i-1][1])
    {
      case 'A': // DAS coeffitients
      {
        // Read cue number
        int cueNr=atoi(argv[i-1]+2);
        if (cueNr<1)
        {
          fprintf(stderr,"Incorrect number of cue in the -A option.\n");
          exit_with_help();
        }
        double cueA=atof(argv[i]);
        if (cueA<0)
        {
          fprintf(stderr,"Incorrect value of the a paramater for DAS.\n");
          exit_with_help();
        }
        tmpDasA=(double**)realloc(tmpDasA, (++tmpDasACount)*sizeof(double*));
        tmpDasA[tmpDasACount-1]=Malloc(double, 2);
        tmpDasA[tmpDasACount-1][0]=cueNr;
        tmpDasA[tmpDasACount-1][1]=cueA;
        break;
      }
      case 'b':
        pred_param->matches=atoi(argv[i]);
        if (pred_param->matches<1)
        {
          fprintf(stderr,"Incorrect value of the -b option.\n");
          exit_with_help();
        }
        break;
      case 'm':
        pred_param->measure=atoi(argv[i]);
        if ((pred_param->measure<1) || (pred_param->measure>3))
        {
          fprintf(stderr,"Incorrect value of the -m option.\n");
          exit_with_help();
        }
        break;
      case 'o':
        pred_param->oaoAlg=atoi(argv[i]);
        if ((pred_param->oaoAlg<1) || (pred_param->oaoAlg>4))
        {
          fprintf(stderr,"Incorrect value of the -o option.\n");
          exit_with_help();
        }
        break;
      case 'a':
        pred_param->oaaAlg=atoi(argv[i]);
        if ((pred_param->oaaAlg<1) || (pred_param->oaaAlg>3))
        {
          fprintf(stderr,"Incorrect value of the -a option.\n");
          exit_with_help();
        }
        break;
      case 'u':
        pred_param->multiCueAlg=atoi(argv[i]);
        if ((pred_param->multiCueAlg<0) || (pred_param->multiCueAlg>1))
        {
          fprintf(stderr,"Incorrect value of the -u option.\n");
          exit_with_help();
        }
        break;
      case 'c':
        if ((atoi(argv[i])!=0) && (atoi(argv[i])!=1))
        {
          fprintf(stderr,"Incorrect value of the -c option.\n");
          exit_with_help();
        }
        options->saveConfidence=(atoi(argv[i])==1);
        break;
      case 't':
        options->targetFileName=Malloc(char, strlen(argv[i])+1);
        strcpy(options->targetFileName, argv[i]);
        break;
      case 's':
        options->saveFFileName=Malloc(char, strlen(argv[i])+1);
        strcpy(options->saveFFileName, argv[i]);
        break;
      case 'v':
        options->saveOutputsFileName=Malloc(char, strlen(argv[i])+1);
        strcpy(options->saveOutputsFileName, argv[i]);
        break;
      case 'l':
        if ((atoi(argv[i])!=0) && (atoi(argv[i])!=1))
        {
          fprintf(stderr,"Incorrect value of the -l option.\n");
          exit_with_help();
        }
        options->loadF=(atoi(argv[i])==1);
        break;
      case 'x':
        if ((atoi(argv[i])!=0) && (atoi(argv[i])!=1))
        {
          fprintf(stderr,"Incorrect value of the -x option.\n");
          exit_with_help();
        }
        options->matchModels=(atoi(argv[i])==1);
        break;
      case 'p':
        if ((atoi(argv[i])!=0) && (atoi(argv[i])!=1))
        {
          fprintf(stderr,"Incorrect value of the -p option.\n");
          exit_with_help();
        }
        options->labelOutputs=(atoi(argv[i])==1);
        break;
      default:
        fprintf(stderr,"unknown option\n");
        exit_with_help();
    }
  }

  // Determine filenames
  // Check if we have a proper amount of arguments
  if ((((argc-i)%2)!=1) || (argc-i<3))
  {
    fprintf(stderr,"Incorrect number of arguments.\n");
    exit_with_help();
  }

  // Count the cues, but only if we have multi-cue combination enabled
  if (pred_param->multiCueAlg)
  {
    options->cues=((argc-i-1)/2);
    pred_param->cues=options->cues;

    if (options->cues<2)
    {
      fprintf(stderr,"At least two model files and test sets are needed to do cue accumulation!\n");
      exit_with_help();
    }

    // If DAS, take care of DAS a parameters
    if (pred_param->multiCueAlg==MULTICUEALG_DAS)
    {
      // Allocate memory
      pred_param->dasA = Malloc(double *, options->cues);
      // Set defaults
      for (int i=0;i<options->cues; ++i)
      {
        pred_param->dasA[i]=Malloc(double, 1);
        pred_param->dasA[i][0]=1.0/((double)options->cues);
      }
      // Read the params that were provided
      for (int i=0; i<tmpDasACount; ++i)
      {
        if ((tmpDasA[i][0]<1) || (tmpDasA[i][0]>options->cues))
        {
          fprintf(stderr,"Incorrect number of cue in the -A option.\n");
          exit_with_help();
        }
        pred_param->dasA[(int)(tmpDasA[i][0])-1][0] = tmpDasA[i][1];
      }
    }
  }
  else
  {
    options->cues=1;
    pred_param->cues=1;
  }

  // Allocate the test and model filename list
  options->inputFileNames=Malloc(char*, options->cues);
  options->modelFileNames=Malloc(char*, options->cues);

  // Copy the model and test filenames
  for (int j=0; j<options->cues; ++j)
  {
    options->inputFileNames[j]=Malloc(char, strlen(argv[i+j*2])+1);
    strcpy(options->inputFileNames[j], argv[i+j*2]);
    options->modelFileNames[j]=Malloc(char, strlen(argv[i+j*2+1])+1);
    strcpy(options->modelFileNames[j], argv[i+j*2+1]);
    //printf("T:%s\n",options->inputFileNames[j]);
    //printf("M:%s\n",options->modelFileNames[j]);
  }

  // Copy the output/confidence file name.
  options->outputFileName=Malloc(char, strlen(argv[argc-1])+1);
  strcpy(options->outputFileName, argv[argc-1]);

  // Check the file names
  bool goodFileNames=true;
  for (int j=0; j<options->cues; ++j)
    if ((strlen(options->inputFileNames[j])==0) ||
        (strlen(options->modelFileNames[j])==0))
    {
      goodFileNames=false;
      break;
    }

  if ((!goodFileNames) || (strlen(options->outputFileName) == 0))
  {
    exit_with_help();
  }

  // Free temporary memory
  if (tmpDasA)
  {
    for (int i=0; i<tmpDasACount; ++i)
      free(tmpDasA[i]);
    free(tmpDasA);
  }
}




/**
 * Reads line from file.
 */
char* readLine(FILE *input, char *&line)
{
  int len;

  if(fgets(line,max_line_len,input) == NULL)
    return NULL;

  while(strrchr(line,'\n') == NULL)
  {
    max_line_len *= 2;
    line = (char *) realloc(line, max_line_len);
    len = strlen(line);
    if(fgets(line+len,max_line_len-len,input) == NULL)
      break;
  }
  return line;
}


/**
 * Decodes a line from text to array of nodes.
 */
bool decodeTestVector(const char *line, int &max_nr_attr, svm_node *&x, double &target)
{
  // Some definitions
  #define SKIP_TARGET\
  while(isspace(*line)) ++line;\
  while(!isspace(*line)) ++line;

  #define SKIP_ELEMENT\
  while(*line!=':') ++line;\
  ++line;\
  while(isspace(*line)) ++line;\
  while(*line && !isspace(*line)) ++line;

  int i = 0;

  // Read target
  if(sscanf(line,"%lf",&target)!=1)
    return false;
  SKIP_TARGET

  // Read values
  while(sscanf(line,"%d:%lf",&x[i].index,&x[i].value)==2)
  {
    SKIP_ELEMENT;
    ++i;
    if(i>=max_nr_attr-1)  // need one more for index = -1
    {
      max_nr_attr *= 2;
      x = (svm_node *) realloc(x,max_nr_attr*sizeof(svm_node));
    }
  }
  x[i].index = -1;

  // Everything ok
  return true;
}


bool decodeFValues(const char *line, int count, double *f_values, double &target)
{
  int scanned;
  // Read target
  if(sscanf(line," %lf%n",&target, &scanned)!=1)
    return false;
  line+=scanned;

  for (int i=0; i<count; ++i)
  {
    if(sscanf(line," %lf%n",&(f_values[i]), &scanned)!=1)
      return false;
    line+=scanned;
  }

  // Everything ok
  return true;
}



/**
 * Reads lines from test files and for each line performs
 * the prediction using svm_predict.
 */
void predict(const svm_model* const * models, const Options *options,
             const predict_parameter *pred_param)
{
  // Initialize variables
  int correct = 0;
  int total = 0;
  int nr_class = models[0]->nr_class;
  int nr_hyp = models[0]->nr_hyp;
  int cues = options->cues;
  double error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;

  // Allocate line
  char* line = (char *) malloc(max_line_len*sizeof(char));

  // Allocate x vectors and initial size of x vectors if needed
  svm_node **xes=0;
  int *max_nr_attr_tab=0;
  if (!options->loadF)
  {
    xes = Malloc(svm_node*, cues);
    max_nr_attr_tab = Malloc(int, cues);
    for (int i=0; i<cues; ++i)
    {
      max_nr_attr_tab[i]=max_nr_attr;
      xes[i]=(svm_node *) malloc(max_nr_attr*sizeof(svm_node));
    }
  }

  // Allocate arrays for fvalues
  double **f_values = Malloc(double*, cues);
  for (int i=0; i<cues; ++i)
    f_values[i]=Malloc(double, nr_hyp);

  // Allocate class and confidence
  int *classes = (int*)malloc(sizeof(int)*nr_class);
  double *confidence = (double*)malloc(sizeof(double)*nr_class);
  double *outputs = (double*)malloc(sizeof(double)*nr_hyp);
  int *outputLabels1 = (int*)malloc(sizeof(int)*nr_hyp);
  int *outputLabels2 = (int*)malloc(sizeof(int)*nr_hyp);
  int outputsCount=0;

  // Print info
  printf("Predicting... ");
  fflush(stdout);

  // Time statistics
  double avgTimeReal=0;
  double avgTime2Real=0;
  double avgTimeVirt=0;
  double avgTime2Virt=0;
  double avgTimeProf=0;
  double avgTime2Prof=0;
  long countTime=0;

  // Read the input files and predict
  while(1)
  {
    // Initialize variables
    double v;
    double target=0;
    // Read a line from each input file and decode
    bool end=false;
    for (int i=0; i<cues; ++i)
    {
      // Read a line from file
      if(readLine(options->inputFilePtrs[i], line)==NULL)
      {
        end=true;
        break;
      }
      // Decode the line
      double tmp_target;
      if (options->loadF)
      {
        if (!decodeFValues(line, nr_hyp, f_values[i], tmp_target))
        {
          end=true;
          break;
        }

        // Convert the values if necessary to match the fx files created for different order of classes in the model
        if ((options->classOrders) && (options->classOrders[i]))
        {
          double *tmpF = Malloc(double, nr_hyp);
          memcpy(tmpF,f_values[i], sizeof(double)*nr_hyp);
          if (models[i]->param.svm_type==ONE_AGAINST_ALL)
          {
             for (int j=0; j<nr_hyp; ++j)
                f_values[i][options->classOrders[i][j]] = tmpF[j];
          }
          else if (models[i]->param.svm_type==C_SVC)
          {
            int p=0;
            for(int u=0; u<nr_class; ++u)
              for(int v=u+1; v<nr_class; ++v)
              {
                // Translate the class numbers
                int a = options->classOrders[i][u];
                int b = options->classOrders[i][v];
                bool reverse=false;
                if (a>b)
                { int tmp=a; a=b; b=tmp; reverse=true; }
                // Get new p value according to eq: p=a(n-(1+a)/2)+b-a-1 (assumption a<b)
                int newP=(a*nr_class-(a+a*a)/2)+b-a-1;
                // Do conversion
                if (reverse)
                  f_values[i][newP] = -tmpF[p];
                else
                  f_values[i][newP] = tmpF[p];
                ++p;
              }
          }
          free(tmpF);
        }
      }
      else
      {
        if (!decodeTestVector(line, max_nr_attr_tab[i], xes[i], tmp_target))
        {
          end=true;
          break;
        }
      }

      // Save target or check target
      if (i==0)
        target=tmp_target;
      else
      {
        if (target!=tmp_target)
        {
          fprintf(stderr, "ERROR: Target labels doesn't match!");
          end=true;
          break;
        }
      }
    }

    // Was everything ok?
    if (end)
      break;

    // Start measuring time
    timerReset();

    // Invoke svm_predict
    // xes=0 if the options.loadF=true and precomputed values of f(x) will be used
    if(svm_predict(models, xes, f_values, pred_param, target, &v, classes, confidence, outputs, outputLabels1, outputLabels2, &outputsCount ))
      ++correct;

    // Stop measuring time and add statistics
    TimeInterval timeI = timerQuery();
    avgTimeReal+=timeI.real;
    avgTime2Real+=timeI.real*timeI.real;
    avgTimeVirt+=timeI.virt;
    avgTime2Virt+=timeI.virt*timeI.virt;
    avgTimeProf+=timeI.prof;
    avgTime2Prof+=timeI.prof*timeI.prof;
    countTime++;

    // Perform some calculations
    error += (v-target)*(v-target);
    sumv += v;
    sumy += target;
    sumvv += v*v;
    sumyy += target*target;
    sumvy += v*target;
    ++total;

    // Print into the target file
    if (options->targetFilePtr)
      fprintf(options->targetFilePtr, "%g\n",target);

    // Print into the f value file
    if (options->saveFFilePtr)
    {
      fprintf(options->saveFFilePtr, "%g ",target);
      for (int i=0; i<nr_hyp; ++i)
        fprintf(options->saveFFilePtr, "%g ",f_values[0][i]);
      fprintf(options->saveFFilePtr, "\n");
    }

    // Print into the V (score) file
    if (options->saveOutputsFilePtr)
    {
      if (options->labelOutputs)
      {
        for (int i=0; i<outputsCount; ++i)
        {
          if (outputLabels2[i]<0)
            fprintf(options->saveOutputsFilePtr, "%d:%.10g ",outputLabels1[i], outputs[i]);
          else
            fprintf(options->saveOutputsFilePtr, "%d_%d:%.10g ",outputLabels1[i], outputLabels2[i], outputs[i]);
        }
        fprintf(options->saveOutputsFilePtr, "\n");
      }
      else
      {
        for (int i=0; i<outputsCount; ++i)
          fprintf(options->saveOutputsFilePtr, "%.10g ",outputs[i]);
        fprintf(options->saveOutputsFilePtr, "\n");
      }
    }

    // Print into the output file
    if (options->saveConfidence)
    { // Write full info including confidence
      for (int i=0; i<nr_class; ++i)
        fprintf(options->outputFilePtr, "%d:%g ", classes[i], confidence[i]);
      fprintf(options->outputFilePtr, "\n");
    }
    else
    { // Write standard output info i.e. the best match only
      fprintf(options->outputFilePtr,"%g\n",v);
    }
  } // END while(1)


  // Printout the results
  printf("Done!\n");
  fflush(stdout);
  printf("Accuracy = %g%% (%d/%d) (classification)\n",
    (double)correct/total*100,correct,total);
  printf("Mean squared error = %g (regression)\n",error/total);
  printf("Squared correlation coefficient = %g (regression)\n",
    ((total*sumvy-sumv*sumy)*(total*sumvy-sumv*sumy))/
    ((total*sumvv-sumv*sumv)*(total*sumyy-sumy*sumy))
    );

  // Print statistics
  avgTimeReal/=countTime;
  avgTime2Real/=countTime;
  avgTimeVirt/=countTime;
  avgTime2Virt/=countTime;
  avgTimeProf/=countTime;
  avgTime2Prof/=countTime;
  printf("Time: real=%.1f[+/-]%.1fms virt=%.1f[+/-]%.1fms prof=%.1f[+/-]%.1fms", 
         avgTimeReal*1000.0, sqrt(avgTime2Real-avgTimeReal*avgTimeReal)*1000.0,
         avgTimeVirt*1000.0, sqrt(avgTime2Virt-avgTimeVirt*avgTimeVirt)*1000.0,
         avgTimeProf*1000.0, sqrt(avgTime2Prof-avgTimeProf*avgTimeProf)*1000.0);

  // Clean up
  free(classes);
  free(confidence);
  free(outputs);
  free(outputLabels1);
  free(outputLabels2);
  free(line);
  for(int i=0; i<cues; ++i)
    free(f_values[i]);
  free(f_values);
  if (!options->loadF)
  {
    for(int i=0; i<cues; ++i)
      free(xes[i]);
    free(xes);
    free(max_nr_attr_tab);
  }
}



/**
 * Check if several models match and can be used for cue combination.
 */
bool matchModels(const svm_model* const* models, int count)
{
  for (int i=1; i<count; ++i)
  {
    if ((models[i]->param.svm_type!=models[0]->param.svm_type) ||
        (models[i]->nr_class!=models[0]->nr_class) ||
        (models[i]->nr_hyp!=models[0]->nr_hyp))
      return false;
    for(int j=0; j<models[0]->nr_class; ++j)
      if (models[i]->label[j]!=models[0]->label[j])
        return false;
  }
  return true;
}



/**
 * Calculates the proper order of classes in the models.
 */
void getClassOrders(const svm_model* const* models, Options *options)
{
  // Initialization
  const svm_model *firstModel=models[0];
  int nrClass = firstModel->nr_class;
  int modelCount = options->cues;
  options->classOrders = Malloc(int*, modelCount);
  int **classOrders = options->classOrders;
  classOrders[0]=0; // First model is the one we use as an example

  // Analyse all models
  for (int i=1; i<modelCount; ++i)
  {
    // Assume that the models match exatly
    classOrders[i]=0;
    // Check if the number of classes is the same
    // If not, just skip the model (the matchModels function will protest later)
    if ( (firstModel->nr_class!=models[i]->nr_class) || (firstModel->nr_hyp!=models[i]->nr_hyp) )
      continue;
    // Check if the class labels are the same (but not necesarily ordered the same way)
    bool error=false;
    for(int j=0; j<nrClass; ++j)
    {
      int k;
      for(k=0; k<nrClass; ++k)
        if (models[i]->label[j]==firstModel->label[k])
          break;
      if (k==nrClass)
      {
        error=true;
        break;
      }
    }
    if (error)
      continue;
    // Check if the labels match for the models
    bool modelsMatch=true;
    for(int j=0; j<nrClass; ++j)
    {
      if (models[i]->label[j]!=firstModel->label[j])
      {
        modelsMatch=false;
        break;
      }
    }
    // Act accordingly
    if (!modelsMatch)
    { // Models do not match, find the correct order
      classOrders[i]=Malloc(int, nrClass);
      for (int j=0; j<nrClass; ++j)
      {
        classOrders[i][j]=0;
        for(int k=0; k<nrClass; ++k)
          if (models[i]->label[j]==firstModel->label[k])
            classOrders[i][j]=k;
      } // for
    } // if
  } // for
} // getClassOrders


/** Main */
int main(int argc, char **argv)
{
  // Define variables
  Options options;
  predict_parameter pred_param;
  svm_model** models;

  // Parse command line parameters
  parse_command_line(argc, argv, &options, &pred_param);

  // Open the files that should be opened
  // Open test set files
  options.inputFilePtrs=Malloc(FILE*, options.cues);
  for (int i=0; i<options.cues; ++i)
  {
    options.inputFilePtrs[i] = fopen(options.inputFileNames[i],"r");
    if(options.inputFilePtrs[i] == NULL)
    {
      fprintf(stderr,"Can't open input file %s\n", options.inputFileNames[i]);
      exit(1);
    }
  }

  // Open output file
  options.outputFilePtr = fopen(options.outputFileName,"w");
  if(options.outputFilePtr == NULL)
  {
    fprintf(stderr,"Can't open output file %s\n",options.outputFileName);
    exit(1);
  }

  // Open target file
  if (options.targetFileName)
  {
    options.targetFilePtr = fopen(options.targetFileName,"w");
    if(options.targetFilePtr == NULL)
    {
      fprintf(stderr,"Can't open file %s\n", options.targetFileName);
      exit(1);
    }
  }

  // Open saveF file
  if (options.saveFFileName)
  {
    options.saveFFilePtr = fopen(options.saveFFileName,"w");
    if(options.saveFFilePtr == NULL)
    {
      fprintf(stderr,"Can't open file %s\n", options.saveFFileName);
      exit(1);
    }
  }

  // Open saveV (score) file
  if (options.saveOutputsFileName)
  {
    options.saveOutputsFilePtr = fopen(options.saveOutputsFileName,"w");
    if(options.saveOutputsFilePtr == NULL)
    {
      fprintf(stderr,"Can't open file %s\n", options.saveOutputsFileName);
      exit(1);
    }
  }

  // Load models
  models=Malloc(svm_model*, options.cues);
  for (int i=0; i<options.cues; ++i)
  {
    printf("Loading model %d... ", i+1);
    fflush(stdout);
    if((models[i]=svm_load_model(options.modelFileNames[i], options.loadF))==0)
    {
      fprintf(stderr, "Can't open model file %s\n", options.modelFileNames[i]);
      exit(1);
    }
    printf("Done!\n");
    fflush(stdout);
  }

  // Correct the order of classes in the models in case of multiple models
  if (options.matchModels)
  {
    // Find the proper order of classes
    getClassOrders(models, &options);

    // Correct the loaded models
    for (int i=1; i<options.cues; ++i)
      if (options.classOrders[i])
        svm_reorganize_model(models[i], options.classOrders[i]);
  }

  // Check if the models match
  if (!matchModels(models, options.cues))
  {
    fprintf(stderr, "The models do not match! Try using the -x option.\n");
    exit(1);
  }

  // Do the prediction
  predict(models, &options, &pred_param);

  // Destroy models
  for (int i=0; i<options.cues; ++i)
    svm_destroy_model(models[i]);
  free(models);

  // Destroy memory allocated in options and close the files
  if (options.targetFileName)
  {
    free(options.targetFileName);
    fclose(options.targetFilePtr);
  }
  if (options.saveFFileName)
  {
    free(options.saveFFileName);
    fclose(options.saveFFilePtr);
  }
  if (options.saveOutputsFileName)
  {
    free(options.saveOutputsFileName);
    fclose(options.saveOutputsFilePtr);
  }
  for (int i=0; i<options.cues; ++i)
  {
    free(options.inputFileNames[i]);
    fclose(options.inputFilePtrs[i]);
    free(options.modelFileNames[i]);
  }
  free(options.inputFileNames);
  free(options.inputFilePtrs);
  free(options.modelFileNames);
  free(options.outputFileName);
  fclose(options.outputFilePtr);
  if (options.classOrders)
  {
    for (int i=0; i<options.cues; ++i)
      free(options.classOrders[i]);
    free(options.classOrders);
  }

  // Everything ok
  return 0;
}




