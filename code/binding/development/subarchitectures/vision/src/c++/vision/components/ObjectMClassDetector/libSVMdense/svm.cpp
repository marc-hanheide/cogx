#include<libSVMdense/svm.h>
#include<vector>
#include<string>
#include<algorithm>
#include<iostream>
#include<fstream>
#include<ostream>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

using namespace SVMLight_ns;

SVMlight::SVMlight() {
 	initVariables();
}

void SVMlight::initVariables() {
  max_training_samples = 0;
  current_training_sample = 0;
  allocatedWordLength = 0;
  words = 0;
  m_model = 0;
  alpha_in = 0;
  kernel_cache =0;
  docs = 0;
  target = 0;
  
  strcpy (learn_parm.predfile, "trans_predictions");
  strcpy (learn_parm.alphafile, "");
  
  learn_parm.type = CLASSIFICATION;
  learn_parm.biased_hyperplane=1;
  learn_parm.sharedslack=0;
  learn_parm.remove_inconsistent=0;
  learn_parm.skip_final_opt_check=0;
  learn_parm.svm_maxqpsize=10;
  learn_parm.svm_newvarsinqp=0;
  learn_parm.svm_iter_to_shrink=-9999;
  learn_parm.maxiter=100000;
  learn_parm.kernel_cache_size=40;
  learn_parm.svm_c=0.0;
  learn_parm.eps=0.1;
  learn_parm.transduction_posratio=-1.0;
  learn_parm.svm_costratio= 1;
  learn_parm.svm_costratio_unlab=1.0;
  learn_parm.svm_unlabbound=1E-5;
  learn_parm.epsilon_crit=0.001;
  learn_parm.epsilon_a=1E-15;
  learn_parm.compute_loo=0;
  learn_parm.rho=1.0;
  learn_parm.xa_depth=0;
  
  kernel_parm.kernel_type= 0;
  kernel_parm.poly_degree=3;
  kernel_parm.rbf_gamma=1.0;
  kernel_parm.coef_lin=1;
  kernel_parm.coef_const=1;
  
  maxNormalizer = 0;
  maxNormalized = false;
}

SVMlight::SVMlight(long maxSamples) {
	initVariables();
	set_max_training_samples(maxSamples);
}


SVMlight::~SVMlight() {	
	if (alpha_in) {
		free(alpha_in);
		alpha_in = 0;
	}	
	if (m_model) {
		free_model(m_model,0);
		m_model = 0;
	}
	clear_training_samples();
	if (words) {
		free(words);
		words = 0;
	}
	
	if (maxNormalizer) {
		free(maxNormalizer);
		maxNormalizer = 0;
	}	
	
}

void SVMlight::clear_training_samples() {	
   if (docs) {   	
   		for(int i=0; i< current_training_sample; i++) { 
   			if (docs[i]) free_example(docs[i],1);
   			docs[i] = 0;
   		}
   		free(docs);
   		docs = 0;
   }
   
   if (target) {
   	free(target);
   	target = 0;
   }
   
}

void SVMlight::set_max_training_samples(long maxSamples) {	
	clear_training_samples();
	printf("Clearing all training samples...\n");
	
	docs = (DOC **)my_malloc(sizeof(DOC *)* (maxSamples + 1));
  	target = (double *)my_malloc(sizeof(double) * maxSamples + 1); /* target values */
  	
  	max_training_samples = maxSamples;
	current_training_sample = 0;	
}

bool SVMlight::add_training_sample(const std::vector<float> &features, float targetValue) {
	
	if(current_training_sample == max_training_samples) {
		printf("Running out of memory! Sample has not been stored!\n");
		return false;
	}		
		
	if (allocatedWordLength != features.size() && words != 0) {
		printf("Dimension does not match previous dimensions! Not considering sample!\n");
		return false;
	}	

	const int dimensions = features.size();
		
	if (words == 0) {
		  words = (WORD *)my_malloc(sizeof(WORD)*(dimensions + 10));
		  allocatedWordLength = dimensions;
	}	
	
	if (maxNormalizer == 0) {
		maxNormalizer = Malloc(FVAL, allocatedWordLength);
		for(unsigned int i = 0; i < allocatedWordLength; ++i)
			maxNormalizer[i] = 0;
	} else if (maxNormalized)
			maxUnNormalize();
				
	for(int k = 0; k < dimensions; k++) {
		words[k].wnum= k + 1;
     	words[k].weight= features[k];
     	//std::cout << features[k] << std::endl;
     	if (fabsf(features[k]) > maxNormalizer[k])
     		maxNormalizer[k] = fabsf(features[k]);
	} 
	words[dimensions].wnum = 0;
	
	docs[current_training_sample] = create_example(current_training_sample,0,0,1, create_svector(words, dimensions, "",1.0));
	target[current_training_sample++] = targetValue;
	
	return true;
}

bool SVMlight::add_training_sample(const float* features, int dimensions, float targetValue) {
	
	if(current_training_sample == max_training_samples) {
		printf("Running out of memory! Sample has not been stored!\n");
		return false;
	}		
		
	if (allocatedWordLength != static_cast<unsigned int>(dimensions) && words != 0) {
		printf("Dimension does not match previous dimensions! Not considering sample!\n");
		return false;
	}
		
	if (words == 0) {
		  words = (WORD *)my_malloc(sizeof(WORD)*(dimensions + 10));
		  allocatedWordLength = dimensions;		  
	}
	
	if (maxNormalizer == 0) {
			maxNormalizer = Malloc(FVAL, allocatedWordLength);
			for(unsigned int i = 0; i < allocatedWordLength; ++i)
				maxNormalizer[i] = 0;
		} else if (maxNormalized)
			maxUnNormalize();		
			
	for(int k = 0; k < dimensions; k++) {
		words[k].wnum= k + 1;
     	words[k].weight= features[k];
     	
     	if (fabsf(features[k]) > maxNormalizer[k])
     	     		maxNormalizer[k] = fabsf(features[k]);
	} 
	words[dimensions].wnum = 0;
	
	docs[current_training_sample] = create_example(current_training_sample,0,0,1, create_svector(words, dimensions, "",1.0));
	target[current_training_sample++] = targetValue;
	
	return true;
}

#ifdef DENSE
void SVMlight::save_features(const char* featureFile) {
	if(current_training_sample == 0) {
	  	printf("No examples to save!\n");
	  	return;
	}
	
	std::ofstream binarySave (featureFile, std::ios::out | std::ios::binary);
		
	if (!binarySave.is_open()) {
		std::cout << "Error opening " << featureFile << "!" << std::endl;
		return;	
	}
	
	binarySave.write(reinterpret_cast<char*>(&current_training_sample), sizeof(current_training_sample));
	binarySave.write(reinterpret_cast<char*>(&allocatedWordLength), sizeof(allocatedWordLength));
	
	for(int i = 0; i < current_training_sample; ++i) {
		binarySave.write(reinterpret_cast<char*>(&target[i]), 
										sizeof(target[i]));	
		for(unsigned int j = 0; j < allocatedWordLength; ++j) {			
			binarySave.write(reinterpret_cast<char*>(&(docs[i]->fvec->words[j])), 
					sizeof(docs[i]->fvec->words[j]));			
		}		
	}
	
	binarySave.close();
}


void SVMlight::loadFeatures(const char* fileName, int& posSamples, int& negSamples) {
	posSamples = 0;
	negSamples = 0;
	clear_training_samples();
		
	std::ifstream binaryLoad (fileName, std::ios::in | std::ios::binary);
			
	if (!binaryLoad.is_open()) {
		std::cout << "Error opening " << fileName << "!" << std::endl;
		return;	
	}
		
	long numSamples;			
	unsigned int featureDim;
	
	binaryLoad.read(reinterpret_cast<char*>(&numSamples), sizeof(numSamples));
	binaryLoad.read(reinterpret_cast<char*>(&featureDim), sizeof(featureDim));
	
	double target;
	
	std::cout << "Number of samples: "<< numSamples << std::endl;
	std::cout << "Feature dimension: " << featureDim << std::endl;
	
	std::vector<float> fv(featureDim, 0);
	
	set_max_training_samples( numSamples );
	
	for(int i = 0; i < numSamples; ++i) {
		binaryLoad.read(reinterpret_cast<char*>(&target), sizeof(target));
		
		for(unsigned int j = 0; j < featureDim; ++j) {			
			binaryLoad.read(reinterpret_cast<char*>(&fv[j]), sizeof(fv[j]));			
		}		
		add_training_sample(fv, target);
		
		if (target == 1.0)
			posSamples++;
		else
			negSamples++;		
	}
		
	std::cout << "Loaded " << posSamples << " positive samples and  " << negSamples << " negative Samples..." << std::endl;
		
	binaryLoad.close();	
}

#else
void SVMlight::save_features(const char* featureFile) {
	std::cout << "Not implemented yet!!" << std::endl;
}

void SVMlight::loadFeatures(const char* fileName, int& posSamples, int& negSamples) {
	std::cout << "Not implemented yet!!" << std::endl;
}

#endif

void SVMlight::save_features_text(const char* featureFile) {
	std::ofstream of(featureFile);
		
	if (!of.is_open()) {
		std::cout << "Could not open " << featureFile << " for output!" << std::endl;
		return;
	}
	
	for(int i = 0; i < current_training_sample; ++i) {
		if (target[i] == 1)
			of << "+";
		
		of << target[i] << " ";
		for(unsigned int j = 0; j < allocatedWordLength; ++j) {	
			of << j + 1 << ":" << docs[i]->fvec->words[j] << " ";				
		}		
		of << std::endl;
	}
	
	of.close();	
}


float SVMlight::proposeC() {

	if(current_training_sample == 0) {
	  	printf("No examples to learn from!\n");
	  	return 0;
	}
	
	float r_delta_avg = estimate_r_delta_average(docs, current_training_sample - 1, &kernel_parm );
	
	float c =  1.0/ (r_delta_avg * r_delta_avg);
	
	if (verbosity >= 1) {
		printf("Slack (C) estimation:\n");
		printf("=====================\n");
		printf("avg x*x^-1: %.15f\n\n", c);
	}
	
	return c;
}

#ifdef DENSE
void SVMlight::maxNormalize() {	
	for(unsigned int j = 0; j < allocatedWordLength; ++j) {		
		for(int i = 0; i < current_training_sample - 1; ++i) {
			if (maxNormalizer[j] != 0)
				docs[i]->fvec->words[j] /= maxNormalizer[j];
		}		
	}
	
	maxNormalized = true;
	
}
#else
void SVMlight::maxNormalize() {
	printf("Max normalize not implemented for sparse feature vectors!\n");
}
#endif

#ifdef DENSE
void SVMlight::maxUnNormalize() {
	for(unsigned int j = 0; j < allocatedWordLength; ++j) {
		
		for(int i = 0; i < current_training_sample - 1; ++i) {		
			docs[i]->fvec->words[j] *= maxNormalizer[j];
		}	
	}	
	maxNormalized = false;
	
}
#else
void SVMlight::maxUnNormalize() {
	printf("Max unnormalize not implemented for sparse feature vectors!\n");
}
#endif

float SVMlight::proposeSigma(float subSet) {
	if(current_training_sample < 3) {
	  	printf("No examples to learn from!\n");
	  	return 0;
	}
	
	if (subSet < 0)
		subSet = 0;
	if (subSet > 1)
		subSet = 1;
	
	std::vector<int> indices;
	int usedSamples = current_training_sample - 1;
	for(int i = 0; i < current_training_sample - 1; ++i)
		indices.push_back(i);
	
	if (subSet > 0.0) {
		random_shuffle(indices.begin(), indices.end());
		usedSamples = static_cast<int>(floorf(usedSamples * subSet));
	} 	
		
	std::vector<float> dists;
	dists.reserve((usedSamples) * (usedSamples));
	
	// Preprocess 
	std::vector<double> squares(usedSamples);
	for(int i = 0; i < usedSamples; ++i) {		
		squares[i] = sprod_ss(docs[indices[i]]->fvec, docs[indices[i]]->fvec);
	}
		
	
	for(int i = 0; i < usedSamples; ++i) {
		//printf("%d\n", i);
		for(int j = i + 1; j < usedSamples; ++j) {
			
			float len = sqrt(squares[i] - 2 * sprod_ss(docs[indices[i]]->fvec, docs[indices[j]]->fvec) + squares[j]);			
			dists.push_back(len);
			
		}
	}
	
	sort(dists.begin(), dists.end());
	
	
	float maxDist = dists.back();
	float minDist = dists.front();
	
	float median = dists[dists.size() / 2];
	
	if (verbosity >= 1) {
		printf("RBF sigma estimation:\n");
		printf("=====================\n");
		printf("min pairwise sample dist: %f\n", minDist);
		printf("median pairwise sample dist: %f\n", median);
		printf("max pairwise sample dist: %f\n\n", maxDist);
	}
	
	return median;	
}


void SVMlight::train(bool probabilities) {
  
   if(current_training_sample == 0) {
  	printf("No examples to learn from!\n");
  	return;
  }  
  
  if(m_model != 0) {
  	free_model(m_model, 0);
  	m_model = 0;
  }	
  
  m_model = new MODEL(); //(MODEL *)my_malloc(sizeof(MODEL));

  if(learn_parm.svm_iter_to_shrink == -9999) {
    if(kernel_parm.kernel_type == LINEAR) 
      learn_parm.svm_iter_to_shrink=2;
    else
      learn_parm.svm_iter_to_shrink=100;
  }

  if(kernel_parm.kernel_type == LINEAR) { /* don't need the cache */
  		kernel_cache=NULL;
  } else {  
  		 /* Always get a new kernel cache. It is not possible to use the
  		same cache for two different training runs */
  		kernel_cache=kernel_cache_init(current_training_sample-1,learn_parm.kernel_cache_size);
   }
     
  if(learn_parm.type == CLASSIFICATION) {
    	svm_learn_classification(docs, target, current_training_sample-1, allocatedWordLength, &learn_parm,
    	 &kernel_parm, kernel_cache, m_model, alpha_in);
  }
  else if(learn_parm.type == REGRESSION) {
    svm_learn_regression(docs,target,current_training_sample-1,allocatedWordLength,&learn_parm,
			 &kernel_parm,&kernel_cache,m_model);
  }
  else if(learn_parm.type == RANKING) {
    svm_learn_ranking(docs,target,current_training_sample-1,allocatedWordLength,&learn_parm,
		      &kernel_parm,&kernel_cache,m_model);
  }
  else if(learn_parm.type == OPTIMIZATION) {
    svm_learn_optimization(docs,target,current_training_sample-1,allocatedWordLength,&learn_parm,
			   &kernel_parm,kernel_cache,m_model,alpha_in);
  }
	     
  if(kernel_cache) {
	  /* Free the memory used for the cache. */
	  kernel_cache_cleanup(kernel_cache);
	  kernel_cache = 0;
  }
  
  // Store the maxes with the model
  m_model->maxNormalized = maxNormalized;
  m_model->maxNormalizer = (FVAL*)my_malloc(sizeof(FVAL) * m_model->totwords);
  for(int i = 0; i < m_model->totwords; ++i) {
	  m_model->maxNormalizer[i] = maxNormalizer[i];
  }
  
  if (probabilities)
	  trainProbabilities();
  
  if(m_model->kernel_parm.kernel_type == SVMLight_ns::LINEAR) /* linear kernel */
    add_weight_vector_to_linear_model(m_model);
  
  if(m_model->kernel_parm.kernel_type == SVMLight_ns::HISTOGRAM_INTERSECT)
	precompute_fast_histogram_intersection_kernel_tables(m_model, &kernel_parm);
}

float SVMlight::predict(std::vector<float> &vals, bool probabilities)
{
  //printf("vals: %d %d %d\n", vals[0], vals[100], vals[1234]);
  //printf("svm_predict ...\n");
  DOC *testdoc;   /* test example */
  WORD *testwords;
  long max_words_doc;
  long queryid,slackid;
  
  long pred_format;
  long j;
  unsigned int f=0;
  double t1,runtime=0;
  double dist,costfactor;
    
  verbosity=0;
  //format=1;
  pred_format=1;
  
    if(verbosity>=2) {
        printf("Classifying test examples.."); fflush(stdout);
    }
    
    if(m_model == 0) {
    	printf("No valid model, train or load first.."); fflush(stdout);
    	return 0;
    }

	max_words_doc = vals.size();  
    testwords = (WORD *)my_malloc(sizeof(WORD)*(max_words_doc+1));

    queryid=0;
    slackid=0;
    costfactor=1;
    //parse_document(line,words,&doc_label,&queryid,&slackid,&costfactor,&wnum,
    //                max_words_doc,&comment);
	//--- convert array to words ---//
    if (!m_model->maxNormalized) {
    	for(f=0; f < vals.size(); ++f)
    	{
    		testwords[f].wnum=f+1;
    		testwords[f].weight=vals[f];		
    	}
    } else {
    	for(f=0; f < vals.size(); ++f) {
    	    testwords[f].wnum=f+1;
    	    testwords[f].weight=vals[f] / m_model->maxNormalizer[f];		
    	}
    }
	//printf("\n");
	testwords[ vals.size()].wnum=0;
	//printf("Number of words: %d\n", f);
	
    if(m_model->kernel_parm.kernel_type == 0)    /* linear kernel */
	{
        for(j=0;(testwords[j]).wnum != 0;j++)   /* Check if feature numbers   */
		{
            if((testwords[j]).wnum>m_model->totwords) /* are not larger than in     */
            (testwords[j]).wnum=0;               /* model. Remove feature if   */
        }                                        /* necessary.                 */
        testdoc = create_example(-1,0,0,0.0, create_svector(testwords, vals.size(),"",1.0));
        t1=get_runtime();
        dist=classify_example_linear(m_model,testdoc);
        runtime+=(get_runtime()-t1);
        free_example(testdoc,1);       
    }
    else 
	{                             /* non-linear kernel */
        testdoc = create_example(-1,0,0,0.0, create_svector(testwords, vals.size(),"",1.0));
        t1=get_runtime();
        dist=classify_example(m_model,testdoc);
        runtime+=(get_runtime()-t1);
        free_example(testdoc,1);       
    }
  
  free(testwords);

  if(verbosity>=2) {
    printf("done\n");
    printf("Runtime (without IO) in cpu-seconds: %.2f\n",
	(float)(runtime/100.0));
    
  }
  if (probabilities)
	  return sigmoid_predict(dist, m_model->A, m_model->B);
  else
	  return dist;
}

float SVMlight::predict(float* vals, int featureDim, bool probabilities)
{
  //printf("vals: %d %d %d\n", vals[0], vals[100], vals[1234]);
  //printf("svm_predict ...\n");
  DOC *testdoc;   /* test example */
  WORD *testwords;
  long max_words_doc;
  long queryid,slackid;
  
  long pred_format;
  long j;
  int f=0;
  double t1,runtime=0;
  double dist,costfactor;
    
  verbosity=0;
  //format=1;
  pred_format=1;
  
    if(verbosity>=2) {
        printf("Classifying test examples.."); fflush(stdout);
    }
    
    if(m_model == 0) {
    	printf("No valid model, train or load first.."); fflush(stdout);
    	return 0;
    }

	max_words_doc = featureDim;  
    testwords = (WORD *)my_malloc(sizeof(WORD)*(max_words_doc+1));

    queryid=0;
    slackid=0;
    costfactor=1;
    //parse_document(line,words,&doc_label,&queryid,&slackid,&costfactor,&wnum,
    //                max_words_doc,&comment);
	//--- convert array to words ---//
    if (!m_model->maxNormalized) {
    	for(f=0; f < featureDim; ++f)
    	{
    		testwords[f].wnum=f+1;
    		testwords[f].weight=vals[f];		
    	}
    } else {
    	for(f=0; f < featureDim; ++f) {
    	    testwords[f].wnum= f+1;
    	    testwords[f].weight= vals[f] / m_model->maxNormalizer[f];		
    	}
    }
	//printf("\n");
	testwords[featureDim].wnum=0;
	//printf("Number of words: %d\n", f);
	
    if(m_model->kernel_parm.kernel_type == SVMLight_ns::LINEAR)    /* linear kernel */
	{
        for(j=0;(testwords[j]).wnum != 0;j++)   /* Check if feature numbers   */
		{
            if((testwords[j]).wnum>m_model->totwords) /* are not larger than in     */
            (testwords[j]).wnum=0;               /* model. Remove feature if   */
        }                                        /* necessary.                 */
        testdoc = create_example(-1,0,0,0.0, create_svector(testwords, featureDim, "" ,1.0));
        t1=get_runtime();
        dist=classify_example_linear(m_model,testdoc);
        runtime+=(get_runtime()-t1);
        free_example(testdoc,1);       
    }
    else 
	{                             /* non-linear kernel */
        testdoc = create_example(-1,0,0,0.0, create_svector(testwords, featureDim, "" ,1.0));
        t1=get_runtime();
        dist=classify_example(m_model,testdoc);
        runtime+=(get_runtime()-t1);
        free_example(testdoc,1);       
    }
  
  free(testwords);

  if(verbosity>=2) {
    printf("done\n");
    printf("Runtime (without IO) in cpu-seconds: %.2f\n",
	(float)(runtime/100.0));
    
  }
  if (probabilities)
	  return sigmoid_predict(dist, m_model->A, m_model->B);
  else
	  return dist;
}


void SVMlight::load_model(const char* modelfile, int format)
{  
  //Release old model if existing
  if (m_model) {
  	free_model(m_model, 0);
  	m_model = 0;
  }
  
  if (format) 
  {
    m_model=read_binary_model(modelfile);
  } 
  else 
  {
    m_model=read_model(modelfile);
    if(m_model->kernel_parm.kernel_type == SVMLight_ns::LINEAR) /* linear kernel */
	{
        /* compute weight vector */
		//printf("linear\n");
        add_weight_vector_to_linear_model(m_model);
    }
  }
  
  if (m_model->kernel_parm.kernel_type == SVMLight_ns::HISTOGRAM_INTERSECT) {	  
	  precompute_fast_histogram_intersection_kernel_tables(m_model, &kernel_parm);	  
  }
 
}

void SVMlight::save_model(const char* modelfile, int format)
{  
  if (!m_model->supvec) {
  	printf("Model is invalid! Maybe you only have training samples of one class?\n");
  	return; 
  }
  	
  if (format) 
  {
    write_binary_model(modelfile, m_model);
  } 
  else 
  {
  	write_model(modelfile, m_model);   
  }
 
}

void SVMlight::read_binary_data(std::string fileName, std::vector<std::vector<float> > &data, std::vector<float> &target) {
	DOC **documents = 0;
	double *labels = 0;
	long int totalWords;
	long int totalDocuments;
	
	read_binary_documents(fileName.c_str(), &documents, &labels, &totalWords, &totalDocuments);
	data.clear();
	target.clear();
	
	for(int i = 0; i < totalDocuments; i++) {
		std::vector<float> nf;
		target.push_back(labels[i]);
		for(int j = 0; j < documents[i]->fvec->n_words; j++) {
			nf.push_back(documents[i]->fvec->words[j]);
		}
		data.push_back(nf);
	}
	
	if (documents) {   	
   		for(int i=0; i< totalDocuments; i++) { 
   			if (documents[i]) free_example(documents[i],1);
   			documents[i] = 0;
   		}
   		free(documents);
   		documents = 0;
   	}
   
   if (labels) {
   		free(labels);
   		labels = 0;
   }
			
   return;
}

bool SVMlight::getSupportVectorIDs(std::vector<int>& ids, float classID) {
	if (!m_model->supvec) {
	  	printf("Model is invalid! Maybe you only have training samples of one class or not trained the model yet?\n");
	  	return false; 
	 }
	
	ids.clear();
	
	for (int i = 1; i < m_model->sv_num && m_model->supvec[i] != 0; ++i) {		
		long int& supID = m_model->supvec[i]->docnum;
		if(target[supID] == classID)
			ids.push_back(supID);		
	}
	
	return true;
}

void SVMlight::deactivateSamples(std::vector<int> ids) {
	for(unsigned int i = 0; i < ids.size(); i++) {
		if (ids[i] < current_training_sample)
			docs[ids[i]]->costfactor = 0.0;
	}
}

void SVMlight::activateSamples(std::vector<int> ids) {
	for(unsigned int i = 0; i < ids.size(); i++) {
		if (ids[i] < current_training_sample)
			docs[ids[i]]->costfactor = 1.0;
	}	
}

void SVMlight::trainProbabilities() {	
	if(m_model == 0) {
	  	printf("No valid model, train or load first.."); fflush(stdout);
	   	return ;
	}
	
	 if(current_training_sample == 0) {
	  	printf("No examples to learn from!\n");
	  	return;
	  }  

	std::vector<int> indices(current_training_sample);
	for(int i = 0; i < current_training_sample; i++) {
			indices[i] = i;
	}
		
	//Setup batches
	const int nfolds = 5;
	std::vector<std::vector<int> > batch(nfolds);
	random_shuffle(indices.begin(), indices.end());
	
	for(int i = 0; i < current_training_sample; i++)
		batch[i % nfolds].push_back(indices[i]);
	
	for(unsigned int i = 0; i < batch.size(); i++)
		sort(batch[i].begin(), batch[i].end());
	
	
	double* dec_values = new double[current_training_sample];
	for(int i = 0; i < nfolds; i++) {
		
		//Deactivate batch for training
		deactivateSamples(batch[i]);
		
		//Count positive and negative samples
		int p_count = 0;
		int n_count = 0;
		
		//Get number of positive and negative training samples
		for(int j = 0; j < nfolds; j++) {
			if (j == i)
				continue;
			
			for(unsigned int k = 0; k < batch[j].size(); k++)
				if(target[batch[j][k]] > 0)
					p_count++;
				else
					n_count++;			
		}
		
		//Missing training examples for one class..
		if(p_count==0 && n_count==0) {
			for(unsigned int j = 0; j < batch[i].size();j++)
				dec_values[batch[i][j]] = 0;
		} else if( p_count > 0 && n_count == 0) {
				for(unsigned int j = 0; j < batch[i].size();j++)
					dec_values[batch[i][j]] = 1;							
		} else if(p_count == 0 && n_count > 0) {
				for(unsigned int j = 0; j < batch[i].size();j++)
					dec_values[batch[i][j]] = -1;							
		} else {
			train();
		
			// Get margins for all training samples
			for(unsigned int j = 0; j < batch[i].size();j++) {
				dec_values[batch[i][j]] = classify_example(m_model,docs[batch[i][j]]);
			}
		}		
		
		//Reactive current batch
		activateSamples(batch[i]);
	}
	
	sigmoid_train(current_training_sample, dec_values, target, m_model->A, m_model->B);
	delete[](dec_values);
}

float SVMlight::train_crossValidateC(float minC, float maxC, float stepC, const char* logFile, bool trainProbabilities, int folds, int usedFolds) {	
	
	if(current_training_sample == 0) {
	  	printf("No examples to learn from!\n");
	  	return 0;
	}  
	
	if(usedFolds > folds) {
		printf("Number of used folds (%d) > folds (%d)!\n", folds, usedFolds);
		return 0;
	}
	
	if(minC > maxC) {
		printf("minC (%f) > maxC (%f)!\n", minC, maxC);
		return 0;
	}

	std::vector<int> indices(current_training_sample);
	for(int i = 0; i < current_training_sample; i++) {
			indices[i] = i;
	}
		
	//Setup batches
	const int nfolds = folds;
	std::vector<std::vector<int> > batch(nfolds);
	random_shuffle(indices.begin(), indices.end());
	
	for(int i = 0; i < current_training_sample; i++)
		batch[i % nfolds].push_back(indices[i]);
	
	for(unsigned int i = 0; i < batch.size(); i++)
		sort(batch[i].begin(), batch[i].end());
	
	//Determine number of tests
	//int num = static_cast<int>(ceilf((maxC - minC) / stepC));
	
	std::vector< float > cvError;
	for (float c = minC; c <= maxC; c+= stepC) {
			
		std::vector< float > foldError;		
		for (int i = 0; i < usedFolds; ++i) {
			//Deactivate batch for training
			deactivateSamples(batch[i]);

			//Count positive and negative samples
			int p_count = 0;
			int n_count = 0;

			//Get number of positive and negative training samples
			for (int j = 0; j < nfolds; j++) {
				if (j == i)
					continue;

				for (unsigned int k = 0; k < batch[j].size(); k++)
					if (target[batch[j][k]] > 0)
						p_count++;
					else
						n_count++;
			}
			
			//Not use this batch if one class is missing
			if (p_count == 0 || n_count == 0)
				continue;

			learn_parm.svm_c = pow(2, c);
			train();

			// Get margins for all training samples
			float correct = 0;
			float wrong = 0;
			for (unsigned int j = 0; j < batch[i].size(); j++) {
				double margin = classify_example(m_model, docs[batch[i][j]]);
				if ( (target[batch[i][j]] >= 0 && margin >= 0) || (target[batch[i][j]] < 0 && margin < 0) ) {
					if (margin > 0)
						correct += learn_parm.svm_costratio;
					else
						correct += 1;
				} else {
					if (margin < 0)
						wrong += learn_parm.svm_costratio;
					else
						wrong += 1;					
				}
			}
			foldError.push_back( wrong * 1.0f / (wrong + correct) );

			//Reactive current batch
			activateSamples(batch[i]);
			
			printf("Fold error (c= %f) %d/%d: %.2f%%\n", pow(2, c), i + 1, usedFolds, foldError.back() * 100 );
			
		}
		
		float cError = 0;
		for (unsigned int i = 0; i < foldError.size(); ++i)
			cError += foldError[i];
		
		cvError.push_back(cError / foldError.size());
		
		printf("Cross validation error (c= %f): %.2f%%\n", pow(2, c), cvError.back() * 100);
	}
	
	float minError = 0;
	int index = 0;
	
	if (verbosity >= 1) {
		printf("Crossvalidation summary:\n");
		printf("========================\n");
	}
	
	for(unsigned int i = 0; i < cvError.size(); ++i) {
		if (verbosity >= 1) printf("c = %f (2^%f):\t\%.20f\n", pow(2, minC + i * stepC), minC + i * stepC, cvError[i]);
		if(i == 0 || cvError[i] < minError) {
			minError = cvError[i];
			index = i;
		}
	}
	
	if(logFile) {
		FILE* log = fopen(logFile, "w");
		
		fprintf(log, "Crossvalidation summary:\n");
		fprintf(log, "========================\n");
		
		for(unsigned int i = 0; i < cvError.size(); ++i) {
			fprintf(log, "c = %f (2^%f):\t\%.20f\n", pow(2, minC + i * stepC), minC + i * stepC, cvError[i]);
			
		}
		
		fclose(log);
	}
	
		
	learn_parm.svm_c = pow(2, minC + index * stepC);
	train(trainProbabilities);
	
	printf("Setting C to %f with on X-V error of: %.2f%%...\n", learn_parm.svm_c, minError * 100);
	return minC + index * stepC;		
}

void SVMlight::train_crossValidate_GridSearchC_Sigma(float minC, float maxC, float stepC, float minKernel, float maxKernel, float kernelStep, float& return_c, float& return_sigma, const char* logFile, bool trainProbabilities, int folds, int usedFolds) {	
	if(current_training_sample == 0) {
	  	printf("No examples to learn from!\n");
	  	return;
	}  
	
	if(usedFolds > folds) {
		printf("Number of used folds (%d) > folds (%d)!\n", folds, usedFolds);
		return;
	}
	
	if(minC > maxC) {
		printf("minC (%f) > maxC (%f)!\n", minC, maxC);
		return;
	}

	std::vector<int> indices(current_training_sample);
	for(int i = 0; i < current_training_sample; i++) {
			indices[i] = i;
	}
		
	//Setup batches
	const int nfolds = folds;
	std::vector<std::vector<int> > batch(nfolds);
	random_shuffle(indices.begin(), indices.end());
	
	for(int i = 0; i < current_training_sample; i++)
		batch[i % nfolds].push_back(indices[i]);
	
	for(unsigned int i = 0; i < batch.size(); i++)
		sort(batch[i].begin(), batch[i].end());
	
	//Determine number of tests
	int numSigmas = 0;
	
	std::vector< float > cvError;
	for ( float sigma = minKernel; sigma <= maxKernel; sigma+=kernelStep, ++numSigmas) {		
		kernel_parm.rbf_gamma = pow(2, sigma);
		for (float c = minC; c <= maxC; c+=stepC) {

			std::vector< float> foldError;
			for (int i = 0; i < usedFolds; ++i) {
				//Deactivate batch for training
				deactivateSamples(batch[i]);

				//Count positive and negative samples
				int p_count = 0;
				int n_count = 0;

				//Get number of positive and negative training samples
				for (int j = 0; j < nfolds; j++) {
					if (j == i)
					continue;

					for (unsigned int k = 0; k < batch[j].size(); k++)
					if (target[batch[j][k]]> 0)
					p_count++;
					else
					n_count++;
				}

				//Not use this batch if one class is missing
				if (p_count == 0 || n_count == 0)
				continue;

				learn_parm.svm_c = pow(2, c);
				train();

				// Get margins for all training samples
				float correct = 0;
				float wrong = 0;
				for (unsigned int j = 0; j < batch[i].size(); j++) {
					double margin = classify_example(m_model, docs[batch[i][j]]);
					if ( (target[batch[i][j]] >= 0 && margin >= 0) || (target[batch[i][j]] < 0 && margin < 0) ) {
						if (margin> 0)
						correct += learn_parm.svm_costratio;
						else
						correct += 1;
					} else {
						if (margin < 0)
						wrong += learn_parm.svm_costratio;
						else
						wrong += 1;
					}
				}
				foldError.push_back( wrong * 1.0f / (wrong + correct) );

				//Reactive current batch
				activateSamples(batch[i]);

				printf("Fold error (c= %f, sigma= %f) %d/%d: %.2f%%\n", pow(2,c), pow(2, sigma), i + 1, usedFolds, foldError.back() * 100 );

			}

			float cError = 0;
			for (unsigned int i = 0; i < foldError.size(); ++i)
			cError += foldError[i];

			cvError.push_back(cError / foldError.size());

			printf("Cross validation error (c= %f, sigma= %f): %.2f%%\n", pow(2, c), pow(2, sigma), cvError.back() * 100);
		}
	}
	
	float minError = 0;
	int index = 0;
	
	if(logFile) {
		FILE* log = fopen(logFile, "w");
		
		fprintf(log, "Crossvalidation summary:\n");
		fprintf(log, "========================\n");
		
		for(unsigned int i = 0; i < cvError.size(); ++i) {
			fprintf(log, "c = %f (2^%f), sigma = %f (2^%f):\t\%.20f\n", pow(2, minC + (i % numSigmas) * stepC), minC + (i % numSigmas) * stepC, pow(2, minKernel + (i / numSigmas) * kernelStep), minKernel + (i / numSigmas) * kernelStep, cvError[i]);
			
		}	
		
		fclose(log);
	}
	
	if (verbosity >= 1) {
		printf("Crossvalidation summary:\n");
		printf("========================\n");
	}
	
	for(unsigned int i = 0; i < cvError.size(); ++i) {
		if (verbosity >= 1) printf("c = %f (2^%f), sigma = %f (2^%f):\t\%.20f\n", pow(2, minC + (i % numSigmas) * stepC), minC + (i % numSigmas) * stepC, pow(2, minKernel + (i / numSigmas) * kernelStep), minKernel + (i / numSigmas) * kernelStep, cvError[i]);

		if(i == 0 || cvError[i] < minError) {
			minError = cvError[i];
			index = i;
		}
	}	
	
		
	learn_parm.svm_c = pow(2, minC + (index % numSigmas) * stepC);
	kernel_parm.rbf_gamma = pow(2, minKernel + (index / numSigmas) * kernelStep);
	train(trainProbabilities);
	
	printf("Setting C to %f and kernel parameter to %f with on X-V error of: %.2f%%...\n", learn_parm.svm_c, kernel_parm.rbf_gamma, minError * 100);
	
	return_c = minC + (index % numSigmas) * stepC;
	return_sigma = minKernel + (index / numSigmas) * kernelStep;
	
	return;		
}





// Platt's binary SVM Probablistic Output: an improvement from Lin et al. (copied from libSVM)
void SVMlight::sigmoid_train(
	int l, const double *dec_values, const double *labels, 
	double& A, double& B)
{
	double prior1=0, prior0 = 0;
	int i;

	for (i=0;i<l;i++)
		if (labels[i] > 0) prior1+=1;
		else prior0+=1;
	
	int max_iter=100; 	// Maximal number of iterations
	double min_step=1e-10;	// Minimal step taken in line search
	double sigma=1e-3;	// For numerically strict PD of Hessian
	double eps=1e-5;
	double hiTarget=(prior1+1.0)/(prior1+2.0);
	double loTarget=1/(prior0+2.0);
	double *t=Malloc(double,l);
	double fApB,p,q,h11,h22,h21,g1,g2,det,dA,dB,gd,stepsize;
	double newA,newB,newf,d1,d2;
	int iter; 
	
	// Initial Point and Initial Fun Value
	A=0.0; B=log((prior0+1.0)/(prior1+1.0));
	double fval = 0.0;

	for (i=0;i<l;i++)
	{
		if (labels[i]>0) t[i]=hiTarget;
		else t[i]=loTarget;
		fApB = dec_values[i]*A+B;
		if (fApB>=0)
			fval += t[i]*fApB + log(1+exp(-fApB));
		else
			fval += (t[i] - 1)*fApB +log(1+exp(fApB));
	}
	for (iter=0;iter<max_iter;iter++)
	{
		// Update Gradient and Hessian (use H' = H + sigma I)
		h11=sigma; // numerically ensures strict PD
		h22=sigma;
		h21=0.0;g1=0.0;g2=0.0;
		for (i=0;i<l;i++)
		{
			fApB = dec_values[i]*A+B;
			if (fApB >= 0)
			{
				p=exp(-fApB)/(1.0+exp(-fApB));
				q=1.0/(1.0+exp(-fApB));
			}
			else
			{
				p=1.0/(1.0+exp(fApB));
				q=exp(fApB)/(1.0+exp(fApB));
			}
			d2=p*q;
			h11+=dec_values[i]*dec_values[i]*d2;
			h22+=d2;
			h21+=dec_values[i]*d2;
			d1=t[i]-p;
			g1+=dec_values[i]*d1;
			g2+=d1;
		}

		// Stopping Criteria
		if (fabs(g1)<eps && fabs(g2)<eps)
			break;

		// Finding Newton direction: -inv(H') * g
		det=h11*h22-h21*h21;
		dA=-(h22*g1 - h21 * g2) / det;
		dB=-(-h21*g1+ h11 * g2) / det;
		gd=g1*dA+g2*dB;


		stepsize = 1; 		// Line Search
		while (stepsize >= min_step)
		{
			newA = A + stepsize * dA;
			newB = B + stepsize * dB;

			// New function value
			newf = 0.0;
			for (i=0;i<l;i++)
			{
				fApB = dec_values[i]*newA+newB;
				if (fApB >= 0)
					newf += t[i]*fApB + log(1+exp(-fApB));
				else
					newf += (t[i] - 1)*fApB +log(1+exp(fApB));
			}
			// Check sufficient decrease
			if (newf<fval+0.0001*stepsize*gd)
			{
				A=newA;B=newB;fval=newf;
				break;
			}
			else
				stepsize = stepsize / 2.0;
		}

		if (stepsize < min_step)
		{
			std::cout << "Line search fails in two-class probability estimates\n";
			break;
		}
	}

	if (iter>=max_iter)
		std::cout << "Reaching maximal iterations in two-class probability estimates\n";
	free(t);
}

double SVMlight::sigmoid_predict(double decision_value, double A, double B)
{
	double fApB = decision_value*A+B;
	if (fApB >= 0)
		return exp(-fApB)/(1.0+exp(-fApB));
	else
		return 1.0/(1+exp(fApB)) ;
}


float SVMlight::trainWithLeaveOneOutXValidationJ(float sampleRatio, const char* path, const char* prefix, bool probabilities) {
	printf("Performing leave one out cross-validation on costfactor...\n");
	learn_parm.compute_loo = 1;
	
	float bestC = 0;
	float minLOO = 0;
	float stepSize = sampleRatio * 1.5f / 30;
	bool first = true; 
	
	for(float c = stepSize; c <= stepSize * 30; c += stepSize) {
		learn_parm.svm_costratio = c;
		train();
		if (path && prefix) {
			char fn[3000];
			sprintf(fn, "%s/%s-j=%f", path, prefix,c);
			save_model(fn, 1); //1 = binary
		}
		if (first || m_model->loo_error < minLOO) {
			minLOO = m_model->loo_error;
			bestC = c;
			first = false;
		} else {
			break;
		} 
		printf("Leave one out error for c=%f: %f\n", c, m_model->loo_error);		
	}	
	
	printf("First level done! Found optimal c between %f and %f...\n", bestC, bestC + stepSize);
	
	first = true;
	float L1BestC = bestC;
	for(float c = L1BestC; c <= L1BestC + stepSize; c += stepSize * 1.0f / 20) {
		learn_parm.svm_costratio = c;
		train();
		if (path && prefix) {
			char fn[3000];
			sprintf(fn, "%s/%s-j=%f", path, prefix,c);
			save_model(fn, 1); // 1 = binary
		}
		if (first || m_model->loo_error < minLOO) {
			minLOO = m_model->loo_error;
			bestC = c;
			first = false;
		} else {
			break;
		} 
		printf("Leave one out error for c=%f: %f\n", c, m_model->loo_error);		
	}	
	
	printf("Second level done! Found optimal c between %f and %f...\n", bestC, bestC + stepSize * 1.0f / 20);
	
	learn_parm.compute_loo = 0;
	learn_parm.svm_costratio = bestC;
	train(probabilities);
	return bestC;	
}

float SVMlight::trainWithLeaveOneOutXValidationC(float sampleRatio, const char* path, const char* prefix, bool probabilities, float min_slack_C, float max_slack_C) {
	printf("Performing leave one out cross-validation on slack...\n");
	learn_parm.svm_costratio = 1.0f * sampleRatio;
	learn_parm.compute_loo = 1;
	
	
	float stepSize = 10;
	float gradientThres = 1.2;
	
	std::vector<float> error;
	std::vector<float> cs;
	for(float c = min_slack_C; c <= max_slack_C; c *= stepSize) {
		printf("Performing Leave one out crossvalidation c=%f\n", c);
		learn_parm.svm_c = c;
		train();
		
		if (path && prefix) {
			char fn[3000];
			sprintf(fn, "%s/%s-c=%f", path, prefix,c);
			save_model(fn, 1); // 1 = binary
		}		
		
		error.push_back(m_model->loo_error);
		cs.push_back(c);
		 
		printf("Leave one out error for c=%f: %f\n", c, m_model->loo_error);		
	}	
	
	float bestC = min_slack_C;
	for(int i = error.size() -2; i >= 0; i--)
		if ( error[i] / error[i+1] > gradientThres) {			
			bestC = cs[i];
			break;			
	}
	
	printf("First level done! Found optimal c between %f and %f...\n", bestC, bestC * stepSize);
	
	error.clear();
	cs.clear();
	for(float c = bestC; c <= bestC*stepSize; c += bestC) {
		printf("Performing Leave one out crossvalidation c=%f\n", c);
		learn_parm.svm_c = c;
		train();
		
		if (path && prefix) {
			char fn[3000];
			sprintf(fn, "%s/%s-c=%f", path, prefix,c);
			save_model(fn, 1); // 1 = binary
		}
		
		error.push_back(m_model->loo_error);
		cs.push_back(c);
		printf("Leave one out error for c=%f: %f\n", c, m_model->loo_error);		
	}	
		
	for(int i = error.size() - 2; i >= 0; i--) {		
		if ( error[i] / error[i+1] > gradientThres) {			
			bestC = cs[i];				
			break;		
		}
	}
	
	printf("Second level done! Found optimal c for c=%f...\n", bestC);
		
	learn_parm.compute_loo = 0;
	learn_parm.svm_c = bestC;
	train(probabilities);		
	return bestC;
}
