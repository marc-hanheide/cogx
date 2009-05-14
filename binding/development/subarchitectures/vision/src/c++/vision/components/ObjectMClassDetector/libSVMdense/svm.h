
#ifndef SVM_CLASSIFY_H
#define SVM_CLASSIFY_H


#include <libSVMdense/svm_common.h>
#include <libSVMdense/svm_learn.h>

#include<vector>


class SVMlight {
	public:
		SVMlight();
		SVMlight(long maxSamples);
		~SVMlight();
		float predict(std::vector<float> &vals, bool probabilities = false);
		float predict(float* vals, int featureDim, bool probabilities = false);
		
		void load_model(const char*, int format);
		bool valid_model(){ return m_model!=0 && m_model->supvec!=0; };
		void set_max_training_samples(long maxSamples);
		void clear_training_samples();
		
		bool add_training_sample(const std::vector<float> &features, float targetValue);
		bool add_training_sample(const float* features, int dimensions, float targetValue);
		
		void train(bool probabilities = false);
		float trainWithLeaveOneOutXValidationJ(float sampleRatio, const char* path = 0, const char* prefix = 0, bool probabilities = false);
		float trainWithLeaveOneOutXValidationC(float sampleRatio, const char* path = 0, const char* prefix = 0, bool probabilities = false, float min_slack_C = 1e-6, float max_slack_C = 1e6);
		float train_crossValidateC(float minC, float maxC, float stepC, const char* logFile, bool trainProbabilities, int folds = 5, int usedFolds = 5) ;
		void train_crossValidate_GridSearchC_Sigma(float minC, float maxC, float stepC, float minKernel, float maxKernel, float kernelStep, float &return_c, float &return_sigma, const char* logFile, bool trainProbabilities, int folds = 5, int usedFolds = 5); 
		
		float proposeC();
		float proposeSigma(float subSet = 0.0);
		void maxNormalize();	
		
		bool getSupportVectorIDs(std::vector<int>& ids, float classID = -1);
		const SVMLight_ns::MODEL* get_model_ptr() const { return this->m_model; }
		
		void save_model(const char* modelfile, int format);
		void set_verbosity(long level) {SVMLight_ns::verbosity = level;};
		static void read_binary_data(std::string fileName, std::vector<std::vector<float> > &data, std::vector<float> &target);
		void save_features(const char* featureFile);
		void loadFeatures(const char* fileName, int& posSamples, int& negSamples);
		void save_features_text(const char* featureFile);			
	private:
		void initVariables();
		double sigmoid_predict(double decision_value, double A, double B);
		void sigmoid_train(int l, const double *dec_values, const double *labels, double& A, double& B);
		void deactivateSamples(std::vector<int> ids);
		void activateSamples(std::vector<int> ids);
		void trainProbabilities();
		void maxUnNormalize();
						
		long max_training_samples;
		long current_training_sample;
		
		unsigned int allocatedWordLength;
		SVMLight_ns::DOC **docs;
		SVMLight_ns::WORD *words;		
		SVMLight_ns::MODEL* m_model; 
	    double *alpha_in;
	    double *target;
  		SVMLight_ns::KERNEL_CACHE *kernel_cache;
  		FVAL* maxNormalizer;
  		bool maxNormalized;
  		
  	public:
  		SVMLight_ns::LEARN_PARM learn_parm;
  		SVMLight_ns::KERNEL_PARM kernel_parm;
};

#endif
