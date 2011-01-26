/** @file data_structs.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */
#ifndef SMLEARNING_DATASTRUCTS_H_
#define SMLEARNING_DATASTRUCTS_H_

#include <Demo/Common/Tools.h>

#include <netcdf.h>
#include <boost/filesystem/operations.hpp>
#include <metalearning/RNN.h>
#include <tools/math_helpers.h>
#include <tools/helpers.h>

using namespace golem;
using namespace std;
using namespace boost::filesystem;

/** \namespace smlearning
    \brief The highest namespace in the hierarchy of this package
*/
namespace smlearning {

/**
   \enum chunk_flags
   \brief flags for storing different Chunk parts in feature vectors
*/
enum chunk_flags {

        _object = 1L << 0, /**< For storing object features. */
        _effector_pos = 1L << 1, /**< For storing effector position. */
	_effector_orient = 1L << 2, /**< For storing effector orientation. */
	_end_effector_pos = 1L << 3, /**< For storing end effector position. */
	_end_effector_orient = 1L << 4, /**< For storing end effector orientation. */
        _action_params = 1L << 5 /**< For storing other motor action features. */
};

inline chunk_flags
operator&(chunk_flags __a, chunk_flags __b)
{ return chunk_flags(static_cast<int>(__a) & static_cast<int>(__b)); }

inline chunk_flags
operator|(chunk_flags __a, chunk_flags __b)
{ return chunk_flags(static_cast<int>(__a) | static_cast<int>(__b)); }

inline chunk_flags
operator^(chunk_flags __a, chunk_flags __b)
{ return chunk_flags(static_cast<int>(__a) ^ static_cast<int>(__b)); }

inline chunk_flags&
operator|=(chunk_flags& __a, chunk_flags __b)
{ return __a = __a | __b; }

inline chunk_flags&
operator&=(chunk_flags& __a, chunk_flags __b)
{ return __a = __a & __b; }

inline chunk_flags&
operator^=(chunk_flags& __a, chunk_flags __b)
{ return __a = __a ^ __b; }

inline chunk_flags
operator~(chunk_flags __a)
{ return chunk_flags(~static_cast<int>(__a)); }

/**
   \enum feature_selection
   \brief flags for choosing the type of feature selection method 
*/
enum feature_selection {
	_basis, /**< First vector stores motor command information */
	_markov, /**< Each vector stores motor command information */
	_obpose, /**< For automata generation: input are action related features, states are object f., output is object next time f. */
	_efobpose /**< For automata generation: input are action related features, states are object and effector f., output is object next time f. */
};


/**
   \class LearningData
   \brief Learning data format and corresponding methods and data conversions
*/
struct LearningData {
	
	/** Data chunk */
	struct Chunk {
		typedef std::vector<Chunk> Seq;
		
		/** Data chunk time stamp */
		golem::SecTmReal timeStamp;

		struct Object {
			/** Object GLOBAL pose */
			golem::Mat34 objectPose;
			/** Object orientation in Euler coordinates */
			golem::Real obRoll, obPitch, obYaw;
			/** feature vector (here any vectorial representation is possible) */
			FeatureVector featureVector;
		};

		struct Action {
			/** Arm state - (joint) dynamic configuration */
			golem::GenConfigspaceState armState;
			/** End-effector GLOBAL pose */
			golem::Mat34 effectorPose;
			/** End-effector orientation in Euler coordinates */
			golem::Real efRoll, efPitch, efYaw; 
			/** horizontal angle */
			golem::Real horizontalAngle;
			/** speed ( 3 (fast), 4 (middle), 5 (low) */
			golem::Real pushDuration;
			/** direction vector (target pose) */
			golem::Mat34 endEffectorPose;
			/** target effector orientation in Euler coordinates */
			golem::Real endEfRoll, endEfPitch, endEfYaw;
			/** feature vector (here any vectorial representation is possible) */
			FeatureVector featureVector;
		};
		/** object related chunk part */
		Object object;
		/** action related chunk part */
		Action action;
		/** feature vector corresponding to the whole chunk (any vectorial representation is possible) */
		FeatureVector featureVector;
		/** a label for this chunk */
		Real label;
			
		
	};

	typedef std::vector<Chunk::Seq> DataSet;

	/** (Dynamic) Effector bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::Action::effectorPose */
	golem::Bounds::Seq effector;
	/** (Dynamic) Object bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::Object::objectPose */
	golem::Bounds::Seq object;
	/** (Static) Obstacles bounds in GLOBAL coordinates (usually ground plane) */
	golem::Bounds::Seq obstacles;
	
	/** Time-dependent data */
	Chunk::Seq currentChunkSeq;
	//DataSet data;
	/** current predicted polyflap poses sequence */
	vector<Mat34> currentPredictedPfSeq;
	/** current predicted effector poses sequence */
	vector<Mat34> currentPredictedEfSeq;
	/** size of motor vector for NN training for basis representation */
	static const int motorVectorSizeBasis = 3;
	/** size of motor vector for NN training for markov representation */
	static const int motorVectorSizeMarkov = 3;	
	/** size of polyflap pose vector for NN training */
	static const int pfVectorSize = 6;
	/** size of effector pose vector for NN training */
	static const int efVectorSize = 6;
	struct CoordinateLimits {
		/** assumed minimum value for polyflap Z-coordinate location during experiment (in xml file should have value of -0.01... bug in PhysX?) */
		Real minZ;
		/** assumed minimum value for polyflap X-coordinate location during experiment */
		Real minX;
		/** assumed minimum value for polyflap Y-coordinate location during experiment */
		Real minY;	
		/** assumed maximum value for polyflap Z-coordinate location during experiment */
		Real maxZ;
		/** assumed maximum value for polyflap X-coordinate location during experiment */
		Real maxX;
		/** assumed maximum value for polyflap Y-coordinate location during experiment */
		Real maxY;
		/** assumed minimum value for duration of trajectory when pushing */
		Real minDuration;
		/** assumed maximum value for duration of trajectory when pushing */
		Real maxDuration;
		
	};
	CoordinateLimits coordLimits;
	
	/** Record validity */
	//bool bArmState;
	//bool bEffectorPose;
	//bool bObjectPose;
	//bool bFtsData;
	//bool bImageIndex;
	//bool bEffector;
	//bool bObject;
	//bool bObstacles;


	
	/** Reset to default (empty)*/
	void setToDefault(CoordinateLimits limits) {
		effector.clear();
		object.clear();
		obstacles.clear();
		coordLimits.minX = limits.minX;
		coordLimits.minY = limits.minY;
		coordLimits.minZ = limits.minZ;
		coordLimits.maxX = limits.maxX;
		coordLimits.maxY = limits.maxY;
		coordLimits.maxZ = limits.maxZ;
		coordLimits.minDuration = limits.minDuration;
		coordLimits.maxDuration = limits.maxDuration;
		//data.clear();
		//bArmState = false;
		//bEffectorPose = false;
		//bObjectPose = false;
		//bFtsData = false;
		//bImageIndex = false;
		//bEffector = false;
		//bObject = false;
		//bObstacles = false;
	}
	/** Check if the data is valid */
	bool isValid() const {
		// if (!data.empty()) // must not be empty
		// 	return false;
		//if (bEffector && effector.empty())
		//	return false;
		//if (bObject && object.empty())
		//	return false;
		//if (bObstacles && obstacles.empty())
		//	return false;

		return true;
	}

	///
	///Write DataSet vector to a file
	///
	static bool write_dataset (string fileName, const DataSet& data, const CoordinateLimits& limits);
	///
	///Read DataSet vector from a file
	///
	static bool read_dataset (string fileName, DataSet& data, CoordinateLimits& limits);

	///
	///print a DataSet vector
	///
	static void print_dataset (const DataSet &d);

	///
	///print coordinate limits for a dataset
	///
	static void print_dataset_limits (const CoordinateLimits limits);

	///
	///print a chunk (features)
	///
	static void print_Chunk (const Chunk& c);

	///
	///check limit parameters correspondence
	///
	static bool check_limits (CoordinateLimits params1, CoordinateLimits params2);

	///
	///Concatenate data sequences
	///
	static bool concatenate_datasets (string dir, string writeFileName);

	///
	///write a netcdf nc file format for feature vectors using basis representation
	///
	template<class Normalization>
	static bool write_nc_file_NNbasis (string fileName, const DataSet& data, Normalization normalize, CoordinateLimits limits) {

		fileName += ".nc";

		FeatureVector inputVector;
		FeatureVector targetVector;
		vector<int> seqLengthsVector;
		size_t numTimesteps_len = 0;
		const int motorVectorSize = motorVectorSizeBasis;
		const int inputSize = motorVectorSize + pfVectorSize + efVectorSize;
		const int outputSize = pfVectorSize;

		DataSet::const_iterator d_iter;

		for (d_iter = data.begin(); d_iter != data.end(); d_iter++) {
			Chunk::Seq seq = *d_iter;

			size_t seqSize = seq.size() - 1;
			seqLengthsVector.push_back ( seqSize );
			numTimesteps_len += seqSize;

			Chunk::Seq::const_iterator s_iter;
			for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
				if (s_iter+1 != seq.end()) {
					if (s_iter == seq.begin()) {
						//motor command
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, /*_action_params*/ _end_effector_pos );
						//completing with feature vector
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, _effector_pos | _effector_orient | _object);
						if (d_iter == data.begin())
							assert (inputSize == inputVector.size());

						/*//zero padding
						for (int i=0; i<motorVectorSize; i++)
							inputVector.push_back (0.0);
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, _effector_pos | _object);*/
					}
					
					else {
						//zero padding
						for (int i=0; i<motorVectorSize; i++)
							inputVector.push_back (0.0);
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, _effector_pos | _effector_orient | _object);
					}
				}
				if (s_iter != seq.begin()) {
					//target vector corresponds to polyflap pose vector
					write_chunk_to_featvector (targetVector, *s_iter, normalize, limits, _object);
					if (s_iter == seq.begin()+1 && d_iter == data.begin())
						assert (targetVector.size() == outputSize);
				}
			
			}
		}
		//netcdf file storing
		if (!write_nc_data (fileName, data.size(), inputSize, outputSize, inputVector, targetVector, seqLengthsVector, numTimesteps_len))
			return false;


		return true;
	}


	///
	///write a netcdf nc file format for feature vectors using basis representation
	///
	template<class Normalization>
	static bool write_nc_file_NNmarkov (string fileName, const DataSet& data, Normalization normalize, CoordinateLimits limits) {
		fileName += ".nc";

		FeatureVector inputVector;
		FeatureVector targetVector;
		vector<int> seqLengthsVector;
		size_t numTimesteps_len = 0;
		const int motorVectorSize = motorVectorSizeMarkov;

		const int inputSize = motorVectorSize + efVectorSize + pfVectorSize;
		const int outputSize = pfVectorSize;

		DataSet::const_iterator d_iter;

		for (d_iter = data.begin(); d_iter != data.end(); d_iter++) {
			Chunk::Seq seq = *d_iter;

			size_t seqSize = seq.size() - 1;
			seqLengthsVector.push_back ( seqSize );
			numTimesteps_len += seqSize;

			Chunk::Seq::const_iterator s_iter;

			for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
				if (s_iter+1 != seq.end()) {
					//input feature vector
					write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, _end_effector_pos | _effector_pos | _effector_orient | _object);
					if (s_iter == seq.begin() && d_iter == data.begin())
						assert (inputVector.size() == inputSize);
				
				}
				if (s_iter != seq.begin()) {
					//target vector corresponds to polyflap pose vector
					write_chunk_to_featvector (targetVector, *s_iter, normalize, limits, _object);
					if (s_iter == seq.begin()+1 && d_iter == data.begin())
						assert (targetVector.size() == outputSize);

				}
			}
		}
		//netcdf file storing
		if (!write_nc_data (fileName, data.size(), inputSize, outputSize, inputVector, targetVector, seqLengthsVector, numTimesteps_len))
			return false;
	
	
		return true;
	}

	
	///
	///write a chunk to feature vector
	///
	template<typename T, class Normalization>
	static void write_chunk_to_featvector (vector<T>& featVector, const Chunk& chunk, Normalization normalize, CoordinateLimits coordLimits, chunk_flags flags = _object | _effector_pos | _effector_orient | _end_effector_pos | _end_effector_orient | _action_params ) {
		try { 
			if ( flags & _action_params ) {
				featVector.push_back (normalize(chunk.action.pushDuration, coordLimits.minDuration, coordLimits.maxDuration));
				featVector.push_back (normalize(chunk.action.horizontalAngle/180.0*REAL_PI, -REAL_PI, REAL_PI));
			}
			if ( flags & _end_effector_pos ) {
				featVector.push_back (normalize(chunk.action.endEffectorPose.p.v1, coordLimits.minX, coordLimits.maxX));
				featVector.push_back (normalize(chunk.action.endEffectorPose.p.v2, coordLimits.minY, coordLimits.maxY));
				featVector.push_back (normalize(chunk.action.endEffectorPose.p.v3, coordLimits.minZ, coordLimits.maxZ));
			}
			if ( flags & _end_effector_orient ) {
				featVector.push_back (normalize(chunk.action.endEfRoll, -REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.action.endEfPitch,-REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.action.endEfYaw, -REAL_PI, REAL_PI));
			}
		
			if ( flags & _effector_pos ) {
			
				featVector.push_back (normalize(chunk.action.effectorPose.p.v1, coordLimits.minX, coordLimits.maxX));
				featVector.push_back (normalize(chunk.action.effectorPose.p.v2, coordLimits.minY, coordLimits.maxY));
				featVector.push_back (normalize(chunk.action.effectorPose.p.v3, coordLimits.minZ, coordLimits.maxZ));
			}
			if ( flags & _effector_orient ) {
				featVector.push_back (normalize(chunk.action.efRoll, -REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.action.efPitch,-REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.action.efYaw, -REAL_PI, REAL_PI));
			}
			if ( flags & _object ) {
				featVector.push_back (normalize(chunk.object.objectPose.p.v1, coordLimits.minX, coordLimits.maxX));
				featVector.push_back (normalize(chunk.object.objectPose.p.v2, coordLimits.minY, coordLimits.maxY));
				featVector.push_back (normalize(chunk.object.objectPose.p.v3, coordLimits.minZ, coordLimits.maxZ));
				featVector.push_back (normalize(chunk.object.obRoll, -REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.object.obPitch, -REAL_PI, REAL_PI));
				featVector.push_back (normalize(chunk.object.obYaw, -REAL_PI, REAL_PI));
			}
		} catch ( ... ) {
			cerr << "Error normalizing effector feature vector " << endl;
		};
	}

	///
	///write a chunk to feature vector with memory assigned
	///
	template<typename T, class Normalization>
	static void write_chunk_to_featvector (vector<T>& featVector, const Chunk& chunk, int& vectorIndex, Normalization normalize, CoordinateLimits coordLimits, chunk_flags flags = _object | _effector_pos | _effector_orient | _end_effector_pos | _end_effector_orient | _action_params ) {
		try { 
			if ( flags & _action_params ) {
				featVector[vectorIndex++] = normalize(chunk.action.pushDuration, coordLimits.minDuration, coordLimits.maxDuration);
				featVector[vectorIndex++] = normalize(chunk.action.horizontalAngle/180.0*REAL_PI, -REAL_PI, REAL_PI);
			}
			if ( flags & _end_effector_pos ) {
				featVector[vectorIndex++] = normalize(chunk.action.endEffectorPose.p.v1, coordLimits.minX, coordLimits.maxX);
				featVector[vectorIndex++] = normalize(chunk.action.endEffectorPose.p.v2, coordLimits.minY, coordLimits.maxY);
				featVector[vectorIndex++] = normalize(chunk.action.endEffectorPose.p.v3, coordLimits.minZ, coordLimits.maxZ);
			}
			if ( flags & _end_effector_orient ) {
				featVector[vectorIndex++] = normalize(chunk.action.endEfRoll, -REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.action.endEfPitch,-REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.action.endEfYaw, -REAL_PI, REAL_PI);
			}
			if ( flags & _effector_pos ) {

				featVector[vectorIndex++] = normalize(chunk.action.effectorPose.p.v1, coordLimits.minX, coordLimits.maxX);
				featVector[vectorIndex++] = normalize(chunk.action.effectorPose.p.v2, coordLimits.minY, coordLimits.maxY);
				featVector[vectorIndex++] = normalize(chunk.action.effectorPose.p.v3, coordLimits.minZ, coordLimits.maxZ);
			}
			if ( flags & _effector_orient ) {
				featVector[vectorIndex++] = normalize(chunk.action.efRoll, -REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.action.efPitch, -REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.action.efYaw, -REAL_PI, REAL_PI);
			}
			if ( flags & _object ) {
				featVector[vectorIndex++] = normalize(chunk.object.objectPose.p.v1, coordLimits.minX, coordLimits.maxX);
				featVector[vectorIndex++] = normalize(chunk.object.objectPose.p.v2, coordLimits.minY, coordLimits.maxY);
				featVector[vectorIndex++] = normalize(chunk.object.objectPose.p.v3, coordLimits.minZ, coordLimits.maxZ);
				featVector[vectorIndex++] = normalize(chunk.object.obRoll, -REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.object.obPitch, -REAL_PI, REAL_PI);
				featVector[vectorIndex++] = normalize(chunk.object.obYaw, -REAL_PI, REAL_PI);
			}
		} catch ( ... ) {
			cerr << "Error normalizing effector feature vector " << endl;
		};			

	}



	///
	///almost automatically generated netcdf function to store netcdf data files
	///
	static bool write_nc_data (string fileName, size_t numSeqs_len, int inputVectorSize, int targetVectorSize, FeatureVector& inputVector, FeatureVector& targetVector, vector<int>& seqLengthsVector, size_t& numTimesteps_len);


	///
	///generation of n-fold cross-validation sets from a particular sequence file
	///and using a certain write_netcdf_file function
	///
	template<class Function, class Normalization>
	static bool write_n_fold_cross_valid_sets (string seqFileName, int n, Function write_netcdf_file, Normalization normalize, CoordinateLimits limits, string target_dir, bool print_data = false) {
		
		DataSet data;
		if (n < 2) {
			cout << "You have to use at least 2 cross-validation sets" << endl;
			return false;
		}
		if (!read_dataset (seqFileName, data, limits)){
			cout << "file " + seqFileName + " could not be read" << endl;
			return false;
		}

		boost::regex seqfile_re (".*/(.*)$");
		boost::cmatch match;
		string seqBaseFileName;
		if (boost::regex_match(seqFileName.c_str(), match, seqfile_re)) {
			seqBaseFileName = string(match[1].first, match[1].second);
		}
		else
			seqBaseFileName = seqFileName;
		cout << "seqFileBaseName: " << seqBaseFileName << endl;
		cout << "seqFile: " << seqFileName << endl;

	
		vector<DataSet> partitions_testing;
		vector<DataSet> partitions_training;
	
		// initialize random seed:
		srand ((unsigned)time(NULL) );
	
		// generate random number:
		int randNr;

		long int partitionSize = data.size() / n;
		vector<bool> availablePartitions;
	
		for (int i=0; i<n; i++) {
			DataSet partition_testing;
			DataSet partition_training;
			partitions_testing.push_back (partition_testing);
			partitions_training.push_back (partition_training);
			availablePartitions.push_back (true);		
		}

		DataSet::const_iterator s;
		for (s=data.begin(); s!=data.end(); s++) {
			bool available_partitions = false;
			//check available partitions
			for (int i=0; i<n; i++)
				if (availablePartitions[i] == true) {
					available_partitions = true;
					break;
				}

			if (available_partitions) {
				bool available_partition_found = false;
				do {
					randNr = rand() % n;
					if (availablePartitions[randNr]) {
						partitions_testing[randNr].push_back(*s);
						available_partition_found = true;
						if (partitions_testing[randNr].size() == partitionSize)
							availablePartitions[randNr] = false;
					}				
				} while (!available_partition_found);
			}
			else
				break;
		}

		//store partitions in cdl files
		for (int i=0; i< n; i++) {
			cout << "size of testing partition " << i << ": " << partitions_testing[i].size() << endl;
			if (print_data)
				print_dataset (partitions_testing[i]);
			stringstream testingFileName;
			testingFileName << target_dir << "/" << seqBaseFileName << "_" << n << "_foldcv_set-" << i << "_testing";
			write_netcdf_file (testingFileName.str(), partitions_testing[i], normalize, limits);
			for (int j=0; j<n; j++)
				if (i != j) {
					DataSet::const_iterator s;
					for (s=partitions_testing[j].begin(); s!=partitions_testing[j].end(); s++)
						partitions_training[i].push_back (*s);
				}
			cout << "size of training partition " << i << ": " << partitions_training[i].size() << endl;
			if (print_data)
				print_dataset(partitions_training[i]);
			stringstream trainingFileName;
			trainingFileName << target_dir << "/" << seqBaseFileName << "_" << n << "_foldcv_set-" << i << "_training";
			write_netcdf_file (trainingFileName.str(), partitions_training[i], normalize, limits);
		
		}

		return true;
	
	}



	///
	///load a sequence into inputs and target vectors (for NN training) (NN basis representation)
	///
	template<class Normalization>
	static void load_NNsequence_basis (vector<float>& inputVector, vector<float>& targetVector, const Chunk::Seq seq, Normalization normalize, CoordinateLimits limits) {
		int contInput = 0;
		int contTarget = 0;
		const int motorVectorSize = motorVectorSizeBasis;

		Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter!= seq.end(); s_iter++) {
			//put inputs and targetPatterns data
			if (s_iter+1 != seq.end()) {
				if (s_iter == seq.begin () ) {
					//motor command
					write_chunk_to_featvector (inputVector, *s_iter, contInput, normalize, limits,  /*_action_params*/ _end_effector_pos );
					//completing with feature vector
					write_chunk_to_featvector (inputVector, *s_iter, contInput, normalize, limits, _effector_pos | _effector_orient | _object);

					//zero padding
					/*for (int i=0; i<motorVectorSize; i++)
						inputVector[contInput++] = 0.0;
					write_chunk_to_featvector (inputVector, *s_iter, contInput, normalize, limits, _effector_pos | _effector_orient | _object);*/
			
				}
				else {
					//zero padding
					for (int i=0; i<motorVectorSize; i++)
						inputVector[contInput++] = 0.0;
					write_chunk_to_featvector (inputVector, *s_iter, contInput, normalize, limits, _effector_pos | _effector_orient | _object);
				
				}	
			}
			if (s_iter != seq.begin()) {
				write_chunk_to_featvector (targetVector, *s_iter, contTarget, normalize, limits, _object);
			}
		}
	}

	///
	///load a sequence into inputs and target vectors (for NN training) (NN basis representation)
	///
	template<class Normalization>
	static void load_NNsequence_markov (vector<float>& inputVector, vector<float>& targetVector, const Chunk::Seq seq, Normalization normalize, CoordinateLimits limits) {
		int contInput = 0;
		int contTarget = 0;

		Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter!= seq.end(); s_iter++) {
			if (s_iter+1 != seq.end()) {
				//input feature vector
				write_chunk_to_featvector (inputVector, *s_iter, contInput, normalize, limits, _end_effector_pos | _effector_pos | _effector_orient | _object );
				
			}
			if (s_iter != seq.begin() ) {
				//target vector corresponds to polyflap pose vector
				write_chunk_to_featvector (targetVector, *s_iter, contTarget, normalize, limits, _object);
			}
			
		}
	}

	///
	///load training data in RNNLIB format
	///
	template<class Normalization >
	static rnnlib::DataSequence* load_NNtrainSeq ( Chunk::Seq& seq, unsigned int featureSelectionMethod, Normalization normalize, CoordinateLimits limits) {
		int motorVectorSize;
		if (featureSelectionMethod == _basis)
			motorVectorSize = motorVectorSizeBasis;
		else if (featureSelectionMethod == _markov)
			motorVectorSize = motorVectorSizeMarkov;
		rnnlib::DataSequence* trainSeq = new rnnlib::DataSequence (motorVectorSize+pfVectorSize+efVectorSize, pfVectorSize);
		vector<int> inputShape, targetShape;

		//TODO: correct here sequence size:
		if (featureSelectionMethod == _basis) {
			inputShape.push_back (seq.size() - 1);
			targetShape.push_back (seq.size() - 1);
		}
		else if (featureSelectionMethod == _markov) {
			inputShape.push_back (seq.size() - 1);
			targetShape.push_back (seq.size() - 1);
		}
		trainSeq->inputs.reshape(inputShape);
		trainSeq->targetPatterns.reshape(targetShape);
		// cout << "input size: " << trainSeq->inputs.data.size() << endl;
		// cout << "output size: " << trainSeq->targetPatterns.data.size() << endl;
		if (featureSelectionMethod == _basis)
			load_NNsequence_basis (trainSeq->inputs.data, trainSeq->targetPatterns.data, seq, normalize, limits);
		else if (featureSelectionMethod == _markov)
			load_NNsequence_markov (trainSeq->inputs.data, trainSeq->targetPatterns.data, seq, normalize, limits);
		return trainSeq;
	
	
	}


	///
	///Writing a dataset in CrySSMEx format. It works as a regression method.
	///All data are vectorial.
	///\param featureSelectionMethod allows different feature selection methods and state spaces.
	template<class Normalization>
	static void write_cryssmexdataset_regression (string writeFileName, DataSet& data, Normalization normalize, CoordinateLimits limits, int featureSelectionMethod) {
		writeFileName += ".cry";

		assert (data.size() >= 1);
		int inputVectorSize, stateVectorSize, outputVectorSize;

		if (featureSelectionMethod == _obpose) {
			inputVectorSize = motorVectorSizeMarkov + efVectorSize;
			stateVectorSize = pfVectorSize;
			outputVectorSize = pfVectorSize;
		}
		else if (featureSelectionMethod == _efobpose) {
			inputVectorSize = motorVectorSizeMarkov /*+ efVectorSize*/;
			stateVectorSize = efVectorSize + pfVectorSize;
			outputVectorSize = pfVectorSize;
		}
		ofstream writeFile(writeFileName.c_str(), ios::out);

		writeFile << "# input dim" << endl;
		writeFile << inputVectorSize << endl;
		writeFile << "# state dim" << endl;
		writeFile << stateVectorSize << endl;
		writeFile << "# output dim" << endl;
		writeFile << outputVectorSize << endl;
		
		writeFile << "# nr input symbols" << endl << "0.0" << endl;
		writeFile << "# output examples" << endl << "0.0" << endl;
		// writeFile.precision(20);

		DataSet::const_iterator d_iter;
		for (d_iter = data.begin(); d_iter != data.end(); d_iter++) {
			Chunk::Seq seq = *d_iter;

			Chunk::Seq::const_iterator s_iter;

			for (s_iter = seq.begin(); s_iter != seq.end(); s_iter++) {
				if (s_iter+1 != seq.end()) {

					FeatureVector inputVector, stateVector, outputVector;

					if (featureSelectionMethod == _obpose) {
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, /*_action_params |*/ _end_effector_pos | _effector_pos | _effector_orient );
						write_chunk_to_featvector (stateVector, *s_iter, normalize, limits, _object);
						write_chunk_to_featvector (outputVector, *(s_iter+1), normalize, limits, _object);

					}
					else if (featureSelectionMethod == _efobpose) {
						write_chunk_to_featvector (inputVector, *s_iter, normalize, limits, /*_action_params |*/ _end_effector_pos );
						write_chunk_to_featvector (stateVector, *s_iter, normalize, limits, _effector_pos | _effector_orient | _object);
						write_chunk_to_featvector (outputVector, *(s_iter+1), normalize, limits, _object);
					}
					if (s_iter == seq.begin() ) {
						assert (inputVector.size() == inputVectorSize);
						assert (stateVector.size() == stateVectorSize);
						assert (outputVector.size() == outputVectorSize);
					}
					write_vector (writeFile, inputVector, _text);
					write_vector (writeFile, stateVector, _text);
					write_vector (writeFile, outputVector, _text);
					writeFile << endl;
				}
			}
		}
		writeFile.close();
	
	}

	

};

}; /* smlearning namespace */

#endif /* SMLEARNING_DATASTRUCTS_H_*/
