#include <libPPProcess/preprocess.h>
#include <libSVMdense/svm.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "sw_svm.h"
#include "parameters.h"

using namespace std;

SW_Train::SW_Train(const SW_Params& sw_params, const LearningParams& learning_params, const preAndPostProcessParams& pp_params) :	
	SW_Test(sw_params, learning_params, pp_params) {
	
}

IplImage* SW_Train::getCrop(IplImage* img, const AnnoRect& anno) {
	fprintf(stdout, "  pos: %dx%d\n", anno.left(), anno.top());					
	fprintf(stdout, "  crop wxh: %dx%d\n", anno.w(), anno.h());
				
	// Add two pixels to the window in order to compute gradients properly (Annotations are already extended in main.cpp)
	IplImage* crop = cvCreateImage(cvSize( m_swParams.windowWidth + 2, m_swParams.windowHeight + 2), img->depth, img->nChannels);
	IplImage* inputImg = img;
				
	IplImage* scaled = 0;
	
	AnnoRect cropAnno(anno);
	
	//Resize image
	//std::cout << cropAnno.w() << "       " << cropAnno.h() << std::endl;
	
	if (cropAnno.w() != m_swParams.windowWidth + 2 || cropAnno.h() != m_swParams.windowHeight + 2) {
		const int scaledWidth = static_cast<int>(round(img->width * 1.0f / cropAnno.scale()));
		const int scaledHeight = static_cast<int>(round(img->height * 1.0f / cropAnno.scale()));
		
		scaled = cvCreateImage(cvSize(scaledWidth, scaledHeight), img->depth, img->nChannels);
		
		//IplImage* pCopy = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
		//cvSmooth(img, pCopy, CV_GAUSSIAN ,((int)floorf(cropAnno.scale() * 3)) / 2 * 2 + 1, ((int)floorf(cropAnno.scale() * 3)) / 2  * 2 + 1 );
		//cvSmooth(img, pCopy, CV_GAUSSIAN , 0, 0, cropAnno.scale(), cropAnno.scale());
    	cvResize(img, scaled, CV_INTER_LINEAR);
		//cvReleaseImage(&pCopy);	
					
		int newX1 = static_cast<int>(round(cropAnno.left() *  1.0f / cropAnno.scale()));
		int newY1 = static_cast<int>(round(cropAnno.top() *  1.0f / cropAnno.scale()));
		int newX2 = newX1 + m_swParams.windowWidth + 2;
		int newY2 = newY1 + m_swParams.windowHeight + 2;
				
		cropAnno.setCoords(newX1, newY1, newX2, newY2);
		cropAnno.setScale(1.0);
					
		inputImg = scaled;
	}
				
	const int width = inputImg->width;
	const int height = inputImg->height;
						
	if(cropAnno.right() >= width || cropAnno.bottom() >= height || cropAnno.left() < 0 || cropAnno.top() < 0) {	 	
	 	const int leftmost = cropAnno.left() >= 0 ? 0 : -cropAnno.left();
	 	const int rightmost = cropAnno.right() >= width ? width - cropAnno.left() - 1 : cropAnno.w() - 1;
	 	const int topmost = cropAnno.top() >= 0 ? 0 : -cropAnno.top(); 
	 	const int bottommost = cropAnno.bottom() >= height ? height - cropAnno.top()  - 1 : cropAnno.h() - 1;
				 	
	 	const int imgL = cropAnno.left() < 0 ? 0 : cropAnno.left() ;
	 	const int imgT = cropAnno.top() < 0 ? 0 : cropAnno.top() ;
				 	
	 	cvSetImageROI(inputImg, cvRect(imgL , imgT , rightmost - leftmost + 1, bottommost - topmost + 1));
	 	cvSetImageROI(crop, cvRect(leftmost, topmost, rightmost - leftmost + 1, bottommost - topmost + 1));
	 	cvCopy(inputImg, crop);
	 	cvResetImageROI(inputImg);
	 	cvResetImageROI(crop);
			 				 				
	 	blurEdges( (unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep, leftmost, topmost, rightmost, bottommost, m_preProcessParams.blurKernelWidth);			 	
	} else {	
		cvSetImageROI(inputImg, cvRect(cropAnno.left() , cropAnno.top() , cropAnno.w(), cropAnno.h()));
		cvCopy(inputImg, crop);
		cvResetImageROI(inputImg);	
	}
				
	if (scaled)
		cvReleaseImage(&scaled);
				
	return crop;
}


void SW_Train::extractSamples(const AnnotationList& posFileList, const AnnotationList& negFileList) {
	const bool saveCrops= false;	
	
	int maxPosSamples = annoCount(posFileList);
	if (m_preProcessParams.mirror)
		maxPosSamples *= 2;
	
	int maxNegSamples = annoCount(negFileList) + m_LearningParams.maxRetrainingSamples;
	
	posSamples = 0;
	negSamples = 0;
	
	//--- Setup SVM light data structures ---//
	if (m_swParams.classifier == SVM)
		m_svm.set_max_training_samples( maxPosSamples + maxNegSamples );
	
	//--- Setup Boosting data structures ---//
	if (m_swParams.classifier == AdaBoost) {
		m_boostingData.clear();
		m_boostingData.reserve (maxPosSamples + maxNegSamples);
	}
		
	// Reserve memory for bounding boxes
	m_NegRects.clear();
	m_NegRectFileIndex.clear();
	
	m_NegRects.reserve( maxNegSamples );
	m_NegRectFileIndex.reserve( maxNegSamples );

	m_PosRects.clear();
	m_PosRectFileIndex.clear();
		
	m_PosRects.reserve(maxPosSamples);
	m_PosRectFileIndex.reserve(maxPosSamples);

	float* fv = new float[m_featureDim];
	bool invalidWindow;
	
	//--- compute features on positive samples ---//
	for(unsigned int i = 0; i < posFileList.size(); ++i) {
				
		//--- load image and crop center window --//
		fprintf(stdout, "Processing image: \"%s\"\n", posFileList[i].imageName().c_str());
		IplImage* img = cvLoadImage(posFileList[i].imageName().c_str());
		
		if (img == 0 || img->nChannels != 3)
		{
			fprintf(stderr, "ERROR: Failed loading image: %s\n", posFileList[i].imageName().c_str());
			continue;
		}
		
		 if (m_preProcessParams.sharpenPercentTrain !=0 ) {
			 sharpen(reinterpret_cast<unsigned char*>(img->imageData), img->width, img->height, img->widthStep, 3, m_preProcessParams.sharpenPercentTrain);
		 }
		
		fprintf(stdout, "  wxh: %dx%d\n", img->width, img->height);
		
		const Annotation& currentAnnotation = posFileList[i];
		for(unsigned int j = 0; j < currentAnnotation.size(); ++j) {			
			
			IplImage* crop = getCrop(img, currentAnnotation[j]);
			
			if (saveCrops) {
				string::size_type pos = posFileList[i].imageName().rfind("/");
				string posSaveName = string("/tmp/poscrop-")+"-"+QString::number(j).toStdString() + posFileList[i].imageName().substr(pos+1,string::npos);
				fprintf(stderr, "saving image: %s\n", posSaveName.c_str());
				cvSaveImage(posSaveName.c_str(), crop);
			}
					
			float* nextfv = fv;
			invalidWindow = false;
			for(unsigned int v = 0; v < features.size(); ++v) {
				features[v]->on_new_Image((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep);
				features[v]->on_new_Scale((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep, 1.0, 0, 0);
				nextfv = features[v]->on_get_FeatureVector(0, 0, m_swParams.windowWidth + 2, m_swParams.windowHeight + 2, nextfv);								
				if (nextfv == 0) {
					invalidWindow = true;
					break;
				}
			}
			if (invalidWindow) {
				cvReleaseImage(&crop);
				continue;
			}
						
			if (m_swParams.normalizeCues > 0)
				normalizeCues(fv);
			
			// Add to SVM structure
			if (m_swParams.classifier == SVM) {
				if (m_svm.add_training_sample(fv, m_featureDim, 1.0))
					posSamples++;
			}

			// ... or to Boosting structure
			if (m_swParams.classifier == AdaBoost) {
				m_boostingData.push_back(FeatureVector(fv, m_featureDim, 1));
				posSamples++;
			}
			
			m_PosRects.push_back(currentAnnotation[j]);
			m_PosRectFileIndex.push_back(i);
			
			// Extract features from mirrored image
			if (m_preProcessParams.mirror) {
				cvFlip(crop, NULL, 1);
				
				float* nextfv = fv;
				invalidWindow = false;
				for(unsigned int v = 0; v < features.size(); ++v) {
					features[v]->on_new_Image((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep);
					features[v]->on_new_Scale((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep, 1.0, 0, 0);
					nextfv = features[v]->on_get_FeatureVector(0, 0, m_swParams.windowWidth + 2, m_swParams.windowHeight + 2, nextfv);								
					if (nextfv == 0) {
						invalidWindow = true;
						break;
					}
				}			
				
				if (invalidWindow) {
					cvReleaseImage(&crop);
					continue;		
				}
				
				if (m_swParams.normalizeCues > 0)							
					normalizeCues(fv);
				
				// Add to SVM structure
				if (m_swParams.classifier == SVM) {
					if (m_svm.add_training_sample(fv, m_featureDim, 1.0))
						posSamples++;
				}

				// ... or to Boosting structure
				if (m_swParams.classifier == AdaBoost) {
					m_boostingData.push_back(FeatureVector(fv, m_featureDim, 1));
					posSamples++;
				}
					
				m_PosRects.push_back(currentAnnotation[j]);
				m_PosRectFileIndex.push_back(i);				
			}	
			cvReleaseImage(&crop);
		}
		cvReleaseImage(&img);
							
	}
	
	//--- compute features on negative samples ---//
	for(unsigned int i = 0; i < negFileList.size(); ++i) {
				
		//--- load image and crop center window --//
		fprintf(stdout, "Processing image: \"%s\"\n", negFileList[i].imageName().c_str());
		IplImage* img = cvLoadImage(negFileList[i].imageName().c_str());
		
		if (img == 0 || img->nChannels != 3)
		{
			fprintf(stderr, "ERROR: Failed loading image: %s\n", negFileList[i].imageName().c_str());
			continue;
		}
				
		if (m_preProcessParams.sharpenPercentTrain !=0 ) {
			 sharpen(reinterpret_cast<unsigned char*>(img->imageData), img->width, img->height, img->widthStep, 3, m_preProcessParams.sharpenPercentTrain);
		}
				
		fprintf(stdout, "  wxh: %dx%d\n", img->width, img->height);
		
		const Annotation& currentAnnotation = negFileList[i];
		
		for(unsigned int j = 0; j < currentAnnotation.size(); ++j) {			
			
			IplImage* crop = getCrop(img, currentAnnotation[j]);
			
			if (saveCrops) {
				string::size_type pos = negFileList[i].imageName().rfind("/");
				string negSaveName = string("/tmp/negcrop-")+"-"+QString::number(j).toStdString() + negFileList[i].imageName().substr(pos+1,string::npos);
				fprintf(stderr, "saving image: %s\n", negSaveName.c_str());
				cvSaveImage(negSaveName.c_str(), crop);
			}	
			
			float* nextfv = fv;
			invalidWindow = false;
			for(unsigned int v = 0; v < features.size(); ++v) {
				features[v]->on_new_Image((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep);
				features[v]->on_new_Scale((unsigned char*)crop->imageData, crop->width, crop->height, crop->widthStep, 1.0, 0, 0);
				nextfv = features[v]->on_get_FeatureVector(0, 0, m_swParams.windowWidth + 2, m_swParams.windowHeight + 2, nextfv);
				if (nextfv == 0) {
					invalidWindow = true;
					break;
				}
			}
			
			cvReleaseImage(&crop);
			
			if (invalidWindow)
				continue;
			
			if (m_swParams.normalizeCues > 0)						
				normalizeCues(fv);
			
			// Add to SVM structure
			if (m_swParams.classifier == SVM) {
				if (m_svm.add_training_sample(fv, m_featureDim, -1.0))
					negSamples++;
			}

			// ... or to Boosting structure
			if (m_swParams.classifier == AdaBoost) {
				m_boostingData.push_back(FeatureVector(fv, m_featureDim, 0));
				negSamples++;
			}
			
			m_NegRects.push_back(currentAnnotation[j]);
			m_NegRectFileIndex.push_back(i);				
			
		}
		cvReleaseImage(&img);							
	}
	
	delete[] fv;
}

/*void SW_Train::loadFeatures(const char* fileName) {
	posSamples = 0;
	negSamples = 0;
	
	if (m_swParams.classifier == SVM) {
		m_svm.loadFeatures(fileName, posSamples, negSamples);
		
		m_svm.learn_parm.svm_costratio =  1.0f * negSamples / posSamples;
	}
}*/


void SW_Train::loadFeatures(const char* fileName) {
	posSamples = 0;
	negSamples = 0;
	
	m_svm.clear_training_samples();
	m_boostingData.clear();
		
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
	
	if (m_swParams.classifier == SVM)
		m_svm.set_max_training_samples( numSamples );
	else
		m_boostingData.reserve(numSamples);
	
	assert(static_cast<int>(featureDim) == m_featureDim);
	
	for(int i = 0; i < numSamples; ++i) {
		binaryLoad.read(reinterpret_cast<char*>(&target), sizeof(target));
		
		for(unsigned int j = 0; j < featureDim; ++j) {			
			binaryLoad.read(reinterpret_cast<char*>(&fv[j]), sizeof(fv[j]));			
		}		
		
		if (m_swParams.classifier == SVM) {			
			m_svm.add_training_sample(fv, target);
		}
		
		if (m_swParams.classifier == AdaBoost) {
			int btarget = 0;
			if (target == 1.0)
				btarget = 1;
			
			m_boostingData.push_back(FeatureVector(fv, btarget));
		}
		
		if (target == 1.0)
			posSamples++;
		else
			negSamples++;		
	}
		
	std::cout << "Loaded " << posSamples << " positive samples and " << negSamples << " negative Samples..." << std::endl;
		
	binaryLoad.close();	
	
	if (m_swParams.classifier == SVM) {

		if (m_LearningParams.svm_costFactor == 0) {
			m_svm.learn_parm.svm_costratio = 1.0f * negSamples / posSamples;
		} else {
			m_svm.learn_parm.svm_costratio = m_LearningParams.svm_costFactor;
		}
	}
}


bool SW_Train::train(bool withHardExamples) {
	if(m_swParams.classifier == SVM) {
		return trainSVM(withHardExamples);
	}  else {
           cerr << " m_swParams.classifier = "<<m_swParams.classifier <<" type not supported"<<endl;
        }
	
/*	if (m_swParams.classifier == AdaBoost) {
		trainAdaBoost(withHardExamples);		
	}*/
	return true;
}

/*void SW_Train::trainAdaBoost(bool withHardExamples) {
	printf("Training a AdaBoost classifier on %d positive and %d negative examples...\n", posSamples, negSamples);
	printf("Number of rounds: %d...\n", m_LearningParams.boosting_rounds);
	
	Data boostData(m_featureDim, 2);
	boostData.addSampleVector(m_boostingData);
	
	m_boost.trainClassifier(&boostData, m_LearningParams.boosting_rounds, &std::cout);
	m_boost.printEnsemble();
	
	 std::string fileName;
	  if (!withHardExamples) 
	  	fileName = m_LearningParams.modelFile + "-withOutHardSamples";
	  else
	  	fileName = m_LearningParams.modelFile;
	  
	m_boost.saveClassifier(fileName);	
	
} */


/*void SW_Train::trainSVM(bool withHardExamples) {	
  //--- Running SVM learning algorithm from SVM dense ---//
  printf("Training a SVM on %d positive and %d negative examples...\n", posSamples, negSamples);  
  m_svm.set_verbosity(1);  
   	 
  m_svm.learn_parm.svm_c = m_LearningParams.svm_slack;
  m_svm.learn_parm.type = SVMLight_ns::CLASSIFICATION;			// 0 = classification 
  m_svm.kernel_parm.kernel_type = SVMLight_ns::LINEAR;   			// 0 = linear kernel
 
  if (m_LearningParams.svm_slack == 0) {
  	
  	if(m_LearningParams.svm_costFactor == 0) {
  	  	
  		//if (!withHardExamples)
  			m_svm.learn_parm.svm_costratio = 1.0f * negSamples / posSamples;
	  	
	  	if (m_LearningParams.svm_slackCV) {
	  		std::string intermediatePath;
	  		size_t pos = m_LearningParams.modelFile.rfind("/");
	  		if (pos == string::npos)
	  			intermediatePath = "./";
	  		else
	  			intermediatePath = m_LearningParams.modelFile.substr(0, pos);
	  	
	  		std::string prefix;
	  		if (!withHardExamples) 
	  			prefix = "intermediate-WithOutHard";
	  		else
	  			prefix = "intermediate-WithHard";
	  		  	  	
	  		m_LearningParams.svm_slack = m_svm.trainWithLeaveOneOutXValidationC(1.0f * negSamples / posSamples, intermediatePath.c_str(), prefix.c_str(), m_LearningParams.svm_trainProbSigmoid, m_LearningParams.svm_min_slack_C, m_LearningParams.svm_max_slack_C);
	  	} else {
	  		m_svm.learn_parm.svm_c = 0.0;
	  		m_svm.train(m_LearningParams.svm_trainProbSigmoid);	  		
	  	}
  	}  else {
  		m_svm.learn_parm.svm_c = 0.0;
  		m_svm.learn_parm.svm_costratio = m_LearningParams.svm_costFactor;
  		m_svm.train(m_LearningParams.svm_trainProbSigmoid);  		
  	}
  
  
  } else {
  	 m_svm.learn_parm.svm_c = m_LearningParams.svm_slack;
  	 if (m_LearningParams.svm_costFactor == 0)
  	 	//if (!withHardExamples)
  	 		m_svm.learn_parm.svm_costratio = 1.0f * negSamples / posSamples;
  	 else
  	 	m_svm.learn_parm.svm_costratio = m_LearningParams.svm_costFactor;
  	 m_svm.train(m_LearningParams.svm_trainProbSigmoid);
  }
 
  printf("Ratio of neg/pos samples %.3f...\n", m_svm.learn_parm.svm_costratio); 
  
  
  std::string fileName;
  if (!withHardExamples) 
  	fileName = m_LearningParams.modelFile + "-withOutHardSamples";
  else
  	fileName = m_LearningParams.modelFile;
  
   m_svm.save_model(fileName.c_str(), 1); // 1 = save binary, 0 = ascii
}  */

bool SW_Train::trainSVM(bool withHardExamples) {	
	
  if ( m_LearningParams.svm_kernelType != SVMLight_ns::LINEAR &&
	   m_LearningParams.svm_kernelType != SVMLight_ns::RBF &&
	   m_LearningParams.svm_kernelType != SVMLight_ns::HISTOGRAM_INTERSECT) {
	  printf("Only linear, RBF and histogram intersection kernels are supported!...\n");
	  return false;
  }
	    
  //--- Running SVM learning algorithm from SVM dense ---//
  printf("Training a SVM on %d positive and %d negative examples...\n", posSamples, negSamples);  
  m_svm.set_verbosity(1);  
   	 
  m_svm.learn_parm.svm_c = m_LearningParams.svm_slack;
  m_svm.learn_parm.type = SVMLight_ns::CLASSIFICATION;			// 0 = classification 
  m_svm.kernel_parm.kernel_type =  m_LearningParams.svm_kernelType;   			// 0 = linear kernel 2 = RBF, 5 = histogram intersection
  m_svm.kernel_parm.rbf_gamma = m_LearningParams.svm_kernel_sigma;
  
  //This needs to be set from outside later
  m_svm.kernel_parm.histIntMethod = SVMLight_ns::KERNEL_PARM::piecewise_linear;
  m_svm.kernel_parm.noHistInt_Bins = 30;
     
  if(m_LearningParams.svm_maxNormalize)
	  m_svm.maxNormalize(); 
  
  if (m_LearningParams.svm_costFactor == 0) {
		//if(!withHardExamples) { 
			m_svm.learn_parm.svm_costratio = 1.0f * negSamples / posSamples;
		//}
	} else {
		m_svm.learn_parm.svm_costratio = m_LearningParams.svm_costFactor;
  }
  
  if (m_LearningParams.svm_slackCV) {
	  if (m_svm.kernel_parm.kernel_type == SVMLight_ns::LINEAR || m_svm.kernel_parm.kernel_type == SVMLight_ns::HISTOGRAM_INTERSECT) {
		  
		  if (m_LearningParams.svm_autoRange) {
			  float c = m_svm.proposeC();
			  m_LearningParams.svm_min_slack_C = logf(c / 10) / logf(2);
			  m_LearningParams.svm_max_slack_C = logf(c * 10) / logf(2);
		  }	
		  
		  cout << "Scanning range for c: [2^" << m_LearningParams.svm_min_slack_C << ", 2^" <<  m_LearningParams.svm_max_slack_C << "]" << std::endl;
		  	  
		  std::string logFile(m_LearningParams.modelFile);
		  std::string logFile1 = logFile + "-cvlog1";
		  
		  float stepC = fabsf(m_LearningParams.svm_min_slack_C - m_LearningParams.svm_max_slack_C) / (m_LearningParams.svm_step_C - 1);	  
		  float c = m_svm.train_crossValidateC(m_LearningParams.svm_min_slack_C, m_LearningParams.svm_max_slack_C, stepC, logFile1.c_str(), false, m_LearningParams.folds, m_LearningParams.used_folds);
	  
		  std::string logFile2 = logFile + "-cvlog2";
		  m_svm.train_crossValidateC(c - 0.5f * stepC, c +  stepC * 0.5f, stepC / (m_LearningParams.svm_step_C - 1), logFile2.c_str(), m_LearningParams.svm_trainProbSigmoid, m_LearningParams.folds, m_LearningParams.used_folds);
	  }
	  
	  if (m_svm.kernel_parm.kernel_type == SVMLight_ns::RBF) {
		  
		  if (m_LearningParams.svm_autoRange) {
			  float s = 1.0f / m_svm.proposeSigma(m_LearningParams.svm_autoRangeSubset);
			  m_LearningParams.svm_min_kernel_sigma = logf(s / 10) / logf(2);
			  m_LearningParams.svm_max_kernel_sigma = logf(s * 10) / logf(2);
			  
			  m_svm.kernel_parm.rbf_gamma = pow(2, m_LearningParams.svm_max_kernel_sigma);			  
			  m_LearningParams.svm_min_slack_C = logf(m_svm.proposeC() / 10) / logf(2);
			  
			  m_svm.kernel_parm.rbf_gamma = pow(2, m_LearningParams.svm_min_kernel_sigma);	
			  m_LearningParams.svm_max_slack_C = logf(m_svm.proposeC() * 10) / logf(2);
		  }
		  
		  cout << "Scanning range for c: [2^" << m_LearningParams.svm_min_slack_C << ", 2^" <<  m_LearningParams.svm_max_slack_C << "]" << std::endl;
		  cout << "Scanning range for sigma: [2^" << m_LearningParams.svm_min_kernel_sigma << ", 2^" <<  m_LearningParams.svm_max_kernel_sigma << "]" << std::endl;
		  		 	  
		  float c;
		  float sigma;
		  
		  std::string logFile(m_LearningParams.modelFile);
		  if (!withHardExamples)
			  logFile.append("-withoutHardSamples");
		  std::string logFile1 = logFile + "-cvlog1";
		 		  
		  float stepC = fabsf(m_LearningParams.svm_min_slack_C - m_LearningParams.svm_max_slack_C) / (m_LearningParams.svm_step_C - 1);
		  float stepSigma = fabsf(m_LearningParams.svm_min_kernel_sigma - m_LearningParams.svm_max_kernel_sigma) / (m_LearningParams.svm_step_kernel_sigma - 1);	
		  m_svm.train_crossValidate_GridSearchC_Sigma(m_LearningParams.svm_min_slack_C, m_LearningParams.svm_max_slack_C, stepC, m_LearningParams.svm_min_kernel_sigma, m_LearningParams.svm_max_kernel_sigma, stepSigma, c, sigma, logFile1.c_str(), false, m_LearningParams.folds, m_LearningParams.used_folds);
		  
		  std::string logFile2 = logFile + "-cvlog2";
		 
		  m_svm.train_crossValidate_GridSearchC_Sigma(c - 0.5f * stepC, c + 0.5f * stepC, stepC / (m_LearningParams.svm_step_C - 1), sigma - 0.5f * stepSigma , sigma + 0.5f * stepSigma, stepSigma / (m_LearningParams.svm_step_kernel_sigma - 1), c, sigma, logFile2.c_str(), false, m_LearningParams.folds, m_LearningParams.used_folds);
		  
	  }
	  
  } else
	  m_svm.train(m_LearningParams.svm_trainProbSigmoid);  	

  printf("Ratio of neg/pos samples %.3f...\n", m_svm.learn_parm.svm_costratio); 
  
  std::string fileName;
  if (!withHardExamples) 
  	fileName = m_LearningParams.modelFile + "-withOutHardSamples";
  else
  	fileName = m_LearningParams.modelFile;
  
   m_svm.save_model(fileName.c_str(), 1); // 1 = save binary, 0 = ascii
   
   return true;
} 

void SW_Train::saveSupportVectors(const AnnotationList& posFileList, const AnnotationList& negFileList, bool containsHardExamples) {
	std::vector<int> svIDs;
		
	if (m_swParams.classifier != SVM || !m_svm.getSupportVectorIDs(svIDs, -1)) {
		printf("Unable to retrieve support vectors...\n");
		return;
	}
	
	AnnotationList supportVectorsNeg(negFileList);
	for(unsigned int i = 0; i < supportVectorsNeg.size(); ++i) {
		supportVectorsNeg[i].clear();		
	}
	
	for(unsigned int i = 0; i < svIDs.size(); ++i) {
		supportVectorsNeg[ m_NegRectFileIndex[svIDs[i] - posSamples ] ].addAnnoRect( m_NegRects[svIDs[i] - posSamples ]);
	}
	
	
	if (!m_svm.getSupportVectorIDs(svIDs, 1)) {
		printf("Unable to retrieve support vectors...\n");
		return;
	}
	
	AnnotationList supportVectorsPos(posFileList);
	for(unsigned int i = 0; i < supportVectorsPos.size(); ++i) {
		supportVectorsPos[i].clear();		
	}
	
	for(unsigned int i = 0; i < svIDs.size(); ++i) {
		supportVectorsPos[ m_PosRectFileIndex[svIDs[i]]  ].addAnnoRect( m_PosRects[svIDs[i]]);
	}
	
	if(containsHardExamples) {
		supportVectorsNeg.save(m_LearningParams.modelFile + "-negativeSVs-hardExamples.idl"); 
		supportVectorsPos.save(m_LearningParams.modelFile + "-positiveSVs-hardExamples.idl");
	} else {
		supportVectorsNeg.save(m_LearningParams.modelFile + "-negativeSVs.idl"); 
		supportVectorsPos.save(m_LearningParams.modelFile + "-positiveSVs.idl");
	}	
}
 
void SW_Train::extractHardSamples(const AnnotationList& negFileList) {
  	//--- Find hard training examples ---//
	
	if (m_swParams.classifier == SVM) { 
		if (!m_LearningParams.svm_trainProbSigmoid)	
			setMinDetectionMargin(0);
		else
			setMinDetectionMargin(0.5);
	}
	
	if (m_swParams.classifier == AdaBoost) {
		setMinDetectionMargin(0.5);
	}
  	
  	AnnotationList falsePositives(negFileList);  	  	  	
  
  	//We don't want extra borders	
  	preAndPostProcessParams origPP(m_preProcessParams);
  	  	
  	m_preProcessParams.borderfactor = 1.0;
  	m_preProcessParams.scaledHeight = m_swParams.windowHeight + 2;
  	m_preProcessParams.scaledWidth = m_swParams.windowWidth + 2;
  	m_preProcessParams.minHeight = 0;
  	m_preProcessParams.maxHeight = 0;
  	m_preProcessParams.minWidth = 0;
  	m_preProcessParams.maxWidth = 0;  	
  	m_preProcessParams.extraBoundary = 0;
  					
  	std::vector<AnnoRect> rects;
	std::vector<int> rectFileIndex;
  	
  	for(unsigned int i = 0; i < falsePositives.size(); i++) {
  		IplImage* negImg = cvLoadImage(falsePositives[i].imageName().c_str());
  		falsePositives[i].clear();
  		
  		if (m_preProcessParams.sharpenPercentTest !=0 ) {
  			 sharpen(reinterpret_cast<unsigned char*>(negImg->imageData), negImg->width, negImg->height, negImg->widthStep, 3, m_preProcessParams.sharpenPercentTest);
  		}
  		  		
  		testImageMultiScale(negImg, falsePositives[i], 1, 1, 100, 0.2, 1);
  		cvReleaseImage(&negImg);
  		
  		falsePositives[i].sortByScore();
  		fprintf(stderr, "Found %d false positives on %s.\n", falsePositives[i].size(), falsePositives[i].imageName().c_str());
  		
  		// Save all false positives in a big vector
  		for(unsigned int j = 0; j < falsePositives[i].size(); j++) {
  			rects.push_back(falsePositives[i].annoRect(j));
  			rectFileIndex.push_back(i);
  		}  		
  	}
  	
  	falsePositives.save(m_LearningParams.modelFile + "-hardExamples.idl");  	  	 
  	
  	AnnotationList reallyUsed(negFileList);
  	for(unsigned int i = 0; i < reallyUsed.size(); ++i) {
  		reallyUsed[i].clear();
  	}
  	
  	unsigned int overAllFP = rects.size();
  	
  	float* fv = new float[m_featureDim];
  	
  	printf("Drawing hard examples..\n");
  	unsigned int retrainSamples = 0; 
  	for(unsigned int i = m_LearningParams.maxRetrainingSamples; i > 0; i--) {
  		
  		//If all rects are computed we should break
  		if (rects.size() == 0)
  			break;
  		
  		int nextImage = rand() % (overAllFP--);
  		reallyUsed[rectFileIndex[nextImage]].addAnnoRect(rects[nextImage]);
  		
  		rects.erase(rects.begin() + nextImage);
  		rectFileIndex.erase(rectFileIndex.begin() + nextImage);
  		retrainSamples++;
  	}
  	reallyUsed.save(m_LearningParams.modelFile + "-hardExamples-reallySelected.idl");
  		
  	unsigned int doneSamples = 1;
  	for(unsigned int i = 0; i < reallyUsed.size(); ++i) {
  		IplImage* negImg = cvLoadImage(reallyUsed[i].imageName().c_str());  		
  		  		
  		for(unsigned int  j = 0; j < reallyUsed[i].size(); ++j, ++doneSamples) {  		
  			//Add the boundary to compute the gradients for some features  		
  			AnnoRect cropRect(reallyUsed[i][j]);
  			int n_x1 = static_cast<int>(round(cropRect.left() - 1 * cropRect.scale()));
  			if (n_x1 < 0) n_x1 = 0;
  		
  			int n_y1 = static_cast<int>(round(cropRect.top() - 1 * cropRect.scale()));
  			if (n_y1 < 0) n_y1 = 0;
  		
  			int n_x2 = static_cast<int>(round(n_x1 + m_swParams.windowWidth * cropRect.scale() + 2 * cropRect.scale()));
  		
  			int n_y2 = static_cast<int>(round(n_y1 + m_swParams.windowHeight * cropRect.scale() + 2 * cropRect.scale()));
  		  		
  			cropRect.setCoords(n_x1, n_y1, n_x2, n_y2);
  			cropRect.setScale(cropRect.w() * 1.0f / (m_swParams.windowWidth + 2) );
  		
  			IplImage* fpImg = getCrop(negImg, cropRect);
  		
  			/*static int count = 0;
  			char fn[1024];
  			sprintf(fn, "/tmp/check-%d.png", count++);
  			cvSaveImage(fn, fpImg);*/
  		  		
  			if (!fpImg)
  				continue;
			
  			printf("Compute %d/%d/(max)%d features for %s at %d,%d:%f...\n", doneSamples, retrainSamples, m_LearningParams.maxRetrainingSamples, falsePositives[i].imageName().c_str(), cropRect.centerX(), cropRect.centerY(), cropRect.scale());
		
  			float* nextfv = fv;
  			bool invalidWindow = false;
  			for(unsigned int v = 0; v < features.size(); ++v) {
  				features[v]->on_new_Image((unsigned char*)fpImg->imageData, fpImg->width, fpImg->height, fpImg->widthStep);
  				features[v]->on_new_Scale((unsigned char*)fpImg->imageData, fpImg->width, fpImg->height, fpImg->widthStep, 1.0, 0, 0);
  				nextfv = features[v]->on_get_FeatureVector(0, 0, m_swParams.windowWidth + 2, m_swParams.windowHeight + 2, nextfv);
  				if (nextfv == 0) {
  					invalidWindow = true;
  					break;
  				}
  			}  			
		
  			cvReleaseImage(&fpImg);
		
  			if (invalidWindow)
  				continue;
		
  			if (m_swParams.normalizeCues > 0)							
  				normalizeCues(fv);
											
  			// Add to SVM structure
  			if (m_swParams.classifier == SVM) {
  				if (m_svm.add_training_sample(fv, m_featureDim, -1.0))
  					negSamples++;
  			}

  			// ... or to Boosting structure
  			if (m_swParams.classifier == AdaBoost) {
  				m_boostingData.push_back(FeatureVector(fv, m_featureDim, 0));
  				negSamples++;
  			}
  			
  			m_NegRects.push_back(reallyUsed[i][j]);
  			m_NegRectFileIndex.push_back(i);  			
  		}  		
		cvReleaseImage(&negImg);							
	} 	 
  	
  	
  	delete[] fv;
  	m_preProcessParams = origPP;
}


