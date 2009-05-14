#include <fstream>
//#include <libTiffStreamReader/StreamReader.h>
#include "sw_svm.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>



bool SW_Test::load_ClassifierModel(bool binarymodel) {
	fprintf(stdout, "Loading model file %s...\n", m_LearningParams.modelFile.c_str());
	if (std::ifstream(m_LearningParams.modelFile.c_str())) {
		
		if (m_swParams.classifier == SVM) {
			//This needs to be set from outside later
			m_svm.kernel_parm.histIntMethod = SVMLight_ns::KERNEL_PARM::piecewise_linear;
			m_svm.kernel_parm.noHistInt_Bins = 30;
			m_svm.load_model(m_LearningParams.modelFile.c_str(), int(binarymodel));
		} else {
		        cerr << "SW_Test::load_ClassifierModel: m_swParams.classifier = "<< m_swParams.classifier << " type not supported"<<endl;
                }
		//if (m_swParams.classifier == AdaBoost)
		//	m_boost.loadClassifier(m_LearningParams.modelFile);
		
		return true;
	}
	else {
		fprintf(stdout, "Model file %s does not exist\n", m_LearningParams.modelFile.c_str());
		return false;
	}
}


int SW_Test::testImage_slidingWindow(IplImage* img, Annotation& anno, const float scale, const float xoffset, const float yoffset, float xbound, float ybound) {	
		
	//Debugging
	//char fn[500];
	//sprintf(fn, "/tmp/debug-scale-%.2f.png", scale);
	//cvSaveImage(fn, img);
		
	int counter =  0;	
	float* fv = new float[m_featureDim];	
	
	//ROI handling
	unsigned char* startAddr = (unsigned char*)img->imageData;
	int rowStep = img->widthStep;
	int w = img->width;
	int h = img->height;
	if (img->roi) {
		startAddr = (unsigned char*)img->imageData + img->widthStep * img->roi->yOffset + img->roi->xOffset * 3;
		w = img->roi->width;
		h = img->roi->height;
	}

	for(unsigned int v = 0; v < features.size(); ++v) {
		features[v]->on_new_Scale(startAddr, w, h, rowStep, scale, img->roi->xOffset * 1.0f / scale, img->roi->yOffset * 1.0f / scale);
	}
	
	for(int y = 1; y < h - 1; y += m_swParams.strideY ) {
		for(int x = 1; x < w - 1; x += m_swParams.strideX) {
			
			float* nextfv = fv;
			bool invalidWindow = false;
			for(unsigned int v = 0; v < features.size(); ++v) {				
				nextfv = features[v]->on_get_FeatureVector(x - 1, y - 1, m_swParams.windowWidth + 2, m_swParams.windowHeight + 2, nextfv);
				if (nextfv == 0) {
					invalidWindow = true;
					break;
				}
			}
			
			if (invalidWindow)
				continue;
			
			++counter;
						
			if (m_swParams.normalizeCues > 0)
				normalizeCues(fv);
								
			double margin;
			
			if (m_swParams.classifier == SVM)
				margin = m_svm.predict(fv, m_featureDim, m_LearningParams.svm_trainProbSigmoid);
			//else
			//	margin = m_boost.evaluateFeaturePoint(fv, m_featureDim, true);
		            
			if (margin > m_dMinMargin) {
				//fprintf(stdout, "Prediction: Margin: %f, %d\n", margin,  xoffset);				
				int posX = static_cast<int>(round((x + xoffset - xbound) / scale));
				int posY = static_cast<int>(round((y + yoffset - ybound) / scale));
				int scaled_windowWidth = static_cast<int>(round(m_swParams.windowWidth * 1.0f / scale));
				int scaled_windowHeight = static_cast<int>(round(m_swParams.windowHeight * 1.0f / scale));
								
				AnnoRect r(posX, posY, posX + scaled_windowWidth, posY + scaled_windowHeight, margin);
				r.setScale( 1/ scale);
				anno.addAnnoRect(r);			
			}
			
		}
	}
	
	delete[] fv;
	
	return counter;

}

int SW_Test::testImage(IplImage* img, Annotation& anno, const int noSteps, const float scale, float xbound, float ybound) {
	fprintf(stdout, " Image Dimensions: %dx%d\n", img->width, img->height);

	// Maybe we should have noStepsX and noStepsY
	int stepX = m_swParams.strideX / noSteps;
	int stepY = m_swParams.strideY / noSteps;
	
	assert(stepX > 0 && stepY > 0);
	
	int counter = 0;
	for(int i = 0; i < noSteps; ++i) {		
		const int xoffset = i * stepX;
		const int yoffset = i * stepY;
		
		fprintf(stdout, "    xoff: %d, yoff: %d\n", xoffset, yoffset);		

		int roiX = 0;
		int roiY = 0;
		int roiW = img->width;
		int roiH = img->height;
		
		bool hadROI = false;
		if (img->roi) {
			roiX = img->roi->xOffset;
			roiY = img->roi->yOffset;
			roiW = img->roi->width;
			roiH = img->roi->height;
			hadROI = true;
		}
		
		cvSetImageROI(img, cvRect(roiX + xoffset, roiY + yoffset, roiW - xoffset, roiH - yoffset));		
		counter += testImage_slidingWindow(img, anno, scale, xoffset, yoffset, xbound, ybound );		
		cvResetImageROI(img);
		
		if (hadROI)
			cvSetImageROI(img, cvRect(roiX , roiY, roiW , roiH ));	
					
		
	}
	return counter;
	//fprintf(stdout, "  Checked %d windows on this scale\n", counter);
}

int SW_Test::testImageMultiScale(IplImage* img, Annotation& anno, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType) {	
	int counter=0;
	const unsigned int descriptorHeight = m_swParams.windowHeight + 2;	
	const unsigned int descriptorWidth = m_swParams.windowWidth + 2;
	
	float minS = std::min(img->width * 1.0f / descriptorWidth, img->height * 1.0f / descriptorHeight);
	
	const int extraBoundary = static_cast<int>(round(m_preProcessParams.extraBoundary * minS));
	const int kernelWidth = m_preProcessParams.blurKernelWidth;
			
	const int xbound = extraBoundary;
	const int ybound = extraBoundary;
		
	//Add some extra boundary
	IplImage* processedImage = img;
	IplImage* extraBoundaryImage = 0;
		
	if (extraBoundary > 0) {
		extraBoundaryImage = cvCreateImage(cvSize(img->width + 2 * xbound, img->height + 2 * ybound), img->depth, img->nChannels);
			
		cvSetImageROI(extraBoundaryImage, cvRect(xbound, ybound, img->width, img->height));
		cvCopy(img, extraBoundaryImage);
		cvResetImageROI(extraBoundaryImage);
		
		blurEdges((unsigned char*)extraBoundaryImage->imageData, extraBoundaryImage->width, extraBoundaryImage->height,
			extraBoundaryImage->widthStep, xbound, ybound, extraBoundaryImage->width - 1 - xbound,
			extraBoundaryImage->height - 1 - ybound, kernelWidth);
		
		processedImage = extraBoundaryImage;
	}
		
	for(unsigned int v = 0; v < features.size(); ++v) {
		features[v]->on_new_Image((unsigned char*)img->imageData, processedImage->width, processedImage->height, processedImage->widthStep);
	}
	
	IplImage* lastImage = processedImage;
	float lastScale = 1.0;

	for(float scale= minScale; scale <= maxScale; scale = scaleType == 0 ? scale + scaleStep : scale * (1+scaleStep)) {
		
		unsigned int h = static_cast<unsigned int>(processedImage->height * 1.0/scale);
		unsigned int w = static_cast<unsigned int>(processedImage->width * 1.0/scale);
		
		if (h + 2 * m_preProcessParams.extraBoundary < descriptorHeight || w + m_preProcessParams.extraBoundary * 2 < descriptorWidth) break;
		fprintf(stdout, "------------ Scale: %.2f ----------------\n", scale);
	
		IplImage* scaled = cvCreateImage(cvSize(w,h), processedImage->depth, processedImage->nChannels);			
		cvResize(lastImage, scaled, CV_INTER_LINEAR);
	
		fprintf(stdout, " Adding extra boundary of %d pixel(s)...\n", m_preProcessParams.extraBoundary);
		int boundX = static_cast<int>(round(xbound * 1.0f / scale - m_preProcessParams.extraBoundary));
		int boundY = static_cast<int>(round(ybound * 1.0f / scale - m_preProcessParams.extraBoundary));
					
		cvSetImageROI(scaled, cvRect(boundX, boundY, scaled->width - 2 *boundX, scaled->height - 2 *boundY));			
		counter += testImage(scaled, anno, noSteps, 1.0f /scale, m_preProcessParams.extraBoundary, m_preProcessParams.extraBoundary);
		cvResetImageROI(scaled);
		
		//Debugging:
		//char fn[500];
		//sprintf(fn, "/tmp/gauss-%f.png", scale);			
		//cvSaveImage(fn, scaled);
	    //sprintf(fn, "/tmp/smoothed-%f.png", scale);	
		//cvSaveImage(fn, pCopy);
		
		//MipMap kind of resizing.. avoids of resizing artifacts while Gaussian smoothing is not necessary
		if(scale * (1 + scaleStep) / lastScale >= 2.0) {
			if(lastScale != 1.0) {
				cvReleaseImage(&lastImage);
				lastImage = 0;
			}
			lastImage = scaled;
			lastScale = scale;
		} else 
			cvReleaseImage(&scaled);					
	}
	
	if(lastImage != 0 && lastImage != processedImage)
		cvReleaseImage(&lastImage);
	
	fprintf(stdout, "Checked %d windows on multiple scales\n", counter);
	
	if (extraBoundaryImage)
		cvReleaseImage(&extraBoundaryImage);
	
	return counter;
}

int SW_Test::testImageMultiScale(std::string fileName, Annotation& anno, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType) {
	fprintf(stdout, "Testing image: %s\n", fileName.c_str());
	
	IplImage* img = cvLoadImage(fileName.c_str());
	if (img == 0 || img->nChannels != 3) {
		fprintf(stdout, "ERROR: Failed loading image: %s\n", fileName.c_str());
		return 0;
	}
	
	if (m_preProcessParams.sharpenPercentTest !=0 ) {		
		sharpen(reinterpret_cast<unsigned char*>(img->imageData), img->width, img->height, img->widthStep, 3, m_preProcessParams.sharpenPercentTest);
	}
			
	const int rval = testImageMultiScale(img, anno, noSteps, minScale, maxScale, scaleStep, scaleType);
	reShapeBoundingBoxes(anno, m_preProcessParams, m_swParams.windowWidth, m_swParams.windowHeight);
	
	cvReleaseImage(&img);
	
	return rval;
}

/*int SW_Test::testImageMultiScaleFromStream(Annotation& anno, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType) {
	static StreamReader reader;
	
	std::string oldDir = reader.getBasePath();
	
	if(oldDir.compare(anno.imageName()) != 0) {
		reader.openStream(anno.imageName().c_str());
	}
		
	if (!reader.isValid()) {
		return 0;
	}
	
	reader.loadFrame(anno.frameNr());
	const unsigned short *img = reader.getColorFrame(m_preProcessParams.ahd_interpolate);
	const unsigned int width = reader.getWidth();
	const unsigned int height = reader.getHeight();
	
	if (m_pTestedImage) {
		delete[] m_pTestedImage;
		m_pTestedImage = 0;
	}
		
	m_pTestedImage = new unsigned char[width * height * 3];
		
	for(unsigned int i = 0; i < width * height; i++) {
		m_pTestedImage[i * 3] = static_cast<unsigned char>((img[i * 3 + 2] >> 8));
		m_pTestedImage[i * 3 + 1] = static_cast<unsigned char>((img[i * 3 + 1] >> 8 ));
		m_pTestedImage[i * 3 + 2] = static_cast<unsigned char>((img[i * 3] >> 8));					
	}
	
	if (m_preProcessParams.sharpenPercentTest != 0) {
		sharpen(m_pTestedImage, width, height, width * 3* sizeof(unsigned char), 3, m_preProcessParams.sharpenPercentTest);
	}
	
	IplImage* testImage = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvSetImageData(testImage, m_pTestedImage, width * 3 * sizeof(unsigned char));
	
	const int rval = testImageMultiScale(testImage, anno, noSteps, minScale, maxScale, scaleStep, scaleType);
	
	reShapeBoundingBoxes(anno, m_preProcessParams, m_swParams.windowWidth, m_swParams.windowHeight);
	
	cvReleaseImageHeader(&testImage);
	return rval;
} */

SW_Test::~SW_Test() {
	if(m_pTestedImage)
		delete[](m_pTestedImage);
}

int SW_Test::testAllMultiScale(AnnotationList& list, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType, std::string incrementalStore)
{
	int counter = 0;
	for(unsigned int i = 0; i < list.size(); ++i) {	
		list[i].clear();
		if(!list[i].isStream())	{	
			counter += testImageMultiScale(list[i].imageName(), list[i], noSteps, minScale, maxScale, scaleStep, scaleType);
		} else {
			//counter += testImageMultiScaleFromStream(list[i], noSteps,minScale, maxScale, scaleStep, scaleType);
                        cerr << "stream processing disabled!"<<endl;
		}
	
		if (incrementalStore.compare("") != 0) {
			list[i].sortByScore();
			list.save(incrementalStore);						
		}
	}
	fprintf(stdout, "Checked %d windows on all images\n", counter);
	return counter;
}

void SW_Test::load_TradeOffModel(const char* fileName) {
	std::ifstream binaryRead(fileName, std::ios::in | std::ios::binary);
	
	if (!binaryRead.is_open()) {
		std::cout << "Error opening " << fileName << "!" << std::endl;
		return;	
	}
	
	m_w.clear();
	m_d.clear();
	m_b = 0;
	
	unsigned int N;
	unsigned int M;
	
	binaryRead.read(reinterpret_cast<char*>(&m_b), sizeof(m_b));	
	binaryRead.read(reinterpret_cast<char*>(&M), sizeof(M));
	
	m_w.resize(M);
	m_d.reserve(M);
	
	double k;
	for(unsigned int i = 0; i < M; ++i) {		
		binaryRead.read(reinterpret_cast<char*>(&k), sizeof(k));
		m_d.push_back(k);
		binaryRead.read(reinterpret_cast<char*>(&k), sizeof(k));
		binaryRead.read(reinterpret_cast<char*>(&N), sizeof(N));
		m_w[i].reserve(N);
		for(unsigned int j = 0; j < N; ++N) {
			binaryRead.read(reinterpret_cast<char*>(&k), sizeof(k));
			m_w[i].push_back(k);
		}
	}
}

void SW_Test::addFeature(FeatureVisitor* newFeature) {
	features.push_back(newFeature);
	
	// Figure out the full length of the feature vector
	m_featureDim = 0;
	for(unsigned int v = 0; v < features.size(); ++v) {
		m_featureDim += features[v]->on_get_featureDim(); 
	}	
}

void SW_Test::addFeature(std::vector<FeatureVisitor*>& newFeature) {
	
	for (unsigned int i = 0; i < newFeature.size(); ++i)
		features.push_back(newFeature[i]);
	
	// Figure out the full length of the feature vector
	m_featureDim = 0;
	for(unsigned int v = 0; v < features.size(); ++v) {
		m_featureDim += features[v]->on_get_featureDim(); 
	}	
	
	maxNormalizer.resize(m_featureDim, 1);
}



void SW_Test::normalizeCues(float* fv) {
	
	float* nextfv = fv;
	for(unsigned int v = 0; v < features.size(); ++v) {
		unsigned int cueLength = features[v]->on_get_featureDim();
		
		if (m_swParams.normalizeCues == 1) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i];
			}
			//cout << sum << endl;
			sum = sum + cueLength;		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}
		}
		
		if (m_swParams.normalizeCues == 2) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}
		}
		
		if (m_swParams.normalizeCues == 4) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
				if (nextfv[i] / sum > 0.2)
					nextfv[i] = 0.2;
				else
					nextfv[i] /= sum;
			}
			
			sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}		
		}
		
		nextfv += cueLength;
	}
}

void SW_Test::clearFeatures() {
	features.clear();
	m_featureDim = 0;
}


