#include <libPPProcess/nonmaxsuppression.h>
#include <cassert>
#include <fstream>
#include <iostream>

using namespace std;

void NonMaxSuppresion::addDataPoints(const Annotation& detections) {
	
	const unsigned int dataDim = 3;
	data.clear();	

	if(detections.size() == 0)
		return;
	
	// Figure out the classifier's windowSize
	bbWidth = abs(detections[0].x2() - detections[0].x1()) / detections[0].scale();
	bbHeight = abs(detections[0].y2() - detections[0].y1()) / detections[0].scale();
	fileName = detections.imageName();
	frameNr = detections.frameNr();
	imageFromStream = detections.isStream();
	
	for(unsigned int i = 0; i < detections.size(); i++) {	
		std::vector<float> d(dataDim);			
		d[0] = detections[i].centerX();
		d[1] = detections[i].centerY();
		d[2] = detections[i].logScale();
	
		//cout << "Detections scale: "  << detections[i].scale()  << "Log Scale: " << detections[i].logScale() << endl;
		
		assert(initalBandwidth.size() == dataDim);
	
		DataPoint nd(d, detections[i].score());	
		
		reset_H(nd);
	
		data.push_back(nd);
	}
	return;
}

void NonMaxSuppresion::reset_H(DataPoint& nd) { 
	nd.H[0] = (initalBandwidth[0] * exp(nd.data[2]) ) * (initalBandwidth[0] * exp(nd.data[2]));
	nd.H[1] = (initalBandwidth[1] * exp(nd.data[2]) ) * (initalBandwidth[1] * exp(nd.data[2]));
	nd.H[2] = (log(initalBandwidth[2])) * (log(initalBandwidth[2]));

	nd.detH12 =  1.0 / sqrt(nd.H[0] * nd.H[1] * nd.H[2]);
}




float NonMaxSuppresion::mahalanobisDist(const std::vector<float>& p1, const std::vector<float>& p2, const std::vector<float>& H) {	
	assert(p1.size() == p2.size());
	assert(H.size() == p1.size());
	
	float dist = 0;
	for(unsigned int i = 0; i < p1.size(); i++) {
		float diff = p1[i] - p2[i];
		dist += diff * 1.0 / H[i] * diff;
	}
	
	return dist;	
}

void NonMaxSuppresion::calcWeights(const std::vector<float>& p1) {
	float sumWeight = 0;
	
	for(unsigned int i = 0; i < data.size(); i++) {
		data[i].weight =  data[i].detH12 * data[i].score * exp( - mahalanobisDist(data[i].data, p1, data[i].H) / 2.0);
		sumWeight += data[i].weight;
	}
	
	//Normalize
	for(unsigned int i = 0; i < data.size(); i++) 
		data[i].weight /= sumWeight;
	
	return;
}

std::vector<float> NonMaxSuppresion::calcAdaptedBandWidth() {
	std::vector<float> H_h(initalBandwidth.size(), 0.0);
		 
	for(unsigned int j = 0; j < data.size(); j++) {
		for(unsigned int i = 0; i < initalBandwidth.size(); i++) {						
			if (data[j].H[i] > 0)
				H_h[i] += 1.0 / data[j].H[i] * data[j].weight;
		}		
	}
	
	for(unsigned int i = 0; i < H_h.size(); i++)			
		H_h[i] = 1.0 / H_h[i];
	
	return H_h;		
}

std::vector<float> NonMaxSuppresion::runMeanShift(const std::vector<float>& p, float epsilon, int maxIter) {
	std::vector<float> mean(p);			
	std::vector<float> lastMean;
	float change;
	epsilon*=epsilon;
	//cout << "Start ...." << endl;
	int iter = 0;
	do {
		lastMean = mean;
		change = 0;
		
		calcWeights(mean);
		std::vector<float> H_h = calcAdaptedBandWidth();
		//cout << "Mean_x: " << mean[0] << " Mean_y: " << mean[1] << " Mean_s: " << mean[2] << endl;
		
		//Update mean
		for(unsigned int i = 0; i < mean.size(); i++) {			
			mean[i] = 0;
			
			for(unsigned int j = 0; j < data.size(); j++)
				mean[i] += data[j].weight * 1.0 / data[j].H[i] * data[j].data[i];
			
			mean[i] *= H_h[i];
			change += (lastMean[i] - mean[i]) * (lastMean[i] - mean[i]);
			//cout << mean[i] << " ";			
		}
		
		//Debugging
		/*float conf = getModeConfidence(mean);
		cout << "f(y) = " <<  conf << endl;*/
		
		iter++;
		//cout << endl << sqrt(change) << endl;
	} while (change > epsilon && !(maxIter > 0 && iter > maxIter));
	//cout << iter << endl;
	
	return mean;
}

float NonMaxSuppresion::getModeConfidence(const std::vector<float>& p1) {
	float confidence = 0;
	const unsigned int numSamples = data.size();
	
	for(unsigned int i = 0 ; i < numSamples; i++) {
		confidence += data[i].detH12 * data[i].score * exp( - mahalanobisDist(data[i].data, p1, data[i].H) / 2.0);
		
		/*cout << "data[i]: "  << data[i].detH12 << endl;
		cout << "dist[i]: "  << exp( - mahalanobisDist(data[i].data, p1, data[i].H) / 2.0) << endl;
		cout << "prod[i]: "  << data[i].detH12 * exp( - mahalanobisDist(data[i].data, p1, data[i].H) / 2.0) << endl;*/
	}
	
	
		
	//confidence = confidence / (numSamples * pow(2*  3.14159265, 1.5));
	
	return confidence * 1.0f / numSamples / sqrt(8* M_PI * M_PI * M_PI);
}

void NonMaxSuppresion::getModes(Annotation &detections, int minDetectionsPerMode, float epsilon, int maxIter, int scoreMode) {	
	const float maxThres = 1;
	
	if (data.size() == 0)
		return;
	
	//ofstream f("modes.txt");	
	std::vector< std::vector<float> > modes;
	std::vector<int> modesNoHypos;
	std::vector<float> modesAccumulatedScore;
	std::vector<float> modesMaxScore;
	Annotation initialDetections;

	for(unsigned int i = 0; i < data.size(); i++) {				
		std::vector<float> currentMode = runMeanShift(data[i].data, epsilon, maxIter);
		
		bool discard = false; 
		for(unsigned int j = 0; j < modes.size(); j++) {
			float meanScale = (exp(currentMode[2]) + exp(modes[j][2])) / 2.0; 
			
			std::vector<float> H_ij(3);
			H_ij[0] =  (initalBandwidth[0] *  meanScale) * (initalBandwidth[0] * meanScale);
			H_ij[1] = (initalBandwidth[1] * meanScale) * (initalBandwidth[1] * meanScale);
			H_ij[2] = (initalBandwidth[2]) * (initalBandwidth[2]);
			
			if (mahalanobisDist(currentMode, modes[j], H_ij) <= maxThres) {				 
				discard = true;
				modesNoHypos[j]++;
				modesAccumulatedScore[j]+=data[i].score;
				if(modesMaxScore[j] < data[i].score)
					modesMaxScore[j] = data[i].score;				
				break;
			}
		}
		
		if (!discard) {
			modes.push_back(currentMode);
			modesNoHypos.push_back(1);
			
			modesAccumulatedScore.push_back(data[i].score);
			modesMaxScore.push_back(data[i].score);
			
			AnnoRect r;
			r.setScale(exp(currentMode[2]));
			
			r.setX1(static_cast<int>(round(currentMode[0] - (bbWidth * r.scale()) / 2.0)));
			r.setX2(static_cast<int>(round(currentMode[0] + (bbWidth * r.scale()) / 2.0)));
			r.setY1(static_cast<int>(round(currentMode[1] - (bbHeight * r.scale()) / 2.0)));
			r.setY2(static_cast<int>(round(currentMode[1] + (bbHeight * r.scale()) / 2.0)));
			r.setScore(getModeConfidence(currentMode));
			
			initialDetections.addAnnoRect(r);
			//f << currentMode[0] << "\t" << currentMode[1] << "\t" << exp(currentMode[2]) << "\t" <<  getModeConfidence(currentMode) <<  endl;					
		}				
	}
	for(unsigned int i=0; i<initialDetections.size(); ++i)
	{		
		if (modesNoHypos[i] >= minDetectionsPerMode)
		{
			if (scoreMode == 1)
				initialDetections[i].setScore(modesMaxScore[i]);
			else if (scoreMode == 2)				
				initialDetections[i].setScore(modesAccumulatedScore[i] / modesNoHypos[i]);
							
			detections.addAnnoRect(initialDetections[i]);
		}
	}	
	detections.sortByScore();
	return;
}
