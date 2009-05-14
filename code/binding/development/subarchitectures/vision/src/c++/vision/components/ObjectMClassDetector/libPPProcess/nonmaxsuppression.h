#ifndef NONMAXSUPPRESSION_
#define NONMAXSUPPRESSION_

#include<vector>
#include<libAnnotation/annotation.h>

class DataPoint{
	public:		
		std::vector<float> data;			// data dimensions (e.g.: x,y,s)
		float score;						// sample score
		float weight;						// normalized weight in kernel 
		std::vector<float> H;               // bandwidth matrix (diag) H_i
		float detH12;						// 1/ sqrt(det(H_i))
		DataPoint(std::vector<float> d, float s) : data(d), score(s) {H.resize(d.size());}; 
};

class NonMaxSuppresion {
	public:
		NonMaxSuppresion(const std::vector<float>& bandwidth) {initalBandwidth = bandwidth;};
		void addDataPoints(const Annotation& detections);
		void getModes(Annotation &detections, int minDetectionsPerMode = 4, float epsilon = 0.0001, int maxIter = 0, int scoreMode = 0) ;
		void clearData() {data.clear();};
				
	private:
		void reset_H(DataPoint& nd); 
		std::vector<float> runMeanShift(const std::vector<float>& p, float epsilon, int maxIter);
		std::vector<float> calcAdaptedBandWidth();
		float mahalanobisDist(const std::vector<float>& p1, const std::vector<float>& p2, const std::vector<float>& H);
		void calcWeights(const std::vector<float>& p1);
		float getModeConfidence(const std::vector<float>& p1);
						
		std::vector<float> initalBandwidth;		
		std::vector<DataPoint> data;
		std::string fileName;
		unsigned int frameNr;
		bool imageFromStream;
		float bbWidth;
		float bbHeight;		
	};

#endif /*NONMAXSUPPRESSION_*/
