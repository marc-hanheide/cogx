#ifndef CVHOGMODEL_H
#define CVHOGMODEL_H

#include <string>
#include <libAnnotation/annotationlist.h>
#include <libHOG/HOG.h>

#include <libSVMdense/svm.h>
#include <libPPProcess/preprocess.h>

//int annoCount(AnnotationList& annotations);

namespace HOG {

class HOGModel{
    public:
		struct SVMParams{
			std::string svmModelFile;
			std::string sampleFile;
			unsigned int noRandomPerNegative;
			float negativeScaleStep;
			int noNegativeScales;
			int kernelType;
			float costFactor;
			float slack;
			float minDetectionMargin;
			unsigned int maxRetrainingSamples;
			bool slackCV;
			bool trainProbSigmoid;
			SVMParams() : svmModelFile("svmHOG.model"), sampleFile("trainingSamples.svm"), 
				noRandomPerNegative(10), negativeScaleStep(1.2), noNegativeScales(1), 
				kernelType(0), costFactor(3), slack(0.01), minDetectionMargin(0), 
				maxRetrainingSamples(0), slackCV(false), trainProbSigmoid(false) {};
		};

        bool loadModel(bool binarymodel);
		// TODO: saveModel???

        void setHOGParams(const HOGParams& params);
		const HOGParams& getHOGParams() const { return m_hogParams; };

        void setSVMParams(const SVMParams& params);
		const SVMParams& getSVMParams() const { return m_SVMParams; };

        void setPreProcessParams(const preAndPostProcessParams& params);
		const preAndPostProcessParams& getPreProcessParams() const { return m_preProcessParams; }

		const SVMlight& getSVM() const { return svm; }
		SVMlight& getSVM() { return svm; }

    private:
        SVMlight svm;
        HOGParams m_hogParams;
        SVMParams m_SVMParams;
        preAndPostProcessParams m_preProcessParams; // TODO: preAndPostProcessParams --> PreAndPostProcessParams or PPParams
};

}


#endif
