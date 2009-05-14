
#ifndef CARHOGJBWRAPPER_H
#define CARHOGJBWRAPPER_H

#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"


class HOGParams; //!< Just a predefinition
class JointBoostClassifier;
//class preAndPostProcessParams; //!< Just a predefinition

/** \brief This is a lightweight class, that wraps HogTest
 * This class is developed to provide an easy way to call the HOG from other libraries
 * HOGWrapper should depend on as few files as possible. This wrapper allows HogTest
 * to have a private implementation (HogTest implementation and public interface
 * can change, still no effect on the user of HOGWrapper).
 */
class CarHOGJBWrapper {
	public:
		CarHOGJBWrapper(int theta, const char * classImgName);
		~CarHOGJBWrapper();
		bool loadConfig(const char* configFile); 
                void loadModel(const char* modelName);
                bool trainModel(const char* idlFile, const char *modelFile, int numBoostingRounds);
		void classifyRawColorInterleaved(const unsigned char* data, int w, int h, int rowstep, std::vector<float>& results);

	private:
		HOGParams* _hparams;
                JointBoostClassifier* _jbc;
                uint hogDim;
                IplImage * _image_classes;
};

#endif
