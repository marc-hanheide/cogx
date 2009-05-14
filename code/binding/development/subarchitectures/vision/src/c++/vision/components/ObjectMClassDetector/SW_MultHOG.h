#include <string>
#include <vector>
#include <libFeatures/featurevector.hh>
#include <libPPProcess/ppParams.h>
//#include <libPPProcess/preprocess.h>
#include <libAnnotation/annotationlist.h>
#include <libHOG/HOG.h>
#include <libSVMdense/svm.h>
#include <opencv/cv.h>

#include "parameters.h"
#include "features_sw.h"


using namespace HOG;
using namespace std;

class SW_MultHOG{

  struct HOGSVM{
    preAndPostProcessParams  ppParams;
    HOGParams               hogParams;
    LearningParams          svmParams;
    SW_Params                swParams;
    SVMlight*                   m_svm;
    vector<float>             vlinsvm;
    int                        hogDim;
    int                  hogStepSizeX;
    int                  hogStepSizeY;
    string                  modelname;
    HOGSVM():m_svm(0){};
    //svm model pointer needs to live in copied objects...
    //~HOGSVM(){ if(m_svm) {delete(m_svm);}};

  };
  int m_imaxHogDim; 
 public:
  vector < HOGSVM > m_vHOGSVM;
  public:
  SW_MultHOG():m_imaxHogDim(0){};
  void processImageFromCamera();
  inifile::IniFile loadConfigAndModel(const char* name,const char* confFile, const char* svmModelFile);
  int testImage_slidingWindow(IplImage* img, Annotation& anno, const float scale, const float xoffset, const float yoffset, float xbound, float ybound);
int testImage(IplImage* img, Annotation& anno, const int noSteps, const float scale, float xbound, float ybound);

int testImageMultiScale(IplImage* img, Annotation& anno, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType);
 void normalizeCues(float* fv);

};

void blurEdges(unsigned char* data, unsigned int width, unsigned int height, unsigned int rowstep, 
	const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad);
