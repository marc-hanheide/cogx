#ifndef HOGMODELS_H_
#define HOGMODELS_H_

#include <libAnnotation/annotationlist.h>
#include <libHOG/HOGModel.h>
#include <libPPProcess/preprocess.h>
#include <libSVMdense/svm_common.h>
#include <vector>
#include <string>



namespace HOG {
using namespace std;
struct MultHOG{
  HOGParams m_hogParams;
  HOG::HOGModel::SVMParams m_SVMParams;
  preAndPostProcessParams m_preAndPostProcessParams;
  //vector<vector<double> > vHOGClassifier;
  vector<vector<float> > vHOGClassifier;
  vector<SVMLight_ns::MODEL *> vHOGClassifierSVMLM; //free at some point ...
  vector<Annotation> vAnnotations;
  vector<string> vFNames;
};

class HOGModels {
 public: 
  vector<MultHOG> v_multHOG;
  void addHOG(string HOGConfFile, vector<string> & SVMModelFiles);
};
}
#endif
