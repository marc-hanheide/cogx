#include<libHOG/HOGModel.h>

bool HOG::HOGModel::loadModel(bool binarymodel){
    fprintf(stdout, "Loading model file %s...\n", m_SVMParams.svmModelFile.c_str());
    if (std::ifstream(m_SVMParams.svmModelFile.c_str())) {
        svm.load_model(m_SVMParams.svmModelFile.c_str(), int(binarymodel));
        return true;
    }
    else {
        fprintf(stdout, "Model file %s does not exist\n", m_SVMParams.svmModelFile.c_str());
        return false;
    }
}

void HOG::HOGModel::setHOGParams(const HOGParams& params){
	m_hogParams=params;
}

void HOG::HOGModel::setSVMParams(const SVMParams& params) {
	m_SVMParams = params;
}

void HOG::HOGModel::setPreProcessParams(const preAndPostProcessParams& params){
	m_preProcessParams = params;
}


		

