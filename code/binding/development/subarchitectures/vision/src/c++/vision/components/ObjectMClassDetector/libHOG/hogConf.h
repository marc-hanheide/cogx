#ifndef HOGCONFIG_
#define HOGCONFIG_

#include <libHOG/HOGModel.h>
#include <libPPProcess/preprocess.h>
#include <libInifile/inifile.h>

namespace HOG {
inifile::IniFile parseConfFile(const char* fileName, HOGParams &hogParams, preAndPostProcessParams &ppParams, HOGModel::SVMParams &svmParams);
}

#endif
