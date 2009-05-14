#ifndef HOG_FEATURES_H_
#define HOG_FEATURES_H_

#include <libHOG/HOG.h>
#include "features_sw.h"

/*
 * 
 * This produces HOG features
 * 
 */


class HOG_Visitor : public FeatureVisitor {
public:
	HOG_Visitor(const HOGParams& hog_parameters);
	virtual ~HOG_Visitor();
	
	virtual void on_new_Scale(const unsigned char* img, int width, int height, int rowStep, float scale, float originAtFullScaleX, float originAtFullScaleY);
	virtual float* on_get_FeatureVector(int posX, int posY, int width, int height, float* features);
	virtual int on_get_featureDim() const;
	virtual void on_new_Image(const unsigned char* img, int width, int height, int rowStep);

	void setHOGParams(const HOGParams& params) {m_hogParams=params;}
	
	
	
private:	
	HOGParams m_hogParams;
	HOG::ColorGradient* m_cg;
	HOG::CellGrid* m_cgr;
        HOG::BlockGrid* m_bg;
	
	int m_StepSizeX;
	int m_StepSizeY;
};

void parseHOG_parameters(const inifile::IniFile& confFile, HOGParams &hogParams);

#endif /*HOG_FEATURES_H_*/
