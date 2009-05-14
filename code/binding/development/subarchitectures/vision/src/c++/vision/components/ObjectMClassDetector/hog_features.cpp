#include <libHOG/HOG.h>
#include <libInifile/inifile.h>

#include "hog_features.h"

using namespace HOG;
using namespace std;

void parseHOG_parameters(const inifile::IniFile& confFile, HOGParams &hogParams) {	
	
	// The object part of the detector and detector parameters
	hogParams.m_BlurSigma=confFile.GetFloat("m_BlurSigma", "HOG", 0.0);
        hogParams.m_nWindowWidth = confFile.GetInt("m_nWindowWidth", "HOG", 64);
	hogParams.m_nWindowHeight = confFile.GetInt("m_nWindowHeight", "HOG", 128);;
	hogParams.m_nBlockWidth=confFile.GetInt("m_nBlockWidth", "HOG", 2);
	hogParams.m_nBlockHeight=confFile.GetInt("m_nBlockHeight", "HOG", 2);
	hogParams.m_nCellWidth=confFile.GetInt("m_nCellWidth", "HOG", 8);
	hogParams.m_nCellHeight=confFile.GetInt("m_nCellHeight", "HOG", 8);
	hogParams.m_nBlockHorizontalOverlap=confFile.GetInt("m_nBlockHorizontalOverlap", "HOG", 8);
	hogParams.m_nBlockVerticalOverlap=confFile.GetInt("m_nBlockVerticalOverlap", "HOG", 8);

	//Ensure that overlap is a multiple of cellWidth/height
	hogParams.m_nBlockHorizontalOverlap = hogParams.m_nBlockHorizontalOverlap / hogParams.m_nCellWidth * hogParams.m_nCellWidth;
	hogParams.m_nBlockVerticalOverlap = hogParams.m_nBlockVerticalOverlap / hogParams.m_nCellHeight * hogParams.m_nCellHeight;

    hogParams.m_nHistogramBins=confFile.GetInt("m_nHistogramBins", "HOG", 9);
	hogParams.m_nHistogramRange=confFile.GetInt("m_nHistogramRange", "HOG", 180);;
	hogParams.m_nGaussianWindow=confFile.GetFloat("m_nGaussianWindow", "HOG", 0.5);
	hogParams.m_nGammaCompression=confFile.GetInt("m_nGammaCompression", "HOG", 1);
	hogParams.m_nNormType=confFile.GetInt("m_nNormType", "HOG", 2);
	hogParams.m_dClippingThres=confFile.GetFloat("m_dClippingThres", "HOG", 0.2);
	hogParams.m_dNormalizeEpsilon=confFile.GetFloat("m_dNormalizeEpsilon", "HOG", 0.1) * hogParams.m_nBlockWidth * hogParams.m_nBlockHeight * hogParams.m_nHistogramBins;
	
	// libHOG1
	/*hogParams.m_dNormalizeEpsilon=confFile.GetFloat("m_dNormalizeEpsilon", "HOG", 1.0) *
		hogParams.m_nBlockWidth * hogParams.m_nBlockHeight * hogParams.m_nCellWidth * hogParams.m_nCellHeight *
		(2 * hogParams.m_nCellWidth) * (2 * hogParams.m_nCellHeight) * 256 * hogParams.m_nHistogramRange/hogParams.m_nHistogramBins / 100;*/
	hogParams.m_dNormalizeEpsilonHys=confFile.GetFloat("m_dNormalizeEpsilonHys", "HOG", 0.1);
	

	cout << "Performing parameter calculation..." << endl;
	cout << "    Object part of the detector: " << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;
	//cout << "    Context of the detector: " << ppParams.localContextL << ", " << ppParams.localContextR << ", " << ppParams.localContextT << ", " << ppParams.localContextB << endl;

	//Determine width and height in blocks of full detector
	//hogParams.m_nWindowWidth += ppParams.localContextL + ppParams.localContextR;
	//hogParams.m_nWindowHeight += ppParams.localContextT + ppParams.localContextB;

	cout << "    Full size of detector: "  << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;

	hogParams.m_nHOGWidth= 1 + (hogParams.m_nWindowWidth - hogParams.m_nBlockWidth * hogParams.m_nCellWidth) / (hogParams.m_nBlockWidth * hogParams.m_nCellWidth - hogParams.m_nBlockHorizontalOverlap); 
	hogParams.m_nHOGHeight= 1 + (hogParams.m_nWindowHeight - hogParams.m_nBlockHeight * hogParams.m_nCellHeight) / (hogParams.m_nBlockHeight * hogParams.m_nCellHeight - hogParams.m_nBlockVerticalOverlap);

	cout << "    Dimension in blocks: "  << hogParams.m_nHOGWidth << "x" << hogParams.m_nHOGHeight << endl;

	// Ensure that there is no rounding involved and thus that window is centered in the end
	hogParams.m_nWindowWidth = hogParams.m_nHOGWidth * ( hogParams.m_nBlockWidth * hogParams.m_nCellWidth - hogParams.m_nBlockHorizontalOverlap) + hogParams.m_nBlockHorizontalOverlap;
	hogParams.m_nWindowHeight = hogParams.m_nHOGHeight * ( hogParams.m_nBlockHeight * hogParams.m_nCellHeight - hogParams.m_nBlockVerticalOverlap) + hogParams.m_nBlockVerticalOverlap;

	cout << "    Final window dimensions of detector: "  << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;	
}

HOG_Visitor::HOG_Visitor(const HOGParams& hog_parameters) : m_hogParams(hog_parameters), m_cg(0), m_cgr(0), m_bg(0) {
	
}

int HOG_Visitor::on_get_featureDim() const {
	return m_hogParams.m_nHOGHeight * m_hogParams.m_nHOGWidth * m_hogParams.m_nBlockWidth * m_hogParams.m_nBlockHeight * m_hogParams.m_nHistogramBins;
}

float* HOG_Visitor::on_get_FeatureVector(int posX, int posY, int width, int height, float* features) {
	
	int blockPosX = static_cast<int>(round(posX * 1.0f / m_StepSizeX));
	int blockPosY = static_cast<int>(round(posY * 1.0f / m_StepSizeY));
		
	if (blockPosX + m_hogParams.m_nHOGWidth > m_bg->m_nBlocksX || blockPosY + m_hogParams.m_nHOGHeight > m_bg->m_nBlocksY)
		return 0;
			
	m_bg->getHOGDescriptor(blockPosX, blockPosY, features);	
		
	return features + on_get_featureDim();
}


void HOG_Visitor::on_new_Scale(const unsigned char* img, int width, int height, int rowStep, float scale, float originAtFullScaleX, float originAtFullScaleY) {
	delete m_cg;
	delete m_bg;
        delete m_cgr;
	
	m_cg = new ColorGradient(img, width, height, rowStep, m_hogParams.m_nGammaCompression);
        m_cgr = new CellGrid(m_hogParams, *m_cg, 0, 0);
        m_bg = new BlockGrid(*m_cgr);
	//m_bg = new BlockGrid(*m_cg, m_hogParams);			
}

void HOG_Visitor::on_new_Image(const unsigned char* img, int width, int height, int rowStep) {
	m_StepSizeX = (m_hogParams.m_nBlockWidth * m_hogParams.m_nCellWidth) - m_hogParams.m_nBlockHorizontalOverlap;
	m_StepSizeY = (m_hogParams.m_nBlockHeight * m_hogParams.m_nCellHeight) -m_hogParams.m_nBlockVerticalOverlap;	
}

HOG_Visitor::~HOG_Visitor() {
	delete m_bg;
	delete m_cgr;
        delete m_cg;
}
