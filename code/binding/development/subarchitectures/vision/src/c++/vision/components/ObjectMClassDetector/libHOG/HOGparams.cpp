#include <fstream>
#include <sstream>
#include <libHOG/HOGparams.h>
#include <cassert>

using namespace std;

HOGParams::HOGParams()
{  
	m_nWindowWidth=64;
	m_nWindowHeight=128;
	
	m_nHOGWidth=7;//in blocks
	m_nHOGHeight=15;
	
	m_nBlockWidth=2;//in Cells
	m_nBlockHeight=2;
	
	m_nCellWidth=8;//in pixels
	m_nCellHeight=8;
	
	m_nBlockHorizontalOverlap=8; //in pixels
	m_nBlockVerticalOverlap=8;
    
	m_nHistogramBins=9;
	m_nHistogramRange=180;
	
	m_nGaussianWindow=0.5;
	m_nNormType=2;
	
	m_nGammaCompression = 0;	
	m_dClippingThres = 0.2;
	m_dNormalizeEpsilon = 10000;
	m_dNormalizeEpsilonHys = 0.1;
	m_BlurSigma = 0.0;
}

//this is to make creation of new parametersets easier... 
HOGParams::HOGParams(int windowWidth, int windowHeight, int blockSize, int cellSize, int blockOverlap)
{  
	m_nWindowWidth=windowWidth; //in pixels
	m_nWindowHeight=windowHeight;

	m_nBlockWidth=blockSize;//in Cells
	m_nBlockHeight=blockSize;
	
	m_nCellWidth=cellSize;//in pixels
	m_nCellHeight=cellSize;
	
	m_nBlockHorizontalOverlap=blockOverlap; //in pixels
	m_nBlockVerticalOverlap=blockOverlap;
	
	m_nHOGWidth=(windowWidth-blockOverlap)/(blockSize*cellSize-blockOverlap);//in blocks
	m_nHOGHeight=(windowHeight-blockOverlap)/(blockSize*cellSize-blockOverlap);//in blocks
	
	//eg: (64-8)/(2*8-8)=56/8=7
	//eg:(128-8)/(2*8-8)=120/8=15
	//if it doesn't fit completely, int division should make sure it fits inside

	// rest just default for now,     
	m_nHistogramBins=9;
	m_nHistogramRange=180;
	
	m_nGaussianWindow=0.5;
	m_nNormType=2;
	
	m_nGammaCompression = 0;
	m_dClippingThres = 0.2;
	m_dNormalizeEpsilon = 10000;
	m_dNormalizeEpsilonHys = 0.1;
	
	m_BlurSigma = 0.0;
	
}


HOGParams::~HOGParams()
{
}

void HOGParams::print()
{
  std::cout << "\nHOG Parameters:\n\n"
  
  << "Window width in px: " << m_nWindowWidth << "\n"
  << "Window height in px: " << m_nWindowHeight << "\n"
  << "HOG width in HOGs: " << m_nHOGWidth << "\n"
  << "HOG height in HOGs: " << m_nHOGHeight << "\n"
  << "Block width in Cells: " << m_nBlockWidth << "\n"
  << "Block height in Cells: " << m_nBlockHeight << "\n"
  << "Cell Width: " << m_nCellWidth << "\n"
  << "Cell height: " << m_nCellHeight << "\n"
  << "Horizontal Overlap: " << m_nBlockHorizontalOverlap << "\n"
  << "Vertical Overlap: " << m_nBlockVerticalOverlap << "\n\n"
  
  << "No of Histogram Bins/Cell: " << m_nHistogramBins << "\n"
  << "Histogram Range: " << m_nHistogramRange << "\n"
  
  << "Gaussian Spatial Window: " << m_nGaussianWindow <<"\n"   
  << "Normalization NormType: " << m_nNormType <<"\n"
  
  << "Gamma compression: " << m_nGammaCompression << "\n"
  << "Normalization clipping Threshold: " << m_dClippingThres << "\n"
  << "Normalization epsilon: " << m_dNormalizeEpsilon << "\n" 
  << "Normalization epsilon Hysteresis: " << m_dNormalizeEpsilonHys << "\n"
  << "Image blurring sigma: " << m_BlurSigma << "\n";  
}

void HOGParams::save()
{
	std::ostringstream oss;
	oss << m_nWindowWidth <<"x"
	<< m_nWindowHeight <<"-"
	<< m_nHOGWidth <<"x"
	<< m_nHOGHeight <<"X"
	<< m_nBlockWidth<<"x"
	<< m_nBlockHeight<<"X"
	<< m_nCellWidth<<"x"
	<< m_nCellHeight<<"-"
	<< m_nBlockHorizontalOverlap<<"x"
	<< m_nBlockVerticalOverlap<<"_"
	<< m_nHistogramBins<<"_"
	<< m_nHistogramRange<<"-G"
	<< m_nGaussianWindow<<"-L"
	<< m_nNormType << "_"
	<< m_nGammaCompression << "_" 
	<< m_dClippingThres << "_"
	<< m_dNormalizeEpsilon << "_"
	<< m_dNormalizeEpsilonHys << "_"
	<< m_BlurSigma;
	
	std::string filename=oss.str();
	this->save(filename);
}


void HOGParams::save(const std::string& fileName)
{	
	std::ofstream file(fileName.c_str());
	if ( file.is_open() )
	{
		file << (*this);
		file.close();
	}
	else std::cerr << "HOGParams.save(): Could not open File: " << fileName << "\n";  
}

ostream& operator<<(ostream& output, HOGParams& hogp) {
	output << "m_nWindowWidth: " << hogp.m_nWindowWidth << "\n"
		<< "m_nWindowHeight: " << hogp.m_nWindowHeight << "\n"
		<< "m_nHOGWidth: " << hogp.m_nHOGWidth << "\n"
		<< "m_nHOGHeight: " << hogp.m_nHOGHeight << "\n"
		<< "m_nBlockWidth: "<< hogp.m_nBlockWidth << "\n"
		<< "m_nBlockHeight: " << hogp.m_nBlockHeight << "\n"
		<< "m_nCellWidth: " << hogp.m_nCellWidth << "\n"
		<< "m_nCellHeight: " << hogp.m_nCellHeight << "\n"
		<< "m_nBlockHorizontalOverlap: " << hogp.m_nBlockHorizontalOverlap << "\n"
		<< "m_nBlockVerticalOverlap: " << hogp.m_nBlockVerticalOverlap << "\n"

		<< "m_nHistogramBins: " << hogp.m_nHistogramBins << "\n"
		<< "m_nHistogramRange: " << hogp.m_nHistogramRange << "\n"
		
		<< "m_nGaussianWindow: " << hogp.m_nGaussianWindow <<"\n"		
		<< "m_nNormType: " << hogp.m_nNormType<<"\n"
		
		<< "m_nGammaCompression: " << hogp.m_nGammaCompression << "\n"
		<< "m_dClippingThres: " <<  hogp.m_dClippingThres << "\n"
  		<< "m_dNormalizeEpsilon: " <<  hogp.m_dNormalizeEpsilon << "\n"  
  		<< "m_dNormalizeEpsilonHys: " <<  hogp.m_dNormalizeEpsilonHys << "\n"
  		<< "m_BlurSigma: " << hogp.m_BlurSigma << "\n";
	return output;
}

istream& operator>>(istream& input, HOGParams& hogp) {
	std::string name; //for now no checking the variablenames
	input >> name >> hogp.m_nWindowWidth
		>> name >> hogp.m_nWindowHeight
		>> name >> hogp.m_nHOGWidth
		>> name >> hogp.m_nHOGHeight
		>> name >> hogp.m_nBlockWidth
		>> name >> hogp.m_nBlockHeight
		>> name >> hogp.m_nCellWidth
		>> name >> hogp.m_nCellHeight
		>> name >> hogp.m_nBlockHorizontalOverlap
		>> name >> hogp.m_nBlockVerticalOverlap
		
		>> name >> hogp.m_nHistogramBins
		>> name >> hogp.m_nHistogramRange
		
		>> name >> hogp.m_nGaussianWindow
		>> name >> hogp.m_nNormType
		>> name >> hogp.m_nGammaCompression
		>> name >> hogp.m_dClippingThres
		>> name >> hogp.m_dNormalizeEpsilon
		>> name >> hogp.m_dNormalizeEpsilonHys
		>> name >> hogp.m_BlurSigma;
			
		//Make sure window width is not too small (consistency of parameters!)	
		int tightWindowWidth = hogp.m_nHOGWidth * (hogp.m_nBlockWidth * hogp.m_nCellWidth - hogp.m_nBlockHorizontalOverlap) + hogp.m_nBlockHorizontalOverlap;
		int tightWindowHeight = hogp.m_nHOGHeight * (hogp.m_nBlockHeight * hogp.m_nCellHeight - hogp.m_nBlockVerticalOverlap) + hogp.m_nBlockVerticalOverlap;
		assert(tightWindowWidth <= hogp.m_nWindowWidth);
		assert(tightWindowHeight <= hogp.m_nWindowHeight);

		if (hogp.m_nWindowWidth - tightWindowWidth >=hogp.m_nBlockWidth * hogp.m_nCellWidth - hogp.m_nBlockHorizontalOverlap)
			std::cout << "HOGParams.load(): Could have fit at least one more Block Horizontally\n";

		if (hogp.m_nWindowHeight - tightWindowHeight >= hogp.m_nBlockHeight * hogp.m_nCellHeight - hogp.m_nBlockVerticalOverlap)
			std::cout << "HOGParams.load(): Could have fit at least one more Block Vertically\n";
	return input;
}
			


void HOGParams::load(const std::string& filename)
{
	std::ifstream file(filename.c_str());
	if ( file.is_open() )
	{		
		file >> (*this);
		file.close();
	}
	else std::cerr << "HOGParams.load(): Could not open File: " << filename << "\n";
}


