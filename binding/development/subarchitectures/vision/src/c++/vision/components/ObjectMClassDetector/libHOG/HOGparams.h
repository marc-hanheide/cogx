#ifndef HOGPARAMS_H_
#define HOGPARAMS_H_

#include <vector>
#include <iostream>

class HOGParams
{
  
public:
  HOGParams();
  ~HOGParams();
  HOGParams(int windowWidth, int windowHeight, int blockSize, int cellSize, int overlap);

  //Use this to create a filename from the paramSet
  void save(); 
  void save(const std::string& fileName); 
  
  void load(const std::string& fileName);
  void print();
  
public:
	int m_nWindowWidth;
	int m_nWindowHeight;
	int m_nHOGWidth; //not sure if this is so useful... (what if imagesize is different?) - maybe rather imagewidth
	int m_nHOGHeight;
	int m_nBlockWidth;
	int m_nBlockHeight;
	int m_nCellWidth;
	int m_nCellHeight;
	int m_nBlockHorizontalOverlap;
	int m_nBlockVerticalOverlap;
	    
	int m_nHistogramBins;
	int m_nHistogramRange;
	
	float m_nGaussianWindow;
	int m_nNormType;
	int m_nGammaCompression;
	float m_dClippingThres;
	float m_dNormalizeEpsilon;
	float m_dNormalizeEpsilonHys;
	float m_BlurSigma;
	
};

std::ostream& operator<<(std::ostream& output, HOGParams& hogp);
std::istream& operator>>(std::istream& input, HOGParams& hogp);

#endif /*HOGPARAMS_H_*/
