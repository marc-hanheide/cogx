#ifndef HOG_H_
#define HOG_H_

#include <libHOG/HOGparams.h>
#include <vector>
#include <cassert>
#include <iostream>
#include <math.h>

//#include <QtGui/QImage>
typedef float PIXEL; 

namespace HOG
{

  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  CellHistogram
  //
  ////////////////////////////////////////////////////////////////////////
  class CellHistogram
  {
    public:
  	  CellHistogram(const int newRange, const int newNoBins);
  	  CellHistogram();
  	  int size() const {return m_nNoBins;}
  
  	  float& operator[](int i) { assert(i>=0 && i<m_nNoBins); return values[i]; }
  	  const float& operator[] (int i) const {assert(i>=0 && i<m_nNoBins); return values[i];}
  
  	  void insert(float orientation, float magnitude);
  	  void insertInterpolated(float orientation, float magnitude);

  	  int save(const std::string& filename) const;
  
    public:
  	  int m_nRange;
  	  int m_nNoBins;
  	  int m_nBinSize; 
  	  std::vector<float> values;
  };
  std::ostream& operator<< (std::ostream& os, const CellHistogram& hist);
  //std::istream& operator>> (std::istream& is, CellHistogram& hist);
  
  
  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  ColorGradient
  //
  ////////////////////////////////////////////////////////////////////////
  class ColorGradient
  {
	  public:
		  //ColorGradient(const QImage& img, int compressionType, double sigma = 0, bool preComputeOrientation = true, std::ostream* outputChannel = &std::cout);
		  template <typename T> ColorGradient(const T* img, unsigned int width, unsigned int height, unsigned int rowstep, int compressionType, double sigma = 0, bool preComputeOrientation = true, std::ostream* outputChannel = 0);//&std::cout);
		   ~ColorGradient();

		   int width() const {return m_nGradientImageWidth;}
		   int height() const {return m_nGradientImageHeight;}
		   //const PIXEL* maxMagnitudes() {if (!m_redMagnitudes) computeMaxGradients(); return m_maxMagnitudes;}
		   //const PIXEL* maxOrientations() {if (!m_redMagnitudes) computeMaxGradients(); return m_maxOrientations;}
		   const PIXEL* maxMagnitudes() const { return m_maxMagnitudes;}
		   const PIXEL* maxOrientations() const { return m_maxOrientations;}
		   PIXEL* maxMagnitudeAt(float degree, int range);

	  private:
  	  	//void compute(const QImage& image, int compressionType);
  	  	  	  	
  	  	template<typename T> void compute(const T* image, unsigned int width, unsigned int height, unsigned int rowsize, int compressionType);
  	  	
		//void computeColorGradients(const QImage& image, int compressionType=0);
		template<typename T> void computeColorGradients(const T* image, unsigned int width, unsigned int height, unsigned int rowstep, int compressionType, int numChannels = 3, int c0 = 0, int c1 = 1, int c2 = 2);
  	  	
		void computeMaxGradients();
		   
	  private:
	  	std::vector<double> smoothingKernel;
	  
	  
		int m_nGradientImageWidth, m_nGradientImageHeight;

  	  	PIXEL* m_redVGradients,   *m_redHGradients,   *m_redMagnitudes;
  	  	PIXEL* m_greenVGradients, *m_greenHGradients, *m_greenMagnitudes;
  	  	PIXEL* m_blueVGradients,  *m_blueHGradients,  *m_blueMagnitudes;
  	  	PIXEL* m_maxOrientations, *m_maxMagnitudes;
  	  	
  	  	std::vector<PIXEL*> m_magnitudes;
  		bool m_preComputeOrientation;
  		
  		std::ostream* m_output;
  };

template <typename T>ColorGradient::ColorGradient(const T* img, unsigned int width, unsigned int height, unsigned int rowstep,
		int compressionType, double sigma, bool preComputeOrientation, std::ostream* outputChannel) :
	m_redVGradients   (0),
	m_redHGradients   (0),
	m_redMagnitudes   (0),
	m_greenVGradients (0),
	m_greenHGradients (0),
	m_greenMagnitudes (0),
	m_blueVGradients  (0),
	m_blueHGradients  (0),
	m_blueMagnitudes  (0),
	m_maxOrientations (0),
	m_maxMagnitudes   (0),
	m_preComputeOrientation(preComputeOrientation),
	m_output(outputChannel)
{
	if(sigma == 0) {
		smoothingKernel.resize(0);
		compute<T>(img, width, height, rowstep, compressionType);
	} else {
		T* blurred = new T[width * height * 3];
				
		const int kernelWidth = static_cast<int>(ceil(3 * sigma));
		smoothingKernel.resize(2 * kernelWidth + 1);
		
		double nf = 0;
		for(int i = -kernelWidth; i <= kernelWidth; i++) {
			smoothingKernel[i + kernelWidth] = 1.0f / (sqrt(2 * M_PI) * sigma) * exp(- (i)*(i) * 1.0f / (2 * sigma * sigma));
			nf += smoothingKernel[i + kernelWidth];		
		}
	
		for(int i = -kernelWidth; i <= kernelWidth; i++) {
			smoothingKernel[i + kernelWidth] /= nf;
			//std::cout << i << ": " << smoothingKernel[i + kernelWidth] << "\t";
		}
		
		//Blur in X direction
		for(unsigned int y = 0; y < height; y++)
			for(unsigned int x = 0; x < width; x++) {
				
				//Red Channel
				unsigned char newRed = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newRed += static_cast<unsigned char>( img[(((x + k + width) % width) + y * rowstep) * 3] * smoothingKernel[kernelWidth + k] );
					
				//Green Channel
				unsigned char newGreen = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newGreen += static_cast<unsigned char>( img[(((x + k + width) % width) + y * rowstep) * 3 + 1] * smoothingKernel[kernelWidth + k] );
				
				//Blue Channel
				unsigned char newBlue = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newBlue += static_cast<unsigned char>( img[(((x + k + width) % width) + y * rowstep) * 3 + 2] * smoothingKernel[kernelWidth + k] );
				
				//std::cout << "R: " << (int)newRed << " G: " << (int)newGreen << " B: " << (int)newBlue << std::endl;
				blurred[y*width+x] = newRed;
				blurred[y*width+x+1] = newGreen;
				blurred[y*width+x+2] = newBlue;
			}
			
		//Blur in y direction
		for(unsigned int y = 0; y < height; y++)
			for(unsigned int x = 0; x < width; x++) {
				
				//Red Channel
				unsigned char newRed = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newRed +=  static_cast<unsigned char>( blurred[(x + ((y + k + height) % height * rowstep)) * 3]  * smoothingKernel[kernelWidth + k] );
					
				//Green Channel
				unsigned char newGreen = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newGreen += static_cast<unsigned char>( blurred[(x + ((y + k + height) % height * rowstep)) * 3 + 1]  * smoothingKernel[kernelWidth + k] );
				
				//Blue Channel
				unsigned char newBlue = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newBlue += static_cast<unsigned char>( blurred[(x + ((y + k + height) % height * rowstep)) * 3 + 2]  * smoothingKernel[kernelWidth + k] );
				
				//std::cout << "R: " << (int)newRed << " G: " << (int)newGreen << " B: " << (int)newBlue << std::endl;
				blurred[y*width+x] = newRed;
				blurred[y*width+x+1] = newGreen;
				blurred[y*width+x+2] = newBlue;
			}
		
		//std::cout << " done";		
		compute<T>(blurred, width, height, width, compressionType);
		delete[] blurred;	
	}	
}

template<typename T> void ColorGradient::compute(const T* image, unsigned int width, unsigned int height, unsigned int rowstep, int compressionType) 
{
	if (m_output)
		*m_output << "    Computing cell Grid for images size: " << width << "x" << height << " ..." << std::endl;
	
	m_nGradientImageWidth = width;
	m_nGradientImageHeight = height;

	if (m_output) 
		*m_output << "       color gradients ... " << std::endl;
	
	computeColorGradients<T>(image, width, height, rowstep, compressionType);
	if (m_output)
		*m_output << "       max gradients ... " << std::endl;
	
	if (m_preComputeOrientation)
		computeMaxGradients();	
}

template<typename T> void ColorGradient::computeColorGradients(const T* image, unsigned int width, unsigned int height, unsigned int rowstep, int compressionType, int numChannels, int c0, int c1, int c2) {
	//--- store gradients in image (integer array) ---//
	// TODO: Shall we we check if there are previous gradients? (otherwise it leaks)

	const unsigned int w = width;
	const unsigned int h = height;
	const unsigned int imgsize=w*h;
	
	m_redVGradients   = new PIXEL[imgsize*6];
	m_redHGradients   = m_redVGradients+imgsize;
	m_greenVGradients = m_redHGradients+imgsize;
	m_greenHGradients = m_greenVGradients+imgsize;
	m_blueVGradients  = m_greenHGradients+imgsize;
	m_blueHGradients  = m_blueVGradients+imgsize;

	//--- define a pointer to the gradie 
	PIXEL* currentRedHGradient = m_redHGradients;
	PIXEL* currentRedVGradient = m_redVGradients;
	PIXEL* currentGreenHGradient = m_greenHGradients;
	PIXEL* currentGreenVGradient = m_greenVGradients;
	PIXEL* currentBlueHGradient = m_blueHGradients;
	PIXEL* currentBlueVGradient = m_blueVGradients;
	
	T maxVal = std::numeric_limits<T>::max();

	switch (compressionType) {
		case 0:
			// Uncompressed
			for (unsigned int y=0; y<height; ++y )	{	
				for (unsigned int x=0; x<width ; ++x)	{
					
					const T* center = static_cast<const T*>(static_cast<const unsigned char*>(image) + y * rowstep);
					const T* upper =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y - 1) * rowstep);
					const T* lower =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y + 1) * rowstep);
					
					// Image coordinates start from the top left corner
					// We want postive gradients to head right/top
					if (x==0 || y==0 || x==w-1 || y==h-1)
					{
						*currentRedHGradient 	= 0;
						*currentRedVGradient 	= 0;
						*currentGreenHGradient 	= 0;
						*currentGreenVGradient 	= 0;
						*currentBlueHGradient 	= 0;
						*currentBlueVGradient 	= 0;
					}
					else
					{					
						*currentRedHGradient 	= (center[(x+1) * numChannels + c0] - center[(x-1) * numChannels + c0]) * 1.0  / maxVal;
						*currentRedVGradient 	= (lower[x * numChannels + c0] - upper[x * numChannels + c0]) * 1.0 / maxVal;
						*currentGreenHGradient 	= (center[(x+1) * numChannels + c1] - center[(x-1) * numChannels + c1]) * 1.0 / maxVal;
						*currentGreenVGradient 	= (lower[x * numChannels + c1] - upper[x * numChannels + c1]) * 1.0 / maxVal;
						*currentBlueHGradient 	= (center[(x+1) * numChannels + c2] - center[(x-1) * numChannels + c2]) * 1.0 / maxVal;
						*currentBlueVGradient 	= (lower[x * numChannels + c2] - upper[x * numChannels + c2]) * 1.0 / maxVal;
					}
			
					++currentRedHGradient;
					++currentRedVGradient;
					++currentGreenHGradient;
					++currentGreenVGradient;
					++currentBlueHGradient; 
					++currentBlueVGradient;
				}
			}
			break;
		case 1:
			//sqrt RGB compression
			for (unsigned int y=0; y<height; ++y )	{	
				for (unsigned int x=0; x<width ; ++x)	{
					
					const T* center = static_cast<const T*>(static_cast<const unsigned char*>(image) + y * rowstep);
					const T* upper =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y - 1) * rowstep);
					const T* lower =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y + 1) * rowstep);
					
					// Image coordinates start from the top left corner
					// We want postive gradients to head right/top
					if (x==0 || y==0 || x==w-1 || y==h-1)
					{
						*currentRedHGradient 	= 0;
						*currentRedVGradient 	= 0;
						*currentGreenHGradient 	= 0;
						*currentGreenVGradient 	= 0;
						*currentBlueHGradient 	= 0;
						*currentBlueVGradient 	= 0;
					}
					else
					{
						*currentRedHGradient 	= static_cast<PIXEL>(sqrt(center[(x+1) * numChannels + c0] * 1.0 / maxVal) - sqrt(center[(x-1) * numChannels + c0] * 1.0 / maxVal));
						*currentRedVGradient 	= static_cast<PIXEL>(sqrt(lower[x * numChannels + c0] * 1.0 / maxVal) - sqrt(upper[x * numChannels + c0] * 1.0 / maxVal));
						*currentGreenHGradient 	= static_cast<PIXEL>(sqrt(center[(x+1) * numChannels + c1] * 1.0 / maxVal ) - sqrt(center[(x-1) * numChannels + c1] * 1.0 / maxVal));
						*currentGreenVGradient 	= static_cast<PIXEL>(sqrt(lower[x * numChannels + c1] * 1.0 / maxVal ) - sqrt(upper[x * numChannels + c1] * 1.0 / maxVal));
						*currentBlueHGradient 	= static_cast<PIXEL>(sqrt(center[(x+1) * numChannels + c2] * 1.0 / maxVal ) - sqrt(center[(x-1) * numChannels + c2] * 1.0 / maxVal));
						*currentBlueVGradient 	= static_cast<PIXEL>(sqrt(lower[x * numChannels + c2] * 1.0 / maxVal) - sqrt(upper[x * numChannels + c2] * 1.0 / maxVal));	
					}
			
					++currentRedHGradient;
					++currentRedVGradient;
					++currentGreenHGradient;
					++currentGreenVGradient;
					++currentBlueHGradient; 
					++currentBlueVGradient;
				}
			}
			break;
		case 2:
			//log RGB compression 
			for (unsigned int y=0; y<height; ++y )	{	
				for (unsigned int x=0; x<width; ++x)	{
					
					const T* center = static_cast<const T*>(static_cast<const unsigned char*>(image) + y * rowstep);
					const T* upper =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y - 1) * rowstep);
					const T* lower =  static_cast<const T*>(static_cast<const unsigned char*>(image) + (y + 1) * rowstep);
										
					// Image coordinates start from the top left corner
					// We want postive gradients to head right/top
					if (x==0 || y==0 || x==w-1 || y==h-1)
					{
						*currentRedHGradient 	= 0;
						*currentRedVGradient 	= 0;
						*currentGreenHGradient 	= 0;
						*currentGreenVGradient 	= 0;
						*currentBlueHGradient 	= 0;
						*currentBlueVGradient 	= 0;
					}
					else
					{
						*currentRedHGradient 	= static_cast<PIXEL>(log(center[(x+1) * numChannels + c0] * 1.0 / maxVal) - log(center[(x-1) * numChannels + c0] * 1.0 / maxVal));
						*currentRedVGradient 	= static_cast<PIXEL>(log(lower[x * numChannels + c0] * 1.0 / maxVal) - log(upper[x * numChannels + c0] * 1.0 / maxVal));
						*currentGreenHGradient 	= static_cast<PIXEL>(log(center[(x+1) * numChannels + c1] * 1.0 / maxVal) - log(center[(x-1) * numChannels + c1] * 1.0 / maxVal));
						*currentGreenVGradient 	= static_cast<PIXEL>(log(lower[x * numChannels + c1] * 1.0 / maxVal) - log(upper[x * numChannels + c1] * 1.0 / maxVal));
						*currentBlueHGradient 	= static_cast<PIXEL>(log(center[(x+1) * numChannels + c2] * 1.0 / maxVal) - log(center[(x-1) * numChannels + c2] * 1.0 / maxVal));
						*currentBlueVGradient 	= static_cast<PIXEL>(log(lower[x * numChannels + c2] * 1.0 / maxVal) - log(upper[x * numChannels + c2] * 1.0 / maxVal));	
					}
			
					++currentRedHGradient;
					++currentRedVGradient;
					++currentGreenHGradient;
					++currentGreenVGradient;
					++currentBlueHGradient; 
					++currentBlueVGradient;
				}
			}
			break;
	}

}
  
  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  CellGrid
  //
  ////////////////////////////////////////////////////////////////////////
  class CellGrid
  {
    public:
  	  CellGrid(const HOGParams& params, const ColorGradient& gradients, unsigned int x=0, unsigned int y=0);
  	  CellGrid(const HOGParams& params, const ColorGradient& gradients, unsigned int x, unsigned int y, unsigned int width, unsigned int height, const float* spatialMask=0);
	  ~CellGrid();
  
  	  CellHistogram& operator[](int i) {assert(i>=0 && i<m_nCellsX*m_nCellsY); return m_vCells[i];}
  	  const CellHistogram& operator[] (int i) const {assert(i>=0 && i<m_nCellsX*m_nCellsY); return m_vCells[i];}
  
  	  CellHistogram& operator()(int x, int y) {assert(x>=0 && x<m_nCellsX);assert(y>=0 && y<m_nCellsY); return m_vCells[x+y*m_nCellsX];}
  	  const CellHistogram& operator() (int x, int y) const {assert(x>=0 && x<m_nCellsX);assert(y>=0 && y<m_nCellsY); return m_vCells[x+y*m_nCellsX];}
  	  
  	  int save(const std::string& filename) const;
  
    private:
  	  void computeCells(const ColorGradient& gradients, unsigned int x, unsigned int y, unsigned int width, unsigned int height, const float* spatialMask=0);
  	   	
    public:
  	  int m_nCellsX, m_nCellsY;
  	  int m_xBorder, m_yBorder;
  	  HOGParams m_params;
  	  std::vector<CellHistogram> m_vCells;
  	  void printCellGridSparseFormat(std::ostream& out);
			void getCellGridSparseFormat(std::vector<float>& features);
			void getCellGridSparseFormatWordNonWord(std::vector<float>& features, float threshold);

	private:	
	  void computeFromGradients(PIXEL* maxOrientation, PIXEL* maxMagnitude, const int width, const int height);	
  };
  std::ostream& operator<< (std::ostream& os, const CellGrid& grid);
  
 

  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  Block
  //
  ////////////////////////////////////////////////////////////////////////
  class Block
  { 
  	
    public:
	  //Block() : m_params(0) {};
	  Block(const HOGParams& params) : m_params(&params) {};  
	  Block(const CellGrid& cellGrid, int x, int y);
  	  Block(const ColorGradient& colorgradient, const HOGParams& params, int x, int y, float* spatialMask = 0);
	    	  
  	  void computeFromCellGrid(const CellGrid& cellGrid, int x, int y);  	 
  	   	  
  	  void normalize();
  	  void normalize(int normtype);
  
  	  int save(const std::string& filename) const;
 
	public:
  	  const HOGParams* m_params;
  	  
  	  std::vector<float> m_vValues;
  	  
  	  //std::vector<CellHistogram> cellHistograms;
  
  	  //const CellHistogram& operator()(int i, int j) const {assert((i+j*m_params.m_nBlockWidth)>=0 && (i+j*m_params.m_nBlockWidth)<(int)cellHistograms.size()); return cellHistograms[i+j*m_params.m_nBlockWidth];}
  	  //CellHistogram& operator()(int i, int j) {assert((i+j*m_params.m_nBlockWidth)>=0 && (i+j*m_params.m_nBlockWidth)<(int)cellHistograms.size()); return cellHistograms[i+j*m_params.m_nBlockWidth];}
  	  //unsigned noCells() {return cellHistograms.size();}
 	private:
 		friend class BlockGrid; 		  
 		void computeFromGradients(const ColorGradient& colorgradients, const HOGParams& params, int x, int y, float* spatialMask = 0);
  	  
  };
  std::ostream& operator<< (std::ostream& os, const Block& block);
  
  
  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  BlockGrid
  //
  ////////////////////////////////////////////////////////////////////////
  class BlockGrid
  {
  
    public:
      BlockGrid(HOGParams& params) : m_params(params), m_spatialMask(0) { }
  	  BlockGrid(const CellGrid& cellGrid);
          BlockGrid(const CellGrid& cellGrid, const HOGParams& params);
  	  BlockGrid(ColorGradient& colorgradients, const HOGParams& params, unsigned int offsetX = 0, unsigned int offsetY = 0, int widthX = -1, int heightY = -1);
  	  ~BlockGrid();

	  int computeFromCellGrid(const CellGrid& cellGrid);
	  int computeFromGradients(ColorGradient& colorgradients, unsigned int offsetX = 0, unsigned int offsetY = 0, int widthX = -1, int heightY = -1);
  	   
	  void getHOGDescriptor(const int i, const int j, std::vector<float>& values);
	  void getHOGDescriptor(const int i, const int j, float* values); // this assumes that there is enough preallocated space at values
	  /*! \brief Values of a given block
	   * No copying is done at all. This is the most efficient way to get the data.
	   * If hogParam.m_nHOGHeight and hogParam.m_nHOGWidth are 1, this method is equivalent to
	   * getHOGDescriptor, but without copying the data.
	   */
	  const float* getHOGBlock(size_t i, size_t j) const { return &(m_vBlocks[j * m_nBlocksX +i].m_vValues[0]); }

	  //void drawHOGDescriptor(const int istart, const int jstart, QImage &img, const int blockDisplayWidth = 25, const int blockDisplayHeight = 25, bool antialias = true);
	  int getHOGDescriptorLen();
  	  int saveDescriptorSVM(const std::string& fileName, const int i, const int j, const int label);
  	  int save(const std::string& filename) const;
 	
	public:
  	  HOGParams m_params;
  	  std::vector<Block> m_vBlocks;
  	  int m_nBlocksX;
  	  int m_nBlocksY;
  	  int m_stepSizeX;
  	  int m_stepSizeY;
  	  float* m_spatialMask;
  		
  	  //const Block& Block(int i) {assert(i>=0 && i<(int)Blocks.size()); return Blocks[i];}
  	  //const Block& operator() (int i,int j) const { return Blocks[i+j*m_params.m_nHOGWidth];}
  	  //Block& operator()(int i,int j){ return Blocks[i+j*m_params.m_nHOGWidth];}
  };
  std::ostream& operator<< (std::ostream& os, const BlockGrid& grid);
 
  
  ////////////////////////////////////////////////////////////////////////
  //
  //
  //  Visualization / Image Conversion
  //
  ////////////////////////////////////////////////////////////////////////
 /* PIXEL* QImageToPixmap(QImage &img, unsigned& width, unsigned& height, unsigned& nChannels, bool preserveAlpha = false);
 void getQImageFromPixmap(PIXEL* img, QImage& qImage, unsigned width, unsigned height, unsigned nChannels= 3);
 void drawCell(const CellHistogram& cell, QImage& img, const int xoffset, const int yoffset, const int cellDisplayWidth, const int cellDisplayHeight, const bool antialias = true, const float clipValue = 1.0);
 void drawCellGrid(const CellGrid& cg, QImage& img, const int cellDisplayWidth, const int cellDisplayHeight, const bool antialias= true);
 void drawBlockGrid(const BlockGrid& bg, QImage& img, const int blockDisplayWidth, const int blockDisplayHeight, const bool antialias = true);
 void drawBlock(const Block& block, QImage& img, const int xoffset, const int yoffset, const int blockDisplayWidth, const int blockDisplayHeight, const float clipValue, const bool antialias = true);
 void visualizeFV(const std::vector<float>& fv, QImage& img, const HOGParams& params, const int blockDisplayWidth, const int blockDisplayHeight, const bool antialias = true);
  */	
};

#endif /*HOG_H_*/
