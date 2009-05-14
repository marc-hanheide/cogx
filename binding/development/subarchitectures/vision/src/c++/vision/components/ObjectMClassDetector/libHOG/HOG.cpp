#include <libHOG/HOG.h>
#define describe(x) (" " #x ": ") << x
#include <cmath>
#include <cassert>
#include <string>
#include <fstream>
#include <numeric> //for accumulate()
#//include <QPainter>

#include <sys/timeb.h>
#include <time.h>

//for debugging
#include <iomanip>

inline float modulo(float c, int a) {
  float z;
  int b = static_cast<int>(c / a);
  z = c - ( b * a );
  return z;
}

using namespace HOG;

//PIXEL* QImageToPixmap(QImage &img, unsigned int& width, unsigned int& height, unsigned int& nChannels, bool preserveAlpha) 
//{
//	
//	width = img.width();
//	height = img.height();
//
//	nChannels = 1;
//	if (img.format() == QImage::Format_RGB32 || 
//		img.format() == QImage::Format_ARGB32 ||
//		img.format() == QImage::Format_ARGB32_Premultiplied)  {
//		nChannels = 3;
//		if (preserveAlpha) nChannels++;
//	}
//	
//	PIXEL* pixelData = new PIXEL[width * height * nChannels];
//	
//	//	if (*nChannels == 1) {
//	//		for (unsigned int i= 0; i < *height; i++) {
//	//			unsigned char* lineData = img.scanLine(i);
//	//			memcpy(&pixelData[i * *width * *nChannels], lineData, *width * *nChannels);
//	//		} 
//	//	} else {
//
//	for (unsigned int i= 0; i < height; ++i) 
//	{
//		for(unsigned int j = 0; j < width; ++j) 
//		{
//			QRgb col = img.pixel(j,i);
//			pixelData[(i*width+j) * nChannels + 0] = static_cast<PIXEL>(qRed(col));
//			pixelData[(i*width+j) * nChannels + 1] =  static_cast<PIXEL>(qGreen(col));
//			pixelData[(i*width+j) * nChannels + 2] =  static_cast<PIXEL>(qBlue(col));
//			if (nChannels == 4)
//				pixelData[(i * width + j)* nChannels + 3] = static_cast<PIXEL>(qAlpha(col));
//		}
//	} 	
//	
//	return pixelData;	
//	
//}

////Converts to color only so far
/*void HOG::getQImageFromPixmap(PIXEL *img, QImage& qImage, unsigned width, unsigned height, unsigned nChannels) 
{
	unsigned char *shiftedImage = new unsigned char[width * height * 4];
		
	unsigned int size = width * height;
	for(unsigned int i = 0; i < size; ++i) 
	{				
		if (nChannels>=3)
		{
			shiftedImage[i * 4] =     (unsigned char) img[i * nChannels + 2];
			shiftedImage[i * 4 + 1] = (unsigned char) img[i * nChannels + 1];
			shiftedImage[i * 4 + 2] = (unsigned char) img[i * nChannels];
		}
		else if (nChannels==1)
		{
			shiftedImage[i * 4] = (unsigned char) img[i];
			shiftedImage[i * 4 + 1] = (unsigned char) img[i];
			shiftedImage[i * 4 + 2] = (unsigned char) img[i];
		}
		else
			std::cerr << "ERROR: 2 color channels not supported\n";
		
		if (nChannels == 3)
			shiftedImage[i * 4 + 3] = 255;
		else if (nChannels == 4)
			shiftedImage[i * 4 + 3] = (unsigned char)img[i * nChannels + 3];
				
	}
	
	//QImage *qImage;
    qImage = QImage(shiftedImage, width, height, QImage::Format_RGB32);   
} */


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// Visualization
//
//
/*
void HOG::drawBlockGrid(const BlockGrid& bg, QImage& img, const int blockDisplayWidth, const int blockDisplayHeight, const bool antialias) {
	img = QImage(bg.m_nBlocksX*blockDisplayWidth, bg.m_nBlocksY*blockDisplayHeight, QImage::Format_RGB32);
	img.fill(Qt::black);
	unsigned int counter=0;
	
	float clipValue = 0.0;
	for (int y=0; y<bg.m_nBlocksY; ++y)
		for (int x=0; x<bg.m_nBlocksX; ++x, ++counter)
			for(size_t i = 0; i < bg.m_vBlocks[counter].m_vValues.size(); ++i) {
			
				if(bg.m_vBlocks[counter].m_vValues[i] > clipValue)
						clipValue = bg.m_vBlocks[counter].m_vValues[i];
	}
	
	counter = 0;
	for (int y=0; y<bg.m_nBlocksY; ++y)
		for (int x=0; x<bg.m_nBlocksX; ++x)
		{
			drawBlock( bg.m_vBlocks[counter], img, x*blockDisplayWidth, y*blockDisplayHeight, blockDisplayWidth, blockDisplayHeight, clipValue, antialias);
			++counter;
		}	
}

void HOG::drawBlock(const Block& block, QImage& img, const int xoffset, const int yoffset, const int blockDisplayWidth, const int blockDisplayHeight, const float clipValue, const bool antialias)
{
	CellHistogram cumulated(block.m_params->m_nHistogramRange, block.m_params->m_nHistogramBins);
	for(unsigned int i = 0; i < block.m_vValues.size(); i++)
		cumulated[i % block.m_params->m_nHistogramBins] += block.m_vValues[i];
		
	HOG::drawCell(cumulated, img, xoffset, yoffset, blockDisplayWidth, blockDisplayHeight, antialias, clipValue *  block.m_vValues.size()/block.m_params->m_nHistogramBins);
}

void HOG::visualizeFV(const std::vector<float>& fv, QImage& img, const HOGParams& params, const int blockDisplayWidth, const int blockDisplayHeight, const bool antialias) {
	img = QImage(params.m_nHOGWidth*blockDisplayWidth, params.m_nHOGHeight*blockDisplayHeight, QImage::Format_RGB32);
	img.fill(Qt::black);
	unsigned int counter=0;
	
	float clipValue = 0.0;
	for(size_t i = 0; i < fv.size(); ++i) {			
		if(fv[i] > clipValue)
			clipValue = fv[i];
			++counter;
	}
	
	counter = 0;
	
	for (int y=0; y<params.m_nHOGHeight; ++y)
		for (int x=0; x<params.m_nHOGWidth; ++x)
		{
			CellHistogram hist(params.m_nHistogramRange, params.m_nHistogramBins);
			for(int z = 0; z <  params.m_nHistogramBins; z++)
				hist.values[z] = fv[y * params.m_nHOGWidth *  params.m_nHistogramBins + x *  params.m_nHistogramBins];
			drawCell( hist, img, x*blockDisplayWidth, y*blockDisplayHeight, blockDisplayWidth, blockDisplayHeight, antialias, clipValue);
			++counter;
		}
}



void HOG::drawCellGrid(const CellGrid& cg, QImage& img, const int cellDisplayWidth, const int cellDisplayHeight, const bool antialias)
{
	img = QImage(cg.m_nCellsX*cellDisplayWidth, cg.m_nCellsY*cellDisplayHeight, QImage::Format_RGB32);
	img.fill(Qt::black);
	unsigned int counter=0;
	for (int y=0; y<cg.m_nCellsY; ++y)
		for (int x=0; x<cg.m_nCellsX; ++x)
		{
			drawCell( cg.m_vCells[counter], img, x*cellDisplayWidth, y*cellDisplayHeight, cellDisplayWidth, cellDisplayHeight, antialias);
			++counter;
		}
}

void HOG::drawCell(const CellHistogram& cell, QImage& img, const int xoffset, const int yoffset, const int cellDisplayWidth, const int cellDisplayHeight, const bool antialias, const float clipValue)
{

	const int width=cellDisplayWidth;
	const int height=cellDisplayHeight;
    const int step=cell.m_nRange/cell.m_nNoBins;

	QPainter p(&img);
	if (antialias)
		p.setRenderHint(QPainter::Antialiasing);
	p.setPen(Qt::white);
	p.translate(xoffset+width/2,yoffset+height/2);
	QLine l(-width/2,0,width/2,0);
	p.rotate(90+step/2);

    // iterate through each bin, then draw the according line.
    for(int i=0; i<cell.m_nNoBins;i++)
    {
        //int binCenterAngle = i*step + step/2;
        float magnitude=cell.values[i];
		if (magnitude>clipValue)
			magnitude= clipValue;
		else
			magnitude= magnitude * 255.0/clipValue;
				
		//std::cerr << i << " * " << step << " ... => " << binCenterAngle << std::endl;
		//std::cerr << binCenterAngle << " " ;

//		p.save();
		p.setPen(QColor(255,255,255,static_cast<int>(magnitude)));
		p.rotate(-step);
		p.drawLine(l);
//		p.restore();
	}
}     */


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// CellHistogramm
//
//
CellHistogram::CellHistogram(int newRange, int newNoBins):
	m_nRange(newRange),
	m_nNoBins(newNoBins),
	m_nBinSize(newRange/newNoBins),
	values(m_nNoBins,0)
{ }

CellHistogram::CellHistogram():
	m_nRange(180),
	m_nNoBins(9),
	m_nBinSize(180 / 9),
	values(9,0)
{ } 

void CellHistogram::insert(float orientation, float magnitude)
{
	// FUNCTION CHECKED AND WORKING 
	const float positiveOrientation = modulo((orientation + m_nRange), m_nRange);// make sure orientation is positive
	const int bin = static_cast<int>(floorf(positiveOrientation * m_nNoBins/ m_nRange)); //select bin ( orienation/binsize )
	values[bin] += magnitude;
	printf("angle: %f -> bin: %d, value: %f\n", orientation, bin, magnitude); 
}

//
void CellHistogram::insertInterpolated(float orientation, float magnitude)
{
	// FUNCTION CHECKED AND WORKING 
	// value gets put into 2 neigboring bins with a weight.
	const float positiveOrientationShifted = modulo((orientation - m_nBinSize / 2 + 2 * m_nRange), m_nRange); // shifted by half bin size to operate between bin centers
	const int leftBin = static_cast<int>(floorf(positiveOrientationShifted * m_nNoBins / m_nRange)); //compute left bin ( orientation/binsize )
	const float rightWeight = modulo(positiveOrientationShifted,(m_nRange/m_nNoBins)) / m_nBinSize; //compute left weight ( orientation%binsize )
	const float rightMagnitude = magnitude * rightWeight;	
        assert( leftBin >= 0 && leftBin < m_nNoBins );
	// write magnitudes (we normalize later)
	values[leftBin] += magnitude - rightMagnitude;
	values[(leftBin+1)%m_nNoBins] += rightMagnitude;
	//printf("angle: %d -> leftbin: %d, values: %d,%d\n", orientation, leftBin, magnitude*m_nBinSize-rightMagnitude,rightMagnitude); 
}


int CellHistogram::save(const std::string& filename) const
{
	std::ofstream of(filename.c_str());
	if (of.is_open())
		of << *this;
	else
	{
		std::cerr << "CellHistogram::save() opening/creating " << filename << "failed\n";
		return 1;
	}
	of.close();
	return 0;
}
std::ostream& HOG::operator<< (std::ostream& os, const CellHistogram& hist)
{
	os << "CellHistogram: " << hist.size() << "\n";
	for (int i=0; i<hist.size(); ++i)
		os << hist[i] << " ";
	return os << std::endl;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// ColorGradient
//
//
/*ColorGradient::ColorGradient(const QImage& img, int compressionType, double sigma, bool preComputeOrientation, std::ostream* outputChannel) :
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
	m_output (outputChannel)
{	
	if(sigma == 0) {
		smoothingKernel.resize(0);
		compute(img, compressionType);
	} else {
		QImage blurred(img.size(), QImage::Format_RGB32);
				
		const int kernelWidth = static_cast<int>(ceil(3* sigma));
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
		for(int y = 0; y < img.height(); y++)
			for(int x = 0; x < img.width(); x++) {
				
				//Red Channel
				unsigned char newRed = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newRed += static_cast<unsigned char>( qRed(img.pixel((x + k + img.width()) % img.width() , y)) * smoothingKernel[kernelWidth + k] );
					
				//Green Channel
				unsigned char newGreen = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newGreen += static_cast<unsigned char>(qGreen(img.pixel((x + k + img.width()) % img.width(), y)) * smoothingKernel[kernelWidth + k]);
				
				//Blue Channel
				unsigned char newBlue = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newBlue += static_cast<unsigned char>(qBlue(img.pixel((x + k + img.width()) % img.width(), y))  * smoothingKernel[kernelWidth + k]);
				
				//std::cout << "R: " << (int)newRed << " G: " << (int)newGreen << " B: " << (int)newBlue << std::endl;
				blurred.setPixel(x,y, qRgb(newRed, newGreen, newBlue));
			}
			
		//Blur in y direction
		for(int y = 0; y < img.height(); y++)
			for(int x = 0; x < img.width(); x++) {
				
				//Red Channel
				unsigned char newRed = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newRed += static_cast<unsigned char>( qRed(blurred.pixel(x, (y + k + img.height()) % img.height() )) * smoothingKernel[kernelWidth + k] );
					
				//Green Channel
				unsigned char newGreen = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newGreen += static_cast<unsigned char>( qGreen(blurred.pixel(x, (y + k + img.height()) % img.height() )) * smoothingKernel[kernelWidth + k] );
				
				//Blue Channel
				unsigned char newBlue = 0;
				for(int k = -kernelWidth; k <= kernelWidth; k++)
					newBlue += static_cast<unsigned char>( qBlue(blurred.pixel(x, (y + k + img.height()) % img.height() )) * smoothingKernel[kernelWidth + k] );
				
				//std::cout << "R: " << (int)newRed << " G: " << (int)newGreen << " B: " << (int)newBlue << std::endl;
				blurred.setPixel(x,y, qRgb(newRed, newGreen, newBlue));
			}
		
		//std::cout << " done";		
		compute(blurred, compressionType);	
	}
	
} */

ColorGradient::~ColorGradient()
{
	if (m_redVGradients) delete[] m_redVGradients;
	if (m_redMagnitudes) delete[] m_redMagnitudes;
	
	for(unsigned int i = 0; i < m_magnitudes.size(); i++)
		if (m_magnitudes[i])
			delete[] m_magnitudes[i];	
}

/*
void ColorGradient::compute(const QImage& image, int compressionType)
{
	if (m_output)
		*m_output << "    Computing Color Gradients for images size: " << image.width() << "x" << image.height() << " ..." << std::endl;
	m_nGradientImageWidth = image.width();
	m_nGradientImageHeight = image.height();

	if (m_output)
		*m_output  << "       color gradients ... " << std::endl;
	computeColorGradients(image, compressionType);
	if (m_output)
		*m_output  << "       max gradients ... " << std::endl;
	if (m_preComputeOrientation)
		computeMaxGradients();
	//computeMaxGradients();
}

void ColorGradient::computeColorGradients(const QImage& image, int compressionType)
{
	//--- store gradients in image (integer array) ---//
	// TODO: Shall we we check if there are previous gradients? (otherwise it leaks)

	computeColorGradients(image.bits(), image.width(), image.height(), image.bytesPerLine(), compressionType, 4, 3, 2, 1);
}
  */
void ColorGradient::computeMaxGradients()
{
	PIXEL* currentRedHGradient = m_redHGradients;
	PIXEL* currentRedVGradient = m_redVGradients;
	PIXEL* currentGreenHGradient = m_greenHGradients;
	PIXEL* currentGreenVGradient = m_greenVGradients;
	PIXEL* currentBlueHGradient = m_blueHGradients;
	PIXEL* currentBlueVGradient = m_blueVGradients;

	int imgsize = m_nGradientImageWidth * m_nGradientImageHeight;
	m_redMagnitudes   = new PIXEL[imgsize*5];
	m_greenMagnitudes = m_redMagnitudes + imgsize;
	m_blueMagnitudes  = m_greenMagnitudes + imgsize;
	PIXEL* currentRedMagnitude = m_redMagnitudes;
	PIXEL* currentGreenMagnitude = m_greenMagnitudes;
	PIXEL* currentBlueMagnitude = m_blueMagnitudes;

	m_maxMagnitudes   = m_blueMagnitudes + imgsize;
	m_maxOrientations = m_maxMagnitudes + imgsize;
	PIXEL* currentMaxMagnitude = m_maxMagnitudes;
	PIXEL* currentMaxOrientation = m_maxOrientations;

	for (int y=0; y<m_nGradientImageHeight; ++y )
	{
		for (int x=0; x<m_nGradientImageWidth; ++x)
		{
			const PIXEL redDX=   *currentRedHGradient;
			const PIXEL redDY=   *currentRedVGradient;
			const PIXEL greenDX= *currentGreenHGradient;
			const PIXEL greenDY= *currentGreenVGradient;
			const PIXEL blueDX=  *currentBlueHGradient;
			const PIXEL blueDY=  *currentBlueVGradient;

			const PIXEL redMag= PIXEL(sqrt(redDX*redDX+redDY*redDY));
			const PIXEL greenMag= PIXEL(sqrt(greenDX*greenDX+greenDY*greenDY));
			const PIXEL blueMag= PIXEL(sqrt(blueDX*blueDX+blueDY*blueDY));

			*currentRedMagnitude   = redMag;
			*currentGreenMagnitude = greenMag;
			*currentBlueMagnitude  = blueMag;

			if (redMag>greenMag)
			{
				if (redMag>blueMag)
				{
					*currentMaxMagnitude=redMag;
					*currentMaxOrientation=PIXEL(atan2(redDY,redDX)*180.0 / M_PI);
				}
				else
				{
					*currentMaxMagnitude=blueMag;
					*currentMaxOrientation=PIXEL(atan2(blueDY,blueDX)*180.0/M_PI);
				}
			}
			else
			{
				if (greenMag>blueMag)
				{
					*currentMaxMagnitude=greenMag;
					*currentMaxOrientation=PIXEL(atan2(greenDY,greenDX)*180.0/M_PI);
				}
				else
				{
					*currentMaxMagnitude=blueMag;
					*currentMaxOrientation=PIXEL(atan2(blueDY,blueDX)*180.0/M_PI);
				}
			}
			
			++currentRedHGradient;
			++currentRedVGradient;
			++currentGreenHGradient;
			++currentGreenVGradient;
			++currentBlueHGradient; 
			++currentBlueVGradient;

			++currentRedMagnitude;
			++currentGreenMagnitude;
			++currentBlueMagnitude;

			++currentMaxMagnitude;
			++currentMaxOrientation;
		}
	}
}

PIXEL* ColorGradient::maxMagnitudeAt(float degree, int range) {
	PIXEL* currentRedHGradient = m_redHGradients;
	PIXEL* currentRedVGradient = m_redVGradients;
	PIXEL* currentGreenHGradient = m_greenHGradients;
	PIXEL* currentGreenVGradient = m_greenVGradients;
	PIXEL* currentBlueHGradient = m_blueHGradients;
	PIXEL* currentBlueVGradient = m_blueVGradients;

	int imgsize=m_nGradientImageWidth*m_nGradientImageHeight;
	PIXEL *maxMagnitude  = new PIXEL[imgsize];
	PIXEL *currentMaxMagnitude = maxMagnitude;
	m_magnitudes.push_back(maxMagnitude);

	const float cosDeg = cos(degree * 1.0f / 180 * M_PI);
	const float sinDeg = sin(degree * 1.0f / 180 * M_PI);
	const float cosDeg180 = cos((degree + 180) * 1.0f / 180 * M_PI);
	const float sinDeg180 = sin((degree + 180) * 1.0f / 180 * M_PI);

	for (int y=0; y<m_nGradientImageHeight; ++y )
	{
		for (int x=0; x<m_nGradientImageWidth; ++x)
		{
			const PIXEL redDX=   *currentRedHGradient;
			const PIXEL redDY=   *currentRedVGradient;
			const PIXEL greenDX= *currentGreenHGradient;
			const PIXEL greenDY= *currentGreenVGradient;
			const PIXEL blueDX=  *currentBlueHGradient;
			const PIXEL blueDY=  *currentBlueVGradient;
			
			PIXEL redMag;
			PIXEL greenMag;
			PIXEL blueMag;
			
			if (range == 180) {
				redMag =  static_cast<PIXEL>(fabsf(redDY*sinDeg + redDX*cosDeg) + fabsf(redDY*sinDeg180 + redDX*cosDeg180));
				greenMag = static_cast<PIXEL>(fabsf(greenDY*sinDeg + greenDX*cosDeg) + fabsf(greenDY*sinDeg180 + greenDX*cosDeg180));
				blueMag = static_cast<PIXEL>(fabsf(blueDY*sinDeg + blueDX*cosDeg) + fabsf(blueDY*sinDeg180 + blueDX*cosDeg180));
			} else {
				redMag =  static_cast<PIXEL>(fabsf(redDX*sinDeg + redDY*cosDeg));
				greenMag = static_cast<PIXEL>(fabsf(greenDX*sinDeg + greenDY*cosDeg));
				blueMag = static_cast<PIXEL>(fabsf
						(blueDX*sinDeg + blueDY*cosDeg));
			}

			(*currentMaxMagnitude) = greenMag;
			if (redMag > greenMag)			
				(*currentMaxMagnitude) = redMag;
				
			if (blueMag > (*currentMaxMagnitude))
				(*currentMaxMagnitude) = blueMag;
							
			++currentRedHGradient;
			++currentRedVGradient;
			++currentGreenHGradient;
			++currentGreenVGradient;
			++currentBlueHGradient; 
			++currentBlueVGradient;
		
			++currentMaxMagnitude;			
		}
	}
	
	return maxMagnitude;
}


CellGrid::CellGrid(const HOGParams& params, const ColorGradient& gradients, unsigned int x, unsigned int y) :
	m_params(params)
{
	computeCells(gradients, x, y, gradients.width()-x, gradients.height()-y);
}
CellGrid::CellGrid(const HOGParams& params, const ColorGradient& gradients, unsigned int x, unsigned int y, unsigned int width, unsigned int height, const float* spatialMask):
	m_params(params)
{
	computeCells(gradients, x, y, width, height, spatialMask);
}

CellGrid::~CellGrid()
{
}

void CellGrid::computeCells(const ColorGradient& gradients, unsigned int x, unsigned int y, unsigned int width, unsigned int height, const float* spatialMask)
{
	const unsigned int gradientWidth = gradients.width();
	//const unsigned int gradientHeight = gradients.height();
	assert(static_cast<int>(x+width) <= gradients.width());
	assert(static_cast<int>(y+height) <= gradients.height());

	//---  compute border and #cells (ensure to have at lease one pixel border) ---//
	m_nCellsX = ( width / m_params.m_nCellWidth ) ;		
	m_xBorder = ( width - m_nCellsX * m_params.m_nCellWidth ) / 2;
	m_nCellsY = ( height / m_params.m_nCellHeight ) ;
	m_yBorder = ( height - m_nCellsY * m_params.m_nCellHeight) / 2;	

	/*
	std::cerr << "    Computing cell Grid for images size: " << gradients.width() << "x" << gradients.height() << " ..." << std::endl;
	std::cerr << "       offset: " << x << " " << y << "   wxh: "<< width << " " << height << std::endl;
	std::cerr << "       border: " << m_xBorder << " " << m_yBorder << std::endl;
	std::cerr << "       nCells: " << m_nCellsX << " " << m_nCellsY << std::endl;
	*/
	
	//--- resize cell grid ---//	
	m_vCells.resize(m_nCellsX * m_nCellsY, CellHistogram(m_params.m_nHistogramRange, m_params.m_nHistogramBins));	

	//--- compute cells ---//
	const PIXEL* currentMaxMagnitude   = gradients.maxMagnitudes() + (x+m_xBorder) + gradientWidth * (y+m_yBorder);
	const PIXEL* currentMaxOrientation = gradients.maxOrientations() + (x+m_xBorder) + gradientWidth * (y+m_yBorder);
	
	const int gridWidth  = m_nCellsX * m_params.m_nCellWidth;
	const int gridHeight = m_nCellsY * m_params.m_nCellHeight;
	const int cwidth = m_params.m_nCellWidth;
	const int cheight = m_params.m_nCellHeight;
	int leftIndex, topIndex;
	float leftWeight, rightWeight;
	float topWeight, bottomWeight;
		
	for (int yi=0; yi < gridHeight; ++yi )
	{
		for (int xi=0; xi <gridWidth; ++xi)
		{			
			const float& orientation= (*currentMaxOrientation);
			PIXEL magnitude = (*currentMaxMagnitude);
			if (spatialMask!=0)
				magnitude *= spatialMask[xi+width*yi];

			//--- figure out cell indices in horizontal direction ---//
			if (xi<cwidth/2)//if we're in the left half of the left most cell
			{
				leftIndex   = -1;
				leftWeight  = 0;
				rightWeight = ((2 * xi + cwidth + 1) % (2 * cwidth)) * 0.5 / cwidth; //cwidth/2 - xi;
			}
			else if (xi >= m_nCellsX*cwidth - cwidth/2)//if we're in the right half of the right-most cell
			{
				leftIndex   = m_nCellsX-1;
				leftWeight  = (2 * cwidth - (2*xi + cwidth + 1) % (2 * cwidth)) * 0.5/ cwidth; //xi - (m_nCellsX*cwidth - cwidth/2);
				rightWeight = 0;								
			}
			else
			{
				leftIndex   = (xi-cwidth/2) / cwidth;
				rightWeight = ((2*xi + cwidth + 1) % (2*cwidth)) * 0.5/ cwidth; //cwidth - leftWeight;
				leftWeight  = 1.0 - rightWeight; //cwidth -((xi-cwidth/2) % cwidth); // should we convert the modulo operation to a multiplacation by using leftIndex?
			}

			//--- figure out cell indices in vertical direction ---//
			if (yi<cheight/2)//if we're in the top half of the top-most cell
			{
				topIndex   = -1;
				topWeight  = 0;
				bottomWeight = ((2*yi + cheight + 1) % (2*cheight)) * 0.5 / cheight; //(2*yi + cheight + 1) % (2*cheight);
			}
			else if (yi >= m_nCellsY*cheight - cheight/2)//if we're in the bottom half of the bottomest cell
			{
				topIndex     = m_nCellsY-1;
				topWeight    = (2 * cheight -(2*yi + cheight + 1) % (2*cheight)) * 0.5 / cheight;//yi - (m_nCellsY*cheight - cheight/2);
				bottomWeight = 0;
			}
			else
			{
				topIndex   = (yi-cheight/2) / cheight;
				bottomWeight = ((2*yi + cheight + 1) % (2*cheight)) * 0.5 / cheight; //cheight - topWeight;
				topWeight  = 1.0 - bottomWeight;//cheight -((yi-cheight/2) % cheight); // should we convert the modulo operation to a multiplacation by using topIndex?
				
			}

			// entering weights into histogram using bilinear interpolation
			const int bottomIndex= topIndex + 1;
			const int rightIndex= leftIndex + 1;

			//std::cerr << "pos: " << xi << " " << yi << std::endl;
			//std::cerr << "idx: " << leftIndex << " " << topIndex << "\n";
			//std::cerr << " wt: " << leftWeight << " " << rightWeight << " " << topWeight << " " << bottomWeight << std::endl;

			if (leftWeight * topWeight != 0)
			{
				m_vCells[leftIndex + topIndex * m_nCellsX].insertInterpolated(orientation, magnitude * leftWeight * topWeight);
			}
			
			if (leftWeight * bottomWeight != 0)
			{
				m_vCells[leftIndex + bottomIndex * m_nCellsX].insertInterpolated(orientation, magnitude * leftWeight * bottomWeight);
			}
			if (rightWeight*topWeight!=0)
			{
				m_vCells[rightIndex + topIndex * m_nCellsX].insertInterpolated(orientation, magnitude * rightWeight * topWeight);
			}
			if (rightWeight*bottomWeight!=0)
			{
				m_vCells[rightIndex + bottomIndex * m_nCellsX].insertInterpolated(orientation, magnitude * rightWeight * bottomWeight);
			}

			++currentMaxMagnitude;
			++currentMaxOrientation;
		}
		// jump to next line
		currentMaxMagnitude+=gradientWidth - gridWidth;
		currentMaxOrientation+=gradientWidth - gridWidth;
	}
}

void CellGrid::printCellGridSparseFormat(std::ostream& out)
{
		
	int nEntries= 0;
	for (int y= 0; y<m_nCellsY; y++)
		for (int x= 0; x<m_nCellsX; x++)
			for (int binId= 0; binId<m_params.m_nHistogramBins; binId++)
				if (this->operator()(x,y)[binId] > 0)
					++nEntries;
					
	out << nEntries << " ";
	int counter= 0;
	for (int y= 0; y<m_nCellsY; y++)
		for (int x= 0; x<m_nCellsX; x++)
			for (int binId= 0; binId < m_params.m_nHistogramBins; binId++)
				if (this->operator()(x,y)[binId] > 0)
					out << counter++ << ":" << this->operator()(x,y)[binId] << " ";
				else
					++counter;
	out << std::endl;
}

void CellGrid::getCellGridSparseFormat(std::vector<float>& features)
{
	int counter= 0;
	for (int y= 0; y<m_nCellsY; y++)
		for (int x= 0; x<m_nCellsX; x++)
			for (int binId= 0; binId<m_params.m_nHistogramBins; binId++)
				features[counter++]= this->operator()(x,y)[binId];
}

void CellGrid::getCellGridSparseFormatWordNonWord(std::vector<float>& features, float threshold)
{
	//int nEntries= m_nCellsX*m_nCellsY*m_params.m_nHistogramBins;
					
	int counter= 0;
	for (int y= 0; y<m_nCellsY; y++)
		for (int x= 0; x<m_nCellsX; x++)
			for (int binId= 0; binId<m_params.m_nHistogramBins; binId++)
			{
			if (threshold == -1)
			{
				// use adaptive threshold
				float cellSum= 0;
				for (int i= 0; i<m_params.m_nHistogramBins; i++)
					cellSum+= this->operator()(x,y)[i];
				threshold= cellSum/m_params.m_nHistogramBins;
			}
				if (this->operator()(x,y)[binId]>threshold)
					features[counter]= 1;
				else
					features[counter+1]= 1;
				counter+= 2;
			}
}


int CellGrid::save(const std::string& filename) const
{
	std::ofstream of(filename.c_str());
	if (of.is_open())
		of << *this;
	else
	{
		std::cerr << "CellGrid::save() opening/creating " << filename << "failed\n";
		return 1;
	}
	of.close();
	return 0;
}
		

std::ostream& HOG::operator<< (std::ostream& os, const CellGrid& grid)
{
	os << "CellGrid: "<< grid.m_nCellsX<< " "<< grid.m_nCellsY<<"\n";
	os << "CellWH: "<< grid.m_params.m_nCellWidth << " " << grid.m_params.m_nCellHeight << "\n"; 
	
	for (unsigned i=0; i < grid.m_vCells.size(); ++i)
		os << grid.m_vCells[i];
	return os << std::endl;
}


///////////////////////
//     Block      	 //
///////////////////////

Block::Block(const ColorGradient& colorgradient, const HOGParams& params, int x, int y, float* spatialMask) : m_params(&params)
{
	computeFromGradients(colorgradient, *m_params, x, y, spatialMask);
}

Block::Block(const CellGrid& cellGrid, int x, int y) : m_params(&cellGrid.m_params)
{
	computeFromCellGrid(cellGrid, x, y);
}

void Block::computeFromCellGrid(const CellGrid& cellGrid, int x, int y)
{
	//m_params = cellGrid.m_params;
	
	//-- copy cells into local vector ---//
	m_vValues.resize(m_params->m_nBlockHeight * m_params->m_nBlockWidth * cellGrid(x,y).values.size());
	int counter= 0;
	for (int j=0; j<m_params->m_nBlockHeight; ++j)
		for (int i=0; i<m_params->m_nBlockWidth; ++i){
			const float* p=&(cellGrid(x+i,y+j).values[0]);
			for(unsigned int k = 0; k  < cellGrid(x+i,y+j).values.size(); ++k,++p)
				m_vValues[counter++]=*p;
		}

	//--- normalize ---//
	normalize();
}

void Block::computeFromGradients(const ColorGradient& colorgradients, const HOGParams& params, int x, int y, float* spatialMask) {
	
	//m_params = params;
	
	//add 1 to neglect boundary
	const int xOffset = 1 + x;
	const int yOffset = 1 + y;
	const int w = m_params->m_nCellWidth * m_params->m_nBlockWidth;
	const int h = m_params->m_nCellHeight * m_params->m_nBlockHeight;
	
	CellGrid cg(*m_params, colorgradients, xOffset, yOffset, w, h, spatialMask);
	
	m_vValues.resize(m_params->m_nBlockHeight * m_params->m_nBlockWidth * cg(0,0).values.size());	
	int counter= 0;
	for (int j=0; j<m_params->m_nBlockHeight; ++j)
		for (int i=0; i<m_params->m_nBlockWidth; ++i){
			const float* p=&(cg(i,j).values[0]);
			for(unsigned int k=0; k<cg(i,j).values.size(); ++k,++p)
				m_vValues[counter++]=*p;
		}

	//--- normalize ---//
	normalize();	
}


void Block::normalize()
{
	normalize(m_params->m_nNormType);
}

void Block::normalize(int normtype)
{
	float clippingThres = m_params->m_dClippingThres;
	float epsilon = m_params->m_dNormalizeEpsilon;
	float epsilonHys = m_params->m_dNormalizeEpsilonHys;

	if (normtype==0)
	{
		// no normalization
	}
	else if (normtype == 1)//L1 norm
	{
		// equation: v = v/(sum(v) +e)

		//--- sum bins ---// 
		float sum=0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			sum += m_vValues[i];
		float normalizationfactor= sum + epsilon;//1 so as to avoid div/0

		//--- divide each element by normalizationfactor ---//
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			m_vValues[i] = m_vValues[i] / normalizationfactor;
	}
	
	else if (normtype == 2)//L2 norm
	{	
		//equation: v = v/sqrt(sum(v^2)+e)
		
		//--- add the sum of squares ---//
		float sumofsquares=0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			sumofsquares += m_vValues[i] * m_vValues[i];
		const float normalizationfactor= sqrt(sumofsquares+epsilon*epsilon);

		//---divide each element by normalizationfactor ---
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			m_vValues[i]=m_vValues[i]/normalizationfactor;
	}
	else if (normtype == 3)//L1 norm + HYS
	{	
		//equation: v = v/sum(v)+e)
		
		//--- add the sum of squares ---//
		float sum=0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			sum += m_vValues[i];
		float normalizationfactor= sum+epsilon;

		//---divide each element by normalizationfactor ---
		float sum2 = 0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)	{	
			m_vValues[i]= m_vValues[i]/normalizationfactor;
			//Clipping Thres
			if (m_vValues[i] > clippingThres) {
				m_vValues[i] = clippingThres;				
			}
			sum2 += m_vValues[i];
		}
		
		normalizationfactor = sum2 + epsilonHys; //sum2 + epsilon * 1.0f * sum2 / (sum + epsilon);
		for(unsigned int i=0; i<m_vValues.size(); ++i)		
			m_vValues[i]= m_vValues[i]/normalizationfactor;
		
	}	
	else if (normtype == 4)//L2 norm + HYS
	{
		//equation: v = v/sqrt(sum(v^2)+e)
		
		//--- add the sum of squares ---//
		float sumofsquares=0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			sumofsquares += m_vValues[i]*m_vValues[i];
		float normalizationfactor= sqrt(sumofsquares+epsilon*epsilon);

		//---divide each element by normalizationfactor ---
		float sumofsquares2 = 0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)	{	
			m_vValues[i]= m_vValues[i]/normalizationfactor;
			//Clipping Thres
			if (m_vValues[i] > clippingThres) {
				m_vValues[i] = clippingThres;				
			}
			sumofsquares2 += m_vValues[i]*m_vValues[i];
		}
		
		normalizationfactor = sqrt(sumofsquares2 + epsilonHys*epsilonHys);
		for(unsigned int i=0; i<m_vValues.size(); ++i)		
			m_vValues[i]= m_vValues[i]/normalizationfactor;
		
	}
	else if (normtype == 5)//sqrt L1 norm
	{	
		//equation: v = sqrt(v/sum(v)+e))
		
		//--- sum bins ---// 
		float sum=0;
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			sum += m_vValues[i];
		float normalizationfactor= sum + epsilon;//1 so as to avoid div/0

		//--- divide each element by normalizationfactor ---//
		for(unsigned int i=0; i<m_vValues.size(); ++i)
			m_vValues[i] = sqrt(m_vValues[i]/normalizationfactor);		
	}
	

	//float sum=0;
	//for(unsigned int i=0; i<m_vValues.size(); ++i)
	//	sum+=m_vValues[i];
	//std::cerr << "Sum: " << sum;
	//float sum=0;
	//for(unsigned int i=0; i<m_vValues.size(); ++i)
	//	sum+=m_vValues[i]*m_vValues[i];
	//std::cerr << "Sum: " << sum;

}


int Block::save(const std::string& filename) const
{
	std::ofstream of(filename.c_str());
	if (of.is_open())
		of << *this;
	else
	{
		std::cerr << "Block::save() opening/creating " << filename << "failed\n";
		return 1;
	}
	of.close();
	return 0;
}

std::ostream& HOG::operator<< (std::ostream& os, const Block& block)
{
	os << "BlockWH: "<< block.m_params->m_nBlockWidth << " " << block.m_params->m_nBlockHeight << "\n";
	os << "CellWH: "<< block.m_params->m_nCellWidth << " " << block.m_params->m_nCellHeight << "\n"; 
	
	for (unsigned int i=0; i<block.m_vValues.size(); ++i)
	{
		os << block.m_vValues[i] << " ";
		if ((i+1) % block.m_params->m_nHistogramBins == 0)
			os << "\n";
	}
	return os << std::endl;
}



//////////////////////
//   BlockGrid   //
//////////////////////

BlockGrid::BlockGrid(const CellGrid& cellGrid) : m_params(cellGrid.m_params), m_spatialMask(0)
{	
	computeFromCellGrid(cellGrid);
}
BlockGrid::BlockGrid(const CellGrid& cellGrid, const HOGParams& params) : m_params(params), m_spatialMask(0)
{	
	computeFromCellGrid(cellGrid);
}

BlockGrid::BlockGrid(ColorGradient& colorgradients, const HOGParams& params, unsigned int offsetX, unsigned int offsetY, int widthX, int heightY) : m_params(params), m_spatialMask(0)
{

  if (m_params.m_nGaussianWindow != 0) {
  	m_spatialMask = new float[m_params.m_nBlockWidth * m_params.m_nBlockHeight * m_params.m_nCellWidth *  m_params.m_nCellHeight];
  	
    // Setup spatial window
    float sigmaX = m_params.m_nBlockWidth * m_params.m_nCellWidth * m_params.m_nGaussianWindow;
    float sigmaY = m_params.m_nBlockHeight * m_params.m_nCellHeight * m_params.m_nGaussianWindow;
    
    const float centerX =  m_params.m_nBlockHeight * m_params.m_nCellHeight * 1.0f / 2 - 0.5;
    const float centerY =  m_params.m_nBlockWidth * m_params.m_nCellWidth * 1.0f / 2 - 0.5;

    //std::cout << std::setprecision(1);
    float sum = 0.0;
    for(int y = 0; y < m_params.m_nBlockHeight * m_params.m_nCellHeight; y++) {
      for(int x = 0; x < m_params.m_nBlockWidth * m_params.m_nCellWidth; x++) {
        const float mhd = (centerX - x) * (centerX - x) / (sigmaX * sigmaX) + (centerY - y) * (centerY - y) / (sigmaY * sigmaY);
        
        //Not needed in this setup (scrambles Gaussian according to cells)
        //const int col = x % m_params.m_nCellWidth;
        //const int row = y % m_params.m_nCellHeight;
        //const int cellR = y / m_params.m_nCellHeight;
        //const int cellC = x / m_params.m_nCellWidth;
        //const int cellID = cellR * m_params.m_nBlockWidth + cellC;
                                                 
        //m_spatialMask[cellID * m_params.m_nCellHeight * m_params.m_nCellWidth + row * m_params.m_nCellWidth + col] = static_cast<int>(round(10000.0f/(sqrt(2*pi) * sigma * sigma ) * exp(-1.0f/2 * mhd)));
        m_spatialMask[y * m_params.m_nBlockWidth * m_params.m_nCellWidth + x] = 1.0f/(2 * M_PI * sigmaX * sigmaY ) * exp(-0.5f * mhd);
        sum += m_spatialMask[y * m_params.m_nBlockWidth * m_params.m_nCellWidth + x];
      }
    }
    
    //Normalize Gaussian to sum = 1
    for(int y = 0; y < m_params.m_nBlockHeight * m_params.m_nCellHeight; y++) {
      for(int x = 0; x < m_params.m_nBlockWidth * m_params.m_nCellWidth; x++) {
    	  m_spatialMask[y * m_params.m_nBlockWidth * m_params.m_nCellWidth + x] /= sum;
      }
    }
    
    /*for(int y = 0; y < m_params.m_nBlockHeight * m_params.m_nCellHeight; y++) {
      for(int x = 0; x < m_params.m_nBlockWidth * m_params.m_nCellWidth; x++) {
	
    	std::cout << m_spatialMask[y * m_params.m_nBlockWidth * m_params.m_nCellWidth + x] << "\t";
    	}
    	std::cout << std::endl;
    }*/
        
    
  } 	                                                                                                            
 
  computeFromGradients(colorgradients, offsetX, offsetY, widthX, heightY);

}


BlockGrid::~BlockGrid() {
	if( m_spatialMask)
		delete[](m_spatialMask);
}

int BlockGrid::computeFromCellGrid(const CellGrid& cellGrid)
{	
	//if nogood m_params were passed on
	if (((m_params.m_nBlockHorizontalOverlap%m_params.m_nCellWidth)!=0) || ((m_params.m_nBlockVerticalOverlap%m_params.m_nCellHeight)!=0))
	{
		std::cout<<"sorry, I don't work with these m_params!\n";
		return 1;
	}
	else
	{
		const int horizontalOverlap = m_params.m_nBlockHorizontalOverlap/m_params.m_nCellWidth;//in cells
		const int verticalOverlap   = m_params.m_nBlockVerticalOverlap/m_params.m_nCellHeight;
	
		m_stepSizeX = m_params.m_nBlockWidth-horizontalOverlap; //in cells
		m_stepSizeY = m_params.m_nBlockHeight-verticalOverlap;
		m_nBlocksX  = (cellGrid.m_nCellsX-horizontalOverlap)/(m_stepSizeX);
		m_nBlocksY  = (cellGrid.m_nCellsY-verticalOverlap)/(m_stepSizeY); 

	
		m_vBlocks.resize(m_nBlocksX*m_nBlocksY, Block(m_params) );
		for (int j=0; j<m_nBlocksY; ++j)
			for (int i=0; i<m_nBlocksX; ++i)
				m_vBlocks[i+j*m_nBlocksX].computeFromCellGrid(cellGrid, i*m_stepSizeX, j*m_stepSizeY);
		
	}
	return 0;
}


int BlockGrid::computeFromGradients(ColorGradient& colorgradients, unsigned int offsetX, unsigned int offsetY, int widthX, int heightY)
{	
	const int stepSizeX = (m_params.m_nBlockWidth * m_params.m_nCellWidth) - m_params.m_nBlockHorizontalOverlap;
	const int stepSizeY = (m_params.m_nBlockHeight * m_params.m_nCellHeight) -m_params.m_nBlockVerticalOverlap;
	
	int offEndX = 0;
	int offEndY = 0;
	
	if (widthX > -1)
		offEndX = colorgradients.width() - widthX;
	
	if (heightY > -1)
		offEndY = colorgradients.height() - heightY;
	
	m_nBlocksX  = (colorgradients.width()-2- offsetX - offEndX -(m_params.m_nBlockWidth * m_params.m_nCellWidth - stepSizeX) )  / stepSizeX;
	m_nBlocksY  = (colorgradients.height()-2- offsetY - offEndY -(m_params.m_nBlockHeight * m_params.m_nCellHeight - stepSizeY) ) / stepSizeY;  
	//m_nBlocksX  = (colorgradients.width()-2- offsetX - (stepSizeX % (m_params.m_nBlockWidth * m_params.m_nCellWidth)) ) / stepSizeX;
	//m_nBlocksY  = (colorgradients.height()-2- offsetY- (stepSizeY % (m_params.m_nBlockHeight * m_params.m_nCellHeight)) ) / stepSizeY;  
		
	m_vBlocks.resize(m_nBlocksX * m_nBlocksY, Block(m_params));
	for (int j=0; j < m_nBlocksY; ++j)
		for (int i=0; i < m_nBlocksX; ++i)				
			m_vBlocks[i + j * m_nBlocksX].computeFromGradients(colorgradients, m_params, i*stepSizeX + offsetX, j*stepSizeY + offsetY, m_spatialMask);
	return 0;
}

int BlockGrid::save(const std::string& filename) const
{
	std::ofstream file(filename.c_str());
	if ( file.is_open() )
		file << *this;
	else
	{
		std::cerr << "BlockGrid.save(): Could not open File: " << filename << std::endl;
		return 1;
	}

	file.close();
	return 0;
}

std::ostream& HOG::operator<< (std::ostream& os, const BlockGrid& grid)
{
	os << "BlockGrid: "<< grid.m_nBlocksX << " " << grid.m_nBlocksY << "\n";
	for(int i=0; i<grid.m_nBlocksX*grid.m_nBlocksY; ++i)
	{
		os << grid.m_vBlocks[i];
	}
	return os<<std::endl;
}


void BlockGrid::getHOGDescriptor(const int istart, const int jstart, std::vector<float>& values)
{
	const int hogW = m_params.m_nHOGWidth;
	const int hogH = m_params.m_nHOGHeight;
	const int iend = istart + hogW;
	const int jend = jstart + hogH;
	assert(iend<=m_nBlocksX);
	assert(jend<=m_nBlocksY);
	assert(m_vBlocks.size()>0);
	const unsigned int nValuesInBlock = m_vBlocks[0].m_vValues.size();

	//--- resize vector ---//
	values.clear();
	values.reserve(m_params.m_nHOGWidth * m_params.m_nHOGHeight * nValuesInBlock);

	//fprintf(stderr, "iend: %d, jend: %d, nValuesInBlock: %d\n", iend, jend, nValuesInBlock);
	for(int j=jstart; j<jend; ++j)
		for(int i=istart; i<iend; ++i)
		{
			std::vector<float>& bVs = m_vBlocks[j * m_nBlocksX +i].m_vValues;
			//for(unsigned int k = 0; k < bVs.size(); k++)
			//		values.push_back(bVs[k]);
			values.insert(values.end(), bVs.begin(), bVs.end());
		}
}

// assumes preallocated space in values!
void BlockGrid::getHOGDescriptor(const int istart, const int jstart, float* values)
{
	const int hogW = m_params.m_nHOGWidth;
	const int hogH = m_params.m_nHOGHeight;
	const int iend = istart + hogW;
	const int jend = jstart + hogH;
	assert(iend<=m_nBlocksX);
	assert(jend<=m_nBlocksY);
	assert(m_vBlocks.size()>0);
	//const unsigned int nValuesInBlock = m_vBlocks[0].m_vValues.size();

	//--- resize vector ---//
	//values.clear();
	//values.reserve(m_params.m_nHOGWidth*m_params.m_nHOGHeight*nValuesInBlock);

	//fprintf(stderr, "iend: %d, jend: %d, nValuesInBlock: %d\n", iend, jend, nValuesInBlock);
	for(int j=jstart; j<jend; ++j)
		for(int i=istart; i<iend; ++i)
		{
			std::vector<float>& bVs = m_vBlocks[j * m_nBlocksX +i].m_vValues;
			copy(bVs.begin(), bVs.end(), values);
			values+=bVs.size();
			//for(unsigned int k = 0; k < bVs.size(); k++)
			//		values.push_back(bVs[k]);
			//values.insert(values.end(), bVs.begin(), bVs.end());
		}
}

int BlockGrid::saveDescriptorSVM(const std::string& fileName, const int istart, const int jstart, const int label)
{
	//fprintf(stderr, "BlockGrid::saveDescriptorSVM\n");
	assert(m_vBlocks.size()>0);
	unsigned int nValuesInBlock = m_vBlocks[0].m_vValues.size();

	std::ofstream file(fileName.c_str(), std::ofstream::out | std::ofstream::app);
	if ( file.is_open() )
	{
		file << label;
		
		const int iend = istart + m_params.m_nHOGWidth;
		const int jend = jstart + m_params.m_nHOGHeight;
		assert(iend<=m_nBlocksX);
		assert(jend<=m_nBlocksY);

		int counter=0;
		//fprintf(stderr, "iend: %d, jend: %d, nValuesInBlock: %d\n", iend, jend, nValuesInBlock);
		for(int j=jstart; j<jend; ++j)
			for(int i=istart; i<iend; ++i)
				for(unsigned int pos=0; pos<nValuesInBlock; ++pos)
				{
					const float val = m_vBlocks[j* m_nBlocksX + i].m_vValues[pos];
					file << " " << ++counter << ":" << val;
				}
		file << std::endl;
		file.close();
	}
	else
	{
		std::cerr << "BlockGrid.saveSVM(): Could not open File: " << fileName << std::endl;
		return 1;
	}

	//fprintf(stderr, "BlockGrid::saveDescriptorSVM done\n");
	return 0;
}
/*
void  BlockGrid::drawHOGDescriptor(const int istart, const int jstart, QImage &img, const int blockDisplayWidth, const int blockDisplayHeight, bool antialias) 
{
	const int hogW = m_params.m_nHOGWidth;
	const int hogH = m_params.m_nHOGHeight;
	const int iend = istart + hogW;
	const int jend = jstart + hogH;
	assert(iend<=m_nBlocksX);
	assert(jend<=m_nBlocksY);
	assert(m_vBlocks.size()>0);
	
	img = QImage(hogW * blockDisplayWidth, hogH * blockDisplayHeight, QImage::Format_RGB32);
	img.fill(Qt::black);
	
	//fprintf(stderr, "iend: %d, jend: %d, nValuesInBlock: %d\n", iend, jend, nValuesInBlock);
	for(int j=jstart; j<jend; ++j)
		for(int i=istart; i<iend; ++i) {					
			drawBlock(m_vBlocks[j * m_nBlocksX +i], img, (i - istart) * blockDisplayWidth , (j - jstart) * blockDisplayHeight, blockDisplayWidth, blockDisplayHeight, antialias);
		}
		
} */

 int BlockGrid::getHOGDescriptorLen() {	
	return m_params.m_nHOGWidth * m_params.m_nHOGHeight * m_vBlocks[0].m_vValues.size();	
 }
