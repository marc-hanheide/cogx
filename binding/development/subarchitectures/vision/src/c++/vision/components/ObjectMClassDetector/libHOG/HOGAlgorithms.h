#ifndef HOGALGORITHMS_
#define HOGALGORITHMS_

#include<libHOG/HOGparams.h>
#include<libHOG/HOGVisitor.h>
#include<libHOG/HOG.h>
#include<opencv/cv.h>

namespace HOG{
	// algorithms
	
	static bool compatmode = false; // this is a hack should be removed avter CVPR - just here to make it compatible with prev. experiments

	template<typename Visitor> // should be DenseSlidingHOGSingleScaleVisitor
	void denseSlidingHOGSingleScale(
			const HOGParams& hogParam, // HOG parameters
			const unsigned char* data, int width, int height, int rowstep, // image
			Visitor visitor, // visitor to customize the algorithm
			int noSteps = 1)
	{
		 int cellWidth = hogParam.m_nCellWidth;
		 int cellHeight = hogParam.m_nCellHeight;
		 int stepSize = cellWidth/noSteps;

		//fprintf(stdout, " Image Dimensions: %dx%d\n", w, h);

		for(int ii=0; ii<noSteps; ++ii)
		for(int i=0; i<noSteps; ++i)
		{
			if(compatmode)
				if(ii!=i) continue;

			int noCellsX = (width-i*stepSize-2)/cellWidth;
			int noCellsY = (height-ii*stepSize-2)/cellHeight;
			int xoffset = i*stepSize;// +1;
			int yoffset = ii*stepSize;// +1;

			assert(((cellWidth*noCellsX+2 + xoffset) <= width) && "Bad image offset, the new image refers non-allocated data");
			assert(((cellHeight*noCellsY+2 + yoffset) <= height) && "Bad image offset, the new image refers non-allocated data");

			//counter += testImageOnGrid(data + yoffset*rowstep + xoffset*3, cellWidth*noCellsX+2, cellHeight*noCellsY+2, rowstep,
					//anno, scale, int(1.0*xoffset/scale), int(1.0*yoffset/scale));

			//{ const unsigned char* data, int w, int h, int rowstep, Annotation& anno, const float scale, const int xoffset, const int yoffset }

			const unsigned char* newdata = data + yoffset*rowstep + xoffset*3;
			// NEW OFFSET: xoffset = int(1.0*xoffset/scale)
			// NEW OFFSET: yoffset = int(1.0*yoffset/scale)
			int w = cellWidth*noCellsX+2;
			int h = cellHeight*noCellsY+2;

			// int windowWidth = hogParam.m_nWindowWidth+2;
			// int windowHeight = hogParam.m_nWindowHeight+2;

			// IplImage* tmp = 0; // if a temporary image is created it will be stored here, and must be released at the end
			// FIXME: Preprocessing somehow should be done in the visitor (I guess?!)

			//  int extraBoundaryInCells = m_model.getPreProcessParams().extraCells;
			//  int kernelWidth = m_model.getPreProcessParams().blurKernelWidth;
			//  	
			//  int xbound = m_model.getHOGParams().m_nCellWidth * extraBoundaryInCells;
			//  int ybound = m_model.getHOGParams().m_nCellHeight * extraBoundaryInCells;
			//  
			//  //Add some extra boundary
			//  if (extraBoundaryInCells > 0) {				
			//  	tmp = cvCreateImage(cvSize(w+xbound * 2, h+ybound * 2), IPL_DEPTH_8U, 3);
			//  	IplImage* src = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
			//  	cvSetData(src, const_cast<unsigned char*>(newdata), rowstep);
			//  	cvCopyMakeBorder( src, tmp, cvPoint(xbound,ybound), IPL_BORDER_REPLICATE);
			//  	if(kernelWidth != 0)
			//  		blurEdges((unsigned char*)tmp->imageData, tmp->width, tmp->height, tmp->widthStep,
			//  				xbound, ybound, tmp->width- 1 - xbound, tmp->height - 1 - ybound, kernelWidth);		
			//  	fprintf(stderr, "    Adding extra boundary of %d cell(s)...\n", extraBoundaryInCells);					
			//  	////cvSaveImage("extraBoundary.ppm", tmp);
			//  	//fprintf(stderr, "    Debug image saved\n");					
			//  	newdata=(const unsigned char*)tmp->imageData;
			//  	w=tmp->width;
			//  	h=tmp->height;
			//  	rowstep=tmp->widthStep;
			//  }
			
			ColorGradient cg(newdata, w, h, rowstep, 0, hogParam.m_BlurSigma, true, 0);
			BlockGrid blockGrid(cg, hogParam);		

			//std::cout << "Debug: " << blockGrid.m_nBlocksY << "\n";
			//std::cout << "Debug: " << hogParam.m_nHOGHeight << "\n";
			//std::cout << "Debug: " << blockGrid.m_nBlocksX << "\n";
			//std::cout << "Debug: " << hogParam.m_nHOGWidth << "\n";

			// if( (hogParam.m_nHOGWidth==1) && (hogParam.m_nHOGHeight==1) ){ // each descriptor is just one block
			// 	for(int j=0; j<=blockGrid.m_nBlocksY - hogParam.m_nHOGHeight; ++j)
			// 		for(int i=0; i<=blockGrid.m_nBlocksX - hogParam.m_nHOGWidth; ++i)
			// 		{
			// 			visitor.examine_descriptor(blockGrid.getHOGBlock(i,j));
			// 		}
			// } else 
			{
			
				size_t dimensions = hogParam.m_nHOGHeight * hogParam.m_nHOGWidth * hogParam.m_nBlockWidth * hogParam.m_nBlockHeight * hogParam.m_nHistogramBins;
				
				float hogDescriptor[dimensions];
				int xStep = hogParam.m_nCellWidth  * hogParam.m_nBlockWidth  - hogParam.m_nBlockHorizontalOverlap;
				int yStep = hogParam.m_nCellHeight * hogParam.m_nBlockHeight - hogParam.m_nBlockVerticalOverlap;
				for(int j=0; j<=blockGrid.m_nBlocksY - hogParam.m_nHOGHeight; ++j)
					for(int i=0; i<=blockGrid.m_nBlocksX - hogParam.m_nHOGWidth; ++i)
					{
						blockGrid.getHOGDescriptor(i, j, hogDescriptor);
						visitor.examine_descriptor(hogDescriptor);
						//visitor.visited_location(i*xStep+xbound+1, j*yStep+ybound+1);
						visitor.visited_location(std::make_pair(i*xStep+1+xoffset, j*yStep+1+yoffset)); // the +1 is because of the gradient is shifted by 1
						//AnnoRect r(int(1.0f*posX/scale)+xoffset, int(1.0f*posY/scale)+yoffset, int(1.0*(posX+windowWidth)/scale)+xoffset, int(1.0f*(posY+windowHeight)/scale)+yoffset, margin);
						//r.setScale(1/scale);
					}
			}
			//cvReleaseImageHeader(&tmp);
		}
	}

	template<typename Visitor> // should be a DenseSlidingHOGMulitscaleVisitor
	bool denseSlidingHOGMultiScale(
			const HOGParams& hogParam, // HOG parameters
			const unsigned char* data, int width, int height, int rowstep, // image
			float minScale, float maxScale, float scaleStep, // scale parameters
			Visitor visitor, // visitor to customize the algorithm
			int scaleType = 1, // scaling type (0 = linear, 1 = log)
			int noSteps = 1)
	{
		unsigned int descriptorHeight = hogParam.m_nWindowHeight+2;
		unsigned int descriptorWidth =  hogParam.m_nWindowWidth+2;

		IplImage* iplimage = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
		cvSetData(iplimage, const_cast<unsigned char*>(data), rowstep);
		switch(scaleType){
			case 0: { // linear
						for(float scale= minScale; scale <= maxScale; scale+=scaleStep) {
							unsigned int h = static_cast<unsigned int>(height*1.0/scale);
							unsigned int w = static_cast<unsigned int>(width*1.0/scale);
							//if (h + hogParam.m_nCellHeight * hogParam.extraCells * 2 < descriptorHeight || // FIXME: extraCells > 0
							//		w + hogParam.m_nCellWidth * hogParam.extraCells * 2 < descriptorWidth)
							//	break;
							if (h < descriptorHeight ||
									w < descriptorWidth)
								break;

							// resize the image with opencv
							IplImage* tmp = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
							cvResize(iplimage, tmp, CV_INTER_LINEAR );

							visitor.new_scale(scale,(unsigned char*)(tmp->imageData), w, h, tmp->widthStep);

							denseSlidingHOGSingleScale(hogParam, 
									(unsigned char*)tmp->imageData, w, h, tmp->widthStep,
									visitor,
									noSteps);
							//counter+=testImage((unsigned char*)tmp->imageData, w, h, tmp->widthStep, anno, noSteps, 1.0f /scale);
							cvReleaseImage(&tmp);
						}
						break;
					}
			case 1: { // log
						for(float scale= minScale; scale <= maxScale; scale*=(1+scaleStep)) {
							unsigned int h = static_cast<unsigned int>(height*1.0/scale);
							unsigned int w = static_cast<unsigned int>(width*1.0/scale);
							//if (h + hogParam.m_nCellHeight * hogParam.extraCells * 2 < descriptorHeight || // FIXME: extraCells > 0
							//		w + hogParam.m_nCellWidth * hogParam.extraCells * 2 < descriptorWidth)
							//	break;
							if (h < descriptorHeight ||
									w < descriptorWidth)
								break;

							// resize the image with opencv
							IplImage* tmp = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
							cvResize(iplimage, tmp, CV_INTER_LINEAR );

							visitor.new_scale(scale,(unsigned char*)(tmp->imageData), w, h, tmp->widthStep);

							denseSlidingHOGSingleScale(hogParam, 
									(unsigned char*)tmp->imageData, w, h, tmp->widthStep,
									visitor,
									noSteps);
							//counter+=testImage((unsigned char*)tmp->imageData, w, h, tmp->widthStep, anno, noSteps, 1.0f /scale);
							cvReleaseImage(&tmp);
						}
						break;
					}
			default:
					{
						std::cerr << "ERROR: denseSlidingHOGMulitscale: Unknown Scale Type\n";
						return false;
					}
		}
		cvReleaseImageHeader(&iplimage);
		return true;
	}

} // namespace HOG

#endif
