#include "SW_MultHOG.h"
#include "hog_features.h"
#include <libSVMdense/svm.h>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <libPPProcess/nonmaxsuppression.h>


inifile::IniFile SW_MultHOG::loadConfigAndModel(const char*  name,const char*  confFile, const char* svmModelFile){
    HOGSVM s_hogsvm;
    cout <<confFile<<endl;
    const inifile::IniFile iniConfFile = parseConfFile(confFile, s_hogsvm.swParams, s_hogsvm.ppParams, s_hogsvm.svmParams);
    parseHOG_parameters(iniConfFile, s_hogsvm.hogParams);
    if ( m_vHOGSVM.empty() ||
         true ) {//check all other conditions here
      bool binarymodel=true;
      s_hogsvm.m_svm = new SVMlight();
      s_hogsvm.m_svm->load_model(svmModelFile, int(binarymodel));
      
      s_hogsvm.hogDim = s_hogsvm.hogParams.m_nHOGHeight   * s_hogsvm.hogParams.m_nHOGWidth * \
                        s_hogsvm.hogParams.m_nBlockHeight * s_hogsvm.hogParams.m_nBlockWidth * \
                        s_hogsvm.hogParams.m_nHistogramBins;
      s_hogsvm.hogStepSizeX = (s_hogsvm.hogParams.m_nBlockWidth  * s_hogsvm.hogParams.m_nCellWidth) - s_hogsvm.hogParams.m_nBlockHorizontalOverlap;
      s_hogsvm.hogStepSizeY = (s_hogsvm.hogParams.m_nBlockHeight * s_hogsvm.hogParams.m_nCellHeight) -s_hogsvm.hogParams.m_nBlockVerticalOverlap;
      //lin_model_vec expects a properly sized float vec ...
      s_hogsvm.vlinsvm.resize(s_hogsvm.hogDim+1);
      if(m_imaxHogDim < s_hogsvm.hogDim) { m_imaxHogDim = s_hogsvm.hogDim;}
      const SVMLight_ns::MODEL *m = s_hogsvm.m_svm->get_model_ptr();
      cout <<">>> LOADED [ "<< confFile <<" / " << svmModelFile <<" ] hogDIM: "<<s_hogsvm.hogDim<<" svm size: "<<m->totwords<<endl; 
      if(m->totwords+1 != s_hogsvm.hogDim+1) { //static_cast<long>(lin_svm.size())) {
         cerr<<"ERROR: MODEL DIM != HOG DIM: "<< (m->totwords) <<" "<<s_hogsvm.hogDim<<endl;
      } else {
         SVMLight_ns::lin_model_vec((SVMLight_ns::MODEL *) m, s_hogsvm.vlinsvm);
         s_hogsvm.modelname = name;
         m_vHOGSVM.push_back(s_hogsvm);
        
      }
    } else {
      cerr << "SKIPPING "<< confFile <<" / " << svmModelFile <<endl;
    }
    return(iniConfFile);

}
int SW_MultHOG::testImage_slidingWindow(IplImage* img, Annotation& anno, const float scale, const float xoffset, const float yoffset, float xbound, float ybound) {	
		
	//Debugging
	//char fn[500];
	//sprintf(fn, "/tmp/debug-scale-%.2f.png", scale);
	//cvSaveImage(fn, img);
		
	int counter =  0;
        int scounter =0;
        int dcounter =0;	
	float* fv = new float[m_imaxHogDim];	
	
	//ROI handling
	unsigned char* startAddr = (unsigned char*)img->imageData;
	int rowStep = img->widthStep;
	int w = img->width;
	int h = img->height;
	if (img->roi) {
		startAddr = (unsigned char*)img->imageData + img->widthStep * img->roi->yOffset + img->roi->xOffset * 3;
		w = img->roi->width;
		h = img->roi->height;
	}
        /*
	for(unsigned int v = 0; v < features.size(); ++v) {
		features[v]->on_new_Scale(startAddr, w, h, rowStep, scale, img->roi->xOffset * 1.0f / scale, img->roi->yOffset * 1.0f / scale);
	} */
	ColorGradient cg(startAddr, w, h, rowStep, m_vHOGSVM.at(0).hogParams.m_nGammaCompression);
        CellGrid cgr(m_vHOGSVM.at(0).hogParams, cg,0, 0);
        BlockGrid bg(cgr, m_vHOGSVM.at(0).hogParams);
        for(int numc=0; numc< m_vHOGSVM.size();numc++) {
          
          HOGSVM & hs = m_vHOGSVM.at(numc);
          //
          //BlockGrid bg(cgr, hs.hogParams);// BlockGrid bg(cg, hs.hogParams);
          std::vector<Block> & m_vBlocks = bg.m_vBlocks;
          int blockSize = m_vBlocks[0].m_vValues.size();
          int m_nBlocksX = bg.m_nBlocksX; 
          int m_nBlocksY = bg.m_nBlocksY;
          int strideY      = hs.swParams.strideY;
          int strideX      = hs.swParams.strideX;
          float oohogStepSizeX = 1.0f / hs.hogStepSizeX;
          float oohogStepSizeY = 1.0f / hs.hogStepSizeY;
          int HOGWidth  = hs.hogParams.m_nHOGWidth;
          int HOGHeight = hs.hogParams.m_nHOGHeight;
          float minDetectionMargin = hs.svmParams.minDetectionMargin;
          register float margin;
          int hogDim = hs.hogDim;
          vector <float> & lin_svm_model = hs.vlinsvm;
	  for(int y = 1; y < h - 1; y += strideY ) {
            for(int x = 1; x < w - 1; x += strideX) {
			
			//float* nextfv = fv;
			bool invalidWindow = false;
                        margin=0;
			int blockPosX = static_cast<int>(round((x-1) * oohogStepSizeX));
	                int blockPosY = static_cast<int>(round((y-1) * oohogStepSizeY));
		
	                if (blockPosX + HOGWidth > bg.m_nBlocksX || blockPosY + HOGHeight > bg.m_nBlocksY){
                          scounter++;
		          continue; //return 0;
                        }
                        if (hs.swParams.normalizeCues > 0) {  //if normalizaion is needed we go witha a copy of of the descriptor
	                bg.getHOGDescriptor(blockPosX, blockPosY, fv);	
			++counter;
			normalizeCues(fv);
		        int ka=0; 
			for(; ka<hogDim; ka++) 
			    margin += lin_svm_model[ka]*fv[ka]; 
                        margin-= lin_svm_model[ka];
                        } else { // read descriptor directly from block structure don't copy!
                          const int iend = blockPosX + HOGWidth;
                          const int jend = blockPosY + HOGHeight;
                          assert(iend<=m_nBlocksX);
                          assert(jend<=m_nBlocksY);
                          assert(m_vBlocks.size()>0);
                          int cnt=0;
                          //float margin1=0;
                          //const unsigned int nValuesInBlock = m_vBlocks[0].m_vValues.size();
                          for(int j=blockPosY; j<jend; ++j)
                            for(int i=blockPosX; i<iend; ++i) {
                              std::vector<float>& bVs = m_vBlocks[j * m_nBlocksX +i].m_vValues;
			      for(unsigned int k = 0; k <blockSize; k++){
			              margin += lin_svm_model[cnt] * bVs[k];
                                      cnt++;
                              }
		            }
                          margin -= lin_svm_model[cnt];
                        }
			if (margin > minDetectionMargin) {
				//fprintf(stdout, "Prediction: Margin: %f, %d\n", margin,  xoffset);				
				int posX = static_cast<int>(round((x + xoffset - xbound) / scale));
				int posY = static_cast<int>(round((y + yoffset - ybound) / scale));
				int scaled_windowWidth = static_cast<int>(round(hs.swParams.windowWidth * 1.0f / scale));
				int scaled_windowHeight = static_cast<int>(round(hs.swParams.windowHeight * 1.0f / scale));
								
				AnnoRect r(posX, posY, posX + scaled_windowWidth, posY + scaled_windowHeight, margin);
				r.setScale( 1/ scale);
                                //set classifier id in silhouette ...
                                r.setSilhouetteID(numc);
				anno.addAnnoRect(r);
                                dcounter++;
                        }  //else {cout << margin<<endl;}
			
		}
	} 
        }
	delete[] fv;
	
        //cout << "counter: "<<counter<< " scounter: "<<scounter<<" dcounter: "<<dcounter<<endl;
	return counter;

}

int SW_MultHOG::testImage(IplImage* img, Annotation& anno, const int noSteps, const float scale, float xbound, float ybound) {
	fprintf(stdout, "[% 4dx% 4d]\n", img->width, img->height);

	// Maybe we should have noStepsX and noStepsY
	int stepX = m_vHOGSVM.at(0).swParams.strideX / noSteps;
	int stepY = m_vHOGSVM.at(0).swParams.strideY / noSteps;
	
	assert(stepX > 0 && stepY > 0);
	
	int counter = 0;
	for(int i = 0; i < noSteps; ++i) {		
		const int xoffset = i * stepX;
		const int yoffset = i * stepY;
		
		//fprintf(stdout, "xoff: %d, yoff: %d, ", xoffset, yoffset);		

		int roiX = 0;
		int roiY = 0;
		int roiW = img->width;
		int roiH = img->height;
		
		bool hadROI = false;
		if (img->roi) {
			roiX = img->roi->xOffset;
			roiY = img->roi->yOffset;
			roiW = img->roi->width;
			roiH = img->roi->height;
			hadROI = true;
		}
		
		cvSetImageROI(img, cvRect(roiX + xoffset, roiY + yoffset, roiW - xoffset, roiH - yoffset));		
		counter += testImage_slidingWindow(img, anno, scale, xoffset, yoffset, xbound, ybound );		
		cvResetImageROI(img);
		
		if (hadROI)
			cvSetImageROI(img, cvRect(roiX , roiY, roiW , roiH ));	
					
		
	}
	return counter;
	//fprintf(stdout, "  Checked %d windows on this scale\n", counter);
}

int SW_MultHOG::testImageMultiScale(IplImage* img, Annotation& anno, const int noSteps, float minScale, float maxScale, float scaleStep, int scaleType) {	
	int counter=0;
	const unsigned int descriptorHeight = m_vHOGSVM.at(0).swParams.windowHeight + 2;	
	const unsigned int descriptorWidth = m_vHOGSVM.at(0).swParams.windowWidth + 2;
	
	float minS = std::min(img->width * 1.0f / descriptorWidth, img->height * 1.0f / descriptorHeight);
	
	const int extraBoundary = static_cast<int>(round(m_vHOGSVM.at(0).ppParams.extraBoundary * minS));
	const int kernelWidth = m_vHOGSVM.at(0).ppParams.blurKernelWidth;
			
	const int xbound = extraBoundary;
	const int ybound = extraBoundary;
		
	//Add some extra boundary
	IplImage* processedImage = img;
	IplImage* extraBoundaryImage = 0;
		
	if (extraBoundary > 0) {
		extraBoundaryImage = cvCreateImage(cvSize(img->width + 2 * xbound, img->height + 2 * ybound), img->depth, img->nChannels);
			
		cvSetImageROI(extraBoundaryImage, cvRect(xbound, ybound, img->width, img->height));
		cvCopy(img, extraBoundaryImage);
		cvResetImageROI(extraBoundaryImage);
		
		blurEdges((unsigned char*)extraBoundaryImage->imageData, extraBoundaryImage->width, extraBoundaryImage->height,
			extraBoundaryImage->widthStep, xbound, ybound, extraBoundaryImage->width - 1 - xbound,
			extraBoundaryImage->height - 1 - ybound, kernelWidth);
		
		processedImage = extraBoundaryImage;
	}
	
	IplImage* lastImage = processedImage;
	float lastScale = 1.0;

	for(float scale= minScale; scale <= maxScale; scale = scaleType == 0 ? scale + scaleStep : scale * (1+scaleStep)) {
		
		unsigned int h = static_cast<unsigned int>(processedImage->height * 1.0/scale);
		unsigned int w = static_cast<unsigned int>(processedImage->width * 1.0/scale);
		
		if (h + 2 * m_vHOGSVM.at(0).ppParams.extraBoundary < descriptorHeight || w + m_vHOGSVM.at(0).ppParams.extraBoundary * 2 < descriptorWidth) break;
		fprintf(stdout, "*Scale: %02.2f ", scale);
	
		IplImage* scaled = cvCreateImage(cvSize(w,h), processedImage->depth, processedImage->nChannels);			
		cvResize(lastImage, scaled, CV_INTER_LINEAR);
	
		//fprintf(stdout, " adding border of %04dpix, ", m_vHOGSVM.at(0).ppParams.extraBoundary);
		int boundX = static_cast<int>(round(xbound * 1.0f / scale - m_vHOGSVM.at(0).ppParams.extraBoundary));
		int boundY = static_cast<int>(round(ybound * 1.0f / scale - m_vHOGSVM.at(0).ppParams.extraBoundary));
					
		cvSetImageROI(scaled, cvRect(boundX, boundY, scaled->width - 2 *boundX, scaled->height - 2 *boundY));			
		counter += testImage(scaled, anno, noSteps, 1.0f /scale, m_vHOGSVM.at(0).ppParams.extraBoundary, m_vHOGSVM.at(0).ppParams.extraBoundary);
		cvResetImageROI(scaled);
		
		//Debugging:
		//char fn[500];
		//sprintf(fn, "/tmp/gauss-%f.png", scale);			
		//cvSaveImage(fn, scaled);
	        //sprintf(fn, "/tmp/smoothed-%f.png", scale);	
		//cvSaveImage(fn, pCopy);
		
		//MipMap kind of resizing.. avoids of resizing artifacts while Gaussian smoothing is not necessary
		if(scale * (1 + scaleStep) / lastScale >= 2.0) {
			if(lastScale != 1.0) {
				cvReleaseImage(&lastImage);
				lastImage = 0;
			}
			lastImage = scaled;
			lastScale = scale;
		} else 
			cvReleaseImage(&scaled);					
	}
	
	if(lastImage != 0 && lastImage != processedImage)
		cvReleaseImage(&lastImage);
	
	//fprintf(stdout, "Checked %d windows on multiple scales\n", counter);
	
	if (extraBoundaryImage)
		cvReleaseImage(&extraBoundaryImage);
	cout <<endl;
	return counter;
}

void SW_MultHOG::normalizeCues(float* fv) {
	
	float* nextfv = fv;
	//for(unsigned int v = 0; v < features.size(); ++v) {
		unsigned int cueLength = m_vHOGSVM.at(0).hogDim;//features[v]->on_get_featureDim();
		
		if (m_vHOGSVM.at(0).swParams.normalizeCues == 1) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i];
			}
			//cout << sum << endl;
			sum = sum + cueLength;		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}
		}
		
		if (m_vHOGSVM.at(0).swParams.normalizeCues == 2) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}
		}
		
		if (m_vHOGSVM.at(0).swParams.normalizeCues == 4) {
			//L2 normalize each cue
			float sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
				if (nextfv[i] / sum > 0.2)
					nextfv[i] = 0.2;
				else
					nextfv[i] /= sum;
			}
			
			sum = 0;
			for(unsigned int i = 0; i < cueLength; ++i) {
				sum += nextfv[i] * nextfv[i];
			}
			//cout << sum << endl;
			sum = sqrt(sum + cueLength);		
			for(unsigned int i = 0; i < cueLength; ++i) {
				nextfv[i] = nextfv[i] / sum;
			}		
		}
		
	//	nextfv += cueLength;
	//}
} 

void blurEdges(unsigned char* data, unsigned int width, unsigned int height, unsigned int rowstep, 
		const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad){
	    	
	//Blur on the top
 	for(int k = topmost - 1; k >= 0; k--)
    	for(int l = leftmost; l <= rightmost; l++) {
        	int left=  l - kernelRad;
            unsigned int right= l + kernelRad;
            if (left < leftmost)
            	left = leftmost;
            if (right > rightmost)
            	right = rightmost;
            
            float normFac= 1.0f / (right-left+1);
            int red= 0;
            int green= 0;
            int blue= 0;
            for (unsigned int i = left; i <= right; i++) {
				const unsigned char* pixel = &(data[(k+1)*rowstep + i*3]);
				red+= pixel[0];
				green+= pixel[1];
				blue+= pixel[2];
			}
			unsigned char* destpixel = &(data[k*rowstep + l*3]);
			destpixel[0] = static_cast<int>(round(normFac * red));
			destpixel[1] = static_cast<int>(round(normFac * green));
			destpixel[2] = static_cast<int>(round(normFac * blue));
		}
       
   //Blur on the bottom
 	for( unsigned int k = bottommost + 1; k <= height - 1; k++ )
		for(int l = leftmost; l <= rightmost; l++)  {
			int left = l - kernelRad;
			unsigned int right = l + kernelRad;
			if (left < leftmost)
				left = leftmost;
			if (right > rightmost)
				right = rightmost;			
			float normFac = 1.0f / (right - left + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (unsigned int i = left; i <= right; i++) {
				const unsigned char* pixel = &(data[(k-1)*rowstep + i*3]);
				red+= pixel[0];
				green+= pixel[1];
				blue+= pixel[2];
			}
			unsigned char* destpixel = &(data[k*rowstep + l*3]);
			destpixel[0] = static_cast<int>(round(normFac * red));
			destpixel[1] = static_cast<int>(round(normFac * green));
			destpixel[2] = static_cast<int>(round(normFac * blue));
		}
	
	//Blur on the right
	for(unsigned int l = rightmost; l < width; l++)  
		for(unsigned int k = 0; k < height; k++ ) {		
			int top = k - kernelRad;
			unsigned int bottom = k + kernelRad;
			if (top < 0)
				top = 0;
			if (bottom >= height)
				bottom = height - 1;
			float normFac = 1.0f / (bottom - top + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (unsigned int i = top; i <= bottom; i++) {
				const unsigned char* pixel = &(data[i*rowstep + (l-1)*3]);
				red+= pixel[0];
				green+= pixel[1];
				blue+= pixel[2];
			}
			unsigned char* destpixel = &(data[k*rowstep + l*3]);
			destpixel[0] = static_cast<int>(round(normFac * red));
			destpixel[1] = static_cast<int>(round(normFac * green));
			destpixel[2] = static_cast<int>(round(normFac * blue));
		}
		
	//Blur on the left
	for(int l = leftmost - 1; l >= 0; l--)  
		for(unsigned int k = 0; k < height; k++ ) {		
			int top = k - kernelRad;
			unsigned int bottom = k + kernelRad;
			if (top < 0)
				top = 0;
			if (bottom >= height)
				bottom = height - 1;
			float normFac = 1.0f / (bottom - top + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (unsigned int i = top; i <= bottom; i++) {
				const unsigned char* pixel = &(data[i*rowstep + (l+1)*3]);
				red+= pixel[0];
				green+= pixel[1];
				blue+= pixel[2];
			}
			unsigned char* destpixel = &(data[k*rowstep + l*3]);
			destpixel[0] = static_cast<int>(round(normFac * red));
			destpixel[1] = static_cast<int>(round(normFac * green));
			destpixel[2] = static_cast<int>(round(normFac * blue));
		}
}
/*


int main(int argc, char ** argv){

 SW_MultHOG swmh;
 
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("bottles32x96.conf","bottles32x96.model"); 
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 swmh.loadConfigAndModel("mugs32.conf","mugs32.model");
 //swmh.loadConfigAndModel("bottles32x96.conf","bottles32x96.model"); 
 swmh.loadConfigAndModel("bottles32x96.conf","bottles32x96.model"); 
 
 //cerr <<"model loaded"<<endl;
 
 //char * tstimg = "/local/mmarinov/tmp1.png";
 char* tstimg = "/local/mmarinov/test640.png";
 IplImage* img = cvLoadImage(tstimg);//"/local/mmarinov/test8-0000000002.png");
 Annotation anno(tstimg);//"/local/mmarinov/test8-0000000002.png");
 if (img == 0 || img->nChannels != 3) {
   fprintf(stdout, "ERROR: Failed loading image: %s\n", "somenamegoesinhere");
   return 0;
 }
 //AnnotationList finalList;
 swmh.testImageMultiScale(img, anno, 1, 1.0, 100.0, 0.1, 1);
 cvReleaseImage(&img);
 cout<< "anno size = "<<  anno.size()<<endl;
 anno.sortByScore();
 for(int numc=0;numc< swmh.m_vHOGSVM.size(); numc++){
   AnnotationList finalList; 
   std::vector<float> bandwidth(3);
   bandwidth[0] = swmh.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthX;
   bandwidth[1] = swmh.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthY;
   bandwidth[2] = swmh.m_vHOGSVM.at(numc).ppParams.smoothingBandWidthScale;
   NonMaxSuppresion nms(bandwidth);
   //AnnotationList inputList("/local/mmarinov/test8-0000000002.png");
   Annotation classAnno(tstimg);
   cout<< "anno size = "<<  anno.size()<<endl;
   for(unsigned int j = 0; j < anno.size(); j++) {
     if(anno[j].score() >= swmh.m_vHOGSVM.at(numc).ppParams.minNMSScore && anno[j].silhouetteID() == numc) {
       anno[j].setSilhouetteID(-1);
       classAnno.addAnnoRect(anno[j]);
     }
   }
   nms.addDataPoints(classAnno);
   Annotation finalAnno(tstimg);//"/local/mmarinov/test8-0000000002.png"); 
   //finalAnno.setImageName("/local/mmarinov/test8-0000000002.png");
   nms.getModes(finalAnno, swmh.m_vHOGSVM.at(numc).ppParams.minDetectionsPerMode, 0.01, 100, swmh.m_vHOGSVM.at(numc).ppParams.scoreMode);
   //finalAnno.setImageName("/local/mmarinov/test8-0000000002.png");
   
   finalAnno.setImageName(tstimg);//"/local/mmarinov/test8-0000000002.png");
   finalList.addAnnotation(classAnno);
   std::ostringstream oss;
   oss <<"test8-class-"<<numc<<".idl";
   
   finalList.save(oss.str().c_str());
   finalList.clear();
   
   finalList.addAnnotation(finalAnno);
   std::ostringstream os;
   os <<"test8-class-"<<numc<<"-nmm.idl"; 
   finalList.save(os.str().c_str());
   
   
}

}
*/
