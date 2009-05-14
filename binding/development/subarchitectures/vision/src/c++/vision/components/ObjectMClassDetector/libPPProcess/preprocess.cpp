#include <libPPProcess/preprocess.h>
//#include <libTiffStreamReader/StreamReader.h>

using namespace std;

void blurEdges(QImage &blurMe, const int leftmost, const int topmost, const int rightmost, const int bottommost, const int kernelRad){
	    	
	//Blur on the top
 	for(int k = topmost - 1; k >= 0; k--)
    	for(int l = leftmost; l <= rightmost; l++) {
        	int left=  l - kernelRad;
            int right= l + kernelRad;
            if (left < leftmost)
            	left = leftmost;
            if (right > rightmost)
            	right = rightmost;
            
            float normFac= 1.0f / (right-left+1);
            int red= 0;
            int green= 0;
            int blue= 0;
            for (int i = left; i <= right; i++) {
				QRgb pixel= blurMe.pixel(i, k + 1);
				red+= qRed(pixel);
				green+= qGreen(pixel);
				blue+= qBlue(pixel);
			}
			QRgb blured= qRgb(static_cast<int>(round(normFac * red)), static_cast<int>(round(normFac * green)), static_cast<int>(round(normFac * blue)));
			blurMe.setPixel(l, k , blured);
		}
           
       
   //Blur on the bottom
   	for( int k = bottommost + 1; k <= blurMe.height() - 1; k++ )
		for(int l = leftmost; l <= rightmost; l++)  {
			int left = l - kernelRad;
			int right = l + kernelRad;
			if (left < leftmost)
				left = leftmost;
			if (right > rightmost)
				right = rightmost;
			float normFac = 1.0f / (right - left + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (int i = left; i <= right; i++) {
				QRgb pixel= blurMe.pixel(i, k - 1);
				red+= qRed(pixel);
				green+= qGreen(pixel);
				blue+= qBlue(pixel);
			}
			QRgb blured= qRgb(static_cast<int>(round(normFac * red)), static_cast<int>(round(normFac * green)), static_cast<int>(round(normFac * blue)));
			blurMe.setPixel(l, k,  blured);
		}
			
	//Blur on the right
	for(int l = rightmost; l < blurMe.width(); l++)  
		for( int k = 0; k < blurMe.height(); k++ ) {		
			int top = k - kernelRad;
			int bottom = k + kernelRad;
			if (top < 0)
				top = 0;
			if (bottom >= blurMe.height())
				bottom = blurMe.height() - 1;
			float normFac = 1.0f / (bottom - top + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (int i = top; i <= bottom; i++) {
				QRgb pixel= blurMe.pixel(l - 1, i);
				red+= qRed(pixel);
				green+= qGreen(pixel);
				blue+= qBlue(pixel);
			}
			QRgb blured= qRgb(static_cast<int>(round(normFac * red)), static_cast<int>(round(normFac * green)), static_cast<int>(round(normFac * blue)));
			blurMe.setPixel(l, k,  blured);
		}
		
	//Blur on the left
	for(int l = leftmost - 1; l >= 0; l--)  
		for( int k = 0; k < blurMe.height(); k++ ) {		
			int top = k - kernelRad;
			int bottom = k + kernelRad;
			if (top < 0)
				top = 0;
			if (bottom >= blurMe.height())
				bottom = blurMe.height() - 1;
			float normFac = 1.0f / (bottom - top + 1);
			int red = 0;
			int green = 0;
			int blue = 0;
			for (int i = top; i <= bottom; i++) {
				QRgb pixel= blurMe.pixel(l + 1, i);
				red+= qRed(pixel);
				green+= qGreen(pixel);
				blue+= qBlue(pixel);
			}
			QRgb blured= qRgb(static_cast<int>(round(normFac * red)), static_cast<int>(round(normFac * green)), static_cast<int>(round(normFac * blue)));
			blurMe.setPixel(l, k,  blured);
		}
		
	
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

bool adjustBoundingBox(const AnnoRect& rect, AnnoRect& adjustedRect, const preAndPostProcessParams& param) {
	int x1 = rect.x1();
	int x2 = rect.x2();
	if (x2 > x1) {
		int tmp = x2;
		x2 = x1;
		x1 = tmp;
	}
		
	int y1 = rect.y1();
	int y2 = rect.y2();
	if (y2 > y1) {
		int tmp = y2;
		y2 = y1;
		y1 = tmp;
	}
			
	unsigned int origwidth = x1 - x2;
	unsigned int origheight = y1 - y2;
		
	//Skip too small examples
	if (origwidth < param.minWidth || origheight < param.minHeight)
		return false;
			
	//Skip too big examples
	if ((origwidth > param.maxWidth  && param.maxWidth > 0) || (origheight > param.maxHeight && param.maxHeight > 0))
		return false;
		
	unsigned int width = origwidth;
	unsigned int height = origheight;
		
	//Determine which side needs to be extended
	if (1.0 * width / param.scaledWidth  < 1.0 * height / param.scaledHeight) {
		width =  static_cast<unsigned int>(round(1.0 * height / param.scaledHeight * param.scaledWidth));
	}
	else {
		height = static_cast<unsigned int>(round(1.0 * width / param.scaledWidth * param.scaledHeight));
	}
			
	//Add some extra boundary
	width = static_cast<unsigned int>(round(width * param.borderfactor));
	height = static_cast<unsigned int>(round(height * param.borderfactor)); 
			
	//Setup corrected coordinates
	x1 = x1 - origwidth / 2 + width / 2;
	y1 = y1 - origheight / 2 + height / 2;
	x2 = x1 - width;
	y2 = y1 - height;
	
	adjustedRect = AnnoRect(x2, y2, x1, y1);
	adjustedRect.setScale(width * 1.0f / param.scaledWidth);
	
	return true;
}


bool preProcessAnnotation(const AnnoRect& rect, const preAndPostProcessParams& param, const QImage& img, QImage& scaledObject) {
	int x1 = rect.x1();
	int x2 = rect.x2();
	if (x2 > x1) 
	{
		int tmp = x2;
		x2 = x1;
		x1 = tmp;
	}
	
	int y1 = rect.y1();
	int y2 = rect.y2();
	if (y2 > y1) 
	{
		int tmp = y2;
		y2 = y1;
		y1 = tmp;
	}
		
	unsigned int origwidth = x1 - x2;
	unsigned int origheight = y1 - y2;
	
	//Skip too small examples
	if (origwidth < param.minWidth || origheight < param.minHeight)
		return false;
		
	//Skip too big examples
	if ((origwidth > param.maxWidth  && param.maxWidth > 0) || (origheight > param.maxHeight && param.maxHeight > 0))
		return false;
	
	unsigned int width = origwidth;
	unsigned int height = origheight;
	
	//Determine which side needs to be extended
	if (1.0 * width / param.scaledWidth  < 1.0 * height / param.scaledHeight) {
		width =  static_cast<unsigned int>(round(1.0 * height / param.scaledHeight * param.scaledWidth));
	}
	else {
		height = static_cast<unsigned int>(round(1.0 * width / param.scaledWidth * param.scaledHeight));
	}
		
	//Add some extra boundary
	width = static_cast<unsigned int>(round(width * param.borderfactor));
	height = static_cast<unsigned int>(round(height * param.borderfactor)); 
	
	float scaleFactor = 1.0f * param.scaledWidth / width;
		
	//Setup corrected coordinates
	x1 = static_cast<int> (round((x1 - origwidth / 2 + width / 2) * scaleFactor));
	y1 = static_cast<int> (round((y1 - origheight / 2 + height / 2) * scaleFactor));
	x2 = x1 - param.scaledWidth;
	y2 = y1 - param.scaledHeight;
		
	QImage scaledImg = img.scaled(static_cast<unsigned int>(scaleFactor * img.width()), static_cast<unsigned int>(scaleFactor * img.height()), Qt::KeepAspectRatio, Qt::SmoothTransformation);
	scaledObject = scaledImg.copy(x2,y2, param.scaledWidth, param.scaledHeight);								
	/*
	 * Check if cropped area is bigger than input
	 * handle by blurring in last row/column 
	 */
	 if(x1 >= scaledImg.width() || y1 >= scaledImg.height() || x2 < 0 || y2 < 0) {	 	
	 	const int leftmost = x2 >= 0 ? 0 : -x2;
	 	const int rightmost = x1 >= scaledImg.width() ? scaledImg.width() - x2 - 1 : param.scaledWidth - 1;
	 	const int topmost = y2 >= 0 ? 0 : -y2; 
	 	const int bottommost = y1 >= scaledImg.height() ? scaledImg.height() - y2  - 1 : param.scaledHeight - 1;
	 	blurEdges(scaledObject, leftmost, topmost, rightmost, bottommost, param.blurKernelWidth);			 	
	 }	 
		 	
	 return true;		
} 


/*unsigned char* streamAnnoToPixmap(Annotation& anno, unsigned char* pixmap, unsigned int &width, unsigned int &height) {
	static StreamReader reader;
	
	std::string oldDir = reader.getBasePath();
	
	if(oldDir.compare(anno.imageName()) != 0) {
		reader.openStream(anno.imageName().c_str());
	}
		
	if (!reader.isValid()) {
		return 0;
	}
	
	reader.loadFrame(anno.frameNr());
	const unsigned short *img = reader.getColorFrame();
	width = reader.getWidth();
	height = reader.getHeight();
	
	if (pixmap) {
		delete[] pixmap;
		pixmap = 0;
	}
		
	pixmap = new unsigned char[width * height * 4];
		
	for(unsigned int i = 0; i < width * height; i++) {
		pixmap[i * 4] = static_cast<unsigned char>((img[i * 3 + 2] >> 8));
		pixmap[i * 4 + 1] = static_cast<unsigned char>((img[i * 3 + 1] >> 8 ));
		pixmap[i * 4 + 2] = static_cast<unsigned char>((img[i * 3] >> 8));
		pixmap[i * 4 + 3] = 255;			
	}
	
	return pixmap;	
} */

AnnotationList preProcess(AnnotationList& annotations, preAndPostProcessParams& param, const char* outDir) {
	AnnotationList newList;
	//StreamReader reader;
	unsigned int width;
	unsigned int height;
	unsigned char *imgPixmap = 0;
		
	for(unsigned int i = 0; i < annotations.size(); i++) 
	{
		//--- retrieve name, basename ---//
		std::string name = annotations[i].imageName();
		std::string basename = name;
		size_t pos = name.rfind("/");
		if (pos>0)
		{
			basename = name.substr(pos+1,string::npos);
		}
		fprintf(stderr, "Processing %s -> %s ...\n", name.c_str(), basename.c_str());

		//--- load image ---//
		QImage img;
		if (!annotations[i].isStream())
			img = QImage(name.c_str());
		else {
                        cerr <<"streams not supported"<<endl;
			//imgPixmap = streamAnnoToPixmap(annotations[i], imgPixmap, width, height);
			//img = QImage(imgPixmap, width, height, QImage::Format_RGB32);			
		} 
			
		//--- crop each annorect ---//
		for(unsigned int j = 0; j < annotations[i].size(); j++) 
		{
			QImage scaledObject;
			if (!preProcessAnnotation(annotations[i].annoRect(j), param, img, scaledObject))
				continue;
			
			QString fn;
			if (outDir != 0) {
				fn = QString(outDir) + "/" + QString::number(param.scaledWidth) + "x" + QString::number(param.scaledHeight) + 
				    "b" + QString::number(param.borderfactor) + "-"+QString(basename.c_str()) + "-" + QString::number(j);					
			} else
				fn = QString("/tmp/") + QString::number(param.scaledWidth) + "x" + QString::number(param.scaledHeight)+ "b" 
				  + QString::number(param.borderfactor) + "-"+QString(basename.c_str()) + "-" + QString::number(j);			
			
			scaledObject.save(fn + "-" + QString::number(i) + ".png", "PNG");			
			if (param.mirror == 1)
				scaledObject.mirrored(true, false).save(fn +  "-" + QString::number(i) + "-mirrored.png", "PNG");
			
			newList.addAnnotationByName(QString(fn + "-" + QString::number(i) + ".png").toStdString());
		}
	}
	
	if (imgPixmap)
		delete[] imgPixmap;
	
	return newList;
}

/*********************
 *
 *  Duplicates for jittering adds one Pixel boundary and context extensions
 * 
 */
void prepareBoundingBoxes(Annotation& annotations, const preAndPostProcessParams& param, int imageWidth, int imageHeight, int windowWidth, int windowHeight, bool addJitter, bool checkBorder) {
	Annotation newAnnotations(annotations);
	newAnnotations.clear();
	
	const int objectWidth = windowWidth - param.localContextL - param.localContextR;
	const int objectHeight = windowHeight - param.localContextT - param.localContextB;

	if (annotations.size() == 0) {
		AnnoRect newAnnoRect(imageWidth / 2 - objectWidth / 2, imageHeight / 2 - objectHeight / 2, imageWidth / 2 + objectWidth / 2, imageHeight / 2 + objectHeight / 2);
		newAnnoRect.setScale(1.0);
		annotations.addAnnoRect(newAnnoRect);
	}
	
	for(unsigned int j = 0; j < annotations.size(); j++) {
		
		// Add the additional context
		int x1 = annotations.annoRect(j).left() - static_cast<int>(roundf(param.localContextL) * annotations.annoRect(j).scale());
		int y1 = annotations.annoRect(j).top() - static_cast<int>(roundf(param.localContextT) * annotations.annoRect(j).scale());			
		int x2 = x1 + static_cast<int>(roundf((param.localContextL + objectWidth + param.localContextR) * annotations.annoRect(j).scale()));
		int y2 = y1 + static_cast<int>(roundf((param.localContextT + objectHeight + param.localContextB) * annotations.annoRect(j).scale()));
		
		int boundary = static_cast<int>(round(annotations[j].scale()));
		
		// Jitter initial annotations
		if (addJitter)
			for(int jitterX = - param.jitterX; jitterX <= param.jitterX; jitterX++) {
				for(int jitterY = - param.jitterY; jitterY <= param.jitterY; jitterY++) {
					if(!checkBorder || (x1 - 1 + jitterX >= 0 && y1 - 1 + jitterY >= 0 && x2 + jitterX < imageWidth && y2 + jitterY < imageHeight)) {
						newAnnotations.addAnnoRect(AnnoRect(x1 - boundary + jitterX , y1 - boundary + jitterY , x2 + boundary + jitterX, y2 + boundary + jitterY, annotations[j].score(), annotations[j].silhouetteID(), annotations[j].scale() ));												
					}
				}
			}
		else
			newAnnotations.addAnnoRect(AnnoRect( x1 - boundary , y1 - boundary, x2 + boundary, y2 + boundary, annotations[j].score(), annotations[j].silhouetteID(), annotations[j].scale() ));	
	}	
	annotations = newAnnotations;
}

void drawRandomSamples(Annotation &negFile, int imageWidth, int imageHeight, int samplesPerImage, float scaleSteps, int noScales, int windowWidth, int windowHeight) {
	//--- load image and crop random window --//
	fprintf(stdout, "Drawing random samples from image: %s\n", negFile.fileName().c_str());
			
	const int width = imageWidth;
	const int height = imageHeight;
	fprintf(stderr, "  wxh: %dx%d\n", width, height);
		
	for(int k=0; k < samplesPerImage; ++k)	{
		const float scale = pow(scaleSteps, rand() % noScales);
		int posX = 0;
		if ((static_cast<int>(width - 2 - windowWidth * scale)) != 0)
			posX = (rand() % (static_cast<int>(width - 2 - windowWidth * scale)));
			
		int posY = 0;
		if ((static_cast<int>(height - 2 - windowHeight * scale)) != 0 )
			posY = (rand() % (static_cast<int>(height - 2 - windowHeight * scale)));
			
		fprintf(stdout, "  pos: %dx%d:%.2f\n", posX, posY, scale);
		AnnoRect ar(static_cast<int>(1 + posX), static_cast<int>(1 + posY), 1 + posX + static_cast<int>(windowWidth * scale), 1 + posY + static_cast<int>(windowHeight * scale));
		ar.setScale(scale);
		//Remember the random crop
		negFile.addAnnoRect(ar);
	}
	
}

void reShapeBoundingBoxes(Annotation& annotation, const preAndPostProcessParams& param, int windowWidth, int windowHeight) {
	Annotation newAnnotation(annotation);
	newAnnotation.clear();
	
	for(unsigned int i = 0; i < annotation.size(); i++) {
		const double score = annotation[i].score();
		const float scale = annotation[i].scale();
		const int sil = annotation[i].silhouetteID();
				
		const int l = static_cast<int>(round(((annotation[i].left() * 1.0f / scale)  + param.localContextL) * scale));
		const int r = l + static_cast<int>(round( (windowWidth - param.localContextR - param.localContextL)* scale));
		const int t = static_cast<int>(round(((annotation[i].top() * 1.0f / scale) + param.localContextT) * scale));
		const int b = t + static_cast<int>(round((windowHeight - param.localContextT - param.localContextB) * scale));
		
		newAnnotation.addAnnoRect(AnnoRect(l,t,r,b,score,sil,scale));		
		//cout << "reShapeBoundingBoxes: " << annotation[i].left() << ", " << l << "\n";
	}
	
	annotation = newAnnotation;
}
