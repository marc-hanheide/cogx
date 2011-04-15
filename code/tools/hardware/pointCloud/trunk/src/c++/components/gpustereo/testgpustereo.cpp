#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "CensusGPU.h"


int main(int argc, char *argv[]) {
	IplImage *ImgLeft, *ImgRight, *ImgResult;
	CensusGPU census(100);

  if(argc != 3) {
    printf("usage: %s <left rect image> <right rect image>\n", argv[0]);
    return 1;
  }

	ImgLeft = cvLoadImage(argv[1], -1);
	ImgRight = cvLoadImage(argv[2], -1);

	ImgResult = cvCreateImage(cvSize(ImgLeft->width, ImgLeft->height), IPL_DEPTH_8U, 1);


	cvSet(ImgResult, cvScalar(0));

	census.setImages(ImgLeft, ImgRight);
	for (int i = 0; i<10; i++) {
		census.setImages(ImgLeft, ImgRight);
		census.match();
		census.printTiming();
	}
	census.getDisparityMap(ImgResult);

	//cvSaveImage("C:\\images\\stereo-pairs\\tsukuba\\dm.png", ImgResult);

	cvNamedWindow("DM GPU");
	cvShowImage("DM GPU", ImgResult);
	cvWaitKey(0);


	cvFree(&ImgLeft);
	cvFree(&ImgRight);
	cvFree(&ImgResult);

	return 0;
}
