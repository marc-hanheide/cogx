/**
 * @file main
 * @author Johann Prankl
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#include "multiplatform.hpp"

#include <vector>
#include <time.h>
#include OCV_CV_H
#include OCV_CXCORE_H
#include OCV_HIGHGUI_H
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <math.h>
#include <float.h>
#include <limits.h>
#include "SMath.h"
//#include "ELLNamespace.h"
#include "syArray.hpp"
//#include "BlobEllHyp.h"
#include "syEllipse.hpp"
#include "syEdge.hpp"
#include "Intersection.h"
#include "syEllipseDetection.hpp"
#include "syEllMethodSubpixel.hpp"
#include "syEllMethodDualEllipse.hpp"
#include <map>
#include <list>
#include <sstream>

#define STEP true
//#define LOG_IMAGES

/*
 * Flexible parameters for projezierte Marken
 */

/*
#define MAX_DIAMETER 100
#define MIN_DIAMETER 5
#define MIN_CONTRAST 20
#define MAX_CONTRAST_DEV 15.0
#define RAD_IN_OUT 3.0
#define MAX_QUOTIENT 4.0
#define SCALE_FACTOR 1.0
#define MIN_SUPPORT	0.4
#define SUBPIXEL_FITERROR 0.
#define CANNY_LOW 40
#define CANNY_HIGH 50
*/

/*
 * CONFIGURATIONS
 * 1 = BLURRED SYNTETISCHE MARKEN
 * 2 = DSC_GRAY_*
 * 3 = NACHWEIS_HUNDERSTEL
 */

#define CONF 4
#define USE_COLOR true
#define ELLIPSE_METHOD 2 /* 1-No sub pixel, 2-Dual ellipse XXXX-Subpixel accuracy, */
#define SHOW_SCALED false
#define SMOOTH false

#if CONF == 1

	#define MAX_DIAMETER 200
	#define MIN_DIAMETER 3
	#define MIN_CONTRAST 20
	#define MAX_CONTRAST_DEV 15
	#define RAD_IN_OUT 3.0
	#define MAX_QUOTIENT 4.0
	#define SCALE_FACTOR 1
	#define MIN_SUPPORT	0.2
	#define SUBPIXEL_FITERROR 0.5
  #define KERNEL_SIZE 3
	#define CANNY_LOW 70
	#define CANNY_HIGH 140

	#define PLOT_AICON false
	#define LOG_ELLIPSES
	//#define LOG_IMAGES
	
	#define IMAGE "Cal_Image4_am.png"
	#define ELL_WHITE true
	#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/Synthetische Marken/Test2/"
	
	#define SHOW_TRANSF_ELLIPSES false
	#define DECODE_TARGETS false

#elif CONF == 2

	#define MAX_DIAMETER 100
	#define MIN_DIAMETER 10
	#define MIN_CONTRAST 20
	#define MAX_CONTRAST_DEV 15
	#define RAD_IN_OUT 4.0
	#define MAX_QUOTIENT 4.0
	#define SCALE_FACTOR 1.0
	#define MIN_SUPPORT	0.65
	#define SUBPIXEL_FITERROR 0.25

  #define KERNEL_SIZE 3
	#if KERNEL_SIZE == 3
		#define CANNY_LOW 70
		#define CANNY_HIGH 140
	#else
		#define CANNY_LOW 200
		#define CANNY_HIGH 250
	#endif
	
	//#define PLOT_AICON true
	#define PLOT_AICON false
	//#define LOG_ELLIPSES
	//#define LOG_IMAGES
	#define IMAGE "DSC_GRAY_2374.PNG"
	#define ELL_WHITE true
	#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/CodierteMarken/reelleBeispieldaten/photKulissenvermessung/"
	
	#define SHOW_TRANSF_ELLIPSES false
	#define DECODE_TARGETS false

#elif CONF == 3

	#define MAX_DIAMETER 200
	#define MIN_DIAMETER 10
	#define MIN_CONTRAST 20
	#define MAX_CONTRAST_DEV 15
	#define RAD_IN_OUT 3.0
	#define MAX_QUOTIENT 4.0
	#define SCALE_FACTOR 1.0
	#define MIN_SUPPORT	0.70
	#define SUBPIXEL_FITERROR 0.3
  #define KERNEL_SIZE 3
	#define CANNY_LOW 250
	#define CANNY_HIGH 300

	#define PLOT_AICON false
	#define LOG_ELLIPSES
	//#define LOG_IMAGES
	
	#define IMAGE "Cal_Image4_am.png"
	#define ELL_WHITE true
	#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/Nachweis-Hundertstel/Beispielbilder_20091203-01/Verschiebung 0.07px/"
	//#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/Nachweis-Hundertstel/Beispielbilder_20091201_01/Verschiebung 0.07px/"
	//#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/Verschiebetisch/Reihe2/"
	#define SHOW_TRANSF_ELLIPSES false
	#define DECODE_TARGETS false

#elif CONF == 4

	#define MAX_DIAMETER 150
	#define MIN_DIAMETER 10
// werden nur verwendet, wenn USE_COLOR = true
// RAD_IN_OUT in px bestimmt, wie weit entfernt von der Ellipsenkontur die Punkte für den Vergleich genommen werden
	#define RAD_IN_OUT 4.0
	#define MIN_CONTRAST 10
	#define MAX_CONTRAST_DEV 20.0

// max. Verhältnis A/B
	#define MAX_QUOTIENT 3.0
// ev. rausschmeissen, da vielleicht nicht überall verwendet:
	#define SCALE_FACTOR 1.0
// wird auf die Hypothesen angewandt, außer bei DualEllipse (hier wird nachträglich nochmal der Support berechnet)
	#define MIN_SUPPORT	0.6
// nur für Subpixel-Methode (durchschnittliche Abweichung der Subpixel von der berechneten Ellipse)
// kann aber auch für DualEllipse verwendet werden.
	#define SUBPIXEL_FITERROR 2.0

  #define KERNEL_SIZE 5
	#define CANNY_LOW 250
	#define CANNY_HIGH 300
/*Kalibrierbilder 11M mit KERNEL=5
	#define MAX_DIAMETER 200
	#define MIN_DIAMETER 30
	#define MIN_CONTRAST 20
	#define MAX_CONTRAST_DEV 5
	#define RAD_IN_OUT 3.0
	#define MAX_QUOTIENT 4.0
	#define SCALE_FACTOR 1.0
	#define MIN_SUPPORT	0.85
	#define SUBPIXEL_FITERROR 0.1
	#define CANNY_LOW 350
	#define CANNY_HIGH 400
*/

	#define PLOT_AICON false
	#define LOG_ELLIPSES
//	#define LOG_IMAGES
	
	#define IMAGE "image119Image_noise_0.png"
	#define ELL_WHITE true
	#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/highResolutionSyntheticImages/"
	//#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/nonGredeheteEllipsen/"
	
	#define SHOW_TRANSF_ELLIPSES true
	#define DECODE_TARGETS true

#elif CONF == 5
	#define MAX_DIAMETER 100
	#define MIN_DIAMETER 5
	#define MIN_CONTRAST 20
	#define MAX_CONTRAST_DEV 15.0
	#define RAD_IN_OUT 3.0
	#define MAX_QUOTIENT 4.0
	#define SCALE_FACTOR 1.0
	#define MIN_SUPPORT	0.25
	#define SUBPIXEL_FITERROR 1
  #define KERNEL_SIZE 3
	#define CANNY_LOW 30
	#define CANNY_HIGH 60
	
	#define PLOT_AICON false
	#define LOG_ELLIPSES
	//#define LOG_IMAGES
	
	#define IMAGE "Markers_gi.tif"
	#define ELL_WHITE false
	#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/ProjizierteMarken/Test3/"
	//#define DIR "/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/nonGredeheteEllipsen/"
	
	#define SHOW_TRANSF_ELLIPSES false
	#define DECODE_TARGETS false
#endif

#if defined WIN32 || defined WIN64
	#define DIR "T:\\work\\imageproc\\STB-ME\\Beispieldaten\\Synthetische Marken\\Test2\\"
  #define MARKER_IMAGE DIR IMAGE
//  #define MARKER_IMAGE "C:\\"IMAGE
#else
	#define MARKER_IMAGE DIR IMAGE
#endif

using namespace std;
//using namespace ELL;
using namespace Z;
using namespace RTE;

void DrawEll(IplImage *img, RTE::CzEllipse *ell);
void LinkEdges(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *edge, IplImage *img, IplImage *tmp, CzArray<RTE::CzEllipse*> &ells, IplImage *dx, IplImage *dy, IplImage *grayPicture); //test

bool CheckContrast(IplImage *dx, IplImage *dy, 
                   CzArray<CzEdgel> &points, 
                   CzEllipse *ell, 
                   IplImage *grayPicture);


void TraceEdge(IplImage *edge, IplImage *tmp,IplImage *img,int u, int v, uchar r, uchar g, uchar b, CzArray<CzVector2> &points);
int FindCodedSequence(string seq, CzArray<CzArray<int> > &groups, CzArray<string> &stdSequences);
void sortEllipses(CzArray<RTE::CzEllipse*> &ells, CzArray<RTE::CzEllipse*> &sortedElls, bool sortX);
//void Subpixel(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *dx, IplImage *dy, CzArray<CzEdgel> &points, RTE::CzEllipse *ell);
//void DualEllipse(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *dx, IplImage *dy, CzArray<CzEdgel> &points, RTE::CzEllipse *ell,IplImage *img);
 
inline uchar GetPixel(IplImage *img, int x, int y);
inline short GetPixelI16(IplImage *img, int x, int y);
inline void SetPixel(IplImage *img, int x, int y, uchar col);
inline void SetPixel3C(IplImage *img, int x, int y, uchar r, uchar g, uchar b);
inline float GetPixelFloat(IplImage *img, int x, int y);
inline bool InRange(IplImage *img, int x, int y, float ddx, float ddy);

/*
 * Dirty stuff to read Aicon files (copy pasted from excel to txt data, columns splitted with whitespace)
 */

struct POD
{
	float id;
	float x;
	float y;
	float a;
	float b;
	float mx;
	float my;
	float rx;
	float ry;
};

struct SSTD
{
	string id;
	string seq;
	//#define SHOW_TRANSF_ELLIPSES false
};

istream& operator >> (istream& ins, POD& datum)
{
  ins >> datum.id >> datum.x >> datum.y >> datum.mx >> datum.my >> datum.a >> datum.b >> datum.rx >> datum.ry;
  return ins;
}

istream& operator >> (istream& ins, SSTD& datum)
{
  ins >> datum.id >> datum.seq;
  return ins;
}

// dirty global variable for files with ellipse points and gradient
char *ellipses_prefix;

/***************************************************************************
 * Main
 */
int main ( int argc, char** argv ) 
{


	cvNamedWindow ( "Results", 1 );
  CzArray<RTE::CzEllipse*> ells;
	char markerImage[200];

  ellipses_prefix = (char*)"";
  IplImage *img = 0, *drwImg = 0, *img2=0;
  if (argc > 1) {
    if (argc > 2) {
			ellipses_prefix = argv[2];
		}
    img=cvLoadImage ( argv[1], CV_LOAD_IMAGE_GRAYSCALE );   //enforce 8bit grey
    if (img == 0) {
      cerr<<"Image: '";
      for (int i=0; argv[1][i] != 0; i++)
        cerr<<argv[1][i];
      cerr<<"' invalid!!!!"<<endl;
      return -1;
    }

		strcpy(markerImage,argv[1]);

  } else {
    img=cvLoadImage ( MARKER_IMAGE, CV_LOAD_IMAGE_GRAYSCALE );   //enforce 8bit grey
    if (img == 0) {
      cerr<<"Image '"<<MARKER_IMAGE"' invalid!!!!"<<endl;
      return -1;
    }

		strcpy(markerImage,MARKER_IMAGE);
  }

	/*IplImage * diff = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
	IplImage * black = cvLoadImage ("/home/aa/projects/ellipse_detection/STB-ME/Beispieldaten/ProjizierteMarken/Test3/BlackImage_gi.TIF", 0 );
	cvSub(img, black, img);
	IplImage * copyGray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	cvCopy(img, copyGray);
	double minVal, maxVal;
	cvMinMaxLoc(img, &minVal, &maxVal, NULL, NULL, NULL);
	cvThreshold(img, img, 20, 255, CV_THRESH_BINARY);*/
	
	cout << "Image" << markerImage << endl;
  drwImg = cvCreateImage(cvSize(1024,700), IPL_DEPTH_8U, 3);
  img2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
  cvConvertImage(img,img2);
	

/*
	cout<<"-----------------"<<endl;
  for (int i=0;i<img->width;i++) {
  	uchar xxx=((uchar*)(img->imageData + img->widthStep*15))[i];
    int xyy = xxx;
  	cout<<xyy<<"-";
  }
	cout<<endl<<"-----------------"<<endl;
*/



	/*
   * Read file with AICON ellipse coordinates
	 *

	if(PLOT_AICON) {
		CzArray<RTE::CzEllipse*> AICONElls(100);
		RTE::CzEllipse* ell;
		AICONElls.Clear();

		string fileName = "res/AICON_";
		fileName.append(IMAGE);
		fileName.append(".txt");
		cout << fileName.c_str() << endl;
		ifstream file(fileName.c_str());
		POD data[ 100 ];	
		int num_elements;
		for (num_elements = 0; num_elements < 100; num_elements++) {
			if (!(file >> data[ num_elements ])) break;
			ell = new RTE::CzEllipse(data[num_elements].x,data[num_elements].y,data[num_elements].a,data[num_elements].b,acos(data[num_elements].rx));
			AICONElls.PushBack(ell);
			DrawEll(img2, ell);		
		}
	
		file.close();
		cout << "size of aicon ellipsen:" << AICONElls.Size() << endl;
		cvResize(img2,drwImg);

		fileName.append(".png");
		cvSaveImage(fileName.c_str(), drwImg);
		cvShowImage("Results",drwImg);
		cvWaitKey(0); 
		cvConvertImage(img,img2);
	}

	/*
	 * Read AICON standard sequences file
	 *

	int num_elements;
	string file2 = "/home/aa/projects/sigsoft/aiconMarkers/14BitStandardSequences.txt";
	ifstream file(file2.c_str());
	SSTD data [516];
	CzArray<string> stdSequences(516);
	CzArray<CzArray<int> > groups(9);
	groups.Clear();

	for(int g=0; g < 8; g++) {
		CzArray<int> group(100);
		group.Clear();	
		groups.PushBack(group);	
	}

	stdSequences.Clear();
	for (num_elements = 0; num_elements < 516; num_elements++) {
		if (!(file >> data[ num_elements ])) break;
		stdSequences.PushBack(data[num_elements].seq);
		
		//group by number of intersections to speed up search! (0,2,4,6,8,10,12,14)
		groups[stdSequences[num_elements].length()/2].PushBack(num_elements);
	}

//	for(int g=0; g < 8; g++) {
//		cout << "group " << g << " has a length of:" << groups[g].Size() << endl;
//	}

*/


   /********** CANNY, EDGE LINKING AND ELLIPSE FITTING *************/


   struct timespec start1, end1;//start2, start3, end1, end2, end3, end4;  


   CzEllipseDetection *detect = new CzEllipseDetection();

   CzEllipseDetection::sEllipseDetectionConfig *config = detect->GetConfig();
   // initialize configuration with default values
   // possible values:
   //    ELL_CONFIGMETHOD_HYPOTHESES   : no subpixel calculation (fast, most inexact)
   //    ELL_CONFIGMETHOD_DUAL_ELLIPSE : dual ellipse method (slow, most precise)
   config->eMethod = (CzEllipseDetection::eEllipseDetectionConfigMethod)(ELLIPSE_METHOD-1);
   
   // minimum/maximum ellipse diameter in pixel
   config->iMinEllipseDiameter = MIN_DIAMETER;
   config->iMaxEllipseDiameter = MAX_DIAMETER;
   
   // minimum contrast to the background, around 20.0
   config->iMinContrast = MIN_CONTRAST;
   
   // maximum standard deviation of ellipse points
   // use values 5.0 to 15.0
   config->dMaxContrastDev = MAX_CONTRAST_DEV;
   
   // factor outer ellipse to inner ellipse
   config->dRadiusInOut = RAD_IN_OUT;
   
   // maximum quotient long to short ellipse axis
   config->dMaxQuotient = MAX_QUOTIENT;
   
   // minimum pixel support 
   // useful range: 0.6 to 1.0
   config->dMinSupport = MIN_SUPPORT;
   
   config->dMaxHypFiterror = SUBPIXEL_FITERROR;

   // aperture size of sobel operator 
   // use values 3 or 5
   config->iSobelApertureSize = KERNEL_SIZE;
   
   // canny edge detector, low/high border 
   // depend on sobel aparture size
   // usful default values:
   //    sobel aparture = 3: canny low/high = 70/140
   //    sobel aparture = 5: canny low/high = 200/250
   config->iCannyLow = CANNY_LOW;
   config->iCannyHigh = CANNY_HIGH;
   
   config->bGaussSmooth = SMOOTH;

   config->iLinkingGrowCount = 40;

   config->dDualEllipseMinGradient = 10.0;
   config->iDualEllipseBorder = 3;

   // maximum average fiterror (only used with subpixel method)
   // useful range: 0.05 to 0.25
   config->dMaxFiterror = 2.0;
   
   // find white or black ellipses
   config->bEllipseWhite = ELL_WHITE;
   
   // decode the code of the marker around the ellipses
   config->bDecodeMarker = DECODE_TARGETS;

   config->eMarkerCode = CzEllipseDetection::ELL_MARKERCODE_12BIT;
   // quotient outermost to innermost radius of marker
   config->dMarkerQuotientOuterInner = 4.0;
   // outer ring width in pixels
   config->iMarkerRingWidth = 4;

   // debugging options
   // directory to store debugging images and log file
   config->sDebugDirectory = "";
   // create debugging images
   config->bDebugImages = true;
   // create log files
   config->bLogFiles = true;





   getRealTime(&start1);


//for (int i=0; i<10; i++) {
   detect->FindEllipses(img);


   getRealTime(&end1);


cout<<"Found ellipses: "<<detect->GetEllipsesCount()<<endl;
cout<<"Overall time [s]: "<<timespec_diff(&end1, &start1)<<endl;


	#ifdef LOG_ELLIPSES
  char filename[300];
  sprintf(filename, "%sEllipses.csv", ellipses_prefix);
  std::ofstream ellsfile(filename, ios::out);
	#endif

  for (int i=0; i<detect->GetEllipsesCount(); i++)
  {
    CzEllipse *ell = detect->GetEllipse(i);
    DrawEll(img2, ell);
		#ifdef LOG_ELLIPSES
    ellsfile <<"Ellipse " << std::setprecision(10)
             <<i<<" ; "
             <<ell->dX<<" ; "
             <<ell->dY<<" ; ; ; "
             <<ell->dA*2<<" ; "
             <<ell->dB*2<<" ; "
             <<ell->dPhi<<endl;
    DrawEll(img2, ell);
							CvFont myFont;
							cvInitFont(&myFont, CV_FONT_HERSHEY_DUPLEX, 1.5, 1.5, 0, 3);

							std::ostringstream sin;
							sin << ell->iMarkerID;
							std::string val = sin.str();

							cvPutText(img2, val.c_str(), cvPoint(ell->dX + 18,ell->dY+18), &myFont, cvScalar(0.0));
							cvPutText(img2, val.c_str(), cvPoint(ell->dX + 22,ell->dY+18), &myFont, cvScalar(0.0));
							cvPutText(img2, val.c_str(), cvPoint(ell->dX + 22,ell->dY+22), &myFont, cvScalar(0.0));
							cvPutText(img2, val.c_str(), cvPoint(ell->dX + 18,ell->dY+22), &myFont, cvScalar(0.0));
							cvPutText(img2, val.c_str(), cvPoint(ell->dX + 20,ell->dY+20), &myFont, CV_RGB(255,255,255));

		#endif    
  }

  #ifdef LOG_ELLIPSES 
	ellsfile.close();
	#endif

  cvResize(img2,drwImg);

   if(img2->width > drwImg->width)  
      cvShowImage( "Results", drwImg );
   else
      cvShowImage( "Results", img2 );

	cvWaitKey(0);

   detect->Clear();
//}
  cvDestroyWindow("Results");

  if (img!=0) cvReleaseImage(&img);
  if (img2!=0) cvReleaseImage(&img2);
  if (drwImg!=0) cvReleaseImage(&drwImg);

   delete detect;

   return 0;













/*
   ********** CANNY, EDGE LINKING AND ELLIPSE FITTING *************

  IplImage *edge=cvCreateImage( cvGetSize(img), img->depth, img->nChannels );
  cvZero(edge);

  getRealTime(&start1);  //canny

	if(SMOOTH) {
  	cvSmooth( img, edge, CV_BLUR, 3, 3, 0, 0 );
	} else {
		cvCopy(img, edge);
	}
	//edge = img;
  CzEdge *ce = new CzEdge();
  IplImage *dx = cvCreateImage( cvGetSize(img), IPL_DEPTH_16S, 1 );
  IplImage *dy = cvCreateImage( cvGetSize(img), IPL_DEPTH_16S, 1 );   

  ce->Set(false, KERNEL_SIZE);
  ce->Sobel(edge, dx, dy);
	ce->Canny(dx, dy, edge, CANNY_LOW, CANNY_HIGH);
  getRealTime(&end1);
  getRealTime(&start2);
  IplImage *tmp=cvCreateImage( cvGetSize(img), img->depth, img->nChannels );
  LinkEdges(config, edge, img2, tmp, ells, dx, dy, img);
  
	//LinkEdges(edge, img2, tmp, ells, dx, dy, copyGray);
  getRealTime(&end2);
  getRealTime(&end3);
  cvReleaseImage(&edge);
  cvReleaseImage(&tmp);
  
/*
	#ifdef LOG_ELLIPSES
  char filename[300];
  sprintf(filename, "%sEllipses.csv", ellipses_prefix);
  std::ofstream ellsfile(filename, ios::out);
	#endif

  for (unsigned i=0; i<ells.Size(); i++)
  {
    DrawEll(img2, ells[i]);
		#ifdef LOG_ELLIPSES
    ellsfile <<"Ellipse " << std::setprecision(10)
             <<i<<" ; "
             <<ells[i]->dX<<" ; "
             <<ells[i]->dY<<" ; ; ; "
             <<ells[i]->dA*2<<" ; "
             <<ells[i]->dB*2<<" ; "
             <<ells[i]->dPhi<<endl;
		#endif    
  }

  #ifdef LOG_ELLIPSES 
	ellsfile.close();
	#endif
****
  cout<<"Ellipses found: "<<ells.Size()<<endl;

	//We should be able to merge these, i have a matlab script to compare relative precission that needs the files like that
	//that is to compare the sinthetic ellipses without searching around an ellipse like done in evaluate, one on one comparison
	getRealTime(&start3);
	CzArray<RTE::CzEllipse*> sortedElls(100);
	sortedElls.Clear();
	sortEllipses(ells,sortedElls,false);
	getRealTime(&end4);
	cout<<"Time sort ellipses[s]:" << timespec_diff(&end4, &start3)<<endl;

	#ifdef LOG_ELLIPSES
	ofstream myfile;

	strcat(markerImage,"res.txt");
  myfile.open(markerImage);
	#endif

	#ifdef LOG_ELLIPSES
	ells = sortedElls;
	for (unsigned i=0; i<ells.Size(); i++)
		{
			 myfile << "Ellipse " << std::setprecision(20)
											 			<<i<<";"
				                    <<ells[i]->dX<<" ; "
				                    <<ells[i]->dY<<" ; ; ;"
				                    <<ells[i]->dA*2<<" ; "
				                    <<ells[i]->dB*2<<" ; "
				                    <<ells[i]->dPhi<<endl;
		}
	#endif
	
	#if DECODE_TARGETS
		//int codeStart=491; //useful to generate the standard sequences from AICON coded targets
		getRealTime(&start3);
		for (unsigned i=0; i<ells.Size(); i++)
		{
			if(ells[i]->dA > 5 /*&& ells[i]->dX > 300 && ells[i]->dX < 1500 && ells[i]->dY > 2000****) {
				ells[i]->dPhi *= -1;
				double fact=3.04;
				float x0 = -ells[i]->dA*fact;
				float y0 = -ells[i]->dB*fact;

/***begin
				float x1 = ells[i]->dA*fact;
				float y1 = ells[i]->dB*fact;

				float x2 = -ells[i]->dA*fact;
				float y2 = ells[i]->dB*fact;

				float x3 = ells[i]->dA*fact;
				float y3 = -ells[i]->dB*fact;
end***
				float x00 = x0 * cos(ells[i]->dPhi) + y0 * sin(ells[i]->dPhi);
				float y00 = x0 * -sin(ells[i]->dPhi) + y0 * cos(ells[i]->dPhi);

/***begin
				float x10 = x1 * cos(ells[i]->dPhi) + y1 * sin(ells[i]->dPhi);
				float y10 = x1 * -sin(ells[i]->dPhi) + y1 * cos(ells[i]->dPhi);

				float x20 = x2 * cos(ells[i]->dPhi) + y2 * sin(ells[i]->dPhi);
				float y20 = x2 * -sin(ells[i]->dPhi) + y2 * cos(ells[i]->dPhi);

				float x30 = x3 * cos(ells[i]->dPhi) + y3 * sin(ells[i]->dPhi);
				float y30 = x3 * -sin(ells[i]->dPhi) + y3 * cos(ells[i]->dPhi);

				cvCircle(img2, cvPoint(ells[i]->dX + x00,ells[i]->dY+y00), 5, CV_RGB(0,0,255), 5);
				cvCircle(img2, cvPoint(ells[i]->dX + x10,ells[i]->dY+y10), 5, CV_RGB(0,0,255), 5);
				cvCircle(img2, cvPoint(ells[i]->dX + x20,ells[i]->dY+y20), 5, CV_RGB(0,0,255), 5);
				cvCircle(img2, cvPoint(ells[i]->dX + x30,ells[i]->dY+y30), 5, CV_RGB(0,0,255), 5);

				cvLine(img2, cvPoint(ells[i]->dX + x00,ells[i]->dY+y00), cvPoint(ells[i]->dX + x20,ells[i]->dY+y20), CV_RGB(0,0,255), 2);
				cvLine(img2, cvPoint(ells[i]->dX + x20,ells[i]->dY+y20), cvPoint(ells[i]->dX + x10,ells[i]->dY+y10), CV_RGB(0,0,255), 2);
				cvLine(img2, cvPoint(ells[i]->dX + x10,ells[i]->dY+y10), cvPoint(ells[i]->dX + x30,ells[i]->dY+y30), CV_RGB(0,0,255), 2);
				cvLine(img2, cvPoint(ells[i]->dX + x30,ells[i]->dY+y30), cvPoint(ells[i]->dX + x00,ells[i]->dY+y00), CV_RGB(0,0,255), 2);
end***
				//compute src size where the whole ring code fits
				double desiredRadius = 12;
				IplImage * src = cvCreateImage(cvSize(ells[i]->dA*fact*2,ells[i]->dB*fact*2), img->depth, img->nChannels);
				IplImage * dst = cvCreateImage(cvSize(ells[i]->dA*fact*2,ells[i]->dB*fact*2), img->depth, img->nChannels);
				IplImage * dst2 = cvCreateImage(cvSize(desiredRadius*fact*2,desiredRadius*fact*2), img->depth, img->nChannels);

				cvGetRectSubPix( img, src, cvPoint2D32f(ells[i]->dX,ells[i]->dY));

				double a[] = { 	cos(ells[i]->dPhi),  sin(ells[i]->dPhi),  ells[i]->dX,
		             				-sin(ells[i]->dPhi),  cos(ells[i]->dPhi),  ells[i]->dY};

				ells[i]->dA /= desiredRadius;
				ells[i]->dB /= desiredRadius; 
				double scale[] = { 	1./ells[i]->dA,  0,  0,
		             						0,  1./ells[i]->dB,  0,
		             						0, 0, 1};

				CvMat Ma=cvMat(2, 3, CV_64FC1, a);
				CvMat MScale=cvMat(3, 3, CV_64FC1, scale);

				cvGetQuadrangleSubPix(img, dst,&Ma);
				cvWarpPerspective( dst, dst2, &MScale, CV_INTER_CUBIC);
				double minVal, maxVal;
				cvMinMaxLoc(dst2, &minVal, &maxVal, NULL, NULL, NULL);
				




          char filename[300];
          sprintf(filename, "MarkerImage-%.3d.PNG", i);
			  	cvSaveImage(filename, dst2);

				cvThreshold(dst2, dst2, (maxVal - minVal) / 2, 255, CV_THRESH_BINARY);

          sprintf(filename, "MarkerImageTh-%.3d.PNG", i);
			  	cvSaveImage(filename, dst2);








				IplImage * mask = cvCreateImage(cvSize(desiredRadius*fact*2,desiredRadius*fact*2), img->depth, img->nChannels);	
				IplImage * src2 = cvCreateImage(cvSize(desiredRadius*fact*2,desiredRadius*fact*2), img->depth, img->nChannels);
				IplImage * dstSum = cvCreateImage(cvSize(desiredRadius*fact*2,desiredRadius*fact*2), img->depth, img->nChannels);
				cvZero(src2);
				cvZero(dstSum);
				cvZero(mask);

//GO				cvCircle(mask, cvPoint(dst2->width/2, dst2->height/2), desiredRadius*1.5, CV_RGB(255,255,255), 4);
				cvAdd(dst2, src2, dstSum, mask);
				
				CvScalar sumedMask = cvSum(dstSum);
				if(sumedMask.val[0] > 0) {
						//stop decoding, mark is not clean
						continue;
				}

//GO				cvCircle(dst2, cvPoint(dst2->width/2, dst2->height/2), desiredRadius*1.5, CV_RGB(255,255,255), 4);
				double fact2=2.5;
				CvPoint p1,p2;
				CzArray<Intersection> intersections(100);
				intersections.Clear();
				p1 = cvPoint(desiredRadius*fact + desiredRadius*fact2*cos(0*M_PI/180),desiredRadius*fact + desiredRadius*fact2*sin(0*M_PI/180));

				for(unsigned j=1; j <= 360; j=j+1) {
						p2 = cvPoint(desiredRadius*fact + desiredRadius*fact2*cos((j)*M_PI/180),desiredRadius*fact + desiredRadius*fact2*sin((j)*M_PI/180));
						if(abs(GetPixel(dst2, p1.x, p1.y) - GetPixel(dst2, p2.x, p2.y)) > 60) {
								Intersection inter;
								inter.p=p1;
								inter.blackToWhite=(GetPixel(dst2, p1.x, p1.y) < GetPixel(dst2, p2.x, p2.y));
								inter.angle=j;
								intersections.PushBack(inter);
						}
						p1=p2;
				}

//				cout << "Intersections:" << intersections.Size() << endl;
				bool ERRTarget;
				if(intersections.Size() > 0) {
					ERRTarget=false;
					string seq;
					seq = "";
					int totalNum=0;

					for(unsigned j=0; j < intersections.Size(); j++) {
							//cout << "Intersection " << j << " " << intersections[j].blackToWhite << " " << intersections[j].angle << endl;
//GO							cvCircle(dst2, intersections[j].p, 3, CV_RGB(0,0,255), 1);
							if(j > 0) {
								int num = round((intersections[j].angle - intersections[j-1].angle) / (360. / 14.));
								totalNum += num;
								if(num == 0) {
									//error with intersections!!
									ERRTarget=true;
//									cout << "Mark cannot be decoded" << endl;
									break;
								}

								if(intersections[j].blackToWhite) {
									seq += char(64+num);
								} else {
									seq += char(96+num);
								}
							}
					}
			
					//last intersection
					if(intersections[intersections.Size()-1].angle > intersections[0].angle)
						intersections[0].angle += 360;
			
					int num = round(abs(intersections[intersections.Size()-1].angle - intersections[0].angle) / (360. / 14.));
					totalNum += num;					
					if(intersections[0].blackToWhite) {
						seq += char(64+num);
					} else {
						seq += char(96+num);
					}

					if(!ERRTarget) {
						//cout << "std. seq" << i+codeStart << ";" << seq << endl;
						//find sequence in standard sequences!!
						int code = FindCodedSequence(seq, groups, stdSequences);

						if(code!=0) {
//							cout << "Coded target:" << code << " seq:" << seq << " total num:" << totalNum <<  endl;
							CvFont myFont;
							cvInitFont(&myFont, CV_FONT_HERSHEY_DUPLEX, 1.5, 1.5, 0, 3);

							std::ostringstream sin;
							sin << code;
							std::string val = sin.str();

							cvPutText(img2, val.c_str(), cvPoint(ells[i]->dX + x00+2,ells[i]->dY+y00+2), &myFont, cvScalar(0.0));
							cvPutText(img2, val.c_str(), cvPoint(ells[i]->dX + x00,ells[i]->dY+y00), &myFont, CV_RGB(255,255,255));
						} else {
//							cout << "Unable to decode target:" << seq << " total num:" << totalNum << endl;
						}

					}
				}

				#if SHOW_TRANSF_ELLIPSES
/*					cvNamedWindow("window");
					cvNamedWindow("window2");
					cvShowImage("window2", dst);
					cvShowImage("window", dst2);
					cvWaitKey(0);*/
/*          char filename[300];
          sprintf(filename, "MarkerImage-%0.3d.PNG", i);
			  	cvSaveImage(filename, dst2);*******
				#endif
			}  		}
		cout<<"Time transform and decoding[s]:" << timespec_diff(&end4, &start3)<<endl;
		getRealTime(&end4);
	#endif	

	#ifdef LOG_ELLIPSES
  myfile.close();
	#endif

	for (unsigned i=0; i<ells.Size(); i++) delete ells[i];
  ells.Clear();

  cout<<"Time canny[s]: "<<timespec_diff(&end1, &start1)<<endl;
  cout<<"Time linking [s]: "<<timespec_diff(&end2, &start2)<<endl;
  cout<<"Overall time [s]: "<<timespec_diff(&end3, &start1)<<endl;

  cvResize(img2,drwImg);

   if(img2->width > drwImg->width)  
      cvShowImage ( "Results", drwImg );
   else
      cvShowImage ("Results", img2);
  #ifdef LOG_IMAGES
  cvSaveImage("resCanny.jpg",img2);
  #endif

	//save images
	#ifdef LOG_IMAGES
	string fileName = "res/SigSoft_";
	fileName.append(IMAGE);
	cvSaveImage(fileName.c_str(), img2);
	#endif

	cvWaitKey(0);
  cvDestroyWindow("Results");

  if (img!=0) cvReleaseImage(&img);
  if (img2!=0) cvReleaseImage(&img2);
  if (drwImg!=0) cvReleaseImage(&drwImg);


  return 0;*/
}


/********************************** SOME METHODES ****************************/

void sortEllipses(CzArray<RTE::CzEllipse*> &ells, CzArray<RTE::CzEllipse*> &sortedElls, bool sortX) {
	for (unsigned i=0; i<ells.Size(); i++)
  {
		float min,min2;
		if(sortX) {
			min=ells[i]->dX;
			min2=ells[i]->dY;
		} else {
			min=ells[i]->dY;
			min2=ells[i]->dX;
		}

		for(unsigned j=i; j<ells.Size(); j++) {

			if(sortX) {
				if(ells[j]->dX < min && (ells[j]->dX - min) < -1) {
					min = ells[j]->dX;
					min2 = ells[j]->dY;
					CzEllipse * tmp;
					tmp = ells[i];
					ells[i] = ells[j];
					ells[j] = tmp;
				} else if((ells[j]->dX - min) > -1 && (ells[j]->dX - min) < 1) {
					if(ells[j]->dY < min2) {
							min2 = ells[j]->dY;
							CzEllipse * tmp;
							tmp = ells[i];
							ells[i] = ells[j];
							ells[j] = tmp;
					}
				}
			} else {
				//sort by Y
				if(ells[j]->dY < min && (ells[j]->dY - min) < -1) {
					min = ells[j]->dY;
					min2 = ells[j]->dX;
					CzEllipse * tmp;
					tmp = ells[i];
					ells[i] = ells[j];
					ells[j] = tmp;
				} else if((ells[j]->dY - min) > -1 && (ells[j]->dY - min) < 1) {
					if(ells[j]->dX < min2) {
							min2 = ells[j]->dX;
							CzEllipse * tmp;
							tmp = ells[i];
							ells[i] = ells[j];
							ells[j] = tmp;
					}
				}
			} 
		}
		sortedElls.PushBack(ells[i]);
	}
}

inline bool Match(string stdSeq, string seq) {
	if(stdSeq.compare(seq) == 0) return true;
	for(uint i=0; i < stdSeq.length(); i++) {
		//take first character of seq and put it at the end
		string seq2;
		char first = seq[0];
		seq2 = seq.substr(1,seq.length()-1);
		seq2.push_back(first);
		if(stdSeq.compare(seq2) == 0) return true;
		seq=seq2;
	}
 
	return false;
}

int FindCodedSequence(string seq, CzArray<CzArray<int> > &groups, CzArray<string> &stdSequences) {
	uint g = seq.length()/2;
	//cout << "group " << g << " has a length of:" << groups[g].Size() << endl;
	for(uint j=0; j < groups[g].Size(); j++) {
		//cout << stdSequences[groups[g][j]] << endl;
		if(Match(stdSequences[groups[g][j]], seq)) {
			//cout << "Sequence found " <<  seq << " " << stdSequences[groups[g][j]] << endl;
			return groups[g][j] + 1;
		} 
	}

	return 0;
}

void DrawEll(IplImage *img, RTE::CzEllipse *ell)
{
  cvEllipse(img, cvPoint((int)(ell->dX+.5),(int)(ell->dY+.5)), cvSize((int)(ell->dA+.5), (int)(ell->dB+.5)), -ell->dPhi*180./M_PI, 0, 360, CV_RGB(255,0,0), 1, CV_AA, 0);
}

void inline DrawEll2(IplImage *img, double x, double y, double a, double b, double phi, double r, double g, double blue)
{
  cvEllipse(img, cvPoint((int)(x+.5),(int)(y+.5)), cvSize((int)(a+.5), (int)(b+.5)), -phi*180./M_PI, 0, 360, CV_RGB(r,g,blue), 1, CV_AA, 0);
}

/***************************** TEST ***************************/

float inline roundAngle(float angle) {
   angle *= 180 / 3.141692;
   if(angle < 0) angle += 360;
   float parts45 = (int)(angle / 45);
   float mod45 = angle - parts45*(45);
   float rounded = parts45 * 45;
   if(mod45 > (22.5)) {
      rounded += 45;
   }

   if(rounded >= 180) rounded = 180 - rounded;
   return (rounded * 3.141692 / 180); 
}

/*
void LinkEdges(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *edge, IplImage *img, IplImage *tmp, CzArray<RTE::CzEllipse*> &ells, IplImage *dx, IplImage *dy, IplImage *grayPicture)
{
		struct timespec start, end;
		
		cvZero(tmp);
		CzArray<CzEdgel> points(10000);
		CzArray<bool> inlIdx(10000);
		RTE::CzEllipse *ell;
		double ex,ey,ea,eb,phi;
		CzArray<RTE::CzEllipse*> subPixelAccuracyElls(1000);
		subPixelAccuracyElls.Clear();

		CzEdge * ce = new CzEdge();
  	
		/*IplImage *grad = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
		ce->Norm(dx, dy, grad);
		for(unsigned i=0; i < grad->width; i++) {
			for(unsigned j=0; j < grad->height; j++) {
					cout << (float)GetPixelFloat(grad, i,j) << " ";
			}
			cout << ";";
		}

		cout << endl << endl;

		for(unsigned i=0; i < grad->width; i++) {
			for(unsigned j=0; j < grad->height; j++) {
					cout << (float)GetPixelI16(dx, i,j) << " ";
			}
			cout << ";";
		}

		cout << endl << endl;

		for(unsigned i=0; i < grad->width; i++) {
			for(unsigned j=0; j < grad->height; j++) {
					cout << (float)GetPixelI16(dy, i,j) << " ";
			}
			cout << ";";
		}*/

		/*
    // calculate Gradients
  	IplImage *grad = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
		//IplImage *showNorm = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		//cvConvertImage(grad,showNorm);
		getRealTime(&start);  	
		ce->Norm(dx, dy, grad);
		getRealTime(&end);
		cout<<"Time ce->Norm [s]: "<<timespec_diff(&end, &start)<<endl;

		double minVal, maxVal;
		cvMinMaxLoc(grad, &minVal, &maxVal, NULL, NULL, NULL);
		//cvThreshold(dst2, dst2, (maxVal - minVal) / 2, 255, CV_THRESH_BINARY);
		cout << minVal << maxVal << endl;		
		cvScale(grad,grad,1/maxVal,0);	
		cvMinMaxLoc(grad, &minVal, &maxVal, NULL, NULL, NULL);
		cout << minVal << maxVal << endl;	

		cvNamedWindow("gradient");
		cvShowImage("gradient", grad);
		cvWaitKey(0);*****************

		CzArray<CzSegment *> segments(10000);
		segments.Clear();

		getRealTime(&start);
		ce->LinkEdge(edge, segments, dx, dy);
		getRealTime(&end);
		cout<<"Time ce->LinkEdge [s]: "<<timespec_diff(&end, &start)<<endl;

		int ell_count=0;
		for(int s=0; s < (int)segments.Size(); s++) 
		{
      points.Clear();
      points = segments[s]->edgels;
	
      if (points.Size() > 6)
      {
     		RTE::CzEllipse::FitEllipse(points,ex,ey,ea,eb,phi);
				ell = new RTE::CzEllipse(ex,ey,ea,eb,phi); 

				if ((2*ea < (MAX_DIAMETER / SCALE_FACTOR)) && 
            (2*eb > (MIN_DIAMETER / SCALE_FACTOR)) && 
            (ea/eb < MAX_QUOTIENT))
       	{
					bool del_ellipse = false;

					if(USE_COLOR) 
					{
              del_ellipse = !CheckContrast(dx, dy, points, ell, grayPicture);
					} /* end of USE COLOUR, FIT ELLIPSE NOW *******

					inlIdx.Clear();
       		ell->EllipseSupport(points, .5, inlIdx);

	        if (ell->dSupport > MIN_SUPPORT && ell->dFitError<.5 && !del_ellipse) {
						if (ELLIPSE_METHOD == 1) {
              ells.PushBack(ell);
            }	else if (ELLIPSE_METHOD == 2) {

 							//Subpixel(config, dx, dy, points, ell);
              CzEllMethodSubpixel* subp = new CzEllMethodSubpixel();
              subp->Subpixel(dx, dy, points, ell);
              delete subp;

							if(ell->dFitError < SUBPIXEL_FITERROR /*&& ell->dSupport > 1******)
								ells.PushBack(ell);
							else
								delete ell;

						} else if (ELLIPSE_METHOD == 3) {

							//DualEllipse(config, dx, dy, points, ell, img);
              CzEllMethodDualEllipse* dual = new CzEllMethodDualEllipse();
              dual->DualEllipse(5, dx, dy, points, ell);
              delete dual;

              inlIdx.Clear();							
              ell->EllipseSupport(points, .5, inlIdx);

							if(ell->dSupport > MIN_SUPPORT /*&& ell->dFitError < SUBPIXEL_FITERROR*)
								ells.PushBack(ell);
							else
								delete ell;
						} else {
								cout << "Ellipse method not recognized" << endl;
						}
					}	               

					ell_count++;
         } else {
           if(ell) delete ell;         
         }                  
  		 }
     }
}



bool CheckContrast(IplImage *dx, IplImage *dy, 
                   CzArray<CzEdgel> &points, 
                   CzEllipse *ell, 
                   IplImage *grayPicture)
{
		double ex=ell->dX,
           ey=ell->dY,
           ea=ell->dA,
           eb=ell->dB,
           phi=ell->dPhi;

					unsigned int i=0;
					float rad=(RAD_IN_OUT/SCALE_FACTOR);                 

						//Checks for contrast and ellipse color!
            float contrastSigma=0;
            float posSum=0, negSum=0;
						unsigned int pointsSkipped=0;
            if(rad < 2) rad = 2.0;
            for(i=0; i < points.Size(); i++) {
                  float norm = sqrt(pow((float)GetPixelI16(dx, points[i].p.x, points[i].p.y),2) + 
																		pow((float)GetPixelI16(dy, points[i].p.x, points[i].p.y),2));      
                  float dxx = cvRound(rad*GetPixelI16(dx, points[i].p.x, points[i].p.y)/norm);
                  float dyy = cvRound(rad*GetPixelI16(dy, points[i].p.x, points[i].p.y)/norm);

									if( InRange(grayPicture, points[i].p.x, points[i].p.y, dxx, dyy) ) {		
                    
										if(ell->InsideEllipse(ea,eb,ex,ey,   phi,    points[i].p.x + dxx, 
                                                            points[i].p.y + dyy)) {   
                       posSum += (int)(GetPixel(grayPicture, points[i].p.x + dxx, 
                                                             points[i].p.y + dyy));
                       negSum += (int)(GetPixel(grayPicture, points[i].p.x - dxx, 
                                                             points[i].p.y - dyy));
                    } else {
                       negSum += (int)(GetPixel(grayPicture, points[i].p.x + dxx, 
                                                             points[i].p.y + dyy));
                       posSum += (int)(GetPixel(grayPicture, points[i].p.x - dxx, 
                                                             points[i].p.y - dyy));                                 
                    }

									} else
										pointsSkipped++;
            }

						int totalPointss=points.Size()-pointsSkipped;
            int contrast;
            if(ELL_WHITE) {
                  contrast = (posSum / totalPointss) - (negSum / totalPointss);
            } else {
                  contrast = (negSum / totalPointss) - (posSum / totalPointss);
            }

            if(contrast < MIN_CONTRAST)
                  return false;

            if (MAX_CONTRAST_DEV > 0) {
               contrastSigma = 0;
               for(i=0; i < points.Size(); i++) {
                        // compute contrast standard deviation
                        // no need to check for inside/outside ellipse
                        float norm = sqrt(pow((float)GetPixelI16(dx, points[i].p.x, points[i].p.y),2) + 
																					pow((float)GetPixelI16(dy, points[i].p.x, points[i].p.y),2));      
                        float dxx = cvRound(rad*GetPixelI16(dx, points[i].p.x, points[i].p.y)/norm);
                        float dyy = cvRound(rad*GetPixelI16(dy, points[i].p.x, points[i].p.y)/norm);

												if( InRange(grayPicture, points[i].p.x, points[i].p.y, dxx, dyy) ) {
                        	contrastSigma += pow( (float)(GetPixel(grayPicture, points[i].p.x + dxx, 
                                                                              points[i].p.y + dyy))
                                                                              - (int)(GetPixel(grayPicture, points[i].p.x - dxx, 
                                                                              points[i].p.y - dyy)) - contrast, 2);
												}
               }
               contrastSigma = sqrt(contrastSigma/totalPointss);
               if(contrastSigma > MAX_CONTRAST_DEV) {
                  //cerr << "Contrast sigma:" << contrastSigma;                                              
                  return false;
               }
            }
  return true;
}
*/

void inline PrintMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
		printf("\n");
		switch (CV_MAT_DEPTH(A->type))
		{
			case CV_32F:
			case CV_64F:
				for (j = 0; j < A->cols; j++)
					printf ("%8.5f ", (float)cvGetReal2D(A, i, j));
				break;
			case CV_8U:
			case CV_16U:
				for(j = 0; j < A->cols; j++)
				printf ("%6d",(int)cvGetReal2D(A, i, j));
				break;
			default:
			break;
		}
	}
	printf("\n");
}

/*
void Subpixel(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *dx, IplImage *dy, CzArray<CzEdgel> &points, RTE::CzEllipse *ell) {
	CzEllMethodSubpixel* subp = new CzEllMethodSubpixel();
	subp->Subpixel(&*config, &*dx, &*dy, points, &*ell);
	delete subp;
}

void DualEllipse(CzEllipseDetection::sEllipseDetectionConfig *config, IplImage *dx, IplImage *dy, CzArray<CzEdgel> &points, RTE::CzEllipse *ell, IplImage *img) {
  //cout << "Support before:" << ell->dSupport << " " << ell->dFitError << " " << ell->dPhi << " " << ell-> a << " " << ell->dB << " " << ell->dX << " " << ell->dY << endl;
	CzEllMethodDualEllipse* dual = new CzEllMethodDualEllipse();
	dual->DualEllipse(&*config, &*dx, &*dy, points, &*ell, &*img);
	delete dual;
  // cout << "Support after:" << ell->dSupport << " " << ell->dFitError << " " << ell->dPhi << " " << ell-> a << " " << ell->dB << " " << ell->dX << " " << ell->dY << endl;
}
*/

void TraceEdge(IplImage *edge, IplImage *tmp,IplImage *img,int u, int v, uchar r, uchar g, uchar b, CzArray<CzVector2> &points)
{
  bool found=true;
  while (found)
  {
    SetPixel(tmp,u,v,255);
    SetPixel3C(img,u,v,r,g,b);
    points.PushBack(CzVector2(u,v));

    if (u<edge->width-1 && GetPixel(edge,u+1,v)==255 && GetPixel(tmp,u+1,v)==0) u=u+1;
    else if (u>1 && GetPixel(edge,u-1,v)==255 && GetPixel(tmp,u-1,v)==0) u=u-1;
    else if (v<edge->height-1 && GetPixel(edge,u,v+1)==255 && GetPixel(tmp,u,v+1)==0) v=v+1;
    else if (v>1 && GetPixel(edge,u,v-1)==255 && GetPixel(tmp,u,v-1)==0) v=v-1;
    else if (u<edge->width-1 && v<edge->height-1 && GetPixel(edge,u+1,v+1)==255 && GetPixel(tmp,u+1,v+1)==0) { u=u+1; v=v+1;}
    else if (u<edge->width-1 && v>1 && GetPixel(edge,u+1,v-1)==255 && GetPixel(tmp,u+1,v-1)==0) { u=u+1; v=v-1;}
    else if (u>1 && v<edge->height-1 && GetPixel(edge,u-1,v+1)==255 && GetPixel(tmp,u-1,v+1)==0) { u=u-1; v=v+1;}
    else if (u>1 && v>1 && GetPixel(edge,u-1,v-1)==255 && GetPixel(tmp,u-1,v-1)==0) { u=u-1; v=v-1;}
    else found=false;
  }
}



inline uchar GetPixel(IplImage *img, int x, int y)
{
  return ((uchar*)(img->imageData + img->widthStep*y))[x];
}

inline short GetPixelI16(IplImage *img, int x, int y)
{
  return ((short*)(img->imageData + img->widthStep*y))[x];
}

inline float GetPixelFloat(IplImage *img, int x, int y)
{
  return ((float*)(img->imageData + img->widthStep*y))[x];
}

inline void SetPixel(IplImage *img, int x, int y, uchar col)
{
  ((uchar*)(img->imageData + img->widthStep*y))[x] = col;
}
inline void SetPixel3C(IplImage *img, int x, int y, uchar r, uchar g, uchar b)
{
  ((uchar*)(img->imageData + img->widthStep*y))[x*3] = r;
  ((uchar*)(img->imageData + img->widthStep*y))[x*3+1] = g;
  ((uchar*)(img->imageData + img->widthStep*y))[x*3+2] = b;
}
inline bool InRange(IplImage *img, int x, int y, float ddx, float ddy) {
		return ( ((x + ddx) < img->width && (x + ddx) > 0) &&
						 ((x - ddx) < img->width && (x - ddx) > 0) &&
						 ((y - ddy) < img->height && (y - ddy) > 0) &&
						 ((y + ddy) < img->height && (y + ddy) > 0));
}
