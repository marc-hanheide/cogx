#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/random.hpp> 
#include <boost/concept_check.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
  using boost::math::normal; 
typedef boost::variate_generator<boost::mt19937, boost::normal_distribution<> > DistGen;

void on_mouse( int event, int x, int y, int flags, void *genPtr )
{
  if (event != CV_EVENT_LBUTTONUP) return;
  DistGen &gen = *(DistGen*) genPtr;
  double sigma = gen.distribution().sigma();
  double mean = gen.distribution().mean();
  double dx = x-mean;
  double dy = y-mean;
  boost::math::normal s(mean,sigma);
  double pdfX = boost::math::pdf(s, x);
  double pdfY = boost::math::pdf(s, y);
  
  // gcapp.mouseClick( event, x, y, flags, param );
  printf("< %i, %i>, s = %f, m = %f, < %f, %f>, pdf = < %f, %f>, \n", x, y, sigma, mean, dx, dy, pdfX, pdfY);
  
}


int main(int argc, char *argv[]) {

    cv::Mat img;
    cv::namedWindow("img",1);
    double sigma = 30;
    boost::mt19937 igen;
    DistGen gen(igen, boost::normal_distribution<>(250,30));
    
    cvSetMouseCallback("img", on_mouse, (void *) &gen );
    do {
        img = cv::Mat::zeros(500, 500, CV_8U);
	for( int i = 0; i < 1000; i++){
	  cv::Point p(gen(),gen());
	  img.at<uchar>(p.x,p.y) = 255;
	}
        cv::imshow("img", img);
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
