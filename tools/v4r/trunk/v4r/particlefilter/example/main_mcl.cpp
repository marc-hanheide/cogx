

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/utils/distributions.h>
#include <v4r/particlefilter/robot2d.h>
#include <v4r/particlefilter/pfilter.h>

using namespace std;

int main(int argc, char *argv[]) {

    cv::Mat img;
    cv::namedWindow("img",1);
      
    V4R::PRobot::Set particleSet;
    V4R::PFilter filter;  
    particleSet.setRange((cv::Mat_<double>(2, 3) << 0, 499, 0, 499, -M_PI, M_PI));
    filter.addSet(&particleSet);
    filter.initParticles(100, 0);
    filter.uniform();
    filter.print();
        
    V4R::Particle::Set *ps = filter[0]->getSet();
    V4R::Particle *p = ps->createParticle();
    double resampleSigma_ = 0.1;
    boost::math::normal s(0, resampleSigma_);
    double resampleQuantile_ = boost::math::quantile(s, 0.95);
    cout << "resampleSigma: " << resampleSigma_ << ", resampleQuantile: " << resampleQuantile_ << std::endl; 
    

  particleSet.setMotionError(V4R::PRobot::Set::SIGMA_V, 0.02);
  particleSet.setMotionError(V4R::PRobot::Set::SIGMA_W, 0.02);
  particleSet.setMotionError(V4R::PRobot::Set::RATE_INVERT, 0.05);
  particleSet.setMotion(V4R::PRobot::Set::SIGMA_V, 0.01);
  particleSet.setMotion(V4R::PRobot::Set::SIGMA_W, 0.01);
  filter.setResampleParameter(0.02, 0.1);
  double dtsec = 0.01;
    do {
        img = cv::Mat::zeros(500, 500, CV_8U);
        for ( int i = 0; i < filter.nrOfParticles(); i++) {
	  V4R::Particle *particle = filter[i];
	  cv::Point_<double> p;
	  particle->location(p);
	  if(particle->inRange()){
	    img.at<uchar>(p.x,p.y) = 255;
	  }
	  
	  filter.setTime(dtsec);
	  filter.update();
        }
        cv::imshow("img", img);
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    
    return 0;
}
