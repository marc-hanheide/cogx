

#include <opencv2/opencv.hpp>
#include <v4r/geometry/average.h>

using namespace V4R;

int main(int argc, char** argv) {  
    std::cout << "Hello statisic" << std::endl;
    
    std::vector<cv::Point3_<double> > myvector2;    
    myvector2.push_back(cv::Point3d(3,3,3));
    myvector2.push_back(cv::Point3d(10, 10, 10));
    myvector2.push_back(cv::Point3d(-1,-1,4));
    
    
    cv::Point3_<double> a = mean_arithmetic(myvector2);
    std::cout << "avr = " << a << std::endl;
    
    return 0;
}
