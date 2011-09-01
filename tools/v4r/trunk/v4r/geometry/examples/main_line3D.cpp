

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/cvextensions/print_cv.h>
#include <v4r/geometry/line3dbase.h>

using namespace V4R;

int main(int argc, char** argv) {
    LineSegment3Dd la(1,3,2,9,7,3);
    LineSegment3Dd lb(6,3,0,2,11,8);
    std::cout << la.human_readable() << std::endl;
    std::cout << lb.human_readable() << std::endl;
    cv::Point3d p(1,0,0);
    double d = la.distanceToLine(p);
    cv::Point3d pi = la.intersection(lb);
//    std::cout << d << std::endl;
//    std::cout << pi << std::endl;
    return 0;
}
