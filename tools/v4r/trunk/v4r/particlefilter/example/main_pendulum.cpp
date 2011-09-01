

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <v4r/utils/distributions.h>
#include <v4r/particlefilter/pendulum.h>
#include <v4r/particlefilter/pfilter.h>
#include <v4r/cvextensions/operator_cv.h>

#include <v4r/TomGine/shm/tgShape.h>

using namespace V4R;
using namespace std;
using namespace cv;
using namespace TomGine;



int main(int argc, char *argv[]) {

    std::vector<ShapeAppearance>  tgAppearances;
    tgAppearances.push_back(ShapeAppearance(1, 1, 0, 0, 0.5));
    tgAppearances.push_back(ShapeAppearance(2, 1, 0, 1, 0.5));
    tgAppearances.push_back(ShapeAppearance(3, 0, 0, 1, 0.5));
    tgAppearances.push_back(ShapeAppearance(4, 0.25, 0.25, 0.25, 0.35));
    ShmTGAppearance shmAppearances("TG/Appearance");
    ShmTGShapes shmShapes("TG/Shapes");
    ShmTGLines shmLines("TG/Lines");
    shmAppearances.set(tgAppearances);

    cv::Mat img;
    cv::namedWindow("img",1);
    img = cv::Mat::zeros(500, 500, CV_8UC3);
    cv::Mat_<double> RT = (Mat_<double>(3, 3) << 50, 0, img.cols/2., /**/ 0, -50, img.rows/4.*3., /**/ 0, 0, 1 );
    std::cout << RT << std::endl;
    Pendulum::Set set;
    set.setRange((Mat_<double>(3, 2) << -1, 1, -0.5, 1, 0, 1));
    Pendulum& pendulum = *((Pendulum*) set.createParticle());
    double F = Pendulum::potential_force(DEG2RAD(20), 4);
    pendulum.init(cv::Point3d(0,0,6), 4, DEG2RAD(-5), true, F);
    cout << pendulum.human_readable() << endl;

    double dt = 0.1;
    double t = 0.;
    for (cv::Point_<double> pr(-10,-10); pr.x <= 10; pr.x = pr.x+1) {
        for (pr.y = -10; pr.y <= 10;  pr.y = pr.y+1) {
            cv::Scalar col(128, 128, 128);
            if (pr.x == 0) col = cv::Scalar(0,0,255);
            if (pr.y == 0) col = cv::Scalar(0,255,255);
            cv::circle(img, RT*pr, 1, col);
        }
    }
    cv::putText(img, "x", RT*cv::Point2d(3,0.2), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
    cv::putText(img, "y", RT*cv::Point2d(0.2,3), FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255));

    std::vector<ShapeEntry>  tgShapes;
    tgShapes.push_back(ShapeBox(tgAppearances[3], 0.1, 0.1, 0.1));
    tgShapes.push_back(ShapeSphere(tgAppearances[0],  0.001, 3));
    tgShapes.back().position(pendulum.xb()/100., pendulum.yb()/100., pendulum.zb()/100.);

    std::vector<ShapeLine>  tgLines;
    do {
        set.setTime(dt);
        cout << pendulum.human_readable() << endl;
        cv::imshow("img", img);
        cv::Point3_<double> p;
        pendulum.toXYZ(p);
        cv::circle(img, RT*Point2d(pendulum.xb(), pendulum.zb()), 3, cv::Scalar(255, 255, 0));
        cv::circle(img, RT*Point2d(p.x, p.z), 1, cv::Scalar(0, 255, 0));
        pendulum.update();
        t +=dt;
        tgShapes.push_back(ShapeSphere(tgAppearances[1],  0.001, 3));
        tgShapes.back().position(p.x/100., p.y/100., p.z/100.);
        shmShapes.set(tgShapes);
        tgLines.push_back(ShapeLine(tgAppearances[2], pendulum.xb()/100., pendulum.yb()/100., pendulum.zb()/100., p.x/100., p.y/100., p.z/100.));
        shmLines.set(tgLines);
    } while ((cv::waitKey(30) < 0) &&  (t < 10.));

    return 0;
}
