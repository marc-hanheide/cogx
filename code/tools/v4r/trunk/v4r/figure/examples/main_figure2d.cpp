
#include <iostream>
#include <opencv/highgui.h>
#include <v4r/figure/figure.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>


template <typename T>
int str2TypeArray(std::string str, T* des, unsigned int size) {
    std::vector<std::string> values;
    boost::erase_all(str, " ");
    boost::split(values, str, boost::is_any_of(";,"));
    if (values.size() != size) return 1;
    for (unsigned int  i = 0; i < size; i++) {
        des[i] = boost::lexical_cast<T>(values[i]);
    }
    return 0;
}

int program_param(int argc, char *argv[], V4R::Figure::Parameters &param) {

    boost::program_options::options_description desc("Allowed Parameters");
    std::string canvas, roi;
    int angleDeg;
    desc.add_options()//
    ("help", "get this help message")//
    ("size,s", boost::program_options::value<std::string>(&canvas)->default_value("600, 900"), "Canvas size <widht, height> [pix], default: \"400,600\"") //
    ("roi,r", boost::program_options::value<std::string>(&roi)->default_value("-3, 2, 6, -4"),"Region of intrest <x, y, dx, dy> [m], default: \"-3, -2, 6, 4\"")
    ("angle, a", boost::program_options::value<int>(&angleDeg)->default_value(90),"rotation of the axis in grad, default: 0"); 

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    } catch (const std::exception& ex) {
        std::cout << desc << "\n";
        return 1;
    }
    boost::program_options::notify(vm);

    if (str2TypeArray<int>(canvas, (int*) &param.size, 2)) return 1;
    if (str2TypeArray<double>(roi, (double*) &param.roi, 4)) return 1;
    param.alpha = angleDeg*M_PI/180.0;
    return 0;
}

int main(int argc, char *argv[])
{

    V4R::Figure::Parameters param;
    program_param(argc, argv, param);

    V4R::Figure figure;
    figure.init(param);
    figure.clear();
    figure.plotGrid();
    figure.plotDot(1,0.5);
    figure.plotLine(2,1.3, -2.8, 1.8);
    figure.setColor(200,20,200);
    figure.plotArrow(1,1.3, 2, -1.5);
    figure.plotOrigin();
    figure.plotCross( -1,1);
    figure.plotCircle( -1,-1, 3);
   
    double state[3] = { 0.2, 0.4, 0};
    figure.plotState(state);
    double state2[3] = { 0.2, 0.4, M_PI/8};
    figure.plotState(state2);
    
    figure.show();
    
    cv::waitKey(20000);
    return 0;
}
