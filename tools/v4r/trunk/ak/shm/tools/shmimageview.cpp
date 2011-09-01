
#include <stdio.h>
#include "opencv/cv.h"
#include <ak/shm/shmimage.hpp>
#include <ak/shm/shmmanager.hpp>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <opencv/highgui.h>





static void mouseCallBack ( int evt, int x, int y, int flags, void *param ) {
    if ( evt == CV_EVENT_LBUTTONDOWN ) {
        IplImage *pImg = (IplImage *) param;
        unsigned char *p = (unsigned char*) pImg->imageData + y * pImg->width * pImg->nChannels + x * pImg->nChannels;
        std::cout << "xy = [" << std::setw(4) << x <<  ", " << std::setw(4) << y << " ] ";
        std::cout << "bgr = [" << std::setw(4) << (int)p[0] <<  "," << std::setw(4) << (int)p[1] <<  "," << std::setw(4) << (int)p[2] <<  " ] ";
        std::cout << std::endl << std::flush;
    }
    if ( evt == CV_EVENT_MBUTTONDOWN ) {
    }
}

/** Sleep
* @param ms
*/
void sleepMs ( unsigned int ms ) {
    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC);
    xt.nsec += ms * 1000 * 1000;
    boost::thread::sleep(xt);
}

namespace po = boost::program_options;
int main ( int argc, char** argv ) {
    ak::Shm::init();
    std::vector<std::string> nameShmImages;
    int waitMs;
    po::options_description desc("Allowed Parameters");
    desc.add_options()//
    ("help", "get this help message")//
    ("waitkey,w", po::value< int >(&waitMs)->default_value(100), "waitkey time")
    ("shm,s", po::value<std::vector <std::string> >(&nameShmImages)->multitoken(), "shared cv::ak::image");

    po::positional_options_description p;
    p.add("shm", -1);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (const std::exception& ex) {
        std::cout << desc << "\n";
        return 1;
    }
    po::notify(vm);


    if (vm.count("help") || nameShmImages.size() == 0) {
        std::cout << desc << "\n";
        return 1;
    }

    std::vector< ak::ShmImageView > shmImageViews(nameShmImages.size());
    std::vector< boost::shared_ptr<boost::thread> > threads;

    for (unsigned int i = 0; i < shmImageViews.size(); i++)
    {
        ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(nameShmImages[i]);
        if (!pHeader) {
            std::cout << "NO such variable: " << nameShmImages[i] << std::endl;
        } else {
            ak::cv::Image img;
            ak::ShmImage shmImage;
            ak::Shm::ErrorCode err = shmImage.open(nameShmImages[i]);
            if (err == ak::Shm::SUCCESS) {
                shmImage.get(img);
                std::cout << nameShmImages[i] << std::endl;
                std::cout << img.toString() << std::endl;
            }
        }
    }

    int key = 0;
    do {
        for (unsigned int i = 0; i < shmImageViews.size(); i++) {
            shmImageViews[i].setName(nameShmImages[i]);
            shmImageViews[i].show();
            cvSetMouseCallback ( nameShmImages[i].c_str(), mouseCallBack, shmImageViews[i].img().ipl() );
            if (i == shmImageViews.size() - 1)  key = cvWaitKey(100);
        }
    } while ( (( char )  key) != 27 );


    return 0;
}
// kate: indent-mode cstyle; space-indent on; indent-width 0;
