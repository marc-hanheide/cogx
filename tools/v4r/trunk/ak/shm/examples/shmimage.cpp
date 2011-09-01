#include <opencv2/opencv.hpp>
#include <ak/shm/shmimage.hpp>
#include <ak/shm/shmmanager.hpp>
#include <iomanip>
#include <stdio.h>

int main ( int argc, char** argv ) {
    ak::Shm::init();
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );
    std::string fileName(argv[1]);
    IplImage *pImgSrc = cvLoadImage (fileName.c_str(), 1 );
    
 
  
    ak::ShmImage shmImage1;
    shmImage1.openOrCreate(fileName.c_str(), cvSize ( pImgSrc->width, pImgSrc->height ), pImgSrc->depth, pImgSrc->nChannels );
    IplImage image1;
    shmImage1.get(&image1);
    cvCopy(pImgSrc, &image1);
    
    
    ak::ShmImage shmImage2;
    shmImage2.open(fileName.c_str());
    ak::cv::Image image2;
    shmImage2.get(image2);
    
    
    std::cout << "sizeof(IplImage) = " << sizeof(IplImage) << std::endl;
    std::cout << "sizeof(ak::cv::Image) = " << sizeof(ak::cv::Image) << std::endl;
    std::cout << "Shm::SEGMENT_SIZE  = " << ak::Shm::segment_size() << " byte := " << ak::Shm::segment_size()/1024 << " Mb" << std::endl;
    std::cout << "Shm::allocated  = " << shmImage1.getVarHeader()->getSize() << std::endl;
    std::cout << "IplImage.imageSize = " << image1.imageSize << " := "  << setprecision (3) << (image1.imageSize*100.0/ak::Shm::segment_size()) << "%" << std::endl;
    
    
    ak::ShmImageView viewer;
    viewer.setName(fileName);
    viewer.view(100);
    
    return 0;
}
