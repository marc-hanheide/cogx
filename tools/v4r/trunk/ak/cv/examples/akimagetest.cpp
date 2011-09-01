#include <ak/cv/akimage.h>
#include <ak/cv/akimage_serialize.h>

#include <stdio.h>



int main ( int argc, char** argv ) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );
    ak::cv::Image  img;
    
    
    IplImage *pImg = cvLoadImage(argv[1], 1);
    *img.ipl() = *pImg;
    ak::cv::writeXML(img, "/tmp", "img.xml");
    std::cout << img.toString() << std::endl;
    cvNamedWindow ( "Image", 1 );
    cvShowImage ("Image", img.ipl() );
    
    
    ak::cv::Image img2;
    ak::cv::readXML(img2, "/tmp/img.xml");
    cvNamedWindow ( "Image2", 1 );
    cvShowImage ("Image2", img2.ipl() );
    
    
    int key;
    do {
      key = cvWaitKey(1000);
    } while ( ( ( char ) key ) != 27 );
    cvDestroyWindow ( "Image" );
    cvDestroyWindow ( "Image2" );

    return 0;
}
