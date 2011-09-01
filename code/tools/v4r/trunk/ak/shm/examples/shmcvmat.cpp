#include <ak/shm/shmcvmat.hpp>
#include <ak/shm/shmmanager.hpp>
#include <ak/cv/print.h>
#include <stdio.h>

int main ( int argc, char** argv ) {

    ak::Shm::init();
    
    ak::ShmCvMat shmMat;
    shmMat.openOrCreate("intrinsic", 3, 3, CV_64F );
    CvMat *p = &shmMat;
    cvSetIdentity(p);
    cvPrint(p, "intrinsic");
    
    ak::ShmCvMat shmMat2;
    shmMat2.open("intrinsic");
    cv::Mat_<double> M2 = shmMat2.cv();
    std::cout << M2 << std::endl;
    
    ak::ShmCvMat shmMat3;
    shmMat3.open("intrinsic");
    cv::Mat_<double> M3 = shmMat3.cv();
    M3(2,1) = 3;
    cvPrint(p, "intrinsic");
    std::cout << M3 << std::endl;
    
    
    
    return 0;
}
