#include <ak/shm/shmvar.hpp>
#include <ak/shm/shmmanager.hpp>
#include <ak/cv/print.h>
#include <stdio.h>

int main ( int argc, char** argv ) {

    ak::Shm::init();
    ak::ShmVar<double, ak::Shm::VAR_F64>  shmDouble1("MyDouble");
    shmDouble1.unlock();
    double *p1 = shmDouble1.ptr();
    
    shmDouble1.lock();
    *p1 = 100;
    shmDouble1.unlock();
    
    
    ak::ShmVar<double, ak::Shm::VAR_F64>  shmDouble2("MyDouble");
    
    std::cout << "p1 = "  << *p1  << ", p2 = "  << *p1 << std::endl;
    
    double p2 = 200.2;
    shmDouble2.copyTo(p2); /// Locks for you 
    
    std::cout << "p1 = "  << *p1  << ", p2 = "  << *p1 << std::endl;
    
    
    ak::ShmVar<double, ak::Shm::VAR_F64>  shmDouble3;
    shmDouble3.open("MyDouble");
    
    shmDouble3.set(30.0); /// Locks for you and triggers signal if needed
        
    std::cout << "p1 = "  << *p1  << ", p2 = "  << *p1   << ", shmDouble3 = "  << shmDouble3() << std::endl;
    
    
    return 0;
}
