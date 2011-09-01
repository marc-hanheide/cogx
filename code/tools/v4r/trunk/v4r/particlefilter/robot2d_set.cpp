/*
 Copyright (c) 2011, Markus Bader
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include "robot2d.h"
namespace V4R
{

PRobot::Set::Set()
        : Particle::Set()
{
    //std::cout << "PRobot::Set constructor" << std::endl;
    setName("ROBOT2D");
    color_ = cv::Vec<double, 4>(1,0,0.5,1);
    type_ = PRobot::ROBOT2D;
    motionNames_.resize ( 3 );
    motionNames_[0] = "sigma_velocity";
    motionNames_[1] = "sigma_azimuth";
    motionNames_[2] = "rate_invert";
    motionError_.resize ( motionNames_.size(), 0 );
    motion_.resize ( motionNames_.size(), 0 );

    stateInfo_.resize ( 5 );
    stateInfo_[0] = "x";
    stateInfo_[1] = "y";
    stateInfo_[2] = "alpha";
    stateInfo_[3] = "v";
    stateInfo_[4] = "w";
}

PRobot::Set::Set ( PRobot::Set &r )
        : Particle::Set ( r )
{
}

PRobot::Set::~Set()
{
   // std::cout << "PRobot::Set destructor" << std::endl;
}

void PRobot::Set::compute_transformtion(cv::Mat_<double> &A, double dtsec) const {
    A = cv::Mat_<double>::eye ( 5,5 );
    A ( 0,3 ) = dtsec;
    A ( 1,3 ) = dtsec;
    A ( 2,4 ) = dtsec;
}
Particle::Set *PRobot::Set::clone() const {
    Particle::Set *p = new PRobot::Set;
    copyTo(*p);
    return p;
}

Particle* PRobot::Set::createParticle ( )
{
    Particle* particle  = new PRobot();
    particle->setSet ( this );
    nrOfParticles_++;
    return particle;
}

void PRobot::Set::deleteParticle ( Particle* p) {   
    if(nrOfParticles_ == 0){
      std::cout << "D " << human_readable() << std::endl;
      std::cerr << "PRobot::Set::deleteParticle() nothing there to free" << std::endl;
      throw(0);
     }
    nrOfParticles_--;
    delete p;
}


}
