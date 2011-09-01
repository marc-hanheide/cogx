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

#include "pendulum.h"

namespace V4R {

Pendulum::Set::Set()
: Particle::Set()
{
    //std::cout << "Pendulum::Set constructor" << std::endl;
    setName("PENDULUM");
    color_ = cv::Vec<double, 4>(1,0,1,1);
    type_ = Pendulum::PENDULUM;
    motionNames_.resize ( 5 );
    motionNames_[0] = "force";
    motionNames_[1] = "radius";
    motionNames_[2] = "phase";
    motionNames_[3] = "base";
    motionNames_[4] = "theta";
    motionError_.resize ( motionNames_.size(), 0 );
    motion_.resize ( motionNames_.size(), 0 );
    
    stateInfo_.resize ( 10 );
    stateInfo_[0] = "xb";
    stateInfo_[1] = "yb";
    stateInfo_[2] = "zb";
    stateInfo_[3] = "radius";
    stateInfo_[4] = "phi";
    stateInfo_[5] = "wphi";
    stateInfo_[6] = "theta";
    stateInfo_[7] = "wtheta";
    stateInfo_[8] = "force";
    stateInfo_[9] = "phase";
}

Pendulum::Set::Set ( Pendulum::Set &r )
        : Particle::Set ( r )
{
}

Pendulum::Set::~Set()
{
    // std::cout << "Pendulum::Set destructor" << std::endl;
}

void Pendulum::Set::compute_transformtion ( cv::Mat_<double> &A, double dtsec ) const
{
  //  std::cout << "Pendulum::Set::compute_transformtion ( cv::Mat_<double> &A, double dtsec ) destructor" << std::endl;
}

Particle::Set *Pendulum::Set::clone() const{
  Particle::Set *p = new Pendulum::Set;
  copyTo(*p);
  return p;
}

Particle* Pendulum::Set::createParticle ()
{
    Particle* particle  = new Pendulum();
    particle->setSet (this );
    nrOfParticles_++;
    return particle;
}
void Pendulum::Set::deleteParticle(Particle* p) {
    if(nrOfParticles_ == 0){
      std::cerr << human_readable() << std::endl;
      std::cerr << "Pendulum::Set::deleteParticle() nothing there to free" << std::endl;
      throw(0);
    }
    nrOfParticles_--;
    delete p;
}

}
