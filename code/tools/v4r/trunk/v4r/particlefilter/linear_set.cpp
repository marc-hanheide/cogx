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

#include "linear.h"
namespace V4R
{

PLinear::Set::Set()
        : Particle::Set()
{
    //std::cout << "PLinear::Set constructor" << std::endl;
    setName("LINEAR");
    color_ = cv::Vec<double, 4>(0,0,1,1);
    type_ = PLinear::LINEAR;
    motionNames_.resize ( 4 );
    motionNames_[0] = "velocity";
    motionNames_[1] = "inclination";
    motionNames_[2] = "azimuth";
    motionNames_[3] = "invert";
    motionError_.resize ( motionNames_.size(), 0 );
    motion_.resize ( motionNames_.size(), 0 );

    stateInfo_.resize ( 6 );
    stateInfo_[0] = "x";
    stateInfo_[1] = "y";
    stateInfo_[2] = "z";
    stateInfo_[3] = "vx";
    stateInfo_[4] = "vy";
    stateInfo_[5] = "vz";
}

PLinear::Set::Set ( PLinear::Set &r )
        : Particle::Set ( r )
{
}

PLinear::Set::~Set()
{
    //std::cout << "PLinear::Set destructor" << std::endl;
}

void PLinear::Set::compute_transformtion(cv::Mat_<double> &A, double dtsec) const {
    A = cv::Mat_<double>::eye ( 6,6 );
    A ( 0,3 ) = dtsec;
    A ( 1,4 ) = dtsec;
    A ( 2,5 ) = dtsec;
}

Particle::Set *PLinear::Set::clone() const {
    Particle::Set *p = new PLinear::Set;
    copyTo(*p);
    return p;
}

Particle* PLinear::Set::createParticle ( )
{
    Particle* particle  = new PLinear();
    particle->setSet ( this );
    nrOfParticles_++;
    return particle;
}

void PLinear::Set::deleteParticle ( Particle* p) {
    if (nrOfParticles_ == 0) {
      std::cerr << human_readable() << std::endl;
      std::cerr << "PLinear::Set::deleteParticle() nothing there to free" << std::endl;
      throw(0);
    }
    nrOfParticles_--;
    delete p;
}


}
