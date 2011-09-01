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

#include "random.h"
#include <v4r/cvextensions/coordinates_cv.h>
#include <v4r/utils/distributions.h>

using namespace V4R;
using namespace cv;

PRandom::PRandom() 
	: Particle(RANDOM){
	  s_ = cv::Mat_<double>::zeros(6,1); 
}

PRandom::~PRandom() {
  std::cout << "Particle RANDOM destructor" << std::endl;
}

void PRandom::copyTo(Particle *p) const{
    copyDataTo(p, false, false, false);
}

void PRandom::convertFrom(const std::list< boost::shared_ptr<Storage> > &history){    
    boost::shared_ptr<Storage> lastEntry = history.back();
    lastEntry->motion().state().copyTo(s_);
    s_(3,0) = 0;
    s_(4,0) = 0;
    s_(5,0) = 0;
    nrOfUpdates_ = 0;
}

Particle *PRandom::clone() const{
  Particle *p = set_->createParticle();
  copyTo(p);
  return p;
}


void PRandom::state2LinearMotion(cv::Mat_<double> &state, double sec) const{
    s_.copyTo(state);
    state(3,0) = 0;
    state(4,0) = 0;
    state(5,0) = 0;
}

void PRandom::set(const cv::Point3d &position, const cv::Point3d &velocety) {
    s_(0,0) = position.x;
    s_(1,0) = position.y;
    s_(2,0) = position.z;
    s_(3,0) = 0;
    s_(4,0) = 0;
    s_(5,0) = 0;
};

void PRandom::location(cv::Point3_<double> &p, double sec) const {
    if (sec == .0) {
        p = cv::Point3_<double>(s_(0, 0), s_(1, 0), s_(2, 0));
    } else {
        Mat_<double> A;
        set_->compute_transformtion(A, sec);
        Mat_<double> s1 = A * s_;
        p = cv::Point3_<double>(s1(0, 0), s1(1, 0), s1(2, 0));
    }
}

void PRandom::uniform() {
    if (set_->range_.empty()) return;
    s_(0, 0) = distUniform(set_->range_(0,0), set_->range_(0,1));
    s_(1, 0) = distUniform(set_->range_(1,0), set_->range_(1,1));
    s_(2, 0) = distUniform(set_->range_(2,0), set_->range_(2,1));
    s_(3,0) = 0;
    s_(4,0) = 0;
    s_(5,0) = 0;
    confidence_ = 0;
    nrOfUpdates_ = 0;
}

void PRandom::update() {
    Mat_<double> s0(s_,true);
    double sigma = set_->getMotionError(PRandom::Set::VELOCITY);     
    s0(3,0) = distNormal(sigma), s0(4,0) = distNormal(sigma), s0(5,0) = distNormal(sigma);
    //std::cout << transform_ << std::endl;
    //std::cout << xt0 << std::endl;
    s_ = set_->A_ * s0;
    nrOfUpdates_ ++;
}

bool PRandom::inRange() {
    for (int r = 0; r < set_->range_.rows; r++) {
        if ((s_(r,0) < set_->range_(r,0)) || (s_(r,0) > set_->range_(r,1))) return false;
    }
    return true;
}

Particle* PRandom:: ptr(){
    std::cerr << "ptr() not implemented";
    throw(0);
}

Particle::ParticleType PRandom::classType(){
    return Particle::LINEAR;
}
