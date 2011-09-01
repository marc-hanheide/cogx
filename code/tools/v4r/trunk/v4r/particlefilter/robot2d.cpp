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
#include <v4r/cvextensions/coordinates_cv.h>
#include <v4r/utils/distributions.h>

using namespace V4R;
using namespace cv;

PRobot::PRobot()
        : Particle(LINEAR) {
	  s_ = cv::Mat_<double>::zeros(5,1); 
}

PRobot::~PRobot() {
    //std::cout << "Particle LINEAR destructor" << std::endl;
}

void PRobot::copyTo(Particle *p) const {
    copyDataTo(p, false, false, false);
}

void PRobot::convertFrom(const std::list< boost::shared_ptr<Storage> > &history) {
        /// ToDo creation from linear motion
    boost::shared_ptr<Storage> lastEntry = history.back();
    s_(1,0) = lastEntry->motion().x();
    s_(2,0) = lastEntry->motion().x();
    s_(3,0) = 0;
    s_(4,0) = 0;
    s_(5,0) = 0;
    nrOfUpdates_ = 0;
}

Particle *PRobot::clone() const {
    Particle *p = set_->createParticle();
    copyTo(p);
    return p;
}


void PRobot::set(const cv::Point2d &position, const double &orientation) {
    s_ = (cv::Mat_<double>(6, 1) << position.x, position.y, orientation, 0, 0, 0);
};

void PRobot::location(cv::Point_<double> &p, double sec) const {
    if (sec == .0) {
        p = cv::Point_<double>(s_(0, 0), s_(1, 0));
    } else {
        Mat_<double> A;
        set_->compute_transformtion(A, sec);
        Mat_<double> s1 = A * s_;
        p = cv::Point_<double>(s1(0, 0), s1(1, 0));
    }
}

void PRobot::orientation(double &alpha, double sec)  const {
    if (sec == .0) {
        alpha = s_(2, 0);
    } else {
        /// ToDo there is a failue
        Mat_<double> A;
        set_->compute_transformtion(A, sec);
        Mat_<double> s1 = A * s_;
        alpha =  s1(2, 0);
    }
}

void PRobot::uniform() {
    s_ = Mat_<double>::zeros(5,1);
    if (set_->range_.empty()) return;
    s_(0, 0) = distUniform(set_->range_(0,0), set_->range_(0,1));
    s_(1, 0) = distUniform(set_->range_(1,0), set_->range_(1,1));
    double rangeV = 0.1;
    double rangeW = 0.1;
    s_(3, 0) = distUniform(rangeV);
    s_(4, 0) = distUniform(rangeW);
    confidence_ = 0;
    nrOfUpdates_ = 0;
}

void PRobot::update() {
  /// ToDo
//   cv::Mat_<double> A;
//   set_->getTransformation(A, 1);
  // rauschen 
  
  
//   s_ = A * s_;
  
    Mat_<double> s0 = s_;
    
    s0(3,0) = V4R::Distributions::normalDist(s0(3, 0), set_->getMotionError(PRobot::Set::SIGMA_V));
    s0(4,0) = V4R::Distributions::normalDist(s0(4, 0), set_->getMotionError(PRobot::Set::SIGMA_W));
    
    set_->A_(0,3) = set_->A_(0,3)*sin(s_(2,0));
    set_->A_(1,3) = set_->A_(1,3)*cos(s_(2,0));
    
    s_ = set_->A_ * s0;
    
    nrOfUpdates_ ++;
}

void PRobot::state2LinearMotion(cv::Mat_<double> &state, double sec) const{
  /// ToDo it should also involve the roation
    state.create(6,1);
    state(0,0) = s_(0,0) + s_(2,0) * sec;
    state(1,0) = s_(1,0) + s_(3,0) * sec;
    state(2,0) = 0;
    state(3,0) = s_(2,0);
    state(4,0) = s_(3,0);
    state(5,0) = 0;
}

bool PRobot::inRange() {
    for (int r = 0; r < set_->range_.rows; r++) {
        if ((s_(r,0) < set_->range_(r,0)) || (s_(r,0) > set_->range_(r,1))) return false;
    }
    return true;
}

Particle* PRobot:: ptr() {
    std::cerr << "ptr() not implemented";
    throw(0);
}

Particle::ParticleType PRobot::classType() {
    return Particle::LINEAR;
}
