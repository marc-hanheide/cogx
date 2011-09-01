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
#include <v4r/cvextensions/coordinates_cv.h>
#include <v4r/utils/distributions.h>

using namespace V4R;
using namespace cv;

PLinear::PLinear()
        : Particle(LINEAR) {
    s_ = cv::Mat_<double>::zeros(6,1);
}

PLinear::~PLinear() {
    //std::cout << "Particle LINEAR destructor" << std::endl;
}

void PLinear::copyTo(Particle *p) const {
    copyDataTo(p, false, false, false);
}

void PLinear::convertFrom(const std::list< boost::shared_ptr<Storage> > &history) {
    std::list< boost::shared_ptr<Storage> >::const_reverse_iterator it0, it1 = history.rbegin();
    (*it1)->motion().state().copyTo(s_);
    nrOfUpdates_ = 0;
    /*
    int n = 4;
    if ((*it1)->getSet()->type() == RANDOM) {
        if (history.size() > n) {
            cv::Vec<double,3> velAVR(0,0,0);
            it1++;
            it0 = history.rbegin();
            for (int i = 0; (i < n) &&  (it1 != history.rend()); i++, it1++, it0++) {
                double dt = (*it0)->getSet()->dt();
                velAVR[0] += (*(*it0)->particle())[0] - (*(*it1)->particle())[0] / dt;
                velAVR[1] += (*(*it0)->particle())[1] - (*(*it1)->particle())[1] / dt;
                velAVR[2] += (*(*it0)->particle())[2] - (*(*it1)->particle())[2] / dt;
                s_(3,0) = velAVR[0] / (double) (i);
                s_(4,0) = velAVR[1] / (double) (i);
                s_(5,0) = velAVR[2] / (double) (i);
            }
        }
    }
    */

}

Particle *PLinear::clone() const {
    Particle *p = set_->createParticle();
    copyTo(p);
    return p;
}


void PLinear::state2LinearMotion(cv::Mat_<double> &state, double sec) const {
    state.create(6,1);
    state(0,0) = s_(0,0) + s_(3,0) *sec;
    state(1,0) = s_(1,0) + s_(4,0) *sec;
    state(2,0) = s_(2,0) + s_(5,0) *sec;
    state(3,0) = s_(3,0);
    state(4,0) = s_(4,0);
    state(5,0) = s_(5,0);
}

void PLinear::set(const cv::Point3d &position, const cv::Point3d &velocety) {
    s_(0,0) = position.x;
    s_(1,0) = position.y;
    s_(2,0) = position.z;
    s_(3,0) = velocety.x;
    s_(4,0) = velocety.y;
    s_(5,0) = velocety.z;
};

void PLinear::location(cv::Point3_<double> &p, double sec) const {
    if (sec == .0) {
        p = cv::Point3_<double>(s_(0, 0), s_(1, 0), s_(2, 0));
    } else {
        Mat_<double> A;
        set_->compute_transformtion(A, sec);
        Mat_<double> s1 = A * s_;
        p = cv::Point3_<double>(s1(0, 0), s1(1, 0), s1(2, 0));
    }
}

void PLinear::uniform() {
    if (set_->range_.empty()) return;
    s_(0, 0) = distUniform(set_->range_(0,0), set_->range_(0,1));
    s_(1, 0) = distUniform(set_->range_(1,0), set_->range_(1,1));
    s_(2, 0) = distUniform(set_->range_(2,0), set_->range_(2,1));
    double rangeV = 0.1;
    s_(3, 0) = distUniform(rangeV);
    s_(4, 0) = distUniform(rangeV);
    s_(5, 0) = distUniform(rangeV);
    confidence_ = 0;
    nrOfUpdates_ = 0;
}

void PLinear::update() {
    Mat_<double> s0(s_,true);
    cv::Vec3d sp = cartToSphere(cv::Point3d(s0(3, 0), s0(4, 0), s0(5, 0)));
    cv::Vec3d sigmas(set_->getMotionError(PLinear::Set::VELOCITY),
                     set_->getMotionError(PLinear::Set::INCLINATION),
                     set_->getMotionError(PLinear::Set::AZIMUTH));
    V4R::Distributions::normalDist(sp,  sigmas);
    if (distUniform() < set_->getMotionError(PLinear::Set::INVERT)) {
        /// Invert direction
        sp[0] *= -1.;
    }
    cv::Vec3d cp = sphereToCart(sp);
    s0(3,0) = cp[0], s0(4,0) = cp[1], s0(5,0) = cp[2];
    //std::cout << transform_ << std::endl;
    //std::cout << xt0 << std::endl;
    s_ = set_->A_ * s0;
    nrOfUpdates_ ++;
}

bool PLinear::inRange() {
    for (int r = 0; r < set_->range_.rows; r++) {
        if ((s_(r,0) < set_->range_(r,0)) || (s_(r,0) > set_->range_(r,1))) return false;
    }
    return true;
}

Particle* PLinear:: ptr() {
    std::cerr << "ptr() not implemented";
    throw(0);
}

Particle::ParticleType PLinear::classType() {
    return Particle::LINEAR;
}
