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
#include <v4r/cvextensions/coordinates_cv.h>
#include <v4r/utils/distributions.h>

using namespace V4R;
using namespace cv;

template<typename T>
inline bool isinf(T value)
{
    return std::numeric_limits<T>::has_infinity() &&
           value == std::numeric_limits<T>::infinity();
}

Pendulum::Pendulum()
        : Particle(LINEAR) {
    s_ = cv::Mat_<double>::zeros(10,1);
}

Pendulum::~Pendulum() {
    std::cout << "Particle LINEAR destructor" << std::endl;
}

void Pendulum::copyTo(Particle *p) const {
    copyDataTo(p, false, false, false);
}

void Pendulum::convertFrom(const std::list< boost::shared_ptr<Storage> > &history) {
    double alpha_xz, alpha_yz, r_xz, r_yz, dz, dx, dy, v, radius, xb, yb, zb, phi, h, w, Fp, Fk;
    double m = 1.0;
    boost::shared_ptr<Storage> lastEntry = history.back();
    const V4R::LinearMotion<double> &lm = lastEntry->motion();
    
    dz = 1.0 -lm.z();

    //std::cout << lm << std::endl;
    
    alpha_xz = atan2(lm.vx(), lm.vz());
    r_xz = dz/cos(alpha_xz);
    dx = dz/tan(alpha_xz);
    //double dx = lm.vx()/lm.vz()*dz;
    if (alpha_xz == 0) {
        r_xz = dz;
        dx = 0.;
    }
    
    alpha_yz = atan2(lm.vy(), lm.vz());
    r_yz = dz/cos(alpha_yz);
    dy = dz/tan(alpha_yz);
    if (alpha_yz == 0) {
        r_yz = dz;
        dy = 0.;
    }


    v = sqrt(lm.vz()*lm.vz() + lm.vy()*lm.vy() + lm.vz()*lm.vz());
    radius = sqrt(r_xz*r_xz + r_yz*r_yz);



    xb = lm.x()+dx;
    yb = lm.y()+dy;
    zb = 1.;

    
    //Range test
    if ((xb < set_->range_(0,0)) || (xb > set_->range_(0,1))) {
        xb = distUniform(set_->range_(0,0), set_->range_(0,1));
    }
    if ((yb < set_->range_(1,0)) || (yb > set_->range_(1,1))) {
        yb = distUniform(set_->range_(1,0), set_->range_(1,1));
    }
    if ((radius < set_->range_(2,0)) || (radius > set_->range_(2,1))) {
        radius = distUniform(set_->range_(2,0), set_->range_(2,1));
    }
    if (isinf(radius)) {
        std::cerr << "Pendulum::convertFrom() radius is inf!!" << std::endl;
        std::cerr << human_readable() << std::endl;
    }



    phi = alpha_xz;  //shortcut assumint motion only in xz

    h = lift(radius, phi);
    w = v/radius;

    Fp = potential_force(phi, radius);
    Fk = kinetic_force(w, radius);

    //phi0 =  0.3;
    cv::Point3_<double> base = cv::Point3d(xb, yb, zb);

    init(base, radius, phi, true, 0, Fk+Fp);

    //p->copyDataTo(this, false, false, false);

    nrOfUpdates_ = 0;
    confidence_ = 0;
    //uniform();
}

Particle *Pendulum::clone() const {
    Particle *p = set_->createParticle();
    copyTo(p);
    return p;
}

void Pendulum::state2LinearMotion(cv::Mat_<double> &state, double sec) const {
    state.create(6,1);
    cv::Point3_<double> p0;
    location(p0, 0);
    state(0,0) = p0.x, state(1,0) = p0.y, state(2,0) = p0.z;
    double v = wphi() * radius();

    /// assuming only xz motions
    double vx = cos(phi() * v);
    double vy = 0;
    double vz = sin(phi() * v);
    state(3,0) = vx, state(4,0) = vy, state(5,0) = vz;
}
void Pendulum::location(cv::Point3_<double> &p, double sec) const {
    if (sec == .0) {
        toXYZ(p);
    } else {
        double t = phase() + sec;
        double phi = phi_t(force(), radius(), t);
        toXYZ(xb(), yb(), zb(), radius(), phi, theta(), p);
    }
}

void Pendulum::uniform() {

    double radius = 0.8;
    double phi0 =  0.5;
    double xb = -0.0171;
    double yb = 0.6237;
    double zb = 1.;
    double phi = distUniform(-phi0, +phi0);
    double F = Pendulum::potential_force(phi0, radius);


    xb = distUniform(set_->range_(0,0), set_->range_(0,1));
    yb = distUniform(set_->range_(1,0), set_->range_(1,1));
    zb = 1;
    phi0 = distUniform(0, 0.5);

    radius = distUniform(set_->range_(2,0), set_->range_(2,1));;

    cv::Point3_<double> base = cv::Point3d(xb, yb, zb);
    init(base, radius, phi , true, 0, F);

    confidence_ = 0;
    nrOfUpdates_ = 0;

    for (int r = 0; r < s_.rows; r++) {
        if (s_(r,0) != s_(r,0)) {
            std::cerr << "Pendulum::uniform() produced a nan value at: " << set_->getStateInfo(r) << std::endl;
            std::cerr << human_readable() << std::endl;
            throw(0);
        }
    }
}


void Pendulum::update() {

    double e;
    double s = set_->motionError_[Pendulum::Set::BASE];
    cv::Point3_<double> base_e(xb(), yb(), zb());
    base_e.x += distNormal(s), base_e.y += distNormal(s);
    double radius_e = radius();
    e = distNormal(set_->motionError_[Pendulum::Set::RADIUS]);
    radius_e += e;
    double phase_e = phase();
    e = distNormal(pendulum_cycle()*set_->motionError_[Pendulum::Set::PHASE]);
    phase_e += e;
    double theta_e = theta();
    e = distNormal(set_->motionError_[Pendulum::Set::THETA]);
    theta_e += e;
    e += distNormal(set_->motionError_[Pendulum::Set::FORCE]);
    double force_e = force();
    force_e += e;


    if ((base_e.x < set_->range_(0,0)) || (base_e.x > set_->range_(0,1))) {
        base_e.x = distUniform(set_->range_(0,0), set_->range_(0,1));
    }
    if ((base_e.y < set_->range_(1,0)) || (base_e.y > set_->range_(1,1))) {
        base_e.y = distUniform(set_->range_(1,0), set_->range_(1,1));
    }
    if ((radius_e < set_->range_(2,0)) || (radius_e > set_->range_(2,1))) {
        radius_e = distUniform(set_->range_(2,0), set_->range_(2,1));
    }
    if (isinf(radius_e)) {
        std::cerr << "Pendulum::convertFrom() radius is inf!!" << std::endl;
        std::cerr << human_readable() << std::endl;
    }

    init(base_e, radius_e, phase_e, theta_e, force_e);

    s_(IDX_PHACE,0) = phase() + set_->dt();
    s_(IDX_PHI,0) = phi_t(force(), radius(), phase());
    s_(IDX_WPHI,0) = angular_velocity();

    if (inRange() == false) {
        uniform();
    }
    for (int r = 0; r < s_.rows; r++) {
        if (s_(r,0) != s_(r,0)) {
            std::cerr << "Pendulum::update() produced a nan value" << std::endl;
            //throw(0);
        }
    }
    nrOfUpdates_ ++;
}

bool Pendulum::inRange() {
    for (int r = 0; r < s_.rows; r++) {
        if (s_(r,0) != s_(r,0)) {
            return false;
        }
    }
    cv::Mat_<double> m;
    toXYZ(m);
    for (int r = 0; r < set_->range_.rows; r++) {
        if ((m(r,0) < set_->range_(r,0)) || (m(r,0) > set_->range_(r,1))) return false;
    }
    return true;
}

Particle* Pendulum:: ptr() {
    std::cerr << "ptr() not implemented";
    throw(0);
}

Particle::ParticleType Pendulum::classType() {
    return Particle::PENDULUM;
}
