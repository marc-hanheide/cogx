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

using namespace V4R;
using namespace cv;

void Pendulum::init(const cv::Point3_<double> _base, double _radius, double _phi, bool _goes_left, double _theta, double _force) {
    if (_force < 0) {
        _force = Pendulum::potential_force(_phi, _radius);
    }
    if (_radius < 0) {
        std::cerr << "Pendulum::init(base, radius, phi, goes_left, theta, force) negative radius !!" << std::endl;
        std::cerr << "init( " << "_base" << ", " << _radius << ", " << _phi << ", " << _goes_left << ", " << _theta << ", " << _force << ") " << std::endl;
        throw(0);
    }

    s_(IDX_Xb,0) = _base.x;
    s_(IDX_Yb,0) = _base.y;
    s_(IDX_Zb,0) = _base.z;
    s_(IDX_RADIUS,0) = _radius;
    s_(IDX_PHI,0) = _phi;
    s_(IDX_FORCE,0) = _force;
    s_(IDX_THETA,0) = _theta;
    s_(IDX_WTHETA,0) = 0;
    s_(IDX_WPHI,0) = angular_velocity(radius(), force(), phi());
    s_(IDX_PHACE,0) = phase(radius(), phi_max(), phi(), _goes_left);
    for (int r = 0; r < s_.rows; r++) {
        if (s_(r,0) != s_(r,0)) {
            std::cerr << "Pendulum::init(base, radius, phi, goes_left, theta, force) produced a nan value at: " << set_->getStateInfo(r) << std::endl;
            std::cerr << human_readable() << std::endl;
            throw(0);
        }
    }
}
void Pendulum::init(const cv::Point3_<double> _base, double _radius, double _phase, double _theta, double _force) {
    if (_radius < 0) {
        std::cerr << "Pendulum::init(base, radius, phase, theta, force) negative radius !!" << std::endl;
        std::cerr << "init( " << "_base" << ", " << _radius << ", " << _phase << ", " << _theta << ", " << _force << ") " << std::endl;
        throw(0);
    }
    s_(IDX_Xb,0) = _base.x;
    s_(IDX_Yb,0) = _base.y;
    s_(IDX_Zb,0) = _base.z;
    s_(IDX_RADIUS,0) = _radius;
    s_(IDX_FORCE,0) = _force;
    s_(IDX_PHACE,0) = _phase; //normalize_phase (radius(), _phase)
    s_(IDX_THETA,0) = _theta;
    s_(IDX_WTHETA,0) = 0;
    s_(IDX_PHI,0) = phi_t(force(), radius(), phase());
    s_(IDX_WPHI,0) = angular_velocity(radius(), force(), phi());
    for (int r = 0; r < s_.rows; r++) {
        if (s_(r,0) != s_(r,0)) {
            std::cerr << "Pendulum::init(base, radius, phase, theta, force) produced a nan value at: " << set_->getStateInfo(r) << std::endl;
            std::cerr << human_readable() << std::endl;
            throw(0);
        }
    }
}

double Pendulum::normalize_phase (double radius, double phase) {
    double tau = pendulum_cycle(radius);
    while (phase > tau) {
        phase -= tau;
    }
    return phase;
}

double Pendulum::angular_velocity () {
    double w = angular_velocity( radius(),force(), phi());
    return w;
}

void Pendulum::toXYZ(cv::Point3_<double> &p) const {
    toXYZ(xb(), yb(), zb(), radius(),phi(), theta(), p);
}
void Pendulum::toXYZ(cv::Mat_<double> &m) const {
    cv::Point3_<double> p;
    toXYZ(p);
    m.create(3,1);
    m(0,0) = p.x, m(1,0) = p.y, m(2,0) = p.z;
}

double Pendulum::kinetic_force() const {
    return kinetic_force(wphi(), radius());
}
double Pendulum::potential_force() const {
    return potential_force(phi(),radius());
}

double Pendulum::phi_max() const {
    double phi = phi_max(kinetic_force() + potential_force(), radius());
    if (phi != phi) {
        std::cerr << "Pendulum::phi_max()" << std::endl;
        std::cerr << "kinetic_force() = " << kinetic_force();
        std::cerr << ", potential_force() = " << potential_force();
        std::cerr << ", radius() = " << radius();
        throw(0);
    }
    return phi;
}

const double &Pendulum::xb() const {
    return s_(IDX_Xb,0);
}
const double &Pendulum::yb() const {
    return s_(IDX_Yb,0);
}
const double &Pendulum::zb() const {
    return s_(IDX_Zb,0);
}
const double &Pendulum::radius() const {
    return s_(IDX_RADIUS,0);
}
const double &Pendulum::phi() const {
    return s_(IDX_PHI,0);
}
const double &Pendulum::wphi() const {
    return s_(IDX_WPHI,0);
}
const double &Pendulum::theta() const {
    return s_(IDX_THETA,0);
}
const double &Pendulum::wtheta() const {
    return s_(IDX_WTHETA,0);
}
const double &Pendulum::force() const {
    return s_(IDX_FORCE,0);
}
const double &Pendulum::phase() const {
    return s_(IDX_PHACE,0);
}


double Pendulum::kinetic_force(double w, double radius, double m) {
    double v = radius * w;
    double Fk = (m * v * v)/2.0;
    return Fk;
}

double Pendulum::potential_force(double phi, double radius, double m) {
    double h = radius * (1-cos(phi));
    double Fp = Ag * m * h;
    if (Fp != Fp) {
        std::cerr << "Pendulum::potential_force(phi, radius, m)" << std::endl;
        std::cerr << "potential_force( " << phi << ", " << radius << ", " << m << ") " << std::endl;
        throw(0);
    }
    return Fp;
}
double Pendulum::pendulum_cycle(double radius) {
    double tmp = fabs(radius / Ag);
    double T = 2.0 * M_PI * sqrt(tmp);
    if (T != T) {
        std::cerr << "Pendulum::pendulum_cycle(radius)" << std::endl;
        std::cerr << "phase( " << radius << ") " << std::endl;
        std::cerr << "tmp = " << tmp;
        throw(0);
    }
    return T;
}

double Pendulum::pendulum_cycle() const {
    double T = 2.0 * M_PI * sqrt( radius() / Ag);
    return T;
}

double Pendulum::lift(double radius, double phi) {
    double h = radius * (1-cos(phi));
    return h;
}

double Pendulum::lift_max(double F, double m) {
    double h = F/(Ag*m);
    return h;
}

double Pendulum::phi_max(double F, double radius, double m) {
    double h0 = lift_max(F, m);
    double tmp = 1 - h0/radius;
    if (tmp > 1.0) tmp = 1;
    if (tmp < -1.0) tmp = -1;
    double phi0 = acos(tmp);
    if (phi0 != phi0) {
        std::cerr << "Pendulum::phi_max(F, radius, m)" << std::endl;
        std::cerr << "phi_max( " << F << ", " << radius << ", " << m << ") " << std::endl;
        std::cerr << "tmp = " << tmp << std::endl;
        std::cerr << "h0 = " << h0 << std::endl;
        throw(0);
    }
    return phi0;
}

double Pendulum::phi_t(double F, double radius, double t, double m) {
    double phi0 = phi_max(F, radius, m);
    double s_phi = cos(sqrt(Ag / radius) * t);
    double phi = phi0 * s_phi;
    if (phi != phi) {
        std::cerr << "Pendulum::phi_t(F, radius, t, m)" << std::endl;
        std::cerr << "phi_max( " << F << ", " << radius << ", " << t << ", " << m << ") " << std::endl;
        std::cerr << "phi0 = " << phi0 << std::endl;
        std::cerr << "s_phi = " << s_phi << std::endl;
        std::cerr << "phi = " << phi << std::endl;
        throw(0);
    }
    return phi;
}

double Pendulum::phase(double radius, double phi0, double phi, bool goes_left) {
    double t, tmp;
    if (phi0 == phi) {
        t = M_PI / 2.;
    } else {
        tmp = phi / phi0;
        if (tmp > 1.0) tmp = 1;
        if (tmp < -1.0) tmp = -1;
        if (goes_left) {
            t = acos(tmp) / sqrt(Ag/radius);
        } else {
            t = acos(tmp) / sqrt(Ag/radius) + pendulum_cycle(radius) / 2;
        }
    }
    if (t != t) {
        std::cerr << "Pendulum::phase(radius, phi0, phi, goes_left)" << std::endl;
        std::cerr << "phase( " << radius << ", " << phi0 << ", " << phi << ", " << goes_left << ") " << std::endl;
        std::cerr << "pendulum_cycle(radius) = " << pendulum_cycle(radius) << std::endl;
        std::cerr << "tmp = " << tmp << std::endl;;
        throw(0);
    }
    return t;
}

double Pendulum::angular_velocity (double radius, double F, double phi, double m) {
    double Fp = potential_force(phi, radius);
    double Fk = F - Fp;
    double v = sqrt(fabs((2.0 * Fk)/m));
    double w = v / radius;
    if (w != w) {
        std::cerr << "Pendulum::angular_velocity(radius, F, phi)" << std::endl;
        std::cerr << "angular_velocity( " << radius << ", " << F << ", " << phi << ", "  << m << ") " << std::endl;
        std::cerr << "Fp = " << Fp << ", Fk = " << Fk << ", v = " << v << ", w = "  << w << std::endl;
        throw(0);
    }
    return w;
}
void Pendulum::toXYZ(double xb, double yb, double zb, double radius, double phi, double theta, cv::Point3_<double> &p) {
    double dx =  sin(phi) * radius;
    double dz = -cos(phi) * radius;
    double ct = cos(theta);
    double st = sin(theta);
    p.x = xb + dx*ct;
    p.y = yb + dx*st;
    p.z = zb + dz;
    if ((p.x != p.x) || (p.y != p.y) || (p.z != p.z)) {
        std::cerr << "toXYZ() has a nan value" << std::endl;
        std::cerr << "; xb     = " << xb << std::endl;
        std::cerr << "; yb     = " << xb << std::endl;
        std::cerr << "; zb     = " << zb << std::endl;
        std::cerr << "; radius = " << radius << std::endl;
        std::cerr << "; phi    = " << phi << std::endl;
        std::cerr << "; theta  = " << theta << std::endl;
        std::cerr << "; dx     = " << dx << std::endl;
        std::cerr << "; dz     = " << dz << std::endl;
        std::cerr << "; ct     = " << ct << std::endl;
        std::cerr << "; st     = " << st << std::endl;
        std::cerr << "; p.x    = " << p.x << std::endl;
        std::cerr << "; p.y    = " << p.y << std::endl;
        std::cerr << "; p.z    = " << p.z << std::endl;
        throw(0);
    }
}
