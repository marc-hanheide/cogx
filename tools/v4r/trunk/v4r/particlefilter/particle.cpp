/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "particle.h"
#include <sys/stat.h>
#include <boost/random.hpp>
#include <boost/random/uniform_01.hpp>


namespace V4R {
boost::minstd_rand gParticleIntGenStd;
boost::mt19937 gParticleIntGen19937;
boost::variate_generator<boost::mt19937, boost::normal_distribution<> > gParticleNormal(gParticleIntGen19937, boost::normal_distribution<>(0,1));
boost::uniform_01<boost::minstd_rand> gParticleUniform(gParticleIntGenStd);
};

using namespace V4R;
using namespace cv;

unsigned long Particle::particlecount_ = 0;

Particle::Particle(ParticleType type)
        : type_(type)
        , confidence_(0)
	, nrOfUpdates_(0)
	, idUnique_(particlecount_)
{
  particlecount_++;
};


Particle::~Particle()
{
    // std::cout << "Particle Base destructor" << std::endl;
};


cv::Mat_<double> &Particle::state() {
    return s_;
}
const cv::Mat_<double> &Particle::state() const {
    return s_;
}
double &Particle::operator[] ( int row ) {
    return s_(row,0);
}
const double &Particle::operator[] ( int row ) const {
    return s_(row,0);
}
void Particle::copyTo(Particle *p) const {
    std::cerr << "copyTo() not implemented";
    throw(0);
}
void Particle::convertFrom(const std::list< boost::shared_ptr<Storage> > &history) {
    std::cerr << "convertFrom() not implemented";
    throw(0);
}

Particle* Particle::clone() const {
    std::cerr << "clone() not implemented";
    throw(0);
};

void Particle::setSet(Particle::Set *p) {
    set_ = p;
}
const Particle::Set* Particle::getSet() const {
    return set_;
}
Particle::Set* Particle::getSet() {
    return set_;
}
void Particle::copyDataTo(Particle *p, bool force, bool copyType, bool copySetPtr) const {
    if (force) {
        if (copyType) {
            p->type_ = type_;
        }
        if (copySetPtr) {
            p->set_ = set_;
        }
    } else {
        if (p->type_ != type_) {
            std::cerr << "copyDataTo() because of different type not possible";
            throw(0);
        }
        if (p->set_->type() != set_->type()) {
            std::cerr << "copyDataTo() because of different parameter ptr not possible";
            throw(0);
        }
    }
    p->idxFilter_ = idxFilter_;
    p->nrOfUpdates_ = nrOfUpdates_;
    p->confidence_ = confidence_;
    s_.copyTo(p->s_);
};

double Particle::confidence() const {
    return confidence_;
};

void Particle::confidence(double value) {
    confidence_ = value;
};

bool Particle::valid() const {
    if (s_.empty()) return false;
    if (s_.cols != 1) return false;
    if (s_.rows > 1) return false;
    return true;
}

bool Particle::inRange() {
    std::cerr << "inRange() not implemented";
    throw(0);
}

void Particle::uniform() {
    std::cerr << "uniform() not implemented";
    throw(0);
}
void Particle::update() {
    std::cerr << "update() not implemented";
    throw(0);
}

void Particle::state2LinearMotion(cv::Mat_<double> &state, double sec) const{
    std::cerr << "stateLinearized(cv::Mat_<double> &state) not implemented";
    throw(0);
}

void Particle::location(cv::Point3_<double> &p, double sec)  const {
    std::cerr << "location(cv::Point3_<double> &p, double sec) not implemented";
    throw(0);
}

void Particle::location(cv::Point_<double> &p, double sec)  const {
    std::cerr << "location(cv::Point_<double> &p, double sec) not implemented";
    throw(0);
}
void Particle::orientation(cv::Vec<double, 3> &r, double sec) const {
    std::cerr << "orientation(cv::Vec<double, 3> &r, double sec = .0) not implemented";
    throw(0);
}

void Particle::orientation(double &alpha, double sec) const {
    std::cerr << "orientation(double &alpha, double sec = .0) not implemented";
    throw(0);
}

void Particle::set(const cv::Point3d &position, const cv::Point3d &velocety) {
    std::cerr << "set(const cv::Point3d &position, const cv::Point3d &velocety) not implemented";
    throw(0);
}
void Particle::set(const cv::Point2d &position, const double &orientation) {
    std::cerr << "set(const cv::Point2d &position, const double &orientation) not implemented";
    throw(0);
}
double Particle::distUniform() {
    return gParticleUniform();
}
double Particle::distNormal() {
    return gParticleNormal();
}

double Particle::distUniform(double scale) {
    return gParticleUniform() * scale;
}
double Particle::distUniform(double A, double B) {
    return A + gParticleUniform() * (B-A);
}
double Particle::distNormal(double sigma) {
    return gParticleNormal() * sigma;
}
double Particle::distNormal(double sigma, double median) {
    return gParticleNormal() * sigma + median;
}
unsigned int Particle::distNormalAbsIdx(double sigma) {
    return (unsigned int ) abs(gParticleNormal() * sigma) ;
}

unsigned long Particle::idUnique() const{
  return idUnique_;
}

std::string Particle::human_readable(bool more) const {
    std::stringstream ss;
    ss << set_->getName();
    ss << "; id  = " << std::setprecision(10) << std::fixed << idUnique_;
    ss << "; c  = " << std::setprecision(2) << std::fixed << confidence_;
    ss << "; it = " << std::setprecision(2) << std::fixed << nrOfUpdates_;
    ss << "; s = [";
    for (int r = 0; r < s_.rows; r++) {
        for (int c = 0; c < s_.cols; c++) {
            if (c > 0) ss << ", ";
            ss << " " << set_->getStateInfo(r) << " ";
            ss << std::setw(8) << std::showpos << std::setprecision (5) << s_(r,c);
        }
        if (r < s_.rows-1)  ss << "; ";
    }
    ss << "];";
    return ss.str();

}

int medianParticles(std::vector< Particle* > &sortedParticles, double sigma) {
    std::cerr << "medianParticles not implemented";
    throw(0);
}

Particle* Particle:: ptr() {
    std::cerr << "ptr() not implemented";
    throw(0);
}

Particle::ParticleType Particle::type() const {
    return type_;
}

Particle::ParticleType Particle::classType() const {
    std::cerr << "classType() not implemented";
    throw(0);
}

void Particle::free(Particle *p) {
    Particle::Set *pSet = p->getSet();
    pSet->deleteParticle(p);
}

unsigned int Particle::nrOfUpdates() const{
    return nrOfUpdates_;
}


