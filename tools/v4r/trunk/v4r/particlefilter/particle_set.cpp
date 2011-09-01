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
namespace V4R
{


unsigned long Particle::Set::setcount_ = 0;

Particle::Set::Set()
        : time_ ( boost::posix_time::microsec_clock::local_time() )
        , nrOfParticles_(0)
        , id_(-1)
        , idUnique_(setcount_++)
{
    //std::cout << "Particle::Set constructor" << std::endl;
    setName("NA");
    color_ = cv::Vec<double, 4>(0,1,1,1);
}

Particle::Set::Set ( Particle::Set &r ) {
    r.copyTo(*this);
}

Particle::Set::~Set()
{
    // std::cout << "Particle::Set destructor" << std::endl;
}

unsigned int Particle::Set::id() const{
    return id_;
}

void  Particle::Set::id(unsigned int idx) {
    id_ = idx;
}

Particle::ParticleType Particle::Set::type() const{
    return type_;
}

void Particle::Set::copyTo ( Particle::Set &r ) const
{
    if (type_ != r.type_) {
        std::cerr << "copyTo not possible";
        throw(0);
    }
    
    A_.copyTo(r.A_);
    range_.copyTo(r.range_);
    r.motionError_ = motionError_;
    r.motionNames_ = motionNames_;
    r.color_ = color_;
    r.id_ = id_;
    r.time_ = time_;
    r.nrOfParticles_ = nrOfParticles_;
    r.name_ = name_;
    r.dtsec_ = dtsec_;
    r.motionNames_ = motionNames_;
    r.stateInfo_ = stateInfo_;
}

void Particle::Set::setRange ( const cv::Mat_<double> &range )
{
    range.copyTo ( range_ );
    for ( int r = 0; r < range.rows; r++ )
    {
        if ( range ( r,0 ) > range ( r,1 ) )
        {
            range_ ( r,1 ) =  range ( r,0 ), range_ ( r,0 ) =  range ( r,1 );
        }
    }
}

Particle::Set* Particle::Set::clone() const {
    std::cerr << "Particle::Set::clone() not implemented";
    throw ( 0 );
}


std::string Particle::Set::human_readable() const {
    std::stringstream ss;
    ss << "Set " << name_;
    ss << "; id = " << id_;
    ss << "; uid " << idUnique_;
    ss << "; nrOfParticles = " << nrOfParticles_;
    return ss.str();
}

void Particle::Set::compute_transformtion(cv::Mat_<double> &A, double dtsec) const {
    std::cerr << "compute_transformtion(cv::Mat_<double> &A, double dtsec) not implemented";
    throw ( 0 );
}

void Particle::Set::pre_update () {
    compute_transformtion(A_, dtsec_);
}

Particle* Particle::Set::createParticle ( )
{
    std::cerr << "createParticle() not implemented";
    throw ( 0 );
}
void Particle::Set::deleteParticle ( Particle* p)
{
    std::cerr << "deleteParticle() not implemented";
    throw ( 0 );
}

void Particle::Set::setMotionError ( unsigned int defName, double value )
{
    if ( defName >= motionError_.size() ) {
        std::cerr << "setMotionError wrong vector size";
        throw ( 0 );
    }
    motionError_[defName] = value;
}

void Particle::Set::setMotion ( unsigned int defName, double value )
{
    if ( defName >= motion_.size() ) {
        std::cerr << "setMotion wrong vector size";
        throw ( 0 );
    }
    motion_[defName] = value;
}

const boost::posix_time::ptime &Particle::Set::getTime ()
{
    return time_;
}

void Particle::Set::setTime (const boost::posix_time::ptime &t )
{
    dtsec_ = duration2sec(t - time_);
    if (dtsec_ < 0.0) dtsec_ = 0;
    time_ = t;
}

void Particle::Set::setTime(double dtsec) {
    if (dtsec < 0.0) dtsec = 0;
    time_ = time_ + sec2duration(dtsec);
    dtsec_ = dtsec;
}

double Particle::Set::duration2sec (const boost::posix_time::time_duration &dt )
{
    return dt.total_microseconds() / 1000000.;
}

boost::posix_time::time_duration Particle::Set::sec2duration (double dtsec)
{
    return boost::posix_time::microseconds(dtsec*1000000);
}

double Particle::Set::getMotionError(unsigned int defName) const {
    return motionError_[defName];
}

const double &Particle::Set::dt() const {
    return dtsec_;
}

unsigned int Particle::Set::nrOfParticles() {
    return nrOfParticles_;

}

const std::string &Particle::Set::getName() const {
    return name_;
}
void Particle::Set::setName(const std::string &n) {
    name_ = n;
}

const std::string &Particle::Set::getStateInfo(unsigned int idx) const {
    return stateInfo_[idx];
}
}
