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

using namespace V4R;

Particle::Storage::Storage()
        : id_(0)
        , varianz_(-1.)
        , samples_(0) {
}
Particle::Storage::~Storage() {
    //std::cout << "Storage::~Storage()" << std::endl;
    Particle::free(particle_);
    delete set_;
}

Particle::Storage::Storage(unsigned long id, const Particle *particle, const LinearMotion<double> &motion)
        : id_(id) {
    set_ = particle->getSet()->clone();
    particle_ = set_->createParticle();
    particle->copyDataTo(particle_, false, false, false);
    motion.copyTo(motion_);
}
Particle::Storage::Storage(unsigned long id, const Particle *particle)
        : id_(id) {
    set_ = particle->getSet()->clone();
    particle_ = set_->createParticle();
    particle->copyDataTo(particle_, false, false, false);
    particle_->state2LinearMotion(motion_.state());
}

void Particle::Storage::setVarianz(double varianz, unsigned int samples) {
    varianz_ = varianz;
    samples_ = samples;
}

const V4R::LinearMotion<double> &Particle::Storage::motion() const {
    return motion_;
}

const Particle* Particle::Storage::particle() const {
    return particle_;
}

const Particle::Set* Particle::Storage::getSet() const {
    return set_;
}

std::string Particle::Storage::human_readable(bool more) const {
    return particle_->human_readable(more);
}
double Particle::Storage::varianz() const {
    return varianz_;
}
double Particle::Storage::nrOfSamples() const {
    return samples_;
}
