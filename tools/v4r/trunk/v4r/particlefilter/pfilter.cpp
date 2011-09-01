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

#include "pfilter.h"
#include <v4r/cvextensions/print_cv.h>
#include <v4r/geometry/average.h>
#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/random/uniform_01.hpp> // for normal_distribution generator
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <fstream>
#include <time.h>
#include <stdlib.h>

using namespace std;

namespace V4R {

const int PFilter::MAX_HISTORY = 1000;
const double PFilter::QUANTILE_THRESHOLD = 0.8;

PFilter::PFilter() 
: sizeOfHistory_(0)
, updateCount_(0) {
    srand ( time(NULL) );
}
PFilter::~PFilter() {
    for (unsigned int i = 0; i < particles_.size(); i++) {
        delete particles_[i];
    }
}
void PFilter::print() {
    for (unsigned int i = 0; i < particles_.size(); i++) {
        std::cout << particles_[i]->human_readable() << std::endl;
    }
}

void PFilter::resample() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock( mutex_ );
    sort();
    resampleBest();
    injectSet();
    resampleUniform();
    resampleOutOfRange();
}

void PFilter::addSet(Particle::Set *pSet) {
    pSet->id(particleSets_.size());
    particleSets_.push_back(pSet);
}


void PFilter::initParticles(unsigned int nrOfParticles, double minParticleRatePerSet) {
    unsigned int nrOfParticlesPerSet = nrOfParticles/nrOfSets();
    minParticleRatePerSet_  =  minParticleRatePerSet;
    particles_.clear();
    particles_.reserve(nrOfParticlesPerSet*nrOfSets());
    for (unsigned int idSet = 0; idSet < particleSets_.size(); idSet++) {
        for (int i = 0; i < nrOfParticlesPerSet; i++) {
            Particle* particle = particleSets_[idSet]->createParticle();
            particles_.push_back(particle);
        }
    }
}
Particle::Set *PFilter::getSet(unsigned int idx) {
    return particleSets_[idx];
}

Particle* PFilter::operator[](int i) {
    return particles_[i];
}
unsigned int PFilter::nrOfParticles() {
    return particles_.size();
}
unsigned int PFilter::nrOfSets() {
    return particleSets_.size();
}

void PFilter::sort() {
    std::sort(particles_.begin(), particles_.end(), V4R::Particle::higherConfidence);
}

void PFilter::update() {
    BOOST_FOREACH(Particle::Set *set, particleSets_) set->pre_update();
    for (unsigned int i = 0; i < particles_.size(); i++) {
        particles_[i]->update();
        if (!(particles_[i]->inRange())) {
            particles_[i]->uniform();
        }
    }
    if (particles_.size() > 0) {
        LinearMotion<double> motion;
        unsigned int samples = estimate(motion, 0);
        ParticleStoragePtr particleData = ParticleStoragePtr(new Particle::Storage(sizeOfHistory_++, particles_[0], motion));
        particleData->setVarianz(0, samples);
        addToHistory(particleData);
    }
    updateCount_++;
}

    unsigned long PFilter::updateCount (){
      return  updateCount_;
    }
void PFilter::uniform() {
    for (unsigned int i = 0; i < particles_.size(); i++) {
        particles_[i]->uniform();
    }
}

void PFilter::resampleBest() {
    double sigma_scaled = resampleSigma_ * (double) nrOfParticles();
    unsigned int idx = 0;
    unsigned int lowThreshold = resampleThreshold();
    if(lowThreshold == 0) lowThreshold = 1;
    unsigned int upperThreshold = particles_.size();
    for (unsigned int i = lowThreshold; i < upperThreshold; i++) {
        idx = Particle::distNormalAbsIdx(sigma_scaled);
        if (idx < lowThreshold) {
          Particle::free(particles_[i]);
          particles_[i] = particles_[idx]->clone();
        }
    }
}

void PFilter::injectSet() {
    unsigned int minParticlePerSet = ((double) (nrOfParticles()/nrOfSets())) * minParticleRatePerSet_;
    /// Resample particles from other sets
    //for (unsigned int i = 0; i < nrOfParticles(); i++) std::cout << particles_[i]->idSet() << ", ";
    int idx = particles_.size() - 1;
    for (unsigned int idSet = 0; idSet < particleSets_.size(); idSet++) {
        Particle::Set *set = particleSets_[idSet];
        unsigned int nrOfParticlesInSet = set->nrOfParticles();

        if (nrOfParticlesInSet < minParticlePerSet) {
            //std::cout << "I " << set->human_readable();
            //std::cout << "; minParticlePerSet = " << minParticlePerSet << std::endl;
            unsigned int nrOfParticlesToInject = minParticlePerSet - nrOfParticlesInSet;
            //std::cout << " -> s=" << idSet << " +" <<  nrOfParticlesToInject << " -> ";
            for (int i = 0; i < nrOfParticlesToInject; i++) {
                Particle *p = set->createParticle();
                p->convertFrom(history_);
                //std::cout << particles_[0]->human_readable() << std::endl;
                //std::cout << p->human_readable() << std::endl;
                Particle::free(particles_[idx]);
                particles_[idx] = p;
                //std::cout << particles_[idx]->human_readable() << std::endl;
                idx--;
            }
        }
    }
    //for (unsigned int i = 0; i < nrOfParticles(); i++) std::cout << particles_[i]->idSet() << ", ";
    //std::cout << std::endl;
}

void PFilter::setRange ( const cv::Mat_<double> &m) {
    for (unsigned int idSet = 0; idSet < particleSets_.size(); idSet++) {
        particleSets_[idSet]->setRange(m);
    }
}

const boost::posix_time::ptime &PFilter::getTime (unsigned int idSet) {
    particleSets_[idSet]->getTime();
}
void PFilter::setTime (const boost::posix_time::ptime &t) {
    BOOST_FOREACH(Particle::Set *set, particleSets_) set->setTime(t);
}
void PFilter::setTime (double dtsec) {
    BOOST_FOREACH(Particle::Set *set, particleSets_) set->setTime(dtsec);
}
unsigned int PFilter::resampleThreshold() {
    unsigned int threshold = round(resampleSigma_ * (double) (nrOfParticles() * 2));
    if(threshold == 0) threshold = 1;
    return threshold;
}
void PFilter::resampleUniform() {
    /// Resample Uniform
    unsigned int threshold = resampleThreshold(); 
    int nrOfUniformParticles = resampleRandomQuote_ * (double) particles_.size();
    for (unsigned int i = 0; i < nrOfUniformParticles; i++) {
        unsigned int idx = (rand() % (nrOfParticles() - threshold)) + threshold;
        unsigned int idSet = rand() % nrOfSets();
        Particle::free(particles_[idx]);
        particles_[idx] = particleSets_[idSet]->createParticle();
        particles_[idx]->uniform();
    }
}

unsigned int PFilter::estimate(V4R::LinearMotion<double> &motion, double dt){
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock( mutex_ );
    std::vector< cv::Mat_<double> > states(resampleThreshold());
    for(unsigned int i = 0; i < states.size(); i++){
      particles_[i]->state2LinearMotion(states[i]);
    }
    motion = V4R::LinearMotion<double>(states);
    
    /*
    //std::cout << states << std::endl;
    std::vector<cv::Point3_<double> > bestParticles( resampleThreshold());  
    for(unsigned int i = 0; i < bestParticles.size(); i++){
      bestParticles[i].x = states(0,i);
      bestParticles[i].y = states(1,i);
      bestParticles[i].z = states(2,i);
    }
    cv::calcCovarMatrix(states, motion.covarianz(), motion.state(), CV_COVAR_NORMAL | CV_COVAR_ROWS);
    motion.covarianz() = motion.covarianz().reshape(6);
    std::cout << motion.human_readable(true) << std::endl;
    //motion.setLocation(mean_arithmetic(bestParticles));
    std::cout << varianz_arithmetic(bestParticles, mean_arithmetic(bestParticles)) << std::endl;
    return bestParticles.size();
    */
}

void PFilter::resampleOutOfRange() {
    /// Resample Particles out box of interest
    for (std::vector<V4R::Particle*>::iterator p = particles_.begin(); p != particles_.end(); p++) {
        if (!(*p)->inRange()) {
            (*p)->uniform();
        }
    }
}

void PFilter::setResampleParameter(double randomQuote, double sigma) {
    resampleRandomQuote_ = randomQuote;
    resampleSigma_ = sigma;
    boost::math::normal s(0, resampleSigma_);
    double threshold = resampleSigma_ * (double) (nrOfParticles() * 2);
    if(threshold < 1){
         std::cerr << "resampleThreshold is very low meaning: " <<  threshold << std::endl;
         std::cerr << "I will set it to 1" << std::endl;
    }
}

unsigned int PFilter::getHistory(std::vector<ParticleStoragePtr> &history, unsigned int numberOfEntries) {
    if (numberOfEntries > history_.size()) {
        history.reserve(history_.size());
    } else {
        history.reserve(numberOfEntries);
    }

    list<ParticleStoragePtr>::reverse_iterator rit;
    for (rit = history_.rbegin(); rit != history_.rend(); ++rit) {

        history.push_back(*rit);
        if (history.size() == numberOfEntries) {
            break;
        }
    }
    return history.size();
}

ParticleStoragePtr PFilter::lastEstimate() {
    if (history_.empty())
        return ParticleStoragePtr();
    return history_.back();
}

void PFilter::addToHistory(ParticleStoragePtr particleData) {
    while (history_.size() > MAX_HISTORY - 2) {
        history_.pop_front();
    }
    history_.push_back(particleData);
}

void PFilter::saveHistory(const std::string &fileName) {
    /*
        std::string matlabFile = fileName;
        ofstream mfile;
        std::stringstream s;
        std::stringstream A;
        std::stringstream t;
        std::stringstream dt;
        std::stringstream id;

        s << "pf_s = [\n";
        A << "pf_A = [\n";
        t << "pf_t = [\n";
        dt << "pf_dt = [\n";
        id << "pf_id = [\n";

        list<ParticleStoragePtr>::reverse_iterator rit;
        for (rit = history_.rbegin(); rit != history_.rend(); ++rit) {
            ParticleStoragePtr pd = (*rit);
            /// write state
            for (int r = 0; r < pd->s_.rows; r++) {
                s << std::setprecision(10) << std::showpos << std::fixed << pd->s_(r, 0);
                if (r == pd->s_.rows - 1)
                    s << ";\n";
                else
                    s << ", ";
            }
            /// write transformation
            for (int r = 0; r < pd->s_.rows; r++) {
                for (int c = 0; c < pd->s_.cols; c++) {
                    A << std::setprecision(10) << std::showpos << std::fixed << pd->s_(r, c);
                    if ((r == pd->s_.rows - 1) && (c == pd->s_.cols - 1))
                        A << ";\n";
                    else
                        A << ", ";
                }
            }

            /// write time duration
            double microseconds = ((double) pd->dt_.total_microseconds()) / 1000000.;
            dt << std::setprecision(6) << std::fixed << microseconds << ";\n";

            /// write mesurment idx
            id << pd->id_ << ";\n";
        }

        s << "]\n";
        A << "]\n";
        t << "]\n";
        dt << "]\n";
        id << "]\n";
        mfile.open(fileName.c_str());
        mfile << s.str() << A.str() << t.str() << dt.str() << id.str();
        mfile.close();
    */
}

}
