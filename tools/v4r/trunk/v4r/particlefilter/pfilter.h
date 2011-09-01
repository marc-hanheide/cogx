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

#ifndef PFILTER_H
#define PFILTER_H

#include <v4r/particlefilter/particle.h>
#include <boost/foreach.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <list>


#ifndef DEG2RAD
#define DEG2RAD(a) ((a) * M_PI / 180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG(a) ((a) * 180.0 / M_PI)
#endif

namespace V4R
{

class PFilter
{
public:
    static const int MAX_HISTORY;
    static const double QUANTILE_THRESHOLD;

    PFilter();
    ~PFilter();

    void addSet(Particle::Set *pSet);
    void initParticles(unsigned int nrOfParticles, double minParticleRatePerSet = 0.);
    Particle::Set *getSet(unsigned int idx);
    void print();
    void update();
    void uniform();
    void resample();
    void injectSet();
    void setResampleParameter ( double randomQuote, double sigma );
    unsigned int nrOfSets();
    unsigned int nrOfParticles();
    Particle* operator[] ( int i );
    unsigned int getHistory (std::vector< ParticleStoragePtr > &history, unsigned int numberOfMeasurements = 10 );
    /**
     * computes optiomal state based on the first sigma particles
     * @param motion 6x1 [x, y, z, vx, vy, vz]
     * @param dt
     * @return number of samples used for the computation
     **/
    unsigned int estimate(V4R::LinearMotion<double> &motion, double dt = 0);
    ParticleStoragePtr lastEstimate();
    void saveHistory ( const std::string &fileName );
    void setRange ( const cv::Mat_<double> &m);
    const boost::posix_time::ptime &getTime (unsigned int idSet = 0);
    void setTime (const boost::posix_time::ptime &t);
    void setTime (double dt);
    void updateTransformation ();
    unsigned long updateCount ();
private:
    void sort();
    void resampleBest();
    void resampleUniform();
    void resampleOutOfRange();
    unsigned int resampleThreshold(); /// should cover 95/2 % and is based on resampleSigma_
    boost::interprocess::interprocess_mutex mutex_; /// used to prevent concurrent access to the data
protected:

    void addToHistory ( ParticleStoragePtr particleData );
    unsigned long sizeOfHistory_;
    std::vector<Particle::Set*> particleSets_;
    std::vector<Particle*> particles_;
    cv::Mat_<double> transform_;
    std::list< ParticleStoragePtr > history_;
    double minParticleRatePerSet_;
    double resampleSigma_;
    double resampleRandomQuote_;
    unsigned long updateCount_;
};
}

#endif // PFILTER_H
