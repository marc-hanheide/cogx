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

#ifndef PLINEAR_H_
#define PLINEAR_H_

#include <v4r/particlefilter/particle.h>

namespace V4R {

class PLinear : public Particle {
public:
    class Set : public Particle::Set  {
    public:
        static const unsigned int VELOCITY = 0;
        static const unsigned int INCLINATION = 1;
        static const unsigned int AZIMUTH = 2;
        static const unsigned int INVERT = 3;
        Set();
        Set(Set &r);
        virtual ~Set();
        /** 
	 * computes the transformtion matrix
	 * @param A
	 * @param dtsec
	 **/
        virtual void compute_transformtion(cv::Mat_<double> &A, double dtsec) const;
        /** 
	 * creates a particle and sets the pointer within the particle
	 * @return new particle
	 **/
        virtual Particle* createParticle();
        /** 
	 * geneartes a clone, the data must be freed by the hand
	 * @return copy of the current set
	 **/
        virtual Particle::Set* clone() const;
        /** 
	 * relases a particle
	 * @param p
	 **/
        virtual void deleteParticle(Particle* p);
    };


    virtual void state2LinearMotion(cv::Mat_<double> &state, double sec = .0) const;
    virtual void location(cv::Point3_<double> &p, double sec = .0) const;
    virtual void copyTo(Particle *p) const;
    virtual void convertFrom(const std::list< boost::shared_ptr<Storage> > &history);
    virtual Particle* clone() const;
    virtual void set(const cv::Point3d &position, const cv::Point3d &velocety);
    virtual void uniform();
    virtual void update();
    virtual bool inRange();
    virtual Particle* ptr();
    virtual ParticleType classType();
protected:
    virtual ~PLinear();
    PLinear();
};


};

#endif /* PLINEAR_H_ */
