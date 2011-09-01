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

#ifndef PPENDULUM_H_
#define PPENDULUM_H_

#include <v4r/particlefilter/particle.h>

namespace V4R {
class Pendulum : public Particle {
public:
    virtual ~Pendulum();
    static const double Ag = 9.81;
    class Set : public Particle::Set  {
    public:
        static const unsigned int FORCE = 0;
        static const unsigned int RADIUS = 1;
        static const unsigned int PHASE = 2;
        static const unsigned int BASE = 3;
        static const unsigned int THETA = 4;
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
    
    const double &xb() const;
    const double &yb() const ;
    const double &zb() const;
    const double &radius() const;
    const double &phi() const;
    const double &wphi() const;
    const double &theta() const;
    const double &wtheta() const;
    const double &force() const;
    const double &phase() const;


    /**
     * init pendulum
     * @param base mount point
     * @param radius distance of masse to base
     * @param phi current angle between straight down and mass
     * @param goes_left current moving direction of the mass
     * @param theta roation around z
     * @param force total force in the system, if force < 0 it will assume that the phi is at its maximum
     * @see Pendulum::potential_force(phi, radius);
     **/
    void init(const cv::Point3_<double> base, double radius, double phi, bool goes_left, double theta, double force); 
    /**
     * init pendulum
     * @param base mount point
     * @param radius distance of masse to base
     * @param phase phase time t since cycle time tau
     * @param theta roation around z
     * @param force total force in the system, if force < 0 it will assume that the phi is at its maximum
     * @see Pendulum::potential_force(phi, radius);
     **/
    void init(const cv::Point3_<double> base, double radius, double phase, double theta, double force); 
    
    virtual void state2LinearMotion(cv::Mat_<double> &state, double sec = .0) const;
    virtual void location(cv::Point3_<double> &p, double sec = .0) const;

    virtual void copyTo(Particle *p) const;
    virtual void convertFrom(const std::list< boost::shared_ptr<Storage> > &history);
    virtual Particle* clone() const;
    virtual void uniform();
    virtual void update();
    virtual bool inRange();
    virtual Particle* ptr();
    virtual ParticleType classType();

    /**
     * @param w angular velocety dphi/dt
     * @param radius distance of masse to base
     * @param m mass
     * @return kinetic force 
     **/
    static double kinetic_force(double w, double radius, double m = 1.);
    double kinetic_force() const;
    /**
     * @param phi angle
     * @param radius distance of masse to base
     * @param m mass
     * @return potential force 
     **/
    static double potential_force(double phi, double radius, double m = 1.);
    double potential_force() const;
    
    /**
     * Cycletime 
     * @param radius distance of masse to base
     * @return Tau 
     **/
    static double pendulum_cycle(double radius);
    double pendulum_cycle() const;
    
    /**
     * normalized the phase to a value under the cycle time tau 
     * @param radius distance of masse to base
     * @param phase time since max phi
     * @return phase < tau 
     **/
    static double normalize_phase (double radius, double phase);

    /**
     * maximal lift of the pendulum 
     * @param F 
     * @param m mass
     * @return maximal lift at ph0 
     **/
    static double lift_max(double F, double m = 1.);

    /**
     * maximal angle on the maximum lift
     * @param F 
     * @param radius distance of masse to base
     * @param m mass
     * @return maximal angle 
     **/
    static double phi_max(double F, double radius, double m = 1.);
    double phi_max() const;
    /**
     * angle at time t
     * @param F 
     * @param radius distance of masse to base
     * @param t current time
     * @param m mass
     * @return current angle
     **/
    static double phi_t(double F, double radius, double t, double m = 1.0);    
    
    /**
     * current lift of the pendulum 
     * @param l distance of masse to base
     * @param phi angle between straight down and mass
     * @return current lift
     **/
    static double lift(double radius, double phi);
     /**
     * angular velocety at a specific angle 
     * @param radius distance of masse to base
     * @param F Total force in the system
     * @param phi specific angle between straight down and mass
     * @return angular velocety
     **/
    static double  angular_velocity (double radius, double F, double phi, double m = 1.0);
    double  angular_velocity ();
    
     /**
     * current timeoffest to the maximum phi
     * @param radius distance of masse to base
     * @param phi0 maximum angle between straight down and mass
     * @param phi specific angle between straight down and mass
     * @param goes_left current moving direction of the mass
     * @return time since last phi max
     **/
    static double phase(double radius, double phi0, double phi, bool goes_left = true);
    /**
     * computes the mass location in xyz space
     * @param xb x of the fixed mounting position
     * @param yb y of the fixed mounting position
     * @param zb z of the fixed mounting position
     * @param radius distance of mass to base
     * @param phi angle between straight down and mass
     * @param theta angle on the xy plan  
     * @param p computed position 
     **/
    static void toXYZ(double xb, double yb, double zb, double radius, double phi, double theta, cv::Point3_<double> &p);
    void toXYZ(cv::Point3_<double> &p) const;
    void toXYZ(cv::Mat_<double> &m) const;
protected:
    static const unsigned int IDX_Xb = 0;
    static const unsigned int IDX_Yb = 1;
    static const unsigned int IDX_Zb = 2;
    static const unsigned int IDX_RADIUS = 3;
    static const unsigned int IDX_PHI = 4;
    static const unsigned int IDX_WPHI = 5;
    static const unsigned int IDX_THETA = 6;
    static const unsigned int IDX_WTHETA = 7;
    static const unsigned int IDX_FORCE = 8;
    static const unsigned int IDX_PHACE = 9;
    Pendulum();
};

};

#endif /* PLINEAR_H_ */
