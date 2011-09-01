/**
 * @file particle.h
 * @author Markus Bader
 * @brief
 *
 * @see
 **/

#ifndef V4R_PARTICLE_H
#define V4R_PARTICLE_H

#include <v4r/geometry/structs.h>
#include <v4r/geometry/motion.h>
#include <opencv/cv.h>
#include <stdio.h>
#include <list>
#include <iostream>
#include <iomanip>
#include <boost/random.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/thread/xtime.hpp>
//#include <Blort/CModel/ABase/ConfigFile.hh>

namespace V4R {

class Particle {
public:
    enum ParticleType {
        BASE = 0, RANDOM = 1, LINEAR = 2, PENDULUM = 3, ROBOT2D = 4
    };

    /// Class to handle a set of particles
    class Set {
    public:
        /// Constructor
        Set();
        /// Copy constructor
        Set(Set &r);
        /// destructor
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
        /**
        * preprocessing step needed
        * @post Particle::update
        **/
        void pre_update();
        /**
        * sets the state space
        * @param range
        **/
        void setRange(const cv::Mat_<double> &range);
        /// copy function
        void copyTo(Set &r) const;
        /// updates the timestamps and computed the time interval since the last mesurment
        void setTime(const boost::posix_time::ptime &t);
        /// updates the timestamps by adding a duration
        void setTime(double dtsec);
        /// current time
        const boost::posix_time::ptime &getTime();
        /// sets a motion error
        void setMotion(unsigned int defName, double value);
        /// sets a motion error
        void setMotionError(unsigned int defName, double value);
        /// returns a motion error entry
        double getMotionError(unsigned int defName) const;
        /// returns a information about a state entry
        const std::string &getStateInfo(unsigned int idx) const;
        /// returns the number of particles
        std::string human_readable() const;
        /// number of particles within the set
        unsigned int nrOfParticles();
        /// name of the set
        const std::string &getName()  const;
        /// sets name of the set
        void setName(const std::string &n);
        /// duration since last setTime in secounds
        const double &dt() const;
        /// converts boost duration to secounds
        static double duration2sec(const boost::posix_time::time_duration &dt);
        /// converts secounds to boost duration
        static boost::posix_time::time_duration sec2duration (double dtsec);
        /**
         * Returns the id / idx of the set within the filter
         * @return idx
         **/
        unsigned int id() const;
        /**
         * Sets the id / idx of the set within the filter
         * @param idx
         **/
        void id(unsigned int idx);
        /**
         * Returns the id / idx of the set within the filter
         * @return idx
         **/
        ParticleType type() const;
        /// Variables
        cv::Mat_<double> A_; /// transormation matrix
        cv::Mat_<double> range_; /// state space
        std::vector<double> motionError_; /// motion error
        std::vector<double> motion_; /// motion error
        cv::Vec<double, 4> color_;
    protected:
        unsigned long idUnique_;  /// Unique id
        int id_; /// Id of the set defined by the add function of the filter
        ParticleType type_; /// type of particle to manage
        boost::posix_time::ptime time_; /// current time
        unsigned int nrOfParticles_;
        std::string name_;
        double dtsec_; /// dt since last update in secounds
        std::vector<std::string> motionNames_; /// name of the related motion error
        std::vector<std::string> stateInfo_; /// state info optional
        static unsigned long setcount_;
    };

    class Storage {
    public:
        Storage();
        ~Storage();
        /**
         * constructor
         * @param id
         * @param particle current best particle
         * @param state_ current position with linearized veloceties 6x1
         **/
        Storage(unsigned long id, const Particle *particle, const LinearMotion<double> &motion_);
        /**
         * constructor uses the current particle to set the current position and veloceties
         * @param id
         * @param particle current best particle
         **/
        Storage(unsigned long id, const Particle *particle);
        /**
         * @return human readable string / particle details
         **/
        std::string human_readable(bool more = false) const;
        /**
         * @return particle
         **/
        const Particle *particle() const;
        /**
         * @return a linear motion
         **/
        const V4R::LinearMotion<double> &motion() const;
        /**
         * @param varianz overall varianz
         * @param nrOfSamples nr of samples used to compute varianz
         **/
        void setVarianz(double varianz, unsigned int nrOfSamples); 
        /**
         * @return overall varianz
         **/
        double varianz() const;
        /**
         * @return nr of samples used to compute varianz
         **/
        double nrOfSamples() const;
        /**
         * @return Particle::Set
         **/
        const Particle::Set* getSet() const;
        
    private:
        unsigned long id_;
        V4R::LinearMotion<double> motion_;
        Particle* particle_;
        Particle::Set* set_;
        double varianz_;
        unsigned int samples_;
    };
    /// destructor
    ~Particle();
    /// @returns the conficence of the particle between 0 and 1
    double confidence() const;
    /// @param value sets conficence of the particle between 0 and 1 at time t-age
    /// @param age
    void confidence(double value);
    /**
     * used for the sort function for decreasing sort
     * @return true if the confidence of left is hight as right
     **/
    static bool higherConfidence(const Particle* left, const Particle* right) {
        return left->confidence() > right->confidence();
    }
    /**
     * checks if the transform matrix and the state matrix are valid
     * @return true if valid
     **/
    bool valid() const;

    /// @return returns a uniform number between 0 an 1
    static double distUniform();
    /**
     * @param scale
     * @return returns a uniform number between (0 an 1) * scale
     **/
    static double distUniform(double scale);

    /**
     * @param A
     * @param B
     * @return returns a uniform number between A an B
     **/
    static double distUniform(double A, double B);

    /// @return returns a normal distributed number with median 0 and sigma 1
    static double distNormal();
    /**
     * @param sigma
     * @return returns a normal distributed number with median 0
     **/
    static double distNormal(double sigma);
    /**
     * returns only positive int --> actualy it is not a normal dirstibution anymore
     * @param sigma
     * @return returns a normal distributed number with median 0
     **/
    static unsigned int distNormalAbsIdx(double sigma);
    /**
     * @param sigma
     * @param median
     * @return returns a normal distributed number
     **/
    static double distNormal(double sigma, double median);

    /**
     * sets the particle set
     * @param set
     **/
    void setSet(Particle::Set *set);

    /**
     * retuns the corrsponding particle set
     * @return set
     **/
    const Particle::Set* getSet() const;
    /**
     * retuns the corrsponding particle set
     * @return set
     **/
    Particle::Set* getSet();

    ///@return particle index which is unique per set
    unsigned int idx() const;

    ///@param idx particle index which is unique per filter
    void idxFilter(unsigned int idx) {
        idxFilter_ = idx;
    }

    ///@return particle index which is unique per filter
    unsigned int idxFilter() const {
        return idxFilter_;
    }

    ///@return particle state
    cv::Mat_<double> &state();

    ///@return particle state
    const cv::Mat_<double> &state() const;

    /**
     * returns one single value of the particles state
     * @param row row index of the state
     * @param reference to the value
     **/
    double &operator[] ( int row );

    /**
     * returns one single value of the particles state
     * @param row row index of the state
     * @param reference to the value
     **/
    const double &operator[] ( int row ) const;

    /**
     * finds the median particle of the best sigma particle and returns it index
     * @param sortedParticles sorted particles
     * @param sigma to define the numer (0.6 Quantile of sigma ) for the evaluation
     * @return index matching to the input vector
     **/
    static int medianParticles(std::vector<Particle*> &sortedParticles, double sigma);

    /// places the particle on a random place within the state space
    virtual void uniform();
    /**
    * transforms the particle by the transformation defined by the particle parameters
    * @pre Particle::Set::pre_update
    **/
    virtual void update();
    /// @return true if the particle is in state space
    virtual bool inRange();

    /** 
     * returns the state linearized in a certain time
     * @param state
     * @param sec state
     * @return linearized state
     **/
    virtual void state2LinearMotion(cv::Mat_<double> &state, double sec = .0) const;

    /// @return locaction of the particle after a cartain time
    virtual void location(cv::Point3_<double> &p, double sec = .0) const;

    /// @return locaction of the particle after a cartain time
    virtual void location(cv::Point_<double> &p, double sec = .0) const;

    /// @return orientation of the particle after a cartain time
    virtual void orientation(cv::Vec<double, 3> &r, double sec = .0) const;

    /// @return orientation of the particle after a cartain time
    virtual void orientation(double &alpha, double sec = .0) const;

    /**
     * returns a pointer of the corrcect virutal class
     * @return pointer
     **/
    virtual Particle* ptr();
    /**
     * returns id of the current class
     * @return ParticleType id of the virual class
     **/
    virtual ParticleType classType() const;

    /** copies all entries if the particle is of the same type
     *  @param p target
     **/
    virtual void copyTo(Particle *p) const;

    /** places the robot
     **/
    virtual void set(const cv::Point3d &position, const cv::Point3d &velocety);

    /** places the robot
     **/
    virtual void set(const cv::Point2d &position, const double &orientation);
    /**
     * geneartes a clone
     * @post the data must be freed by the hand
     **/
    virtual Particle* clone() const;

    /**
     * converts a particle of an other type the to the current one
     * @param history of last mesurments
     **/
    virtual void convertFrom(const std::list< boost::shared_ptr<Storage> > &history);
    /**
     *  Removes the object
     **/
    static void free(Particle *p);

    /** copies all entries if the particle is of the same type
     *  @param p target
     *  @param force on true it will copy the particle data even if it not of the same type but not the type
     *  @param copyType on true it will copy also the type
     *  @param copySetPtr on true it will copy the pointer to the parameters/set
     **/
    void copyDataTo(Particle *p, bool force, bool copyType, bool copySetPtr) const;
    /**
     * produces a human readable string
     * @param more on true it will print additinal information
     * @return string
     **/
    std::string human_readable(bool more = false) const;
    /**
     * returns id of the current class
     * @return ParticleType id of the virual class
     **/
    ParticleType type() const ;

    /**
     * unique id over all particles
     * @return idUnique_
     **/
    unsigned long idUnique() const;

    /**
     * number of updates of this spezific particle
     * @return iterations_
     **/
    unsigned int nrOfUpdates() const;

protected:
    ParticleType type_;
    unsigned long idUnique_;  /// Unique id
    unsigned int idxFilter_;
    unsigned int nrOfUpdates_;  /// number of updates of this spezific particle
    double confidence_;
    cv::Mat_<double> s_;
    Particle::Set *set_;
protected:
    /// constructor
    Particle(ParticleType type);
    static unsigned long particlecount_;
};

typedef boost::shared_ptr<Particle> ParticlePtr;
typedef boost::shared_ptr<Particle::Set> ParticleSetPtr;
typedef boost::shared_ptr<Particle::Storage> ParticleStoragePtr;
};
#endif //V4R_PARTICLE_H
