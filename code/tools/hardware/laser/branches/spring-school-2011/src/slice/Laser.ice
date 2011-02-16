/**
 * Module to define data structures for laser sensors
 * 
 */

#ifndef LASER_ICE
#define LASER_ICE

#include <cast/slice/CDL.ice>

module Laser {

  sequence<double> RangeSequence;  

  struct Scan2d
  {
    // Timestamp when odometry data was acquired
    cast::cdl::CASTTime time;

    // Array with the range readings [m]
    RangeSequence ranges;

    // The angle of the first beam [rad]
    double startAngle;

    // The angle between the two consecutve range readings [rad]
    double angleStep;   

    // The max range for a valid reading [m]
    double maxRange;

    // The min range for a valid reading [m]
    double minRange;

    // The resolution in range [m]
    double rangeRes;
  };

  interface Scan2dPushClient {

    /**
     * This function is called by the server when new Scan2D data is
     * available to the client
     */
    void receiveScan2d(Scan2d scan);
  };

  interface LaserServer extends cast::interfaces::CASTComponent {

    /**
     * Get the latest Scan2d data available to the server
     */
    Scan2d pullScan2d();

    /** 
     * Use this function if you as a client want to get Scan2d data
     * pushed to you with a certain interval
     *
     * @param client the client to push the data to
     * @param the interval [s] for data to be pushed (-1 as fast as possible)
     */
    void registerScan2dPushClient(Scan2dPushClient *client, double interval);
  };

};

#endif // LASER_ICE
