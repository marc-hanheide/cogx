/**
 * Module to define data structures for sonar sensors
 * 
 */

#ifndef SONAR_ICE
#define SONAR_ICE

#include <cast/slice/CDL.ice>

module Sonar {

  sequence<double> RangeSequence;  

  /**
  * A 2d sonar scan 
  */
  struct SonarScan2d
  {
    // Timestamp when odometry data was acquired
    cast::cdl::CASTTime time;

    // Array with the range readings [m]
    RangeSequence ranges;
  };

//   interface SonarScan2dPushClient {

//     /**
//      * This function is called by the server when new Scan2D data is
//      * available to the client
//      */
//     void receiveSonarScan2d(SonarScan2d scan);
//   };

  interface SonarServer extends cast::interfaces::CASTComponent {

    /**
     * Get the latest SonarScan2d data available to the server
     */
    SonarScan2d pullSonarScan2d();

//     /** 
//      * Use this function if you as a client want to get SonarScan2d data
//      * pushed to you with a certain interval
//      *
//      * @param client the client to push the data to
//      * @param the interval [s] for data to be pushed (-1 as fast as possible)
//      */
//     void registerSonarScan2dPushClient(SonarScan2dPushClient *client, double interval);
  };

};

#endif // SONAR_ICE
