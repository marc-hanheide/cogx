/**
 * Module to define data structures for robot base sensors
 * 
 */

#ifndef ROBOTBASE_ICE
#define ROBOTBASE_ICE

#include <cast/slice/CDL.ice>

module Robotbase {

  /**
   * The robot pose in 2D.
   */
  struct Pose2d {
    // The x-coordinate [m]
    double x;

    // The y-coordinate [m]
    double y;

    // The orientation [rad]
    // The definition of what the orientation actually refers
    // to is tricky. On a differential drive platform it would
    // coincide with the direction of motion.
    double theta;
  };

  /**
   * Sequence that by definition should have zero or one
   * element. Zero if the integrated odometry is not available
   * and one otherwise
   */ 
  sequence<Pose2d> Pose2dOpt;

  /**
   * Sequence that is either zero long if no speed information is available
   * from the platform or contain as many speed readinsg as the platform
   * provides "natively".
   *
   * Different drive: 0=trans.speed, 1= rot.speed
   */ 
  sequence<double> SpeedOpt;

  /**
   * Sequence that is either zero long if no encoder readings are available 
   * or contains as many readings as encoders
   *
   * Diff. drive: 0=right, 1=left
   */
  sequence<long> EncoderOpt;

  struct Odometry
  {
    // Timestamp when odometry data was acquired
    cast::cdl::CASTTime time;

    // Optional integrated odometry
    Pose2dOpt odompose;   

    // Optional speed information
    SpeedOpt speed;

    // Optional encoder data
    EncoderOpt encoder;
  };

  struct MotionCommand
  {
    // The desired translation speed [m/s]
    double speed;

    // The desired rotation speed [rad/s]
    double rotspeed; 
  };

  interface OdometryPushClient {

    /**
     * This function is called by the server when new Odometry data is
     * available to the client
     */
    void receiveOdometry(Odometry odom);
  };

  /**
   * This is the interface for the servers that provide odometry data
   */
  interface RobotbaseServer extends cast::interfaces::CASTComponent {

    /**
     * Pull the last odometry data from the server (will take what is 
     * already there and not wait for new from the hardware)
     */
    Odometry pullOdometry();

    /**
     * Call this function to execute a certain motion command
     * @param cdm the motion command to execute
     * @return true if it can be executed
     */
    void execMotionCommand(MotionCommand cmd);

    /** 
     * Use this function if you as a client want to get Odometry data
     * pushed to you with at a certain interval
     *
     * @param client the client to push the data to
     * @param the interval to push with [s] (-1 as fast as possible)
     */
    void registerOdometryPushClient(OdometryPushClient *client, 
                                    double interval);

  };

};

#endif  // ROBOTBASE_ICE
