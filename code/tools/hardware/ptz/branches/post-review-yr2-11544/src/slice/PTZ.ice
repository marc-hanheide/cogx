/**
 * Module to define data structures for pan-tilt unit.
 * 
 */

#ifndef PTU_ICE
#define PTU_ICE

#include <cast/slice/CDL.ice>

module ptz {

  /**
   * The ptz position.
   */
  struct PTZPose {

    /// The cam pan value [rads]
    double pan;

    /// The cam tilt value [rads]
    double tilt;

    /// The cam zoom value [rads ... corresponds to camera field of view apparently]
    double zoom;

  };

  /**
   * The ptz reading. A position with a timestamp.
   */
  struct PTZReading {

    /// Timestamp when the pose was acquired
    cast::cdl::CASTTime time;

    /// The ptz pose
    PTZPose pose;
  };



  /** This is the interface for getting and setting ptz pose.  **/

  interface PTZInterface {

    /**
     * Get the current ptz pose.
     */  
    ["cpp:const"] PTZReading getPose();

    /**
     * Set the desired ptz pose.
     */
    void setPose(PTZPose cmd);

  };


};

#endif // LASER_ICE
