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

    // Returns true if the PTZ is moving.
    ["cpp:const"] bool isMoving();
  };

  /**
   * value of the completion state of the commands
  */
  enum PTZCompletion {
    FAILED,
    SUCCEEDED,
    COMPINIT
  };

  /**
   * This command returns the current ptz position. All fields are set by the
   * system. The field pose represent the current pose and the field comp returns
   * the completion status of the command. It will change to SUCCEEDED if no
   * errors occur. 
   * @author Torben Toeniges
   */
  class GetPTZPoseCommand {
    PTZPose pose;
    PTZCompletion comp;
  };

  /**
   * This command can be used to set a pose of the PTZ unit.
   * The pose field has to be set by the user and is updated after the completion
   * of the command to the current position of the unit. 
   * Furthermore the comp field shows the completion of the command. 
   * @author Torben Toeniges 
   */
  class SetPTZPoseCommand {
    PTZPose pose;
    PTZCompletion comp;
  };
};

#endif // LASER_ICE
