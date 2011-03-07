//
// = FILENAME
//    HSSPoseArchive.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2006 Patric Jensfelt (Cure)
//                  2010 Patric Jensfelt (HSS)
//
/*----------------------------------------------------------------------*/

#ifndef HSS_PoseArchive_hh
#define HSS_PoseArchive_hh

namespace HSS {

/**
 * This class has a buffer with pose data. You can use this buffer to
 * get poses at time within the archive, possibly using
 * interpolation.
 *
 * @author Patric Jensfelt
 */
class PoseArchive 

public:

  /**
   * Constructor
   */
  PoseArchive(int length = 100);

  /**
   * Destructor
   */
  ~PoseArchive();

  /**
   * Use this function to add new pose data to the archive.
   *
   * @param p pose
   *
   * @return 0 if pose added, 1 if already added, 2 if not moved enough 
   */
  int addPose(const Eigen::Vector3d &p);

  /**
   * Get the last pose added to the archive
   */
  Eigen::Vector3d getLastAddedPose() { return m_LastAddedPose; }

  /**
   * Use this function to get an interpolation from to the poses in
   * the archive to a certain time.
   * 
   * @param t time to interpolate too.
   * @param p pose to put result in
   * 
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   *
   * @see getExtrapolation
   */
  int getInterpolation(const struct timeval &tv, Eigen::Vector3d &p);

  /**
   * Use this function to get estimate of the pose at a certain time
   * and allow for extraplation from the last pose in the archive.
   * 
   * @param t time to extrapolate too.
   * @param p pose to put result in
   * @param maxT max time to extrapolate forward from last pose in archive
   * 
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   *
   * @see getExtrapolation
   */
  int getExtrapolation(const struct timeval &tv, Eigen::Vector3d &p,
                       const double maxT = 0.5);

  /**
   * Use this function to get a prediction for the pose at a certain
   * time in another coordinate system that can be considered to have
   * a fixed transformation with respect to the poses in the
   * archive. This function will first get the pose at the time of
   * pOld, calculate the difference in archive poses to the last known
   * and then use this to get the new pose in the other coordinate
   * system.
   *
   * @param pOld old pose (input) 
   * @param tNew time of interpolated pose
   * @param pNew new predicted pose (output)
   *
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   */
  /*
  int getPoseInterpolation(const Cure::Pose3D &pOld, 
                           const Cure::Timestamp &tNew,
                           Cure::Pose3D &pNew);
  */

  /**
   * Use this function to get a prediction for the pose possibly at a
   * future time in another coordinate system that can be considered
   * to have a fixed transformation with respect to the poses in the
   * archive. This function will first get the pose at the time of the
   * pOld, calculate the difference in archive pose to the
   * extrapolation time and then use this to get the new pose in the
   * other coordinate system.
   *
   * @param pOld old pose (input) 
   * @param t time to extrapolate to
   * @param pNew new predicted pose (output)
   * @param maxT max allowed extrapolation from last archive pose
   *
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   */
  /*
  int getPoseExtrapolation(const Cure::Pose3D &pOld, const Cure::Timestamp &t,
                           Cure::Pose3D &pNew,
                           const double maxT = 0.5);
  */

  /**
   * Use this function to find the transformation from a given pose to
   * the poses in the archive. The pose in the archive to compare
   * against is specified by the timestamp of the input pose.
   *
   * @param p2 input pose from which you want a transformation to
   * the poses in the pose archive.
   * @param T transformation to pose archive pose from the pose you provided
   *
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   */
  //int getTransformationToArchive(const Cure::Pose3D &p2, Cure::Pose3D &T);

  /**
   * Use this function to find the transformation between the poses in
   * the archive and a given pose. The pose in the archive to compare
   * against is specified by the timestamp of the input pose.
   *
   * @param p2 input pose to which you want a transformation from
   * the poses in the pose archive.
   * @param T transformation from pose archive pose to the pose you provided
   *
   * @return 0 if OK, else error code
   *                  archive empty  (TIMESTAMP_ERROR|NO_DATA_AVAILABLE)
   *                  time in future (TIMESTAMP_ERROR|NO_DATA_AVAILABLE|TIMESTAMP_IN_FUTURE)
   *                  time too old   (TIMESTAMP_ERROR | TIMESTAMP_TOO_OLD)
   */
  //int getTransformationFromArchive(const Cure::Pose3D &p2, Cure::Pose3D &T);

protected:
  int getInterpolationOutsideArchive(const struct timeval &tv, 
                                     Eigen::Vector3d &p);
  
  int getExtrapolationOutsideArchive(const struct timeval &tv, 
                                     Eigen::Vector3d &p,
                                     const double maxT);

  int calcExtrapolation(const struct timeval &tv,
                        const Eigen::Vector3d &p2,
                        Eigen::Vector3d &p);

protected:
  /// Last pose added to the PoseArchive but that might not be in the
  /// m_Archive which can be configured to store only poses when the
  /// robot has moved "enough"
  Eigen::Vector3d m_LastAddedPose;

}; // class PoseArchive

}; // namespace HSS

#endif // HSS_PoseArchive_hh
