//
// = LIBRARY
//
// = FILENAME
//    NewWrappedSLAM.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    John Folkesson
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 John Folkesson
//
/*----------------------------------------------------------------------*/

#ifndef NewWrappedSLAM_hh
#define  NewWrappedSLAM_hh
#include <AddressBank/DataSlotAddress.hh>
#include <AddressBank/FileAddress.hh>
#include <Filters/DeadReckoner.hh>
#include <Map/EKFSLAM.hh>
#include <SensorData/SmartData.hh>
#include <Map/PoseProvider.hh>
#include <AddressBank/RLDisplayHelper.hh>
#include <Filters/ScanLineFilter.hh>

#ifndef DEPEND
#endif

namespace Cure {

// Forward declaration(s)
class PoseFilter;
class RLDisplaySICK;
/**
 * This class encapsulated the filters needed to build a map from
 * odometry and scan data. The configuration is handled with a config
 * file (ccf) in the format for the ConfigFileReader.
 *
 * You use it by calling the functions addOdometry and addScan. When
 * addScan returns 1 you know that the filter has been updated, 0 if
 * ok but no update and negative value in case of error.
 *
 * Or you can just push every thing to the input ports from the raw sources.
 *
 *  Inputs;
 *       0 raw cumulated odometry
 *       1 raw SICK scans
 *       2 raw gyro (optional)
 *       3 measurement sets from the camera (optional)
 * 
 * Setting the covariance type the SENSORPOSE in the cfg file will
 * Allow the offset from the robot to the sensor to be included in
 * the EKF state.
 *
 * If using input 3 and scanlines one must be sure to write the scans and 
 * measurementSets in sequence. 
 * One must write scans (for now) to get robolook updates, by seting 
 * useScanLines=false one can avoid using the scans  for SLAM.
 * then no hough is made.
 *
 * @author John Folkesson AND Patric Jensfelt 
 * @see EKFSLAM, DeadReckoner
 */
class NewWrappedSLAM: public DataFilter, public PoseProvider {

public:

  /**
   * This is the easy constructor with default models.  This uses
   * SimpleOdoModel as modelA.  If usefuse is true it sets modelB as
   * GyroModel and fuser to GyroAdjOdometry.  The config must then be
   * set by config(const std::string &cfgFile);
   *
   * So this is the constructor for using the cfg file to set the models,
   * For now there is no real choice of models as we only have 1 implemented.
   * However the cfg file can taylor the SimpleOdoModel to a robot.
   *
   * 
   * 
   * 
   * @param numberOfSensors maximum of 5 for now.
   *
   */
  NewWrappedSLAM(short numberOfSensors = 1, bool usefuse = false);

  /**
   * This is for those that want to pass their own models/fuser or set 
   * the buffer sizes...
   *
   *
   * @param modelA for the DeadReckoner.
   * @param period for the DeadReckoner.
   * @param sensortypes for the EKFLocatizer
   * @param numberofsensors for the EKFLocatizer
   * @param modelB for the DeadReckoner, set to 0 for just odometry.
   * @param fuser for the DeadReckoner, set to 0 for just odometry.
   * @param userotate for the DeadReckoner, set to false for just odometry.
   * @param inputBuffer for the DeadReckoner, 
   *                    this many input records are saved.
   * @param outputBuffer for the DeadReckoner,
   *                     this many output records are stored.
   * 
   * 
   *
   * @see DeadReckoner, EKFLocatizer
   */
  NewWrappedSLAM(PoseErrorModel *modelA, Timestamp & period,
      short numberOfSensors = 0, PoseErrorModel *modelB = 0, FuseFilter *fuser =
          0, bool userotate = false, long inputBuffer = 50, long outputbuffer =
          100);

  ~NewWrappedSLAM();

  /** 
   * @return The Pose of the robot. NOTE that this might
   * not correspond to the current physical position of the robot.
   */
  Pose3D getPose() const;

  /**
   * Use this function to get a prediction of the pose of the robot at
   * the time corresponding to the last written odometry data. If not
   * possible a pose with time 0 will be returned
   * 
   * @return pose
   */
  Pose3D getPosePrediction();

  /** 
   * Call this function to setup the NewWrappedSLAM using info from config file
   * 
   * @return 0 on sucess
   */
  int config(const std::string &cfgFile);

  /**
   * This decreases (or increases for factor<1) the amount of clutter,
   * (unwanted small walls). 
   * @param factor one is no change, >1 makes it harder to add walls,
   *               <1 makies it easier. 
   */
  void removeClutter(double factor);

  /**
   * This increases (or decreases for factor<1) the quality threshold for 
   * the lines.
   * @param factor one is no change, >1 makes it harder to find lines,
   *               <1 makies it easier. 
   */
  void increaseLineQuality(double factor);

  /**
   * This increases (or decreases for factor<1) the parameters
   * for the sensor error model which set the minimum covariance for the 
   * lines.  
   * @param factor one is no change, >1 makes the lines error estimate larger,
   *               <1 makies it smaller. 
   */
  void increaseMinCovariance(double factor);

  /**
   * This sets the overall relative strength of the measurements.
   * Gain less than one gives more weight to deadreckoning.
   *@param gain gain>1 increases the sensitivity to feature measurments.
   */
  void setGain(double gain);

  /**
   * This increases (or decreases for factor<1) the threshold for 
   * making the match to features. 
   * @param factor one is no change, >1 makes it easier to match lines,
   *               <1 makies it harder. 
   */
  void loosenMatching(double factor, int type);

  /**
   * @return 0 if ok
   */
  int addOdometry(const Pose3D &odo);

  int addInertial(const Pose3D &inert);

  /** 
   * @return 0 if no update, 1 if updated and neg if error
   */
  int addScan(const SICKScan &scan);

  /**
   * Use this function to add a set of measurements. This could be of
   * any type, for example, line extracted from a laser scan or point
   * extracted from an image.
   */
  int addMeasurementSet(MeasurementSet &meas);

  int saveMap(const std::string &filename);
  int saveTrackKeys(const std::string &filename);

  /**
   * The idea with this function is that you should be able to change
   * the pose of the robot if you know from some other source that it
   * is somewhere else. This could be useful for example if you have a
   * user interface where the user can specify the location of the
   * robot.
   *
   * @param p the new pose to use
   * @param useCov if true the covariance information from p will be
   * use, otherwise some hardcoded values will be used.
   */
  int resetRobotPose(const Pose3D &p, bool useCov = true);

protected:

  /**
   * Function that makes it possible to hook up this object to
   * Addresses that pump odometry and scan data directly
   * into the objects
   */
  unsigned short write(Cure::TimestampedData& p, const int port = -1);

  /** 
   * Function that helps perfom the stuff that all constructors need
   * to do
   */
  void constructorInit();

  TimestampedData * data(int port) {
    if (port < Ins)
      return &Data[port];
    return Data[port].getTPointer();
  }

private:
  /** Time for last updated  pose */
  Timestamp m_LastTimePoseUpdated;
  /** This could be a odometry model for example*/
  PoseErrorModel * m_ModelA;
  /** 
   * This could be a Gyro model for example
   * If left unset the only the A input will be used.
   */
  PoseErrorModel * m_ModelB;
  /** 
   * This could be a GyroAdjOdometry for example 
   *  The PoseErrorModels will feed this and it will feed the output. 
   */
  FuseFilter * m_Fuser;

protected:
  SmartData Data[4];
  unsigned short m_ConfigModels;
public:
  /** The Bank with all features */
  MapBank m_MapBank;

  /** The map information */
  FeatureMap m_Map;

  /** The Filter that does the actual map track */
  EKFSLAM *m_SLAM;

  /** The deadreckoning estimates */
  DeadReckoner m_Dead;

  /** Extracts lines from scan data */
  ScanLineFilter m_Hough;

  /** Slot for scans */
  DataSlotAddress m_SlotScans;

  /** Slot for all measurements */
  DataSlotAddress m_SlotMeas;

  /** Slot for SensorOffsets */
  DataSlotAddress *m_OffsetSlot;

  /** Class that helps to display scan data */
  RLDisplaySICK *m_ScanDisp;

  /** Class that helps to connect to RoboLook */
  RLDisplayHelper m_Rlph;

  /** Debug file for odometry and measurements to see the sequency we get */
  std::fstream m_walkclocktimefile;

  /** max 5 */
  short m_NumberOfSensors;
  bool useScanlines;
};

inline Pose3D NewWrappedSLAM::getPose() const {
  return m_SLAM->PoseOut;
}

} // namespace Cure

#endif // NewWrappedSLAM_hh
