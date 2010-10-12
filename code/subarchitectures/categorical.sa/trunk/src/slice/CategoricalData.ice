#ifndef CATEGORICALDATA_ICE
#define CATEGORICALDATA_ICE

#include <cast/slice/CDL.ice>
#include <Laser.ice>
#include <Robotbase.ice>

/**
 * Data structures representing the knowledge stored in the categorical layer
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module CategoricalData 
{

  // ---------------------------------------------------------------------
  // Data
  // ---------------------------------------------------------------------

  /** Sequence of bytes. */
  sequence<byte> ByteSeq;


  /** Image buffer. */
  struct ImageData 
  {
    int width;
    int height;
    ByteSeq data;  // width*height bytes of data
  };


  /** Image provided by the DataProvider. */
  class Image
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Real timestamp of the image. */
    cast::cdl::CASTTime realTimeStamp;

    /** Time when the image was placed in the WM. */
    cast::cdl::CASTTime wmTimeStamp;

    /** Image buffer. */
    ImageData imageBuffer;
  };

  /** Laser scan provided by the DataProvider. */
  class LaserScan
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Time when the scan was placed in the WM. */
    cast::cdl::CASTTime wmTimeStamp;

    /** Real timestamp of the scan. */
    cast::cdl::CASTTime realTimeStamp;

    /** Scan buffer. */
    Laser::Scan2d scanBuffer;
  };

  /** Odometry provided by the DataProvider. */
  class Odometry
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Odometry information. */
    Robotbase::Odometry odometryBuffer;

    /** Time when the odomerty was placed in the WM. */
    cast::cdl::CASTTime wmTimeStamp;

    /** Real timestamp of the odometry reading. */
    cast::cdl::CASTTime realTimeStamp;
  };

  /** Target class information provided by the DataProvider. */
  class Target
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Target number. */
    long targetNo;

    /** Target name. */
    string targetName;
  };

};
#endif // CATEGORICALDATA_ICE
