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
  // Place.SA external interface
  // ---------------------------------------------------------------------

  /** Name of unknown place class. */
  const string UnknownClassName = "unknown";

  /** Number of unknown place class. */
  const long UnknownClassNo = 0;


  // ---------------------------------------------------------------------
  // DataProvider
  // ---------------------------------------------------------------------

  /** Types of commands that can be sent to DataProvider. */
  enum DataProviderCommandType
  {
    /** Acquire new data. */
    DpCmdUpdate,

    /** Invalid command. */
    DpCmdInvalid
  };

  /** Command sent to DataProvider. */
  class DataProviderCommand
  {
    /** The command. */
    DataProviderCommandType cmd;
  };

  /** Ack that command was received by DataProvider. */
  class DataProviderCommandAck
  {
    /** The command. */
    DataProviderCommand cmd;

    /** Process that sent the command. */
    string src;
  };


  // ---------------------------------------------------------------------
  // Data
  // ---------------------------------------------------------------------

  /** Status of the data (scans, images, odometry readings) in the working memory. */
  enum DataStatus
  {
    /** No data of this type were found. */
    DsInvalid,

    /** Data are available and valid. */
    DsValid
  };


  /** Sequence of bytes. */
  sequence<byte> ByteSeq;


  /** Image buffer. */
  struct ImageData 
  {
    int width;
    int height;
	// width*height bytes of data
    ByteSeq data;  
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

    /** Status of the image. */
    DataStatus status;
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

    /** Status of the scan. */
    DataStatus status;
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

    /** Status of the data. */
    DataStatus status;
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

    /** Status of the data. */
    DataStatus status;
  };


  // ---------------------------------------------------------------------
  // VisualProcessor
  // ---------------------------------------------------------------------

  class VisualProcessorStatus
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Status of the data. */
    DataStatus status;

    /** Real timestamp of the image. */
    cast::cdl::CASTTime imageRealTimeStamp;

    /** Time when the image was placed in the WM. */
    cast::cdl::CASTTime imageWmTimeStamp;

    /** Time when the processing started. */
    cast::cdl::CASTTime processingStartTimeStamp;

    /** Time when the feature extraction finished. */
    cast::cdl::CASTTime extractionEndTimeStamp;

    /** Time when the classification finished. */
    cast::cdl::CASTTime classificationEndTimeStamp;
  };

  struct ClassifierOutput
  {
    string name;
    double value;
  };

  sequence<ClassifierOutput> ClassifierOutputs;

  struct ClassifierResult
  {
    long classNo;
    string className;
    double confidence;
  };

  sequence<ClassifierResult> ClassifierResults;

  enum SvmMulticlassAlg {SmaOaO, SmaOaA};

  class VisualResults
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Status of the data. */
    DataStatus status;

    /** Real timestamp of the image. */
    cast::cdl::CASTTime imageRealTimeStamp;

    /** Time when the image was placed in the WM. */
    cast::cdl::CASTTime imageWmTimeStamp;

    /** Outputs of the classifier. */
    ClassifierOutputs outputs;

    /** SVM multi-class algorithm */
    SvmMulticlassAlg multiclassAlg;

    /** Hypotheses finding algorithm. */
    long hypFindAlg;

    /** Results produced by the classifier. */
    ClassifierResults results;

    /** Confidence threshold below which the classifier is unconfident. */
    double confidenceThreshold;

    /** Was the classifier confident? */
    bool confident;
  };



  // ---------------------------------------------------------------------
  // LaserProcessor
  // ---------------------------------------------------------------------


  class LaserProcessorStatus
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Status of the data. */
    DataStatus status;

    /** Real timestamp of the scan. */
    cast::cdl::CASTTime scanRealTimeStamp;

    /** Time when the scan was placed in the WM. */
    cast::cdl::CASTTime scanWmTimeStamp;

    /** Time when the processing started. */
    cast::cdl::CASTTime processingStartTimeStamp;

    /** Time when the feature extraction finished. */
    cast::cdl::CASTTime extractionEndTimeStamp;

    /** Time when the classification finished. */
    cast::cdl::CASTTime classificationEndTimeStamp;
  };

  class LaserResults
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Status of the data. */
    DataStatus status;

    /** Real timestamp of the scan. */
    cast::cdl::CASTTime scanRealTimeStamp;

    /** Time when the scan was placed in the WM. */
    cast::cdl::CASTTime scanWmTimeStamp;

	bool useSize;

    /** Outputs of the classifier. */
    ClassifierOutputs outputs;

    /** Outputs of the classifier. */
    ClassifierOutputs sizeOutputs;

    /** SVM multi-class algorithm */
    SvmMulticlassAlg multiclassAlg;

    /** Hypotheses finding algorithm. */
    long hypFindAlg;

    /** Results produced by the classifier. */
    ClassifierResults results;

    /** Results produced by the classifier. */
    ClassifierResults sizeResults;

    /** Confidence threshold below which the classifier is unconfident. */
    double confidenceThreshold;

    /** Was the classifier confident? */
    bool confident;
    bool sizeConfident;
  };


};
#endif // CATEGORICALDATA_ICE
