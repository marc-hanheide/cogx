// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

#ifndef PLACEDATA_ICE
#define PLACEDATA_ICE

#include <cast/slice/CDL.ice>
#include <Laser.ice>
#include <Robotbase.ice>
#include <Video.ice>


module PlaceData
{
  // ---------------------------------------------------------------------
  // Place.SA external interface
  // ---------------------------------------------------------------------

  /** Types of commands that can be sent to Place.SA. */
  enum PlaceCommandType
  {
    /** Acquire new data and update information about the current place. */
    CmdUpdate,

    /** Start performing continuous update. */
    CmdStart,

    /** Stop performing continuous update. */
    CmdStop,

    /** Invalid command. */
    CmdInvalid
  };

  /** Command sent to Place.SA. */
  class PlaceCommand
  {
    /** The command. */
    PlaceCommandType cmd;
  };

  /** Name of unknown place class. */
  const string UnknownClassName = "Area";

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
  // DataSaver
  // ---------------------------------------------------------------------
  /** Types of commands that can be sent to DataSaver. */
  enum DataSaverCommandType
  {
    /** Start sending updates. */
    DsCmdUpdateStart,

    /** Stop sending updates. */
    DsCmdUpdateStop,

    /** Start saving. */
    DsCmdStart,

    /** Stop saving. */
    DsCmdStop,

    /** Pause saving. */
    DsCmdPause,

    /** Unpause saving. */
    DsCmdUnpause,

    /** Sets new target. */
    DsCmdNewTarget,

    /** Invalid command. */
    DsCmdInvalid
  };

  /** Command sent to DataSaver. */
  class DataSaverCommand
  {
    /** The command. */
    DataSaverCommandType cmd;

    /** Directory to which the data will be saved. */
    string dataDirPath;

    /** Name of data config file and data subdirectory. */
    string dataBaseName;

    /** ID of the target class. */
    long targetNo;

    /** Name of the target class*/
    string targetName;
  };

  /** Ack that command was received by DataSaver. */
  class DataSaverCommandAck
  {
    /** The command. */
    DataSaverCommand cmd;

    /** Process that sent the command. */
    string src;
  };

  /** Sent by DataSaver when saving completed. */
  class DataSaverStatus
  {
    /** true if image was saved. */
    bool savedImage;

    /** true if scan was saved. */
    bool savedLaserScan;

    /** true if odometry was saved. */
    bool savedOdometry;

    /** No of frames saved including the last one. */
    long framesSaved;

    /** Frame no. of saved data*/
    long frameNo;

    /** True if error occured. */
    bool error;
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

  /** Types of commands that can be sent to VisualProcessor. */
  enum VisualProcessorCommandType
  {
    /** Start sending updates when input reading data finished. */
    VpCmdUpdateOnReadStart,

    /** Stop sending updates. */
    VpCmdUpdateStop,

    /** Invalid command. */
    VpCmdInvalid
  };

  /** Command sent to VisualProcessor. */
  class VisualProcessorCommand
  {
    /** The command. */
    VisualProcessorCommandType cmd;
  };

  /** Ack that command was received by VisualProcessor. */
  class VisualProcessorCommandAck
  {
    /** The command. */
    VisualProcessorCommand cmd;

    /** Process that sent the command. */
    string src;
  };

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

  /** Types of commands that can be sent to LaserProcessor. */
  enum LaserProcessorCommandType
  {
    /** Start sending updates when input reading data finished. */
    LpCmdUpdateOnReadStart,

    /** Stop sending updates. */
    LpCmdUpdateStop,

    /** Invalid command. */
    LpCmdInvalid
  };

  /** Command sent to LaserProcessor. */
  class LaserProcessorCommand
  {
    /** The command. */
    LaserProcessorCommandType cmd;
  };

  /** Ack that command was received by LaserProcessor. */
  class LaserProcessorCommandAck
  {
    /** The command. */
    LaserProcessorCommand cmd;

    /** Process that sent the command. */
    string src;
  };

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
  // CueIntegrator
  // ---------------------------------------------------------------------

  class IntegratedResults
  {
    /** Frame number. Used for synchronization. */
    long frameNo;

    /** Status of the data. */
    DataStatus status;

    /** True if results from VisualProcessor were integrated. */
    bool usedVision;

    /** True if results from LaserProcessor were integrated. */
    bool usedLaser;

    /** Real timestamp of the image. */
    cast::cdl::CASTTime imageRealTimeStamp;

    /** Time when the image was placed in the WM. */
    cast::cdl::CASTTime imageWmTimeStamp;

    /** Real timestamp of the scan. */
    cast::cdl::CASTTime scanRealTimeStamp;

    /** Time when the scan was placed in the WM. */
    cast::cdl::CASTTime scanWmTimeStamp;

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
  // NodeLabeller
  // ---------------------------------------------------------------------

  /** Types of pull querries which can be sent to NodeLabeller. */
  enum NodeLabellerQueryType
  {
    /** Read-only query, returns place class info about the node. */
    NlQtInfo,
    /** Informs NodeLabeller that new node was created. */
    NlQtNew,
    /** Informs NodeLabeller that the node was updated. */
    NlQtUpdate
  };

  /** Structure containing information about a place class assigned to a node. */
  struct NodePlaceInfo
  {
    // Node info
    long nodeId;
    // Results for the node
    long nodeClassNo;
    string nodeClassName;
    // Results for the area
    long areaClassNo;
    string areaClassName;
  };

  /** Stores information about a node kept by NodeLabeller. */
  struct NodeInfo
  {
    // Node info
    long nodeId;
    bool gateway;
    // Outputs
    ClassifierOutputs nodeAccumulatedOutputs;
    long nodeOutputCount;
    ClassifierOutputs areaAccumulatedOutputs;
    long areaOutputCount;
    // Results
    ClassifierResults nodeResults;
    ClassifierResults areaResults;
  };

  sequence<NodeInfo> NodeInfos;

  /** Reflects the current knowledge of the NodeLabeller
      about the NavGraph nodes. */
  class NodeLabellerData
  {
    NodeInfos nodes;
    NodeInfo currentNode;
    long lastNodeId;
  };

};


#endif // PLACEDATA_ICE
