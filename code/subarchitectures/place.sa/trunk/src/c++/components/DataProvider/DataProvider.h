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

/**
 * The PlaceDataProvider class.
 * \file DataProvider.h
 * \author Andrzej Pronobis
 * \date 2008-07-16
 */

#ifndef __PLACE_DATA_PROVIDER__
#define __PLACE_DATA_PROVIDER__

#include "shared/DataReader.h"

#include <Scan2dReceiver.hpp>
#include <Laser.hpp>
#include <OdometryReceiver.hpp>
#include <Robotbase.hpp>
#include <VideoClient.h>
#include <PlaceData.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <cast/core/CASTTimer.hpp>
#include <cast/core/CASTComponent.hpp>
#include <Ice/Ice.h>

#include <list>
#include <pthread.h>


/**
 * Implements the DataProvider component.
 * Collects and synchronizes multi-sensory data when triggered.
 */
class PlaceDataProvider: public cast::ManagedComponent,
                         public Scan2dReceiver,
                         public OdometryReceiver,
                         public cast::VideoClient
{

  // Servers
  class LaserServer: public Laser::LaserServer,
                     public cast::CASTComponent
  {
  public:
    void pushScanToClients(PlaceData::LaserScanPtr scan);

    Laser::Scan2d pullScan2d(const Ice::Current&);
    void registerScan2dPushClient(const Laser::Scan2dPushClientPrx& clientPrx,
                                  Ice::Double desiredInterval, const Ice::Current&);

  private:

    class Scan2dClient
    {
    public:
      Laser::Scan2dPushClientPrx prx;
      double interval;
      cast::CASTTimer timer;
    };
    std::vector<Scan2dClient> _scanPushClients;

  };

  class RobotbaseServer: public Robotbase::RobotbaseServer,
                         public cast::CASTComponent
  {
  public:
    void pushOdometryToClients(PlaceData::OdometryPtr odom);

    Robotbase::Odometry pullOdometry(const Ice::Current&);
    void execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                           const Ice::Current&);
    void registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& clientPrx,
                                    Ice::Double desiredInterval, const Ice::Current&);

  private:

    class OdometryClient
    {
    public:
      Robotbase::OdometryPushClientPrx prx;
      double interval;
      cast::CASTTimer timer;
    };
    std::vector<OdometryClient> _odomPushClients;

  };


public: // Component management

  PlaceDataProvider();
  virtual ~PlaceDataProvider();

  virtual void configure(const std::map<std::string,std::string> &_config);
  virtual void runComponent();
  virtual void start();
  virtual void stop();


public: // Data receiving/pulling

  void receiveScan2d(const Laser::Scan2d &scan);
  void receiveOdometry(const Robotbase::Odometry &odom);


private: // Data processing

  /** Finds closest (in time) or most recent (depending on refT) scan in the queue. */
  void findLaserScan(const cast::cdl::CASTTime& refT, PlaceData::LaserScanPtr scan);

  /** Finds closest (in time) or most recent (depending on refT) odometry reading in the queue. */
  void findOdometryReading(const cast::cdl::CASTTime& refT, PlaceData::OdometryPtr odom);

  /** Pulls an image. */
  void pullImage(PlaceData::ImagePtr image);

  /** Invoked when new command added to WM. */
  void newDataProviderCommandAdded(const cast::cdl::WorkingMemoryChange & wmc);

  /** Outputs new laser scan to WM. */
  cast::cdl::CASTTime outputLaserScan(const cast::cdl::CASTTime& refT);

  /** Outputs new odometry reading to WM. */
  cast::cdl::CASTTime outputOdometryReading(const cast::cdl::CASTTime& refT);

  /** Outputs new image to WM. Returns timestamp of the image.*/
  cast::cdl::CASTTime outputImage();

  /** Outputs new target information to WM.*/
  void outputTarget();

  /** Adds empty image to WM. Returns data ID. */
  std::string addEmptyImage();

  /** Adds empty odometry to WM. Returns data ID. */
  std::string addEmptyOdometry();

  /** Adds empty scan to WM. Returns data ID. */
  std::string addEmptyLaserScan();

  /** Adds empty scan to WM. Returns data ID. */
  std::string addEmptyTarget();

  /** Adds empty scan to WM. Returns data ID. */
  std::string addEmptyDataProviderCommandAck();

  /** Converts image to proper resolution and gray scale.
    * Always makes a copy of the data. */
  void convertImageBGR24(const Video::Image &src, PlaceData::ImageData &dest) const;

  /** Converts image to proper resolution and gray scale.
   * Always makes a copy of the data. */
  void convertImageRGB24(const Video::Image &src, PlaceData::ImageData &dest) const;



private: // Configuration

  /** Name of the config file group. */
  const std::string _cfgGroup;

  bool _useLaser;
  bool _useVision;
  bool _useOdometry;

  /**
   * The timeWindow specifies the length of the queue in seconds.
   * It indicates the biggest time difference allowed between the 
   * oldest and newest element in the queues.
   */
  double _queueTimeWindow;

  /** Max no of items in the queue. */
  int _queueSize;

  /** Id of the camera to be used. */
  int _cameraId;

  /** Name of the videoserver. */
  std::string _videoServerName;

  int _scanDelay;


private: // Queues

  pthread_mutex_t _scanQueueMutex;
  std::list<Laser::Scan2d> _scanQueue;

  pthread_mutex_t _odometryQueueMutex;
  std::list<Robotbase::Odometry> _odometryQueue;


private: // Thread synchronization

  bool _wasSignal;
  pthread_cond_t _signalCond;
  pthread_mutex_t _signalMutex;


private: // Servers

  RobotbaseServer _robotbaseServer;
  LaserServer _laserServer;


private: // Data sources

  /** Which video server to use. */
  enum VideoServerType
  {
    /** Standard VideoServer. */
    VS_STD
  };
  VideoServerType _videoServerType;

  /** If true, data will be loaded from disk. */
  bool _loadDataFromDisk;

  /** Performes the actual data reading (from disk).*/
  place::DataReader _dataReader;

  /** ICE proxy to the video server. */
  Video::VideoInterfacePrx _videoServer;


private: // WM object IDs

  std::string _imageId;
  std::string _laserScanId;
  std::string _odometryId;
  std::string _targetId;
  std::string _dataProviderCommandAckId;


private:

  /** Frame no. of the data tha will be added to WM now. */
  long _frameNo;

  /** Timestamp when last image was grabbed .*/
  cast::cdl::CASTTime _lastGrabTimestamp;
};

#endif
