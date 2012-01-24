//
// = LIBRARY
//
// = FILENAME
//    testDoorExtractor.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2011 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef DEPEND
#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <time.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#endif

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "Utils/CureDebug.hh"
#include "AddressBank/FileAddress.hh"
#include "AddressBank/DataSlotAddress.hh"
#include "AddressBank/ConfigFileReader.hh"
#include "HSSutils.hh"
#include "HSSDoorExtractor.hh"
#include "HSSutils.hh"
#include "HSSDisplayFunctions.hh"

int g_NextScanId = 0;
bool run = true;
void sigtrap(int signo) { run = false; }

void operator<<(Eigen::Vector3d &sink, const Cure::Pose3D &src)
{
  sink[0] = src.getX();
  sink[1] = src.getY();
  sink[2] = src.getTheta();
}

void operator<<(HSS::Scan2D &sink, const Cure::SICKScan &src)
{
  sink.m_Id = g_NextScanId++;

  sink.range.resize(src.getNPts());
  sink.valid.resize(src.getNPts());
  sink.theta.resize(src.getNPts());
  
  double da = src.getAngleStep();
  //double da = 1.0*M_PI/180;
  
  sink.min_theta = src.getStartAngle();
  sink.max_theta = src.getStartAngle() + (src.getNPts()-1)*da;

  sink.min_range = 0.1;
  sink.max_range = 5.55;

  sink.range_sigma = 0.05;

  for (int i = 0; i < src.getNPts(); i++) {
    sink.theta[i] = src.getStartAngle() + da * i;
    sink.valid[i] = 1;
    sink.range[i] = src.getRange(i);
  }
  sink.tv.tv_sec = src.getTime().Seconds;
  sink.tv.tv_usec = src.getTime().Microsec;
}

int main(int argc, char * argv[])
{
  cure_debug_level = 30;

  const char *optstring = "hc:eH:";
  const char *args = "[-h help] [-c config gile] [-e execMode] [-H peekabot host]";
  std::string configfile = ""; 
  std::string pbHost = "localhost";
  int execMode = 0;
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 'H':
      pbHost = optarg;
      break;
    case 'e':
      execMode = atoi(optarg);
      break;
    case 'c':
      configfile = optarg;
      break;
    case 'h':
    case '?':
      std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
      return -1;
    }
    o = getopt(argc, argv, optstring);
  }

  HSS::DoorExtractor doorExtractor;

  peekabot::PeekabotClient peekabotClient;
  peekabot::GroupProxy peekabotRoot;
    int pbPort = 5050;

  try {
    printf("Trying to connect to Peekabot on host %s and port %d\n",
           pbHost.c_str(), pbPort);

    peekabotClient.connect(pbHost, pbPort);

    peekabot::ObjectProxy tmp;
    peekabotRoot.add(peekabotClient, "HSS", peekabot::REPLACE_ON_CONFLICT);    

  } catch(std::exception &e) {
    fprintf(stderr, "Caught exception when connecting to peekabot (%s)\n",
            e.what());
    return -1;
  }

  if (configfile == "") {
    std::cerr << "You must specify a config file\n";
    std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
    return -1;
  }

  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) return -1;

  if (execMode < 0 || execMode > 1) {
    std::cerr << "Only supports execModes 0-(AFAP), 1-step, not "
              << execMode << "\n";
    execMode = 0;
    std::cerr << "Will use execMode=" << execMode << std::endl;
  }

  Cure::SensorPose laserPoseR;
  if (cfg.getSensorPose(1, laserPoseR)) {
    CureCERR(30) << "Failed to get sensor pose on robot\n";
    return -1;
  }
  Eigen::Vector3d xsR;
  xsR[0] = laserPoseR.getX();
  xsR[1] = laserPoseR.getY();
  xsR[2] = laserPoseR.getTheta();

  Cure::FileAddress odoFile, scanFile;
  Cure::DataSlotAddress poseBuf(100);
  std::string filename, filepath = ".";
  cfg.getFilePath(filepath);

  if (cfg.getSensorFile(0, filename)) return -1;
  std::cerr<<"opening odo "<<filepath<<"/"<<filename<<"\n";
  if (odoFile.openReadFile(filepath + "/" + filename)) return -1;
  if (cfg.getSensorFile(3, filename)) return -1;
  if (scanFile.openReadFile(filepath + "/" + filename)) return -1;
  
  signal(SIGINT, sigtrap);

  Cure::SICKScan cure_scan;
  Cure::Pose3D cure_odom;

  float color_current[3] = {0.5,.5,0.5};

  Eigen::Vector3d odomNew, odomOld;

  int n = 0;
  while (run && scanFile.read(cure_scan) == 0) {

    // Make sure that we read odometry at least up and until the scan
    while (cure_odom.getTime() < cure_scan.getTime()) {
      if (odoFile.read(cure_odom) != 0) {
        CureCERR(30) << "No odometry data\n";
        break;
      }
      poseBuf.write(cure_odom);
    }
    n++;

    // Interpolate to get the robot pose at the time of the laser    
    Cure::Pose3D cure_odomNew;
    poseBuf.read(cure_odomNew, cure_scan.getTime());
    odomNew << cure_odomNew;

    HSS::Scan2D scanNew;
    scanNew << cure_scan;
    scanNew.odom = odomNew;

    doorExtractor.extract(scanNew);

    // Display
    peekabotClient.begin_bundle();
      
    peekabot::GroupProxy current;      
    current.add(peekabotRoot, "current", peekabot::REPLACE_ON_CONFLICT);
      
    peekabot::PointCloudProxy scanPts;
    scanPts.add(current, "scan", peekabot::REPLACE_ON_CONFLICT);
    HSS::displayScan(scanPts, scanNew, 
                     odomNew, xsR, 
                     color_current[0],color_current[1],color_current[2]);

    // Display the detected door
    HSS::displayDoorMeas(peekabotRoot, odomNew, xsR, doorExtractor);
      
    peekabotClient.end_bundle();
    peekabotClient.sync();      

    std::cerr << ".";
    if (execMode == 1) {
      getchar();
    } else if (execMode == 2) {
      sleep(1);
    } else if (execMode == 3) {
      usleep(50000);
    }
  }

  std::cout << "Processed " << n << " scans\n";

  if (!run) {
    std::cout << "Aborted with ctrl-c @ scan time " 
              << cure_scan.getTime() << "\n";    
  }
}
