//
// = LIBRARY
//
// = FILENAME
//    testScanSLAM.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
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
#include "HSSScanMotionDetector.hh"
#include "HSSDoorExtractor.hh"
#include "HSSutils.hh"
#include "HSSCandScan2D.hh"
#include "HSSScan2DFilter.hh"
#include "HSSScan2DMatcher.hh"
#include "HSSMotionEstimator.hh"
#include "HSSMap.hh"
#include "HSSDisplayFunctions.hh"
#include "HSSVirtualScan2DFactory.hh"
#include "HSSGridMap2D.hh"
#include "HSSGridMapFunctor.hh"
#include "HSSGridLineRayTracer.hh"

long g_NextScanId = 0;
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

void testRGB2HSV()
{
  float r,g,b,h,s,v;
  HSS::rgb2hsv(.1,.1,.1,&h,&s,&v);
  std::cerr << "h=" << h << " s=" << s << " v=" << v << std::endl;
  HSS::hsv2rgb(h,s,v,&r,&g,&b);
  std::cerr << "r=" << r << " g=" << g << " b=" << b << std::endl;
}

int main(int argc, char * argv[])
{
  std::cerr << "TODO:\n===============================\n";
  std::cerr << "* All scans should have corridor/corrDir so that we can use the special corridor R matrix if ONE of the two scans involves is a corridor.\n";
  std::cerr << "* Save multi map case, need to store transitions\n";
  std::cerr << "* Estimate motion at the correct point in time. Now we use the position from the previous frame. Replace scan in map after motion has been removed?\n";
  std::cerr << "* Store and use info about where the scan is so that one can calculate an overlap to see if the scan should be matched to or not\n";
  std::cerr << std::endl;
  //std::cerr << "Press ENTER to start\n";
  //getchar();

  cure_debug_level = 30;

  const char *optstring = "hc:d:e:tn:TpaloDs:m:v:";
  const char *args = "[-h help] [-c config file] [-d debug level] "
    "[-e execMode] [-H peekabot host] [-t test color conversions] "
    "[-n process every nth scan (default 1=every)] [-T show traj] "
    "[-p pure dead recknoning] [-a display all scans] [-s max LMap size]"
    "[-o odom only predictions] [-d display labels] [-D no doors] "
    "[-m mapfile] [-v save virtual scans]";
  std::string configfile = ""; 
  std::string pbHost = "localhost";
  std::string mapfile = "";
  std::fstream *fsVirtual = 0;
  std::string virtualScanFilename = "virtual360-scans.txt";
  int execMode = 0;
  int proceverynthscan = 1;
  bool showTraj = false;
  bool pureDeadReckoning = false;
  int maxLMapSize = -1;
  bool noDoors = false;
  bool displayAllScans = false;
  bool displayLabels = false;
  bool odomOnlyPrediction = false;  
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 'v':
      fsVirtual = new std::fstream;
      if (std::string(optarg) == "") {
        fsVirtual->open("virtual360-scans.txt", std::ios::out);
      } else {
        fsVirtual->open(optarg, std::ios::out);
      }
      break;
    case 'D':
      noDoors = true;
      break;
    case 'l':
      displayLabels = true;
      break;
    case 'o':
      odomOnlyPrediction = true;
      break;
    case 'a':
      displayAllScans = true;
      break;
    case 'p':
      pureDeadReckoning = true;
      break;
    case 'T':
      showTraj = true;
      break;
    case 'H':
      pbHost = optarg;
      break;
    case 'm':
      mapfile = optarg;
      break;
    case 'e':
      execMode = atoi(optarg);
      break;
    case 's':
      maxLMapSize = atoi(optarg);
      break;
    case 'n':
      proceverynthscan = atoi(optarg);
      break;
    case 'd':
      cure_debug_level = atoi(optarg);
      break;
    case 'c':
      configfile = optarg;
      break;
    case 't':
      testRGB2HSV();
      return 0;
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
  peekabot::GroupProxy peekabotTraj;
  peekabot::GroupProxy peekabotScans;
  peekabot::GroupProxy peekabotMotionDetection;
  
  int pbPort = 5050;

  try {
    printf("Trying to connect to Peekabot on host %s and port %d\n",
           pbHost.c_str(), pbPort);

    peekabotClient.connect(pbHost, pbPort);

    peekabot::ObjectProxy tmp;
    peekabotRoot.add(peekabotClient, "HSS", peekabot::REPLACE_ON_CONFLICT);    

    peekabotTraj.add(peekabotRoot, "traj", peekabot::REPLACE_ON_CONFLICT);
    peekabotScans.add(peekabotRoot, "scans", peekabot::REPLACE_ON_CONFLICT);

    peekabotMotionDetection.add(peekabotRoot, "motiondetection",
                                peekabot::REPLACE_ON_CONFLICT);

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

  HSS::CandScan2D scanOld;

  HSS::Scan2DFilter scanFilter;
  HSS::MotionEstimator motionEstimator(odomOnlyPrediction);
  HSS::ScanMotionDetector motiondetector;
  HSS::Map hssmap;
  hssmap.setMaxLMapSize(maxLMapSize);

  HSS::CharGridMapFunctor gridfunc;
  HSS::GridMap2D<char> grid(gridfunc, 120, 0.05);
  HSS::GridLineRayTracer<char> raytracer(grid, gridfunc);

  Eigen::Vector3d drX;
  drX.setZero();
  Eigen::Matrix3d drP;
  drP.setIdentity();
  drP *= 1e-12;

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

    HSS::CandScan2D scanNew;
    scanNew << cure_scan;
    scanNew.odom = odomNew;

    if (n == 1) {
      odomOld = odomNew;
      scanOld = scanNew;
    }

    if ((n-1) % proceverynthscan != 0) continue;

    scanFilter.filterScanMotion(scanNew, motiondetector, hssmap.getXrG(),xsR,2);
    scanFilter.filterScanRange(scanNew);
    //scanFilter.filterScanMean(scanNew, 1, 0.1);
    scanFilter.filterScanSmall(scanNew, 4, 0.2);

    HSS::makeScanCandidate(scanNew, xsR);

    if (n == 1) {
      if (mapfile != "") {
        Eigen::Vector3d xr(HSS::icompound(xsR));
        hssmap.init(mapfile, xsR, &xr);
      } else {
        hssmap.init(xsR, scanNew);        
      }
    }

    Eigen::Vector3d delta;
    delta.setZero();
    Eigen::Matrix3d Q;
    Q.setIdentity();
    Q *= 1e-12;
    bool moved = true;
    if (motionEstimator.estimateMotion(odomNew, odomOld, scanNew, scanOld, 
                                       xsR, delta, Q)) {

      std::cerr << "Estimated motion dx=" << delta[0] << " y=" << delta[1]
                << " da=" << delta[2] << std::endl;

      // We only swap reference scan/odom for the motion estimate if
      // we actually used it. Otherwise there is a risk that we will
      // underestimate the motion if we are moving very slowly.
      scanOld = scanNew;
      odomOld = odomNew;

      hssmap.predict(delta, Q);

    } else {
      moved = false;
      //std::cerr << "No motion estimated\n";
    }

    if (n == 1 || moved) {

      { // The pure dead reckoning estimate
        Eigen::MatrixXd J;
        drX = HSS::compound(drX, delta, &J);
        Eigen::Matrix3d Jr(J.block(0,0,3,3));
        Eigen::Matrix3d Ju(J.block(0,3,3,3));
        drP = Jr*drP*Jr.transpose() + Ju*Q*Ju.transpose();
      }

      if (!pureDeadReckoning) {
        hssmap.updateScan(xsR, scanNew);
      }

      if (!noDoors) {
        doorExtractor.extract(scanNew);
        if (!pureDeadReckoning) {
          hssmap.updateDoor(xsR, doorExtractor);
        }
      }

      hssmap.checkForTransitions(xsR, scanNew);

      // Display
      peekabotClient.begin_bundle();
      
      peekabot::GroupProxy current;      
      current.add(peekabotRoot, "current", peekabot::REPLACE_ON_CONFLICT);
      
      peekabot::PointCloudProxy scanPts;
      scanPts.add(current, "scan", peekabot::REPLACE_ON_CONFLICT);
      HSS::displayScan(scanPts, scanNew, 
                       hssmap.getXrG(), xsR, 
                       color_current[0],color_current[1],color_current[2]);
      //HSS::displayPolygon(scanPts, HSS::compound(hssmap.getXrG(),xsR),scanNew.m_BB);

      // Display the current scanner pose
      Eigen::Vector3d xsW(HSS::compound(hssmap.getXrG(), 
                                        xsR));
      HSS::displayStateWithUnc(current, xsW, hssmap.getPrrG(), 0, NULL, 1.0, 
                               color_current[0],
                               color_current[1],
                               color_current[2]);
      
      if (displayAllScans) {
        peekabot::PointCloudProxy scanPtsAll;
        scanPtsAll.add(peekabotScans, "scan");
        HSS::displayScan(scanPtsAll, scanNew, hssmap.getXrG(), xsR, 0,0,0);
      }

      peekabot::GroupProxy lmaproot;
      lmaproot.add(peekabotRoot, "lmaps", peekabot::REPLACE_ON_CONFLICT);
      hssmap.displayMap(lmaproot, displayLabels);

      if (0) {
        peekabot::GroupProxy lmdebugroot;
        lmdebugroot.add(peekabotRoot, "lmdebug", peekabot::REPLACE_ON_CONFLICT);
        hssmap.displayLandmarksInSameFrame(lmdebugroot);
      }

      if (showTraj) {
        peekabot::CubeProxy cube;
        cube.add(peekabotTraj, "traj");
        cube.set_scale(0.1, 0.05, 0.01);
        cube.set_pose(hssmap.getXrG()[0], hssmap.getXrG()[1], 0, 
                      hssmap.getXrG()[2], 0, 0);
      }

      if (fsVirtual) {
        double t0 = HSS::getCurrentTime();
        Eigen::Vector3d xsW(HSS::compound(hssmap.getXrG(), xsR));
        grid.moveCenterTo(hssmap.getXrG());      
        raytracer.addScan2D(scanNew, xsW, scanNew.max_range);
        HSS::displayGrid<char>(peekabotRoot, grid, gridfunc);
        double t1 = HSS::getCurrentTime();
        std::cerr << "Grid stuff took " << t1-t0 << "s\n";

        HSS::Scan2D scan360;
        HSS::VirtualScan2DFactory::getVirtualScan2D(grid, gridfunc, xsW,
                                                    scan360,
                                                    240.0/680,
                                                    -180,
                                                    360,
                                                    5.55);
        if (fsVirtual) {
          char buf[256];
          sprintf(buf, "%ld %ld ", scanNew.tv.tv_sec, scanNew.tv.tv_usec);
          *fsVirtual << buf << xsW[0] << " " << xsW[1] << " " << xsW[2] << " ";
          for (unsigned int i = 0; i < scan360.range.size(); i++) {
            *fsVirtual << scan360.range[i] << " " << scan360.theta[i] << " ";
          }
          *fsVirtual << std::endl;
        }
      }
    
      motiondetector.displayMotion(peekabotMotionDetection);
      //motiondetector.displayFreeSpace(peekabotMotionDetection, true, true);

      // Display the detected door
      HSS::displayDoorMeas(peekabotRoot, hssmap.getXrG(), xsR, doorExtractor);
      
      peekabotClient.end_bundle();
      peekabotClient.sync();      
    }

    //std::cerr << "PrrG=[" << hssmap.getPrrG() << "]\n";

    hssmap.save("mapfile");

    std::cerr << ".";
    if (moved && execMode == 1) {
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

  if (fsVirtual) fsVirtual->close();
  fsVirtual = 0;
}
