//
// = LIBRARY
//
// = FILENAME
//    testScanSLAM2.cc
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

#include "HSSutils.hh"
#include "HSSScanMotionDetector.hh"
#include "HSSAngleHistogram.hh"
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

/*
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
*/

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
  std::cerr << std::endl;
  std::cerr << "Press ENTER to start\n";
  getchar();

  const char *optstring = "hc:e:tn:TpaloDs:m:vS:";
  const char *args = "[-h help] [-c config gile] [-S sensor pose]"
    "[-e execMode] [-H peekabot host] [-t test color conversions] "
    "[-n process every nth scan (default 1=every)] [-T show traj] "
    "[-p pure dead recknoning] [-a display all scans] [-s max LMap size]"
    "[-o odom only predictions] [-d display labels] [-D no doors] "
    "[-m mapfile] [-v save virtual scans]";
  std::string configfile = ""; 
  std::string pbHost = "localhost";
  std::string mapfile = "";
  std::fstream *fsVirtual = 0;
  int execMode = 0;
  int proceverynthscan = 1;
  bool showTraj = false;
  bool pureDeadReckoning = false;
  int maxLMapSize = -1;
  bool noDoors = false;
  bool displayAllScans = false;
  bool displayLabels = false;
  bool odomOnlyPrediction = false;  
  Eigen::Vector3d xsR;

  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 'S':
      {
        std::istringstream str(optarg);
        str >> xsR[0] >> xsR[1] >> xsR[2];
      }
      break;
    case 'v':
      fsVirtual = new std::fstream;
      fsVirtual->open("virtual360-scans.txt", std::ios::out);
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

  std::cerr << "Sensor pose xsR=[" << xsR.transpose() << "]\n";
  getchar();

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
    tmp.assign(peekabotClient, "root");
    peekabotRoot.add(tmp, "HSS", peekabot::REPLACE_ON_CONFLICT);    

    peekabotTraj.add(peekabotRoot, "traj", peekabot::REPLACE_ON_CONFLICT);
    peekabotScans.add(peekabotRoot, "scans", peekabot::REPLACE_ON_CONFLICT);

    peekabotMotionDetection.add(peekabotRoot, "motiondetection",
                                peekabot::REPLACE_ON_CONFLICT);

  } catch(std::exception &e) {
    fprintf(stderr, "Caught exception when connecting to peekabot (%s)\n",
            e.what());
    return -1;
  }

  if (execMode < 0 || execMode > 1) {
    std::cerr << "Only supports execModes 0-(AFAP), 1-step, not "
              << execMode << "\n";
    execMode = 0;
    std::cerr << "Will use execMode=" << execMode << std::endl;
  }

  signal(SIGINT, sigtrap);

  std::fstream scanFile;
  std::fstream odomFile;

  scanFile.open("data/data_cogx/scans.tdf", std::ios::in);
  odomFile.open("data/data_cogx/odom.tdf", std::ios::in);

  float color_current[3] = {0.5,.5,0.5};

  Eigen::Vector3d odom, odomNew, odomOld;
  HSS::CandScan2D scanNew, scanOld;

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

  struct timeval odomTime;

  int n = 0;
  while (run && HSS::readCureFileScan(scanFile, scanNew)) {

    std::cerr << "Read scan with timestamp "
              << HSS::toString(scanNew.tv) << std::endl;
    

    // Make sure that we read odometry at least up and until the scan
    while (odomTime < scanNew.tv) {
      if (!HSS::readCureFileOdom(odomFile, odom, odomTime)) {
        std::cerr << "Could not ready odometry\n";
        break;
      }
      std::cerr << "Read odometry with timestamp "
                << HSS::toString(odomTime) << std::endl;
      //poseBuf.add(odom, odomTime);
    }
    n++;

    // Interpolate to get the robot pose at the time of the laser    
    /*
    if (!poseBuf.getOdomAtTime(scan.tv, odomNew)) {
      std::cerr << "ERROR: Could not get odometry at desired time\n";
      getchar();
    }
    */
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

    if (n == 1) {
      if (mapfile != "") {
        Eigen::Vector3d xr(HSS::icompound(xsR));
        hssmap.init(mapfile, xsR, &xr);
      } else {
        hssmap.init(xsR, scanNew);        
      }
    }

    ////////////////////////////////
    // Check if the scan is corridor like
    HSS::AngleHistogram ah;
    ah.setMinPeakFrac(0.7);
    
    scanNew.m_Corridor = ah.isCorridorLike(scanNew, &scanNew.m_MainDirS);
    
    // Adjust the corridor direction to be in the robot frame
    // instead of sensor frame
    scanNew.m_MainDirR = HSS::pi_to_pi(scanNew.m_MainDirS + xsR[2]);
    
    // We do not want the direction of the corridor to result in any
    // changes of direction of motion so we make sure that it is
    // +-pi/2
    if (scanNew.m_MainDirS < -M_PI_2) scanNew.m_MainDirS += M_PI;
    else if (scanNew.m_MainDirS > M_PI_2) scanNew.m_MainDirS -= M_PI;

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

      {
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
              << HSS::toString(scanNew.tv) << "\n";    
  }

  if (fsVirtual) fsVirtual->close();
  fsVirtual = 0;
}
