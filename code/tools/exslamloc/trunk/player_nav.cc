// = FILENAME
//    player_nav.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <Utils/CureDebug.hh>
#include <Utils/KeyReader.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Map/WrappedSLAM.hh>
#include <Geometry/Line2D.hh>
#include <AddressBank/FileAddress.hh>
#include <AddressBank/RLDisplayHelper.hh>
#include <Navigation/NavGraph.hh>
#include <Navigation/NavGraphNode.hh>
#include <Navigation/NavGraphGateway.hh>
#include <Navigation/BielefeldFollowAlgorithm.hh>
#include <Navigation/InDoorOpeningDetector.hh>
#include <Map/PoseProvider.hh>
#include <Sensory/RANSACSegmentor.hh>
#include <Sensory/LsqLineFitter.hh>
#include <Map/WrappedLocalization.hh>
#include <Navigation/LocalMap.hh>
#include <Utils/HelpFunctions.hh>
#include <Navigation/NavController.hh>
#include "AddressBank/SocketAddress.hh"
#include "AddressBank/AddressFcnHook.hh"

#include "PlayerDataToCure.hh"

#include <stdlib.h>        
#include <signal.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <cstdlib>   // atoi

enum DriveModes {
  DRIVEMODE_KEY=2,
  DRIVEMODE_FOLLOW,
  DRIVEMODE_AUTO
};

Cure::Hal::SocketAddress g_SockScan("scansocket");
Cure::Hal::SocketAddress g_SockOdom("odomsocket");
Cure::Hal::SocketAddress g_SockCmd("cmdsocket");

double g_velTrans = 0;
double g_velSteer = 0;
bool heartbeat = true;
std::string tag = "";  // When non-empty a teh current pose will be
                       // marked in the navigation graph
int g_drivemode = DRIVEMODE_KEY;
bool g_RunningSLAM = false;
Cure::InDoorOpeningDetector doorDetector(0.6, 1.0);
int g_IterToInitialize = 10;
bool inDoor = false;
BielefeldFollowAlgorithm follow;
Cure::SensorPose g_LaserPoseR;
Cure::RANSACSegmentor g_Seg;
Cure::LsqLineFitter g_Lsq;
peekabot::PeekabotClient g_PeekabotClient;
peekabot::GroupProxy g_PeekabotRobot;
peekabot::GroupProxy g_PeekabotLaser;
Cure::NavGraph g_graph;
Cure::LocalMap g_lmap(5.5);
Cure::PoseProvider *g_poseprov = 0;
Cure::DataSlotAddress g_posbuf(100, POSE3D_TYPE, 0x100B, false);
std::list<int> g_waypoints;
Cure::FileAddress g_scanfile, g_odomfile;
bool g_saveData = false;
Cure::SICKScan g_scan; // last received scan

//===========================================================================

bool run = true;
void sigtrap(int signo) { 
  fprintf(stderr, "Pressed ctrl-c\n");
  run = false; 
}

//===========================================================================

class NavCtrl : public Cure::NavController,
                public Cure::NavControllerEventListener {
public:
  NavCtrl(Cure::PoseProvider &pp, Cure::NavGraph &graph, Cure::LocalMap &lmap)
    :Cure::NavController(graph, lmap),
     Cure::NavControllerEventListener("NavCtrl")
  {
    addEventListener(this);
    setTurnAngleIntoSpeed(true, 0.5);
    setMinNonzeroSpeeds(0.03, Cure::HelpFunctions::deg2rad(10));
    setGotoMaxSpeeds(0.4, 0.4);
    setTurnMaxSpeeds(0.2, 0.3);
    setGatewayMaxSpeeds(0.3, 0.3);
    setUsePathTrimming(false);
    setMaxPathTrimDist(0.5);

    setPoseProvider(pp);
  }

  void abortTask(int taskId) {
    CureCERR(30) << "Aborted task\n";
    g_velTrans = 0;
    g_velSteer = 0;
    g_drivemode = DRIVEMODE_KEY;
  }

  void failTask(int taskId, int error) {
    CureCERR(30) << "Failed in task with error " << error << "\n";
    g_velTrans = 0;
    g_velSteer = 0;
    g_drivemode = DRIVEMODE_KEY;
  }

  void doneTask(int taskId) {
    CureCERR(30) << "Task done\n";
    if (!g_waypoints.empty()) {
      g_waypoints.pop_front();
      if (g_waypoints.empty()) {
        g_velTrans = 0;
        g_velSteer = 0;
        g_drivemode = DRIVEMODE_KEY;
      } else {
        gotoNode(0, g_waypoints.front());
        std::cerr << "Heading for "
                  << g_waypoints.front() << " with "
                  << g_waypoints.size() << " waypoints left\n";
        g_drivemode = DRIVEMODE_AUTO;
      }
    }  else {
      std::cerr << "Should not get here with an emoty waypoint list\n";
    }
  }

  void execCtrl(Cure::MotionAlgorithm::MotionCmd &cmd)
  {
    if (g_drivemode != DRIVEMODE_AUTO) return;

    if (cmd.type == Cure::MotionAlgorithm::CMD_TYPE_STOP) {
      CureCERR(50) << "Executing command of type STOP\n";
      g_velTrans = 0;
      g_velSteer = 0;
    } else if (cmd.type == Cure::MotionAlgorithm::CMD_TYPE_VW) {
      CureCERR(50) << "Executing command of type VW, v="
                   << cmd.v << " w=" << cmd.w << std::endl;
      g_velTrans = cmd.v;
      g_velSteer = cmd.w;
    } else {
      CureCERR(20) << "Not implemented support for motion command type "
                   << cmd.type << std::endl;
    }
  }
};

NavCtrl *g_navCtrl = 0;


//===========================================================================

void printKeySetup()
{
  std::cerr << "\n"
            << "Move the robot around using the keyboard or\n"
            << "press 'f' to make the robot follow you around "
            << "(you need to be the closest object)\n"
            << "Press SPACE to stop\n"
            << "+/u/U : increase translation speed\n"
            << "-/n/N : decrease translation speed\n"
            << "h/H   : increase rotation speed\n"
            << "k/K   : decrease rotation speed\n"
            << "j/J   : zero rotation speed\n"
            << "\n"
            << "x/X   : mark a position and give it a tag\n"
            << "?     : print this key mapping info\n";
}

void getColorByIndex(int id, float &r, float &g, float &b)
{
  switch (id) {
    case 1:    
    r = 1.0/0xFF*0x00;
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0x00;
    break;
  case 2:
    r = 1.0/0xFF*0xFF; 
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0x00;
    break;
  case 3:
    r = 1.0/0xFF*0x00;
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0xFF;
    break;
  case 4:
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0x00;
    break;
  case 5:
    r = 1.0/0xFF*0x00; 
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0xFF;
    break;
  case 6:
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0xFF;
    break;
  case 7:
    r = 1.0/0xFF*0x00; 
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0x00;
    break;
  case 8:
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0x24;
    b = 1.0/0xFF*0x00;
    break;
  case 9:
    r = 1.0/0xFF*0x6F;
    g = 1.0/0xFF*0x42;
    b = 1.0/0xFF*0x42;
    break;
  case 10:
    r = 1.0/0xFF*0x8C;
    g = 1.0/0xFF*0x17;
    b = 1.0/0xFF*0x17;
    break;
  case 11:
    r = 1.0/0xFF*0x5C; 
    g = 1.0/0xFF*0x33;
    b = 1.0/0xFF*0x17;
    break;
  case 12:
    r = 1.0/0xFF*0x2F; 
    g = 1.0/0xFF*0x4F;
    b = 1.0/0xFF*0x2F;
    break;
  default:
    fprintf(stderr, 
            "Only handles color with indices 1-12, not %d, using red", id);
    r = 1.0;
    g = 0;
    b = 0;
  }
}

void addDoorpost(double x, double y, double theta, 
                 double width, 
                 peekabot::SphereProxy &node)
{                                 
  peekabot::CubeProxy cpL;
  cpL.add(node, "doorpostleft");
  cpL.set_scale(0.1, 0.1, 2.0);
  cpL.set_pose(0.5*width*cos(theta), 0.5*width*sin(theta), 1.0, 
               theta, 0, 0);
  cpL.set_color(1.0, 0.817, 0.269);

  peekabot::CubeProxy cpR;
  cpR.add(node, "doorpostright");
  cpR.set_scale(0.1, 0.1, 2.0);
  cpR.set_pose(0.5*width*cos(theta+M_PI), 0.5*width*sin(theta+M_PI), 1.0,
               theta, 0, 0);
  cpR.set_color(1.0, 0.817, 0.269);
  
  peekabot::CubeProxy cpT;
  cpT.add(node, "doorposttop");
  cpT.set_scale(width+0.1, 0.1, 0.1);
  cpT.set_pose(0, 0, 2.0, theta, 0, 0);
  cpT.set_color(1.0, 0.817, 0.269);
}

void displayPeekabot(Cure::SICKScan &scan, Cure::PoseProvider *pp,
                     Cure::LocalMap &lmap)
{
  g_PeekabotClient.begin_bundle();

  peekabot::PointCloudProxy scanPts;
  peekabot::VertexSet vs1;
  scanPts.add(g_PeekabotLaser, "scan", peekabot::REPLACE_ON_CONFLICT); 
  for (unsigned int i = 0; i < scan.getNPts(); i++) {
    vs1.add_vertex(scan.getRange(i)*cos(scan.getStartAngle() + scan.getAngleStep()*i),
                   scan.getRange(i)*sin(scan.getStartAngle() + scan.getAngleStep()*i),
                   0);      
  }
  scanPts.set_vertices(vs1);
  scanPts.set_color(0.75,0,0);

  peekabot::PointCloudProxy mapPts;
  peekabot::VertexSet vs2;
  mapPts.add(g_PeekabotClient, "root.lmap", peekabot::REPLACE_ON_CONFLICT); 
  for (unsigned int i = 0; i < lmap.nObst(); i++) {
    vs2.add_vertex(lmap.obstRef(i).x, 
                  lmap.obstRef(i).y, 
                  0.2);
  }
  mapPts.set_vertices(vs2);
  mapPts.set_color(0,0.75,0);
 
  Cure::Pose3D cp = pp->getPose();
  g_PeekabotRobot.set_pose(cp.getX(), cp.getY(), 0, cp.getTheta(), 0, 0);

  peekabot::GroupProxy gp;
  gp.add(g_PeekabotClient, "root.graph", peekabot::REPLACE_ON_CONFLICT);
  int i = 0;
  for (std::list<Cure::NavGraphNode*>::iterator iter = g_graph.m_Nodes.begin();
       iter != g_graph.m_Nodes.end(); iter++) {

    peekabot::SphereProxy sp;
    char name[32];
    sprintf(name, "node%d", (*iter)->getId());
    sp.add(gp, name);
    sp.set_position((*iter)->getX(), (*iter)->getY(), 0);
    
    float r,g,b;
    if ((*iter)->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) {
      sp.set_scale(0.2, 0.2, 0.05);
      getColorByIndex(2, r, g, b);
      
      Cure::NavGraphGateway *door = (Cure::NavGraphGateway*)(*iter);      
      addDoorpost(door->getX(), 
                  door->getY(), 
                  door->getTheta(), 
                  door->getWidth(), sp);
      
    } else {
      getColorByIndex(3+((*iter)->getAreaId()%6), r, g, b);
      sp.set_scale(0.1, 0.1, 0.05);
    }
    sp.set_color(r,g,b);
  }

  for (std::list<Cure::NavGraphEdge>::iterator ei = g_graph.m_Edges.begin();
       ei != g_graph.m_Edges.end(); ei++) {

    Cure::NavGraphNode *n1 = g_graph.getNode(ei->getNodeId1());
    Cure::NavGraphNode *n2 = g_graph.getNode(ei->getNodeId2());

    if (n1 && n2) {
      peekabot::LineCloudProxy lp;
      char name[32];
      if (n1->getId() > n2->getId()) {
        sprintf(name, "edge%06d", n1->getId()*1000+n2->getId());
      } else {
        sprintf(name, "edge%06d", n2->getId()*1000+n1->getId());
      }
      
      lp.add(gp, name, peekabot::REPLACE_ON_CONFLICT);
      lp.add_line(n1->getX(), n1->getY(), 0,
                  n2->getX(), n2->getY(), 0);
      lp.set_color(0., 0., 0.);
      lp.set_opacity(1);
    }
  }

  peekabot::GroupProxy wp;
  wp.add(g_PeekabotClient, "root.walls", peekabot::REPLACE_ON_CONFLICT);
  Cure::FeatureMap *fm = 0;
  if (g_RunningSLAM) {
    fm = &(((Cure::WrappedSLAM*)pp)->m_Map);
  } else {
    fm = &(((Cure::WrappedLocalization*)pp)->m_Map);
  }
  std::list<Cure::Line2D> walls;
  fm->getLine2DWalls(walls);
  if (walls.size() > 0) {
    i = 0;
    for (std::list<Cure::Line2D>::iterator w = walls.begin();
         w != walls.end(); w++) {
      peekabot::PolygonProxy pp;
      char buf[32];
      sprintf(buf, "poly%d", i);
      pp.add(wp, buf);    
      pp.add_vertex(w->StartPoint.getX(), w->StartPoint.getY(), 0);
      pp.add_vertex(w->StartPoint.getX(), w->StartPoint.getY(), 1.0);
      pp.add_vertex(w->EndPoint.getX(), w->EndPoint.getY(), 1.0);
      pp.add_vertex(w->EndPoint.getX(), w->EndPoint.getY(), 0);
      pp.set_color(255./255, 198./255, 0.);
      pp.set_opacity(0.2);
    }
  }

  g_PeekabotClient.end_bundle().status();
  g_PeekabotClient.sync();
}

void* control_thread(void *ptr)
{
  Cure::KeyReader kr;

  while (run) {

    char key;
    while ((key = kr.keyPressed())) {
      //std::cerr << "\nKEY WAS HIT!!!\n\n";
      //fprintf(stderr, "Got key '%c'\n", key);
      switch (key) {
      case '?':
        printKeySetup();
        break;
      case 'q':
      case 'Q':
        run = false;
        break;
      case 'a':
      case 'A':
        if (g_drivemode != DRIVEMODE_AUTO) {
          std::cerr << "Now in AUTO mode\n";
          g_drivemode = DRIVEMODE_AUTO;
        } else {
          g_drivemode = DRIVEMODE_KEY;
          std::cerr << "Now in MANUAL mode\n";
        }
        g_velTrans = 0;
        g_velSteer = 0;
        break;
      case 's':
      case 'S':
        {
          g_waypoints.clear();
          std::fstream fs;
          fs.open("path.txt", std::ios::in);
          int id;
          while (!fs.eof() && fs >> id) {
            g_waypoints.push_back(id);
          }
          if (g_waypoints.empty()) {
            std::cerr << "Failed to read file \"path.txt\", going to node 0\n";
            g_waypoints.push_back(0);
          }
          g_navCtrl->gotoNode(0, g_waypoints.front());
          std::cerr << "Now in AUTO mode, heading for "
                    << g_waypoints.front() << " with "
                    << g_waypoints.size() << " waypoints left\n";
          g_drivemode = DRIVEMODE_AUTO;
        }
        break;

      case 'x':
      case 'X':        
        {
          heartbeat = false;
          g_velTrans = 0;
          g_velSteer = 0;
          std::cerr << "\nMarking current pose\n";
          std::cerr << "Enter tag for current pose:";
          std::cin >> tag;  
          heartbeat = true;
        }
        break;
      case 'f':
      case 'F':
        if (g_drivemode != DRIVEMODE_FOLLOW) {
          std::cerr << "Now following\n";
          g_drivemode = DRIVEMODE_FOLLOW;
        }  else {
          std::cerr << "Now not following\n";
          g_drivemode = DRIVEMODE_KEY;
        }
        break;
      case '+':
      case 'u':
      case 'U':
        g_velTrans += 0.05;
        break;
      case '-':
      case 'n':
      case 'N':
        g_velTrans -= 0.05;
        break;
      case ' ':
        g_velTrans = 0;
        g_velSteer = 0;
        g_drivemode = DRIVEMODE_KEY;
        CureCERR(30) << "Stopping platform\n";
        break;
      case 'h':
      case 'H':
        g_velSteer += 0.05;        
        break;
      case 'j':
      case 'J':
        g_velSteer = 0;
        break;
      case 'k':
      case 'K':
        g_velSteer -= 0.05;        
        break;
      }
      std::cerr << "v=" << g_velTrans << " w=" << g_velSteer << std::endl;
    }

    double maxV = 0.5;
    double maxW = 0.5;
    if (g_velTrans > maxV) g_velTrans = maxV;
    else if (g_velTrans < -maxV) g_velTrans = -maxV;
    if (g_velSteer > maxW) g_velSteer = maxW;
    else if (g_velSteer < -maxW) g_velSteer = -maxW;
    
    usleep(100000);
  }

  return NULL;
}

void processOdom(Cure::Pose3D &odom, Cure::PoseProvider *pp,
                 Cure::NavGraph &graph)
{
  if (heartbeat) std::cerr << "o";
  if (g_saveData) g_odomfile.write(odom);
  
  pp->addOdometry(odom);
  
  Cure::Pose3D p = pp->getPose();

  if (g_IterToInitialize > 0) return;

  graph.addFreePose(p.getX(), p.getY(), p.getTheta());
  if (tag != "") {
    graph.markPose(p.getX(), p.getY(), p.getTheta(), tag);
    tag = "";
  }
}

void processScan(Cure::SICKScan &scan, Cure::PoseProvider *pp,
                 Cure::NavGraph &graph)
{
  if (g_saveData) g_scanfile.write(scan);

  Cure::MeasurementSet measSet;
  g_Seg.segment(scan);
  g_Lsq.doFit(g_Seg.m_Segments, scan);
  Cure::RedundantLine2DRep::makeMeasurementSetFromLines(g_Lsq.m_Lines,
                                                        measSet,
                                                        &g_Seg.m_Segments,
                                                        &scan);
  int ret = pp->addMeasurementSet(measSet);


  if (ret == 1) {
    if (heartbeat) std::cerr << "+";
    g_IterToInitialize--;
  } else if (ret == 0) {
    if (heartbeat) std::cerr << ".";
  } else {
    CureCERR(20) << "Failed to do SLAM update\n";
    return;
  }

  if (g_IterToInitialize > 0) {
    return;
  }
  
  if (g_drivemode == DRIVEMODE_FOLLOW) {
    std::cerr << "In follow mode (1)\n";
    std::string msg;
    if (follow.calcSpeed(scan,g_velTrans, g_velSteer, &msg) != BFARET_WORKING) {
      std::cerr << "Leaving follow mode\n";
      g_velTrans = 0;
      g_velSteer = 0;
      g_drivemode = DRIVEMODE_KEY;
    }
    if (msg != "") std::cerr << msg << std::endl;
  }

  if (doorDetector.inDoorOpening(scan)) {
    if (!inDoor) {
      //std::cout << "\nIn door opening\n"; getchar();
      Cure::Pose3D p = pp->getPose();

      // We add the door where the laser was when the door was
      // detected
      Cure::Pose3D lpW;
      lpW.add(p, g_LaserPoseR);

      // Now calculate the position of the door posts
      double xR = lpW.getX() + (doorDetector.m_RangeR * cos(lpW.getTheta() + doorDetector.m_AngleR - M_PI_2));
      double yR = lpW.getY() + (doorDetector.m_RangeR * sin(lpW.getTheta() + doorDetector.m_AngleR - M_PI_2));
      double xL = lpW.getX() + (doorDetector.m_RangeL * cos(lpW.getTheta() + doorDetector.m_AngleL - M_PI_2));
      double yL = lpW.getY() + (doorDetector.m_RangeL * sin(lpW.getTheta() + doorDetector.m_AngleL - M_PI_2));
      
      // Get the position and orientation of the door
      double x = 0.5 * (xR + xL);
      double y = 0.5 * (yR + yL);
      double dir = atan2(yR - yL, xR - xL);

      graph.addGatewayNode(x,y,dir, doorDetector.m_Width);
    }
  } else {
    if (inDoor) {
      //std::cout << "Leaving door opening\n"; getchar();
      inDoor = false;
    }
  }
}

int odomCallback(Cure::TimestampedData &td)
{
  Cure::Pose3D odom;
  odom = td;

  /*  
  std::cout << "Got odom with t=" << odom.getTime() << ", x="
            << odom.getX() << " y=" << odom.getY()
            << " theta=" << odom.getTheta()
            << std::endl; 
  */

  processOdom(odom, g_poseprov, g_graph);

  return 0;
}

int scanCallback(Cure::TimestampedData &td)
{
  Cure::SICKScan scan;
  scan = td;
  g_scan = scan;

  /*
  std::cout << "Got scan with t=" << scan.getTime() << " and "
            << scan.getNPts() << " points\n";
  */

  scan.setStartAngle(-M_PI_2); // HACK!!!
  processScan(scan, g_poseprov, g_graph);
      
  if (g_IterToInitialize > 0) return 0;
  
  Cure::Pose3D cp = g_poseprov->getPose();
  g_posbuf.write(cp);
  
  // Interpolate to get the best estimate of where the robot was
  // when the scan was acquired
  Cure::Pose3D scan_pos = cp; // HACK!!!
  if (0 && g_posbuf.read(scan_pos, scan.getTime())) {
    // Could not get odometry at scan time
    std::cerr << "Could not get pose at scan time\n";
  } else {
    g_lmap.addScan(scan.getNPts(),
                   scan.getRanges(),
                   scan.getStartAngle(),
                   scan.getAngleStep(),
                   g_LaserPoseR.getX(), 
                   g_LaserPoseR.getY(), 
                   g_LaserPoseR.getTheta(),
                   scan_pos.getX(),
                   scan_pos.getY(),
                   scan_pos.getTheta());
  }
  
  // Predict forward to the last odometry data that we have
  cp = g_poseprov->getPosePrediction();          
  g_lmap.moveRobot(cp);
  
  displayPeekabot(scan, g_poseprov, g_lmap);

  return 0;
}

void runPlayer(const std::string &host, int port)
{
  PlayerCc::PlayerClient robot(host.c_str(),port);
  PlayerCc::Position2dProxy pp(&robot);
  PlayerCc::LaserProxy lp(&robot);

  /* turn on the motors */
  pp.SetMotorEnable(true);

  Cure::Timestamp lastOdomTime(0);
  Cure::Timestamp lastLaserTime(0);

  while(run) {

    robot.Read();

    Cure::Pose3D odom(Cure::PlayerDataToCure::getPose(pp));
    if (odom.getTime() > lastOdomTime) {
      processOdom(odom, g_poseprov, g_graph);
      lastOdomTime = odom.getTime();
    }

    Cure::SICKScan scan = Cure::PlayerDataToCure::getScan(lp);
    if (scan.getNPts() > 0 && scan.getTime() > lastLaserTime) {
      processScan(scan, g_poseprov, g_graph);
      lastLaserTime = scan.getTime();
      
      if (g_IterToInitialize > 0) continue;
      
      Cure::Pose3D cp = g_poseprov->getPose();
      g_posbuf.write(cp);
      
      // Interpolate to get the best estimate of where the robot was
      // when the scan was acquired
      Cure::Pose3D scan_pos;
      if (g_posbuf.read(scan_pos, scan.getTime())) {
        // Could not get odometry at scan time
      } else {
        g_lmap.addScan(scan.getNPts(),
                       scan.getRanges(),
                       scan.getStartAngle(),
                       scan.getAngleStep(),
                       g_LaserPoseR.getX(), 
                       g_LaserPoseR.getY(), 
                       g_LaserPoseR.getTheta(),
                       scan_pos.getX(),
                       scan_pos.getY(),
                       scan_pos.getTheta());
      }
      
      // Predict forward to the last odometry data that we have
      cp = g_poseprov->getPosePrediction();          
      g_lmap.moveRobot(cp);
      
      if (g_drivemode == DRIVEMODE_KEY) {
        // Keyboard mode
        CureCERR(40) << "Sent v=" << g_velTrans << " w=" << g_velSteer 
                     << " in manual mode\n";
        
      } else if (g_drivemode == DRIVEMODE_FOLLOW) {
        std::cerr << "In follow mode (2)\n";
        // Follow mode
        if (follow.calcSpeed(scan,g_velTrans, g_velSteer) != 0) {
          std::cerr << "Leaving follow mode\n";
          g_drivemode = DRIVEMODE_KEY;
          g_velTrans = 0;
          g_velSteer = 0;
        }
        
      } else if (g_drivemode == DRIVEMODE_AUTO) {            
        
        g_navCtrl->updateCtrl();
      } else {
        std::cerr << "g_drivemode=" << g_drivemode << std::endl;
      }
      
      pp.SetSpeed(g_velTrans, g_velSteer);
      
      displayPeekabot(scan, g_poseprov, g_lmap);
    }
  }

  pp.SetSpeed(0,0);
}

void runCure(const std::string &host, int basePort)
{
  g_SockScan.m_Hostname = host;
  g_SockScan.m_Port=basePort;
  g_SockScan.m_Server=false; 
  g_SockScan.startDevice();

  g_SockOdom.m_Hostname = host;
  g_SockOdom.m_Port=basePort+1;
  g_SockOdom.m_Server=false; 
  g_SockOdom.startDevice();

  g_SockCmd.m_Hostname = host;
  g_SockCmd.m_Port=basePort+2;
  g_SockCmd.m_Server=false; 
  g_SockCmd.startDevice();

  Cure::AddressFcnHook scanFcn(scanCallback);
  g_SockScan.push(&scanFcn);

  Cure::AddressFcnHook odomFcn(odomCallback);
  g_SockOdom.push(&odomFcn);

  while(run) {

    if (g_drivemode == DRIVEMODE_KEY) {
      // Keyboard mode
      CureCERR(40) << "Sent v=" << g_velTrans << " w=" << g_velSteer 
                   << " in manual mode\n";
      
    } else if (g_drivemode == DRIVEMODE_FOLLOW) {
      std::cerr << "In follow mode (2)\n";
      // Follow mode
      if (follow.calcSpeed(g_scan,g_velTrans, g_velSteer) != 0) {
        std::cerr << "Leaving follow mode\n";
        g_drivemode = DRIVEMODE_KEY;
        g_velTrans = 0;
        g_velSteer = 0;
      }
      
    } else if (g_drivemode == DRIVEMODE_AUTO) {            
      
      g_navCtrl->updateCtrl();

    } else {
      std::cerr << "g_drivemode=" << g_drivemode << std::endl;
      g_drivemode = DRIVEMODE_KEY;
      g_velTrans = 0;
      g_velSteer = 0;
    }

    Cure::Pose3D cmd;
    Cure::Timestamp ct;
    ct.setToCurrentTime();
    cmd.setTime(ct);
    cmd.setX(g_velTrans);
    cmd.setY(g_velSteer);

    g_SockCmd.write(cmd);

    std::cerr << ".";
    usleep(100000);

  }
}

int main(int argc, char **argv)
{
  cure_debug_level = 30;

  const char *optstring = "h:p:m:d:c:sC";
  const char *args = "[-h host for player] [-p port for player] "
    "[-m map filename] [-d debug level] [-c config file] [-s save data] "
    "[-C use cure]";

  bool hwViaCure = false;
  std::string mapfile = "";
  std::string host = "localhost";
  std::string configfile = "NOTSET";
  int basePort = 3245;
  int port = PlayerCc::PLAYER_PORTNUM;
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 's':
      g_saveData = true;
      break;
    case 'C':
      hwViaCure = true;
      break;
    case 'c':
      configfile = optarg;
      break;
    case 'd':
      cure_debug_level = atoi(optarg);
      break;
    case 'h':
      host = optarg;
      break;
    case 'p':
      port = atoi(optarg);
      break;
    case 'm':
      mapfile = optarg;
      break;
    case '?':
      std::cerr << "Incorrect command line arguments near " <<
        (char) optopt;
      std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
      return -1;
    }

    o = getopt(argc, argv, optstring);
  }

  signal(SIGINT, sigtrap);

  pthread_t tid;
  if (pthread_create(&tid, NULL, control_thread, NULL)) {
    perror("pthread_create");
    return -1;
  }

  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) return -1;

  if (cfg.getSensorPose(1, g_LaserPoseR)) {
    CureCERR(30) << "Failed to get sensor pose on robot\n";
    return -1;
  } else {
    CureCERR(30) << "Got SensorPose " << g_LaserPoseR << std::endl;
  }

  std::string pbRobotFile = "CogX_base.xml";
  std::string pbHost = "localhost";
  int pbPort = 5050;
  std::string usedCfgFile, tmp;
  if (cfg.getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
    pbHost = tmp;
  }
  if (cfg.getString("PEEKABOT_ROBOT_XML_FILE", true, tmp, usedCfgFile) == 0){
    pbRobotFile = tmp;
  }


  try {
    printf("Trying to connect to Peekabot on host %s and port %d\n",
           pbHost.c_str(), pbPort);

    g_PeekabotClient.connect(pbHost, pbPort);

    peekabot::ObjectProxy peekabotRoot;
    peekabotRoot.assign(g_PeekabotClient, "root");    

    g_PeekabotRobot.add(peekabotRoot, "robot", peekabot::REPLACE_ON_CONFLICT);
    
    g_PeekabotRobot.load_scene(pbRobotFile);
    if (pbRobotFile == "Minnie.xml") {
      g_PeekabotLaser.assign(g_PeekabotRobot, "peoplebot_base.rangefinder");
    } else {
      g_PeekabotLaser.assign(g_PeekabotRobot, "chassis.rangefinder");
    }

  } catch(std::exception &e) {
    fprintf(stderr, "Caught exception when connecting to peekabot (%s)\n",
            e.what());
    return -1;
  }

  std::ifstream test(mapfile.c_str());
  if (mapfile != "" && test.is_open()) {
    test.close();
    Cure::WrappedLocalization *loc = new Cure::WrappedLocalization;
    loc->dontDisplay();
    g_RunningSLAM = false;
    g_poseprov = loc;
    
    if (loc->config(configfile)) {
      printf("Failed to config localization with \"%s\"\n",
             configfile.c_str());
      g_poseprov = 0;
    } else if ( loc->loadMap(mapfile) != 0) {
      printf("Failed to load map \"%s\"\n", mapfile.c_str());
      g_poseprov = 0;
    }

    if (g_poseprov == 0) {
      delete loc;
    }
  }

  if (g_poseprov == 0) {
    Cure::WrappedSLAM *slam = new Cure::WrappedSLAM;
    slam->dontDisplay();
    g_poseprov = slam;
    g_RunningSLAM = true;
    slam->getPose();
    if ( slam->config(configfile) ) {
      printf("Failed to config slam with \"%s\"\n", configfile.c_str());
      exit(0);
    }
  }

  if (mapfile == "") {
    mapfile = "tmpmap.metric";
  }

  std::cerr << "mapfile=\"" << mapfile << "\"\n";

  std::string graphfile;
  int pos = mapfile.find_first_of(".");
  if (pos != std::string::npos) {
    std::cerr << "pos=" << pos << std::endl;
    graphfile = mapfile.substr(0,pos) + ".graph";
  } else {
    graphfile = mapfile + ".graph";
  }
  std::fstream f(graphfile.c_str(), std::ios::in);
  if(f.is_open()){
    f.close();
    // Make backup
    std::string backup = graphfile + ".bak";
    std::ofstream(backup.c_str()) << 
      std::ifstream(graphfile.c_str()).rdbuf(); 
  }
  if (!g_RunningSLAM) g_graph.loadGraph(graphfile);
  std::cerr << "graphfile=\"" << graphfile << "\"\n";

  if (g_saveData) {
    g_scanfile.openWriteFile("scans.tdf");
    g_odomfile.openWriteFile("odom.tdf");
  }

  g_navCtrl = new NavCtrl(*g_poseprov, g_graph, g_lmap);
  if (g_navCtrl->config(configfile) != 0) {
    CureCERR(20) << "Failed to config robot shape\n";
    return 1;
  }

  int cntr = 0;

  std::cerr << "\n"
            << "========================== STARTING ======================\n"
            << "\n"
            << "Waiting for connection (can take a few seconds).....\n"
            << "\n";


  std::cerr << "\n\n\n"
            << "NOTE that the initial position of the robot is given\n"
            << "by the ROBOTPOSE variable from the config file or\n"
            << "any of its include files such as robotpose.ccf.\n"
            << "To make the initial pose to be the origin,\n"
            << "removing the file robotpose.ccf should work in most cases\n";
  printKeySetup();

  if (hwViaCure) {
    runCure(host,basePort);
  } else {
    runPlayer(host,port);
  }

  CureCERR(30) << "About to quit\n";
  pthread_join(tid, NULL);

  if (mapfile == "-") {
    std::cerr << "Do you want to save the map? (y/n)\n";
    char a;
    std::cin >> a;
    if (a == 'y' || a == 'Y') {
      std::cerr << "Enter map filename:";
      std::cin >> mapfile;
    }
  }

  if (mapfile != "-" && g_RunningSLAM) {
    ((Cure::WrappedSLAM*)g_poseprov)->saveMap(mapfile);
  }
   
  std::fstream fsg;
  fsg.open(graphfile.c_str(), std::ios::out);
  g_graph.writeToStream(fsg);
  fsg.close();

  std::fstream fsl;
  fsl.open("robotpose.ccf", std::ios::out);
  if (fsl > 0) {
    Cure::Pose3D p = g_poseprov->getPose();
    double ang[3];
    p.getAngles(ang);
    fsl << "ROBOTPOSE\n";
    fsl << p.getX() << " "
        << p.getY() << " "
        << p.getZ() << " "
        << ang[0] << " "
        << ang[1] << " "
        << ang[2] << std::endl;
  }
  fsl.close();

  delete g_poseprov;

  std::cout << "Over and out!!\n";
}
