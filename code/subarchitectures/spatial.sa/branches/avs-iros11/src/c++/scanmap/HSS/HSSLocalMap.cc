//
// = FILENAME
//    HSSLocalMap.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include <fstream>
#include <cstdio>

#include "Eigen/LU"

#include "HSSLocalMap.hh"
#include "HSSutils.hh"
#include "HSSDisplayFunctions.hh"
#include "HSSScan2DMatcher.hh"
#include "HSSMapRobotPose.hh"
#include "HSSAngleHistogram.hh"

namespace HSS {

LocalMap::LocalMap(long id)
  :m_Id(id),
   m_UniformColored(false)
{
  m_StdMeasXY = 0.005;
  m_StdMeasDeg = 0.1;

  m_X.resize(0);
  m_P.resize(0,0);

  m_GlobalT.setZero();
  m_GlobalP.setIdentity();
  m_GlobalP *= 1e-12;

  m_InitScanId = -1;

  m_Localized = true;

  m_LocalizeOnly = false;

  //m_MinDistToAddRefScan = 1;
  //m_MinAngleToAddRefScan = M_PI_2;
  m_MinMaxOverlapToAddRefScan = 0.5;
}

LocalMap::~LocalMap()
{
  finit();
}

void
LocalMap::finit()
{
  m_InitScanId = -1;
  m_Localized = false;

  for (unsigned int i = 0; i < m_MapObjects.size(); i++) {
    delete m_MapObjects[i];
  }
  m_MapObjects.clear();
  m_MapScans.clear();
  m_MapDoors.clear();
}

Eigen::Vector3d 
LocalMap::getXrL() const
{
  if (!hasRobotPose()) {
    std::cerr << "getXrL() LocalMap " << m_Id 
              << " has no robot pose and can therefore not return it\n";
    throw std::exception();
  }

  Eigen::Vector3d x;
  for (int i = 0; i < 3; i++) x[i] = m_X[i];

  return x;
}

Eigen::Matrix3d 
LocalMap::getPrrL() const
{
  if (!hasRobotPose()) {
    std::cerr << "getPrrL() LocalMap " << m_Id 
              << " has no robot pose and can therefore not return P for it\n";
    throw std::exception();
  }

  return m_P.block(0,0,3,3);
}

Eigen::Vector3d 
LocalMap::getXrG() const
{
  if (!hasRobotPose()) {
    std::cerr << "getXrG() LocalMap " << m_Id 
              << " has no robot pose and can therefore not return it\n";
    throw std::exception();
  }

  Eigen::Vector3d x;
  for (int i = 0; i < 3; i++) x[i] = m_X[i];
  x = HSS::compound(m_GlobalT, x);

  return x;
}

Eigen::Matrix3d 
LocalMap::getPrrG() const
{
  if (!hasRobotPose()) {
    std::cerr << "getPrrG() LocalMap " << m_Id 
              << " has no robot pose and can therefore not return P for it\n";
    throw std::exception();
  }

  Eigen::Matrix3d p(m_P.block(0,0,3,3));
  Eigen::Vector3d x;
  for (int i = 0; i < 3; i++) x[i] = m_X[i];
  Eigen::MatrixXd J;
  x = HSS::compound(m_GlobalT, x, &J);
  Eigen::Matrix3d J1(J.block(0,0,3,3));
  Eigen::Matrix3d J2(J.block(0,3,3,3));
  p = J1*m_GlobalP*J1.transpose() + J2*p*J2.transpose();

  return p;  
}

void
LocalMap::init(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  Eigen::Vector3d xr(HSS::icompound(xsR));
  this->extendState(new HSS::MapRobotPose(m_X.size()), xr, 1e6);
  this->extendStateScan(xsR, scan, 1e-12);

  m_InitScanId = -1;

  // Run the update function so that we get the proper uncertainty for
  // the robot pose and the correlation with the ref scan pose
  this->updateScan(xsR, scan);

  m_InitScanId = scan.m_Id;

  m_Localized = true;
}

void 
LocalMap::save(int totMaps, const std::string &mapfile)
{
  FILE *fid;
  fid = fopen(mapfile.c_str(), "w");
  if (fid < 0) { 
    char buf[128];
    sprintf(buf, "/tmp/lmap%03ld", m_Id);
    std::cerr << "HSS::LocalMap(" << m_Id << ")::save Could not open file \"" 
              << mapfile << "\" for writing. Storing in \""
              << buf << "\"\n";
    fid = fopen(buf, "w");
    if (fid < 0) {
      std::cerr << "HSS::LocalMap(" << m_Id << ")::save Failed to open \"" 
                << buf << "\" also for writing, giving up\n";
      return;
    }
  }

  if (m_MapObjects.empty()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::save, empty map\n";
    fprintf(fid, "%d\n0\n", totMaps);
    fclose(fid);
    return;
  }

  int N = m_X.size();

  // Number of maps in total and number of state variables
  // in this map
  fprintf(fid, "%d\n%ld\n%d\n%d\n%ld\n",
          totMaps, m_Id, N, m_MapObjects.size(),m_InitScanId);

  // State variables
  for (int i = 0; i < N; ) {
    // Look for the map objects with statepos equal to i
    bool found = false;    
    for (unsigned int j = 0; j < m_MapObjects.size(); j++) {
      if (m_MapObjects[j]->getStatePos() == i) {
        found = true;
        // Write TYPE
        fprintf(fid, "%d ", m_MapObjects[j]->getType());
        // Write X Y ANG
        for (int k = 0; k < m_MapObjects[j]->getStateSize(); k++) {
          fprintf(fid, "%.6lf ", m_X[i+k]);
        }
        // Write scan data for scan and width for door
        if (m_MapObjects[j]->getType() == MapObject::TYPE_ROBOTPOSE) {
          // Nothing more to add
        } else if (m_MapObjects[j]->getType() == MapObject::TYPE_SCAN) {
          MapScan2D *scan = (MapScan2D*)m_MapObjects[j];
          fprintf(fid, "%ld %d %f %f %f %ld %ld %f %f %f ",
                  scan->m_Id, scan->range.size(),
                  scan->min_range, scan->max_range, scan->range_sigma,
                  scan->tv.tv_sec, scan->tv.tv_usec,
                  scan->odom[0], scan->odom[1], scan->odom[2]);
          for (unsigned k = 0; k < scan->range.size(); k++) {
            fprintf(fid, "%.3f %.8f %d ", 
                    scan->range[k], scan->theta[k], scan->valid[k]);
          }                  

        } else if (m_MapObjects[j]->getType() == MapObject::TYPE_DOOR) {

          MapDoor *door = (MapDoor*)m_MapObjects[j];
          fprintf(fid, "%f ", door->getWidth());

        } else {
          std::cerr << "HSSLocalMap(" << m_Id << ")::save type="
                    << m_MapObjects[j]->getType()
                    << " not dealt with. Logic error!!\n";
          throw std::exception();
        }
        i += m_MapObjects[j]->getStateSize();
        break;
      }
    }
    fprintf(fid, "\n");

    if (!found) {
      std::cerr << "HSSLocalMap(" << m_Id << ")::save. Did not find map object at state pos "
                << i << std::endl;
      throw std::exception();
    }    
  }

  // Covariance matrix upper right triangle (excl robot pose)
  for (int r = 0; r < N; r++) {
    for (int c = r; c < N; c++) {
      fprintf(fid, "%.6e ", m_P(r,c));
    }
    fprintf(fid, "\n");
  }
  fclose(fid);
}

void 
LocalMap::load(const std::string &mapfile,
               const Eigen::Vector3d &xsR)
{
  m_Localized = false;
  
  std::fstream fs;
  fs.open(mapfile.c_str(), std::ios::in);
  if (!fs.is_open()) {
    std::cerr << "HSSLocalMap(" << m_Id << ")::load could not load file \""
              << mapfile << "\"\n";
    throw std::exception();
  }

  int M, N, n;
  fs >> M >> m_Id >> N >> n >> m_InitScanId;
  m_X.resize(N);
  m_P.resize(N,N);
  m_MapObjects.resize(n);

  std::cerr << "Loading " << n << " landmarks from file \""
            << mapfile << "\" (file " << M << ")\n";

  // Read the state vector and accompaning data
  int pos = 0;
  for (int i = 0; i < n; i++) {
    int type;
    fs >> type;
    if (type == MapObject::TYPE_ROBOTPOSE) {

      fs >> m_X[pos] >> m_X[pos+1] >> m_X[pos+2];

      m_MapObjects[i] = new MapRobotPose(pos);
      pos += m_MapObjects[i]->getStateSize();

    } else if (type == MapObject::TYPE_SCAN) {

      fs >> m_X[pos] >> m_X[pos+1] >> m_X[pos+2];

      CandScan2D *scan = new CandScan2D;
      int n_pts;
      fs >> scan->m_Id >> n_pts
         >> scan->min_range >> scan->max_range 
         >> scan->range_sigma >> scan->tv.tv_sec >> scan->tv.tv_usec
         >> scan->odom[0] >> scan->odom[1] >> scan->odom[2];
      scan->alloc(n_pts);
      scan->min_theta = 1e10;
      scan->max_theta =-1e10;
      for (int s = 0; s < n_pts; s++) {
        fs >> scan->range[s] >> scan->theta[s] >> scan->valid[s];
        if (scan->theta[s] < scan->min_theta) scan->min_theta = scan->theta[s];
        if (scan->theta[s] > scan->max_theta) scan->max_theta = scan->theta[s];
      }

      m_MapObjects[i] = new MapScan2D(pos, *scan);
      m_MapScans.push_back((MapScan2D*)m_MapObjects[i]);
      pos += m_MapObjects[i]->getStateSize();

      HSS::makeScanCandidate(*((MapScan2D*)m_MapObjects[i]), xsR);

    } else if (type == MapObject::TYPE_DOOR) {

      double width;
      fs >> m_X[pos] >> m_X[pos+1] >> m_X[pos+2] >> width;

      m_MapObjects[i] = new MapDoor(pos, width);
      m_MapDoors.push_back((MapDoor*)m_MapObjects[i]);
      pos += m_MapObjects[i]->getStateSize();

    } else {
      std::cerr << "HSSLocalMap(" << m_Id << ")::load type=" << type
                << " not dealt with. File read error!!\n";
      throw std::exception();
    }
  }

  // Read the covariance matrix
  for (int r = 0; r < N; r++) {
    for (int c = r; c < N; c++) {
      fs >> m_P(r,c);
      if (r != c) m_P(c,r) = m_P(r,c);
    }
  }

  std::cerr << "Loaded LocalMap(" << m_Id << ") with "
            << m_MapObjects.size() << " map objects ("
            << m_MapScans.size() << " scans and "
            << m_MapDoors.size() << " doors)\n";
}
  
bool
LocalMap::hasRobotPose() const
{
  if (m_MapObjects.empty()) return false;

  return (m_MapObjects[0]->getType() == MapObject::TYPE_ROBOTPOSE);
}

bool
LocalMap::atBorderHeadingOut(const Eigen::Vector3d &xsR)
{
  return false;

  /*
  // If there is no robot pose in the map we have nothing to do in the
  // prediction step
  if (!hasRobotPose()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::atMapBorder No Robot pose!?!?!\n";
    throw std::exception();
  }

  // Loop through the scans
  Eigen::Vector3d xr(getXrL());
  for (unsigned int i = 0; i < m_MapScans.size(); i++) {

    // Pose of the landmark (scan)
    Eigen::Vector3d xl(m_MapScans[i]->getVector3(m_X));

    // Check if this landmark is behind us in which case we
    // do not need to do anything
    if (HSS::distPt2Line(xl[0], xl[1], xr[0], xr[1], xr[2]+M_PI_2) < 0) {
      continue;
    }

    // Check if it is close enough on the side
    if (fabs(HSS::distPt2Line(xl[0], xl[1], xr[0], xr[1], xr[2])) <
        m_MinDistToAddRefScan) {
      // Close enough to the side to say that we have something
      // infront of us
      return false;
    }
  }

  // If we get this far it means that there was no ref scan infront of
  // us
  
  // We should have the closest ref scan far enough behind us
  if (minScanner2ScanDist(xsR) > m_MinDistToAddRefScan - 0.5) {
    return true;
  }

  return false;
  */
}

bool
LocalMap::atBorderHeadingIn(const Eigen::Vector3d &xsR)
{
  return false;

  /*
  // If there is no robot pose in the map we have nothing to do in the
  // prediction step
  if (!hasRobotPose()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::atMapBorder No Robot pose!?!?!\n";
    throw std::exception();
  }

  // Loop through the scans.
  Eigen::Vector3d xr(getXrL());
  for (unsigned int i = 0; i < m_MapScans.size(); i++) {

    // Pose of the landmark (scan)
    Eigen::Vector3d xl(m_MapScans[i]->getVector3(m_X));

    // Check if this landmark is infront of us an not too far away
    double d = HSS::distPt2Line(xl[0], xl[1], xr[0], xr[1], xr[2]+M_PI_2);
    if (d > 0 && d < m_MinDistToAddRefScan) {

      // Check if it is close enough on the side
      if (fabs(HSS::distPt2Line(xl[0], xl[1], xr[0], xr[1], xr[2])) <
          m_MinDistToAddRefScan) {
        // Close enough to the side to say that we have something
        // infront of us and we can thus say that we are heading in to it
        return true;
      }
    }
  }

  return false;
  */
}

void 
LocalMap::extendState(MapObject *mapObject, const Eigen::Vector3d &xnW,
                      double variance)
{
  std::cerr << "Extending state from size=" << m_X.size() << std::endl;
  //std::cerr << "State vector is now X=[" << m_X.transpose() << "]\n";
  //std::cerr << "Before extend P=[" << m_P << "]\n";
  
  m_MapObjects.push_back(mapObject);
  
  int N = m_X.size();
  Eigen::VectorXd Xtmp(m_X);
  m_X.resize(N+3);
  for (int i = 0; i < N; i++) {
    m_X[i] = Xtmp[i];
  }
  m_X[N+0] = xnW[0];
  m_X[N+1] = xnW[1];
  m_X[N+2] = xnW[2];

  Eigen::MatrixXd Ptmp(m_P);
  m_P.resize(N+3,N+3);
  m_P.setZero();
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      m_P(i,j) = Ptmp(i,j);
    }
  }
  m_P.block(N,N,3,3).setIdentity();
  m_P.block(N,N,3,3) *= variance;
  
  //std::cerr << "Extended state to size=" << m_X.size() << std::endl;
  //std::cerr << "State vector is now X=[" << m_X.transpose() << "]\n";
  //std::cerr << "After extend P=[" << m_P << "]\n";
}

void 
LocalMap::extendStateScan(const Eigen::Vector3d &xsR, 
                          const HSS::CandScan2D &scan,
                          double variance)
{
  HSS::MapScan2D *newMapScan = new HSS::MapScan2D(scan);
  newMapScan->setStatePos(m_X.size());
  m_MapScans.push_back(newMapScan);
  std::cerr << "Added ref scan, now have " << m_MapScans.size() << "\n";

  // Pose of sensor 
  Eigen::Vector3d xs(HSS::compound(m_X,xsR));
  
  extendState(m_MapScans.back(), xs, variance);
}

void 
LocalMap::extendStateDoor(const Eigen::Vector3d &xdS, 
                          const Eigen::Vector3d &xsR, 
                          double width)
{
  HSS::MapDoor *newMapDoor = new HSS::MapDoor(width);
  newMapDoor->setStatePos(m_X.size());
  m_MapDoors.push_back(newMapDoor);
  std::cerr << "Added door, now have " << m_MapDoors.size() << "\n";

  // Pose of door 
  Eigen::Vector3d xd(HSS::compound(m_X, HSS::compound(xsR, xdS)));

  extendState(m_MapDoors.back(), xd, 1e6);
}

bool 
LocalMap::predict(const Eigen::Vector3d &delta, const Eigen::Matrix3d &Q)
{
  if (m_MapScans.empty()) {
    std::cerr << "LocalMap(" << m_Id << ")::predict:  ERROR No ref scan\n";
    std::cerr << "predict: Did you forget to call init??\n";
    throw std::exception();
  }

  if (m_MapObjects.empty()) {
    std::cerr << "No map objects at all in LocalMap " << m_Id
              << " no need to predict\n";
    return false;
  }

  // If there is no robot pose in the map we have nothing to do in the
  // prediction step
  if (!hasRobotPose()) {
    std::cerr << "No Robot pose (or not at pose 0) in LocalMap " << m_Id 
              << " no need to predict\n";
    return false;
  }

  //std::cerr << "Prediction started det(P)=" << m_P.determinant() << std::endl;

  // The robot pose changes according to
  // x(k+1) = x(k) + cos(theta(k))*delta_x - sin(theta(k))*delta_y
  // y(k+1) = y(k) + sin(theta(k))*delta_x + cos(theta(k))*delta_y
  // theta(k+1) = theta(k) + delta_a
  
  int N = m_X.size();

  Eigen::Vector3d newX;
  newX = HSS::compound(m_X, delta);
  m_X[0] = newX[0];
  m_X[1] = newX[1];
  m_X[2] = newX[2];
  
  // The upfdate for P is P = F*P*F.transpose() + G*Q*G.transpose();
  // However, F is sparse and consists of 4 blocks
  // F = [Fr 0
  //      0  I];
  // where Fr is of size 3x3 and I is of size (N-3)x(N-3)
  //
  // We can also define the following blocks for P
  // P = [Prr Prl
  //      Plr Pll]
  // where again Prr is size 3x3 and Pll (N-3)x(N-3)
  // 
  // F*P*F^T can then be calculated as
  // 
  // F*P = [Fr 0] [Prr Prl] = [Fr*Prr Fr*Prl]
  //       [ 0 I] [Plr Pll]   [Plr    Pll   ]
  //
  // F*P*F^T = [Fr*Prr Fr*Prl] * [Fr^T 0] = [Fr*Prr*Fr^T Fr*Prl]
  //           [Plr    Pll   ]   [0    I]   [Plr*Fr^T    Pll   ]

  Eigen::MatrixXd Fr(3,3);
  Fr.setIdentity();
  Fr(0,2) = -sin(m_X[2])*delta[0]-cos(m_X[1])*delta[1];
  Fr(1,2) =  cos(m_X[2])*delta[0]-sin(m_X[1])*delta[1];

  Eigen::MatrixXd Gr(3,3);
  Gr.setZero();
  Gr(0,0) = cos(m_X[2]); 
  Gr(0,1) =-sin(m_X[2]); 
  Gr(1,0) = sin(m_X[2]); 
  Gr(1,1) = cos(m_X[2]); 
  Gr(2,2) = 1;
  
  m_P.block<3,3>(0,0) = 
    Fr*m_P.block<3,3>(0,0)*Fr.transpose() + Gr*Q*Gr.transpose();

  if (m_LocalizeOnly) return true;

  if (N > 3) {
    m_P.block(0,3,3,N-3) = Fr*m_P.block(0,3,3,N-3);
    m_P.block(3,0,N-3,3) = m_P.block(0,3,3,N-3).transpose();
  }

  //std::cerr << "Prediction done det(P)=" << m_P.determinant() << std::endl;
  //std::cerr << "State vector after pred X=[" << m_X.transpose() << "]'\n";

  return true;
}

bool 
LocalMap::correct(const Eigen::Vector3d &xsR, HSS::Meas &z) 
{
  //std::cerr << "HSS::correctGlobal starting\n";
  
  // Position of landmark
  Eigen::Vector3d xl;
  xl[0] = m_X[z.getStatePosX()];
  xl[1] = m_X[z.getStatePosY()];
  xl[2] = m_X[z.getStatePosTheta()];
  
  Eigen::Vector3d xsW(HSS::compound(m_X, xsR));
  
  int N = m_X.size();

  /*
  // Build the measurement equation Jacobian
  Eigen::MatrixXd H(3,N);
  H.setZero();
  
  // Dependence on the robot position
  H(0,0) =-cos(m_X[2]+xsR[2]);
  H(0,1) =-sin(m_X[2]+xsR[2]);
  H(0,2) =-sin(m_X[2]+xsR[2])*(xl[0]-m_X[0]-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(sin(m_X[2])*xsR[0]+cos(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(xl[1]-m_X[1]-sin(m_X[2])*xsR[0]-cos(m_X[2])*xsR[1])+sin(m_X[2]+xsR[2])*(-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1]);
  H(1,0) = sin(m_X[2]+xsR[2]);
  H(1,1) =-cos(m_X[2]+xsR[2]);
  H(1,2) =-cos(m_X[2]+xsR[2])*(xl[0]-m_X[0]-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1])-sin(m_X[2]+xsR[2])*(sin(m_X[2])*xsR[0]+cos(m_X[2])*xsR[1])-sin(m_X[2]+xsR[2])*(xl[1]-m_X[1]-sin(m_X[2])*xsR[0]-cos(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1]);
  H(2,0) = 0;
  H(2,1) = 0;
  H(2,2) = -1;
  
  // Dependence on the landmark position
  H(0,z.getStatePos()+0) = cos(m_X[2] + xsR[2]);
  H(0,z.getStatePos()+1) = sin(m_X[2] + xsR[2]);
  H(0,z.getStatePos()+2) = 0;
  H(1,z.getStatePos()+0) =-sin(m_X[2] + xsR[2]);
  H(1,z.getStatePos()+1) = cos(m_X[2] + xsR[2]);
  H(1,z.getStatePos()+2) = 0;
  H(2,z.getStatePos()+0) = 0;
  H(2,z.getStatePos()+1) = 0;
  H(2,z.getStatePos()+2) = 1;
  */

  // Build the measurement equation Jacobian

  // Dependence on the robot position
  Eigen::MatrixXd Hr(3,3);
  Hr.setZero();
  Hr(0,0) =-cos(m_X[2]+xsR[2]);
  Hr(0,1) =-sin(m_X[2]+xsR[2]);
  Hr(0,2) =-sin(m_X[2]+xsR[2])*(xl[0]-m_X[0]-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(sin(m_X[2])*xsR[0]+cos(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(xl[1]-m_X[1]-sin(m_X[2])*xsR[0]-cos(m_X[2])*xsR[1])+sin(m_X[2]+xsR[2])*(-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1]);
  Hr(1,0) = sin(m_X[2]+xsR[2]);
  Hr(1,1) =-cos(m_X[2]+xsR[2]);
  Hr(1,2) =-cos(m_X[2]+xsR[2])*(xl[0]-m_X[0]-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1])-sin(m_X[2]+xsR[2])*(sin(m_X[2])*xsR[0]+cos(m_X[2])*xsR[1])-sin(m_X[2]+xsR[2])*(xl[1]-m_X[1]-sin(m_X[2])*xsR[0]-cos(m_X[2])*xsR[1])+cos(m_X[2]+xsR[2])*(-cos(m_X[2])*xsR[0]+sin(m_X[2])*xsR[1]);
  Hr(2,0) = 0;
  Hr(2,1) = 0;
  Hr(2,2) = -1;
  
  // Dependence on the landmark position
  Eigen::MatrixXd Hl(3,3);
  Hl.setZero();
  Hl(0,0) = cos(m_X[2] + xsR[2]);
  Hl(0,1) = sin(m_X[2] + xsR[2]);
  Hl(0,2) = 0;
  Hl(1,0) =-sin(m_X[2] + xsR[2]);
  Hl(1,1) = cos(m_X[2] + xsR[2]);
  Hl(1,2) = 0;
  Hl(2,0) = 0;
  Hl(2,1) = 0;
  Hl(2,2) = 1;
  
  // S = H*P*H^T + R
  // We calculate S using block calculations as H is very sparse.
  // 
  // H = [ Hr 0 Hl 0 ]
  // 
  // P = [ Pr    Pa   Prl  Pb ]   [ PR PA PL PB ]
  //     [ Pa^T  Pc   Pd   Pe ] =
  //     [ Prl^T Pd^T Pl   Pf ]  
  //     [ Pb^T  Pd^T Pe^T Pg ]  
  //
  // We need to use P*H^T later for the calculation of K so we store 
  // that result so that we can reuse it
  //
  // P * H^T =
  // 
  // [ Pr    Pa   Prl  Pb ]   [ Hr^T ]   
  // [ Pa^T  Pc   Pd   Pe ] * [ 0    ] = 
  // [ Prl^T Pd^T Pl   Pf ]   [ Hl^T ]   
  // [ Pb^T  Pd^T Pe^T Pg ]   [ 0    ]   
  //
  // [ PR PA PL PB ] * H^T = PR*Hr^T+PL*Hl^T
  //

  // We avoid using P = (I-K*H)*P since it is not numerically stable
  // and instead use P = (I-K*H)*P*(I-K*H)^T + K*R*K^T

  double maxChi2 = 7.81;
  if (z.Z.size() == 1) maxChi2 = 3.84;
  else if (z.Z.size() == 2) maxChi2 = 5.99;

  if (m_LocalizeOnly) {

    Eigen::Matrix3d Prr = m_P.block(0,0,3,3);
    Eigen::Vector3d xr;
    for (int i = 0; i < 3; i++) xr[i] = m_X[i];
                                  
    Eigen::MatrixXd PHt(Prr*Hr.transpose());
    Eigen::MatrixXd S(Hr*PHt + z.R);
    Eigen::MatrixXd invS(S.inverse());
    Eigen::MatrixXd chi2(z.innov.transpose() * invS * z.innov);
    z.chi2 = chi2(0,0);
    if (z.chi2 > maxChi2) {
      std::cerr << "Scrapping measurement of ref \"" << z.description << "\" item " << z.refpos
                << " with too large chi2="
                << z.chi2 << std::endl;
      return false;
    }
    Eigen::MatrixXd K(PHt*invS);
    Eigen::MatrixXd KH(K*Hr);
    Eigen::MatrixXd I(3,3);
    I.setIdentity();    
    Eigen::MatrixXd IKH(I-KH);
    Prr = IKH*Prr*IKH.transpose() + K*z.R*K.transpose();
    xr = xr + K*z.innov;

    m_P.block(0,0,3,3) = Prr;
    for (int i = 0; i < 3; i++) m_X[i] = xr[i];

  } else {
    // We start by calculating the matrix P*H^T since we need that 
    Eigen::MatrixXd PHt(m_P.block(0,0,N,3)*Hr.transpose() + m_P.block(0,z.getStatePos(),N,3)*Hl.transpose());
    Eigen::MatrixXd S(Hr*PHt.block(0,0,3,3) + Hl*PHt.block(z.getStatePos(),0,3,3) + z.R);
    
    Eigen::MatrixXd invS(S.inverse());
    
    Eigen::MatrixXd chi2(z.innov.transpose() * invS * z.innov);
    z.chi2 = chi2(0,0);
    if (z.chi2 > maxChi2) {
      std::cerr << "Scrapping measurement of ref \"" << z.description << "\" item " << z.refpos
                << " with too large chi2="
                << z.chi2 << std::endl;
      //if (z.description == "door") getchar();
      return false;
    }
    
    std::cerr << "Performing correction with measurement of ref \"" << z.description << "\" item "
              << z.refpos << "  innov=[" << z.innov.transpose() << "]\n";
    
    Eigen::MatrixXd K(PHt*invS);
    
    /*
      std::cerr << "H=[" << H << "]\n";
      std::cerr << "X=[" << m_X << "]\n";
      std::cerr << "P=[" << m_P << "]\n";
      std::cerr << "Z=[" << z.Z << "]\n";
      std::cerr << "R=[" << z.R << "]\n";
      std::cerr << "innov=[" << z.innov << "]\n";
      std::cerr << "S=[" << S << "]\n";
      std::cerr << "K=[" << K << "]\n";
      std::cerr << "I=[" << I << "]\n";
    */
    
    // We avoid using P = (I-K*H)*P since it is not numerically stable
    // and instead use P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
    //
    // Again we can make use of the structure of H = [Hr 0 Hl 0]
    Eigen::MatrixXd KH(N,N);
    KH.setZero();
    KH.block(0,0,N,3) = K * Hr;
    KH.block(0,z.getStatePos(),N,3) = K * Hl;
    
    Eigen::MatrixXd I(N,N);
    I.setIdentity();
    
    Eigen::MatrixXd IKH(I-KH);
    
    m_P = IKH*m_P*IKH.transpose() + K*z.R*K.transpose();
    m_X = m_X + K*z.innov;

    /*
      std::cerr << "X=[" << m_X << "]\n";
      std::cerr << "P=[" << m_P << "]\n";
    */

    for (unsigned int i = 0; i < m_MapObjects.size(); i++) {
      m_MapObjects[i]->normalize(m_X);
    }
  }

  //std::cerr << "X=[" << m_X.transpose() << "]\n";

  //std::cerr << "HSS::correctGlobal done\n";
  return true;
}

void 
LocalMap::setCorridorR(Eigen::Matrix3d &R, double corrDirNewR, 
                       double angRob, double angLandm)
{
  // We want large uncertainty along the corridor and small
  // across it This uncertainty is expressed in the corridor
  // coordinate system which we know the angle of with respect
  // to the robot. However we need to express it in the frame of
  // the landmark. The orientation of the corridor (x-direction
  // along corridor) in the landmark frame is corrDirNewR+angRob-angLandm
  Eigen::MatrixXd Rot(2,2);
  Rot(0,0) = cos(corrDirNewR+angRob-angLandm);
  Rot(0,1) =-sin(corrDirNewR+angRob-angLandm);
  Rot(1,0) =-Rot(0,1);
  Rot(1,1) = Rot(0,0);

  Eigen::MatrixXd R22(2,2);
  R22.setZero();
  R22(0,0) = 1e6;
  R22(1,1) = HSS::sqr(m_StdMeasXY);
  R22 = Rot * R22 * Rot.transpose();
  
  R.setZero();
  R(0,0) = R22(0,0);
  R(0,1) = R(1,0) = R22(0,1);
  R(1,1) = R22(1,1);
  R(2,2) = HSS::sqr(HSS::deg2rad(m_StdMeasDeg));
}


void 
LocalMap::updateScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  if (m_MapScans.empty()) {
    std::cerr << "LocalMap(" << m_Id << ")::updateScan:  ERROR No ref scan\n";
    std::cerr << "updateScan: Did you forget to call init??\n";
    throw std::exception();
  }

  if (scan.m_Id == m_InitScanId) {
    std::cerr << "LocalMap(" << m_Id << ")::updateScan: "
              << "skipping update with init scan\n";
    return;
  }

  //std::cerr << "Update step started\n";
  //std::cerr << "Before scan update X=[" << m_X.transpose() << "]\n";
  //std::cerr << "Before update\nP=[" << m_P << "]\n";

  // Pose of sensor
  Eigen::Vector3d xs(HSS::compound(m_X,xsR));

  // Vector where measurements will be collected
  std::vector<HSS::Meas> meas;

  //double dmin = 1e10;
  //double amin = 1e10;
  double maxOverlap = 0;

  std::ostringstream resstr;

  // Loop over all the reference scans and check which ones are likely
  // to be matchable
  for (unsigned int i = 0; i < m_MapScans.size(); i++) {

    if (scan.m_Id == m_MapScans[i]->m_Id) {

      std::cerr << "HSSLocalMap(" << m_Id << ")::updateScan running update with ref scan "
                << m_MapScans[i]->m_Id << " itself\n";

      //dmin = 0;
      //amin = 0;
      maxOverlap = 1;
   
      HSS::Meas z;
      z.description = "scan";
      z.Z.setZero();
      z.statepos = m_MapScans[i]->getStatePos();
      z.refpos = i;
      z.R.setIdentity();
      z.R *= 1e-12;
      z.innov.setZero();
      meas.push_back(z);

      continue;
    }

    // Position of landmark
    Eigen::Vector3d xl;
    xl[0] = m_X[m_MapScans[i]->getStatePos()+0];
    xl[1] = m_X[m_MapScans[i]->getStatePos()+1];
    xl[2] = m_X[m_MapScans[i]->getStatePos()+2];

    // Predicted position of sensor in landmark frame
    Eigen::Vector3d xlS(HSS::compound(HSS::icompound(xs), xl));

    /*
    std::cerr << "Predicted measurement xsL=" << xsL[0]
              << " ysL=" << xsL[1]
              << " asL=" << xsL[2]
              << std::endl;
    */

    /*
    double dd = hypot(xlS[0], xlS[1]);
    if (dd < dmin) {
      dmin = dd;      
    }

    if (dd <= m_MinDistToAddRefScan && fabs(HSS::pi_to_pi(xlS[2])) < amin) {
      amin = fabs(HSS::pi_to_pi(xlS[2]));      
    }
    */

    double overlap = estimatedOverlap(*m_MapScans[i], scan, xs);
    if (overlap > maxOverlap) maxOverlap = overlap;
    std::cerr << "[" << i << " o=" << overlap << "] ";
    if (overlap < 0.5) {
      //std::cerr << "Not using this scan since it is too far away\n";
      resstr << "[" << i << " o=" << overlap << "<0.5] ";
      continue;
    }


    /*
    // Is it too far away to even attempt at a match
    if (dd > 2) {
      //std::cerr << "Not using this scan since it is too far away\n";
      resstr << "[" << i << " dd=" << dd << ">2] ";
      continue;
    }
    */

    double t0 = HSS::getCurrentTime();
    int n = -1;
    Eigen::Vector3d T;
    HSS::Scan2DMatcher matcher;
    std::vector< std::pair<int, int> > corrs;
    bool ret = matcher.scanMatch(scan, *m_MapScans[i], xlS, T, n, &corrs);
    double t1 = HSS::getCurrentTime();
    if (ret) {

      std::cout << "Scan matching successful in t=" << t1-t0 << "s "
                << " and " << n << " iterations "
                << " x=" << T[0] << " (" << T[0]-xlS[0] << ")"
                << " y=" << T[1] << " (" << T[1]-xlS[1] << ")"
                << " theta=" << HSS::rad2deg(T[2])
                << "deg (" << HSS::rad2deg(T[2]-xlS[2]) << "deg)"
                << std::endl;
      
      HSS::Meas z;
      z.description = "scan";
      for (int j = 0; j < 3; j++) z.Z[j] = T[j];
      z.statepos = m_MapScans[i]->getStatePos();
      z.refpos = i;
      z.R.setZero();
      z.R(0,0) = z.R(1,1) = HSS::sqr(m_StdMeasXY);
      z.R(2,2) = HSS::sqr(HSS::deg2rad(m_StdMeasDeg));      
      if (scan.m_Corridor) {
        setCorridorR(z.R, scan.m_MainDirR, m_X[2], xl[2]);
      } else if (m_MapScans[i]->m_Corridor) {
        setCorridorR(z.R, m_MapScans[i]->m_MainDirR, xl[2], xl[2]);
      }
      
      // Create the new CandScan2D from scan but only the points that were
      // matched are considered valid.
      HSS::CandScan2D corrScan;
      corrScan = scan;
      corrScan.m_Id *= -1;
      // Set all the points to not valid
      for (unsigned int j = 0; j < corrScan.valid.size(); j++) {
        corrScan.valid[j] = 0;
      }
      // Set to valid just the points which appear in the file as matched
      
      /*
      FILE *corrLog;
      int scanSensIndex, scanRefIndex; 
      corrLog = fopen("corr.log", "r");
      if (corrLog != NULL) {
        while ( fscanf(corrLog, "%d %d", &scanSensIndex, &scanRefIndex) != EOF) {
          corrScan.valid[scanRefIndex-1] = 1;
            // IMPORTANT: we use -1 because in the file the indices were written
            // considering that the first element is in the position one.
        }
        fclose(corrLog);
      } else {
        std::cerr << "Failed corr.log reading" << std::endl;
      }
      */
      
      for (unsigned int j = 0; j < corrs.size(); j++) {
        corrScan.valid[ corrs[j].first ] = 1;
      }
      
      HSS::AngleHistogram ah;
      ah.setMinPeakFrac(0.7);
      if (ah.isCorridorLike(corrScan, &corrScan.m_MainDirS)) {
        if (!(scan.m_Corridor || m_MapScans[i]->m_Corridor)) {
          std::cerr << "I say corridor now but it did not before!!\n";
          getchar();
        }
      } else {
        if ((scan.m_Corridor || m_MapScans[i]->m_Corridor)) {
          std::cerr << "I do not say corridor now but I did before. That is WEIRD!! "
                    << scan.m_Id << ":" << scan.m_Corridor << " " 
                    << m_MapScans[i]->m_Id << ":" << m_MapScans[i]->m_Corridor << std::endl;
          getchar();
        }
      }


      double d = hypot(z.Z[0], z.Z[1]);
      if (d > 0.05) z.R *= d / 0.05;

      z.innov = z.Z - xlS;
      z.innov[2] = HSS::pi_to_pi(z.innov[2]);

      meas.push_back(z);
    } else {
      resstr << "[" << i << " SM failed] ";

      std::cout << "Scan matching failed in t=" << t1-t0 << "s "
                << " and " << n << " iterations\n";
    }
  }   

  //std::cerr << resstr.str() << std::endl;

  for (unsigned int i = 0; i < meas.size(); i++) {

    /*
    if (scan.m_Id != m_MapScans[meas[i].refpos]->m_Id) {
      std::cerr << "HACKING so that we never get an update from a scan\n";
      continue;
    }
    */

    //std::cerr << "Correcting with measurement of ref " << meas[i].refpos 
    //        << " at index " << meas[i].getStatePos() << " in X\n";

    if (correct(xsR, meas[i])) {
      resstr << "[" << meas[i].refpos << " USED chi2=" 
             << meas[i].chi2 << "] ";
    } else {
      resstr << "[" << meas[i].refpos << " SKIPPED chi2=" 
             << meas[i].chi2 << "] ";
    }
  }

  if (!m_Localized &&
      HSSmax(sqrt(m_P(0,0)), sqrt(m_P(1,1))) < 0.2) {
    std::cerr << "LOCALIZED in map " << getId() << std::endl;
    m_Localized = true;
  }

  std::cerr << " maxOverlap=" << maxOverlap << std::endl;
  if (!m_LocalizeOnly &&
      m_Localized &&
      maxOverlap < m_MinMaxOverlapToAddRefScan) {
    // Getting far from other reference scans, time to add a new one
    std::cerr << "Added new ref because little overlap with previous scans "
              << "maxOverlap=" << maxOverlap << std::endl;
    extendStateScan(xsR, scan, 1e6);

    //std::cerr << "Before extra update X=[" << m_X.transpose() << "]\n";
    //std::cerr << "Before extra update\nP=[" << m_P << "]\n";


    HSS::Meas z;
    z.description = "newscan";
    z.statepos = m_MapScans.back()->getStatePos();
    z.refpos = m_MapScans.size()-1;

    z.R.setIdentity();
    z.R *= 1e-12;
    //std::cerr << "Immediately correcting with meas of ref " << z.refpos 
    //        << " at index " << z.getStatePos() << " in X\n";
    correct(xsR, z);
  }

  //std::cerr << "Done with update step, now size(X)=" << X.size() << std::endl;
  //std::cerr << "After update X=[" << X.transpose() << "]\n";
  //std::cerr << "After update\nP=[" << P << "]\n";
}

void
LocalMap::addRefScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  // Check if this scan is not already added as a reference scan in
  // case we do not need to do anything
  for (unsigned int i = 0; i < m_MapScans.size(); i++) {
    if (m_MapScans[i]->m_Id == scan.m_Id) {
      std::cerr << "HSS::LocalMap(" << m_Id << ")::addRefScan scan "
                << scan.m_Id << " already added\n";
      return;
    }
  }

  std::cerr << "HSS::LocalMap(" << m_Id << ")::addRefScan("
                << scan.m_Id << ") added\n";

  // Extend the state vector
  this->extendStateScan(xsR, scan, 1e6);

  // Run the update function so that we get a proper covariance matrix
  this->updateScan(xsR, scan);
}


void 
LocalMap::updateDoor(const Eigen::Vector3d &xsR, 
                     HSS::DoorExtractor &doorExtractor)
{
  if (m_MapScans.empty()) {
    std::cerr << "LocalMap(" << m_Id << ")::updateDoor:  ERROR No ref scan\n";
    std::cerr << "updateDoor: Did you forget to call init??\n";
    throw std::exception();
  }

  //std::cerr << "Before door update X=[" << X.transpose() << "]\n";

  // Pose of sensor in
  Eigen::Vector3d xs(HSS::compound(m_X, xsR));

  // Vector where measurements will be collected
  std::vector<HSS::Meas> meas;

  for (std::list<HSS::RedundantLine2DRep>::iterator iter = 
         doorExtractor.doors().begin();
       iter != doorExtractor.doors().end(); iter++) {

    HSS::Meas z;
    z.description = "door";

    // Pose of the door in the sensor frame
    z.Z[0] = iter->xC();
    z.Z[1] = iter->yC();
    z.Z[2] = iter->theta();

    z.width = 2.0*iter->h();

    // The uncertainty for the door is very large in the direction
    // through the door and not as bit along the threshold of the door
    Eigen::MatrixXd Rxy(2,2);
    Rxy.setZero();
    // We first set the uncertain in door coordinates (x = threshold dir)
    Rxy(0,0) = HSS::sqr(0.1);
    Rxy(1,1) = HSS::sqr(1.0);
    // Now rotate it into the sensor frame
    Eigen::MatrixXd Rot(2,2);
    Rot(0,0) = cos(z.Z[2]);
    Rot(0,1) =-sin(z.Z[2]);
    Rot(1,0) =-Rot(0,1);
    Rot(1,1) = Rot(0,0);
    Rxy = Rot * Rxy * Rot.transpose();

    z.R.setZero();    
    z.R(0,0) = Rxy(0,0);
    z.R(0,1) = z.R(1,0) = Rxy(0,1);
    z.R(1,1) = Rxy(1,1);
    z.R(2,2) = HSS::sqr(HSS::deg2rad(5));

    // Door according to measurement
    Eigen::Vector3d xm = HSS::compound(xs, z.Z);
    
    // Loop over all doors in the map to find a match
    for (unsigned int j = 0; j < m_MapDoors.size(); j++) {
      // Estimated pose of door
      Eigen::Vector3d xdi;
      xdi[0] = m_X[m_MapDoors[j]->getStatePos()+0];
      xdi[1] = m_X[m_MapDoors[j]->getStatePos()+1];
      xdi[2] = m_X[m_MapDoors[j]->getStatePos()+2];

      if (hypot(xdi[0]-xm[0], xdi[1]-xm[1]) < 1) {
        z.statepos = m_MapDoors[j]->getStatePos();
        z.refpos = j;

        Eigen::Vector3d xdS(HSS::compound(HSS::icompound(xs),xdi));
        z.innov = z.Z - xdS;
        z.innov[2] = HSS::pi_to_pi(z.innov[2]);
        // Since the direction of the door is only up to +-pi we can
        // only get innovatios that are +-pi/2 in angle
        if (z.innov[2] < -M_PI_2) z.innov[2] += M_PI;
        if (z.innov[2] > M_PI_2) z.innov[2] -= M_PI;

        break;
      }
    }

    meas.push_back(z);
  }

  for (unsigned int i = 0; i < meas.size(); i++) {

    if (meas[i].getStatePos() < 0) {

      if (m_LocalizeOnly) continue;

      // If the measurement was not matched to anything we extend the
      // statevector to include it
      extendStateDoor(meas[i].Z, xsR, meas[i].width);
      meas[i].description = "newdoor";
      meas[i].statepos = m_MapDoors.back()->getStatePos();
      meas[i].refpos = m_MapDoors.size()-1;
      meas[i].innov.setZero();
      meas[i].R.setIdentity();
      meas[i].R *= 0.1;
    }

    correct(xsR, meas[i]);
    m_MapDoors[meas[i].refpos]->updateWidth(meas[i].width);
  }

  //std::cerr << "Done with update step, now size(X)=" << m_X.size() << std::endl;
  //std::cerr << "After update X=[" << m_X.transpose() << "]\n";
  //std::cerr << "After update\nP=[" << m_P << "]\n";
}

void 
LocalMap::resetRobotPoseState(const Eigen::Vector3d &Xr,
                              const Eigen::Matrix3d &Prr)
{
  if (!hasRobotPose()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::resetRobotPoseState cannot be called for "
              << "map" << getId() << " since there was no robot pose\n";
    throw std::exception();
  }

  // Clear the correlation terms
  int N = m_X.size();
  m_P.block(0,3,3,N-3).setZero();
  m_P.block(3,0,N-3,3).setZero();

  // Set the robot pose and covariance
  for (unsigned int i = 0; i < 3; i++) m_X[i] = Xr[i];
  m_P.block(0,0,3,3) = Prr;

  m_Localized = false;
}

double
LocalMap::minScanner2ScanDist(const Eigen::Vector3d &xsR) const
{
  if (!hasRobotPose()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::minScanner2ScanDist cannot be called for "
              << "map" << getId() << " since there was no robot pose\n";
    throw std::exception();
  }

  // Scanner pose
  Eigen::Vector3d xs(HSS::compound(getXrL(), xsR));

  double minD = 1e10;
  for (unsigned int i = 0; i < m_MapScans.size(); i++) {
    double d = hypot(xs[1] - m_X[m_MapScans[i]->getStatePos()+1],
                     xs[0] - m_X[m_MapScans[i]->getStatePos()+0]);
                     
    if (d < minD) {
      minD = d;
    }
  }

  return minD;
}

int
LocalMap::getScanIndexFromId(int id) const
{
  if (m_MapScans.empty()) {
    std::cerr << "HSS::LocalMap(" << m_Id << ")::getScanIndexFromId no scans!!!\n";
    throw std::exception();
  }

  for (unsigned int i = 0; i < m_MapScans.size(); i++) {
    if (m_MapScans[i]->m_Id == id) return i;
  }

  std::cerr << "HSS::LocalMap(" << m_Id << ")::getScanIndexFromId(" << id << ") id not found\n";
  throw std::exception();
}

double
LocalMap::estimatedOverlap(const HSS::MapScan2D &mapScan, 
                           const HSS::CandScan2D &newScan,
                           const Eigen::Vector3d &xns)
{
  // We approximate the overlap by evaluating a grid of NxN points
  // and check which of these fall within both scan bounding boxes.

  // Pose of mapScan's BB's lower right corner in mapScan frame 
  Eigen::Vector3d mapLL;
  mapLL[0] = mapScan.m_BB(0,0);
  mapLL[1] = mapScan.m_BB(1,0);
  mapLL[2] = mapScan.m_BBdir;

  // Pose of mapScan's BB's lower right corner in map frame
  Eigen::Vector3d xms;
  for (int j = 0; j < 3; j++) xms[j] = m_X[mapScan.getStatePos()+j];
  mapLL = HSS::compound(xms, mapLL);

  // Pose of newScan's BB's lower right corner in newScan frame
  Eigen::Vector3d newLL;
  newLL[0] = newScan.m_BB(0,0);
  newLL[1] = newScan.m_BB(1,0);
  newLL[2] = newScan.m_BBdir;

  // Pose of newScan's BB's lower right corner in map frame
  newLL = HSS::compound(xns, newLL);

  int N = 20;

  // Check how large a fraction of new scan that is in the map scan
  // bounding box
  double overlapNM = 1;
  {
    // Pose of mapScan's BB's lower right corner in newScan's BB's frame
    Eigen::Vector3d ll(HSS::compound(HSS::icompound(newLL), mapLL));
    
    // Direction cosines for the x-axis of mapScan BB in newScan BB frame
    double cosX = cos(ll[2]);
    double sinX = sin(ll[2]);
    // Direction cosines for the y-axis of mapScan BB in newScan BB frame
    double cosY = cos(ll[2]+M_PI_2);
    double sinY = sin(ll[2]+M_PI_2);
    
    double xStep = 0.9999 * newScan.m_BBw / (N-1);
    double yStep = 0.9999 * newScan.m_BBh / (N-1);
    int n = 0;
    for (double x = 0; x <= newScan.m_BBw; x += xStep) {
      for (double y = 0; y <= newScan.m_BBh; y += yStep) {
        double h = distPt2Line(x,y,ll[0],ll[1],cosX, sinX);
        double w = distPt2Line(x,y,ll[0],ll[1],cosY, sinY);
        bool inside = (h <= 0 && h >=-mapScan.m_BBh &&
                       0 <= w && w <= mapScan.m_BBw);
        if (inside) n++;
      }
    }
    overlapNM = (1.0 * n / (N*N));
  }

  // Check how large a fraction of map scan that is in the new scan
  // bounding box
  double overlapMN = 1;
  {
    // Pose of newScan's BB's lower right corner in mapScan's BB's frame
    Eigen::Vector3d ll(HSS::compound(HSS::icompound(mapLL), newLL));
    
    // Direction cosines for the x-axis of newScan BB in mapScan BB frame
    double cosX = cos(ll[2]);
    double sinX = sin(ll[2]);
    // Direction cosines for the y-axis of newScan BB in mapScan BB frame
    double cosY = cos(ll[2]+M_PI_2);
    double sinY = sin(ll[2]+M_PI_2);
    
    double xStep = 0.9999 * mapScan.m_BBw / (N-1);
    double yStep = 0.9999 * mapScan.m_BBh / (N-1);
    int n = 0;
    for (double x = 0; x <= mapScan.m_BBw; x += xStep) {
      for (double y = 0; y <= mapScan.m_BBh; y += yStep) {
        double h = distPt2Line(x,y,ll[0],ll[1],cosX, sinX);
        double w = distPt2Line(x,y,ll[0],ll[1],cosY, sinY);
        bool inside = (h <= 0 && h >=-newScan.m_BBh &&
                       0 <= w && w <= newScan.m_BBw);
        if (inside) n++;
      }
    }
    overlapMN = (1.0 * n / (N*N));
  }

  if (overlapMN < overlapNM) 
    return overlapMN;
  else 
    return overlapNM;
}


void 
LocalMap::displayMap(peekabot::GroupProxy &peekabotRoot,
                     bool globalCoords, bool displayLabels)
{
  peekabot::GroupProxy maproot;
  char name[128];
  sprintf(name, "lmap%02ld", m_Id);
  maproot.add(peekabotRoot, name, peekabot::REPLACE_ON_CONFLICT);

  for (unsigned int i = 0; i < m_MapScans.size(); i++) {

    float r,g,b;
    if (m_UniformColored) {
      HSS::hsv2rgb(m_Id * 35 % 360, 1, 1, &r, &g, &b);
    } else {
      HSS::hsv2rgb(i * 35 % 360, 1, 1, &r, &g, &b);
    }

    // Add the root for this reference scan
    peekabot::GroupProxy refroot;
    char name[128];
    sprintf(name, "scan%02d_id%04ld", i, m_MapScans[i]->m_Id);
    refroot.add(maproot, name, peekabot::REPLACE_ON_CONFLICT);

    Eigen::Vector3d x;
    for (int j = 0; j < 3; j++) x[j] = m_X[m_MapScans[i]->getStatePos()+j];
    Eigen::MatrixXd p(m_P.block(m_MapScans[i]->getStatePos(),
                                m_MapScans[i]->getStatePos(), 3, 3));
    if (globalCoords) {
      Eigen::MatrixXd J;
      x = HSS::compound(m_GlobalT, x, &J);
      Eigen::Matrix3d J1(J.block(0,0,3,3));
      Eigen::Matrix3d J2(J.block(0,3,3,3));
      p = J1*m_GlobalP*J1.transpose() + J2*p*J2.transpose();
    }
    HSS::displayStateWithUnc(refroot, x, p, 
                             0,
                             m_MapScans[i], 1.0, r,g,b);

    //HSS::displayPolygon(refroot, x, m_MapScans[i]->m_BB, r, g, b);

    if (0) {
      peekabot::CylinderProxy cylp;
      sprintf(name, "space");
      cylp.add(refroot, name);
      cylp.set_scale(1, 1, 0.001);
      cylp.set_position(x[0], x[1], 0);
      cylp.set_opacity(0.5);
      cylp.set_color(r,g,b);
    }
      
    if (displayLabels) {
      // Add the ref or id of the scan in the display to make it easy to see which is which
      peekabot::LabelProxy lp;
      lp.add(refroot, "label");
      lp.set_text(name);
      lp.set_pose(x[0], x[1], 0.5, 0, 0, -M_PI_2);
      lp.set_scale(30);    
      lp.set_color(r,g,b);
    }
  }

  for (unsigned int i = 0; i < m_MapDoors.size(); i++) {

    float r,g,b;
    if (m_UniformColored) {
      HSS::hsv2rgb(m_Id * 35 % 360, 1, 1, &r, &g, &b);
    } else {
      HSS::hsv2rgb(i * 35 % 360, 1, 1, &r, &g, &b);
    }

    // Add the root for this reference scan
    peekabot::GroupProxy refroot;
    char name[128];
    sprintf(name, "door%02d", i);
    refroot.add(maproot, name, peekabot::REPLACE_ON_CONFLICT);

    Eigen::Vector3d x;
    for (int j = 0; j < 3; j++) x[j] = m_X[m_MapDoors[i]->getStatePos()+j];
    Eigen::MatrixXd p(m_P.block(m_MapDoors[i]->getStatePos(),
                                m_MapDoors[i]->getStatePos(), 3, 3));
    if (globalCoords) {
      Eigen::MatrixXd J;
      x = HSS::compound(m_GlobalT, x, &J);
      Eigen::Matrix3d J1(J.block(0,0,3,3));
      Eigen::Matrix3d J2(J.block(0,3,3,3));
      p = J1*m_GlobalP*J1.transpose() + J2*p*J2.transpose();
    }

    HSS::displayStateWithUnc(refroot, x, p, 
                             0, NULL, 1.0, r,g,b);

    peekabot::GroupProxy door;
    door.add(refroot, "doorpost");
    door.set_pose(x[0],
                  x[1],
                  0 , 
                  x[2],
                  0, 
                  0);
    
    HSS::addDoorPost(door, m_MapDoors[i]->getWidth(), r,g,b);

    if (displayLabels) {
      // Add the ref or id of the door in the display to make it easy to see which is which
      peekabot::LabelProxy lp;
      lp.add(door, "label");
      lp.set_text(name);
      lp.set_pose(0,0,2.5,0, 0,-M_PI_2);
      lp.set_scale(30);   
      lp.set_color(r,g,b); 
    }
  }
}

}; // namespace HSS
