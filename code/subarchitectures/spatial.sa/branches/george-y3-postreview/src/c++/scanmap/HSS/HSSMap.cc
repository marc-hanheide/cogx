//
// = FILENAME
//    HSSMap.cc
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

#include "HSSMap.hh"

#include <cstdio>
#include <fstream>

namespace HSS {

Map::Map()
{
  m_NextLocalMapId = 0;
  m_CurrLMap = -1;

  m_LMaps.clear();

  m_MaxLMapSize = 5;

  m_SwitchDist = 1;
  m_SwitchHystDist = 0.5;
}

Map::~Map()
{
  finit();
}

void
Map::finit()
{
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    delete m_LMaps[i];
  }
  m_LMaps.clear();

  for (unsigned int i = 0; i < m_Transfs.size(); i++) {
    delete m_Transfs[i];
  }
  m_Transfs.clear();
}

void
Map::init(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    delete m_LMaps[i];
  }
  m_LMaps.clear();
  m_LMaps.push_back(new HSS::LocalMap(m_NextLocalMapId++));
  m_LMaps[0]->init(xsR, scan);
  m_LMaps[0]->setUniformColored(true);
  m_CurrLMap = 0;

  m_LMapsT.resize(1);
  m_LMapsT[0].setZero();
}

void 
Map::init(const std::string &mapfileBasename,
          const Eigen::Vector3d &xsR,
          Eigen::Vector3d *xr, Eigen::Matrix3d *Prr)
{
  // Read the first number in the first file to know how many local
  // maps there are
  char name[256];
  sprintf(name, "%s000.hss", mapfileBasename.c_str());
  std::fstream fs;
  fs.open(name, std::ios::in);
  if (!fs.is_open()) {
    std::cerr << "HSS::Map::init could not open file \""
              << name << "\"\n";
    throw std::exception();
  }

  int M = 0;
  fs >> M;
  std::cerr << "There should be " << M << " local map(s)\n";
  m_LMaps.resize(M);
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    m_LMaps[i] = new LocalMap(-1);

    sprintf(name, "%s%03d.hss", mapfileBasename.c_str(), i);
    m_LMaps[i]->load(name, xsR);
    m_LMaps[i]->setUniformColored(true);
    m_LMaps[i]->setLocalizeOnly(true);
  }

  if (M > 1) {
    std::cerr << "HSS::Map::init Only handling 1 local map, not "
              << M << std::endl;
    throw std::exception();
  }

  m_CurrLMap = 0;
  
  if (xr) {
    Eigen::Matrix3d P;
    if (Prr) {
      P = *Prr;
    } else {
      P.setIdentity();
    }
    m_LMaps[0]->resetRobotPoseState(*xr, P);
  }
}

void
Map::save(const std::string &mapfileBasename)
{
  if (m_LMaps.empty()) {
    std::cerr << "HSSMap::save no maps to save\n";
    return;
  }

  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    char name[256];
    sprintf(name, "%s%03d.hss", mapfileBasename.c_str(), i);
    m_LMaps[i]->save(m_LMaps.size(), name);
  }
}

void 
Map::clearLocalMapAuxBitMaps()
{
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    m_LMaps[i]->setAuxBitMap(0);
  }
}

void 
Map::setAllLocalMapAuxValues(double v)
{
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    m_LMaps[i]->setAuxValue(v);
  }
}

Eigen::VectorXd 
Map::getXrG() const
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::getXrG could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    Eigen::VectorXd ret(3);
    ret.setZero();
    return ret;
  }
  
  return m_LMaps[m_CurrLMap]->getXrG();
}


Eigen::MatrixXd 
Map::getPrrG() const
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::getPrrG could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    Eigen::MatrixXd ret(3,3);
    ret.setIdentity();
    return ret;
  }
  
  return m_LMaps[m_CurrLMap]->getPrrG();
}

int
Map::getIndexFromId(long id)
{
  if (m_LMaps.empty()) {
    std::cerr << "HSS::Map::getIndexFromId you need to call init to create"
              << " first local map before anything else\n";
    throw std::exception();
  }

  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    if (m_LMaps[i]->getId() == id) return i;
  }

  std::cerr << "HSS::Map::getIndexFromId(" << id << ") id not found\n";
  throw std::exception();
}

bool 
Map::predict(const Eigen::Vector3d &delta, const Eigen::Matrix3d &Q)
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::predict could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    return false;
  }

  return m_LMaps[m_CurrLMap]->predict(delta, Q);
}

void 
Map::updateScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::updateScan could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    return;
  }

  m_LMaps[m_CurrLMap]->updateScan(xsR, scan);
}

void 
Map::updateDoor(const Eigen::Vector3d &xsR, HSS::DoorExtractor &doorExtractor)
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::updateDoor could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    return;
  }
  
  m_LMaps[m_CurrLMap]->updateDoor(xsR, doorExtractor);
}

bool
Map::checkForTransitions(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::checkForTransitions could not be done "
              << "since m_CurrLMap=" << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }

  if (!m_LMaps[m_CurrLMap]->hasRobotPose()) {
    std::cerr << "HSS::Map::checkForTransitions error "
              << "localmap " << m_LMaps[m_CurrLMap]->getId()
              << " does not have a robot pose\n";
    throw std::exception();
  }

  if (m_LMaps[m_CurrLMap]->atBorderHeadingIn(xsR)) {
    int next = enteringExistingMap(xsR);
    if (next >= 0) {
      std::cerr << "TIME TO RE_ENTER LOCAL MAP " << next 
                << " FROM LOCAL MAP " << m_CurrLMap
                << std::endl;
      int oldIndex = m_CurrLMap;
      changeToOldLocalMap(xsR, scan, next);
      updateTransitionInfo(oldIndex);
      getchar();

      return true;
    }
  }

  // See if we shold create a new local map. We should do this if we
  // are at the border of the local map (already checked) and it is
  // big enough already
  if (m_MaxLMapSize > 0 &&
      (int)m_LMaps[m_CurrLMap]->m_MapScans.size() >= m_MaxLMapSize) {
    if (m_LMaps[m_CurrLMap]->atBorderHeadingOut(xsR)) {
      std::cerr << "TIME TO CREATE NEW LOCAL MAP AND LEAVE "
                << "LOCAL MAP " << m_CurrLMap
                << std::endl;
      int oldIndex = m_CurrLMap;
      changeToNewLocalMap(xsR, scan);
      updateTransitionInfo(oldIndex);
      //getchar();
      return true;
    }
  }

  return false;
}

int
Map::enteringExistingMap(const Eigen::Vector3d &xsR)
{
  /*
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::enteringExistingMap could not be done "
              << "since m_CurrLMap=" << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }

  if (!m_LMaps[m_CurrLMap]->hasRobotPose()) {
    std::cerr << "HSS::Map::enteringExistingMap error "
              << "localmap " << m_LMaps[m_CurrLMap]->getId()
              << " does not have a robot pose\n";
    throw std::exception();
  }
  
  // Nothing to do since there is only one map, the one we are in
  if (m_LMaps.size() == 1) return -1;
  
  // Pose of robot in local frame
  Eigen::Vector3d xrL(m_LMaps[m_CurrLMap]->getXrL());

  // Pose of sensor in current local frame
  Eigen::Vector3d xsL(HSS::compound(xrL, xsR));

  // Check the landmarks in the other local maps
  double minD = 1e10;
  int closest = -1;

  for (unsigned int i = 0; i < m_LMapObjects.size(); i++) {

    m_Dists[i].scanIndex = -1;
    m_Dists[i].scanD = 1e10;
    m_Dists[i].centerD = hypot(m_LMapsC[i][1] - xsL[1],
                               m_LMapsC[i][0] - xsL[0]);

    if ((int)i == m_CurrLMap) continue;
    
    for (unsigned int j = 0; j < m_LMapObjects[i].size(); j++) {
      
      // We only use scans for this
      if (m_LMapObjects[i][j].getType() != 
          HSS::MapObject::TYPE_SCAN) continue;
      
      // Calculate the distance from sensor to ref scans
      double d = hypot(m_LMapObjects[i][j].xL[1] - xsL[1],
                       m_LMapObjects[i][j].xL[0] - xsL[0]);
      
      if (d < m_Dists[i].scanD) {
        m_Dists[i].scanIndex = j;
        m_Dists[i].scanD = d;
      }
      
      if (d < minD) {
        minD = d;
        closest = i;
      }
    }
    
    if (m_Dists[i].state == DIST_LEAVING) {
      if (m_Dists[i].scanD > m_LMaps[i]->getMinDistToAddRefScan()) {
        m_Dists[i].state = DIST_OUTSIDE;
      }
    }
    
  }

  std::cerr << "Distances: ";
  for (unsigned int i = 0; i < m_Dists.size(); i++) {
    std::cerr << "[i=" << i << " j=" << m_Dists[i].scanIndex 
              << " d=" << m_Dists[i].scanD << " st="
              << m_Dists[i].state << "] ";
  }
  std::cerr << std::endl;
  
  if (closest < 0) {
    std::cerr << "HSS::Map::enteringExistingMap closest<0 should not happen";
    throw std::exception();
  }
  
  if (m_Dists[closest].state == DIST_OUTSIDE &&
      m_Dists[closest].scanD < m_LMaps[closest]->getMinDistToAddRefScan()) {
    return closest;
  }
  */

  return -1;
}

void
Map::changeToNewLocalMap(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan)
{
  if (m_LMaps.empty()) {
    std::cerr << "HSS::Map::changeToNewLocalMap you need to call init to create"
              << " first local map before anything else\n";
    throw std::exception();
  }

  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::changeToNewLocalMap could not be done since "
              << "m_CurrLMap=" << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }
 
  int oldIndex = m_CurrLMap;
  long oldId = m_LMaps[m_CurrLMap]->getId();
  long newId = m_NextLocalMapId++;
 
  // Make sure that this scan is a reference scan in the old local map
  // as well as the new so it can act as the transformation between
  // the maps
  m_LMaps[m_CurrLMap]->addRefScan(xsR, scan);

  m_LMaps.push_back(new HSS::LocalMap(newId));
  m_CurrLMap = m_LMaps.size()-1;
  m_LMaps[m_CurrLMap]->init(xsR, scan);
  m_LMaps[m_CurrLMap]->setUniformColored(true);

  HSS::MapTransformation *t = 
    new HSS::MapTransformation(scan.m_Id, oldId, newId);
  m_LMaps[oldIndex]->m_Transfs.push_back(t);
  m_LMaps[m_CurrLMap]->m_Transfs.push_back(t);
  m_Transfs.push_back(t);

  {
    Eigen::MatrixXd J;
    Eigen::Vector3d xr(HSS::compound(m_LMaps[oldIndex]->m_X, xsR, &J));
    Eigen::Matrix3d J1(J.block(0,0,3,3));
    Eigen::Matrix3d P(J1*m_LMaps[oldIndex]->m_P.block(0,0,3,3)*J1.transpose());

    Eigen::Vector3d glT(m_LMaps[oldIndex]->getGlobalTransfT());
    Eigen::Matrix3d glP(m_LMaps[oldIndex]->getGlobalTransfP());
    
    glT = HSS::compound(glT, xr, &J);
    J1 = J.block(0,0,3,3);
    Eigen::Matrix3d J2(J.block(0,3,3,3));
    glP = J1*glP*J1.transpose() + J2*P*J2.transpose();

    m_LMaps[m_CurrLMap]->setGlobalTransfT(glT);
    m_LMaps[m_CurrLMap]->setGlobalTransfP(glP);
  }
}

void
Map::changeToOldLocalMap(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan,
                         int nextIndex)
{
  if (m_LMaps.size() < 2) {
    std::cerr << "HSS::Map::changeToOldLocalMap you need to call init to create"
              << " first and need at least 2 local maps for this function\n";
    throw std::exception();
  }

  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::changeToOldLocalMap could not be done since "
              << "m_CurrLMap=" << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }
 
  if (nextIndex < 0 || nextIndex >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::changeToOldLocalMap could not be done since "
              << "nextIndex=" << nextIndex << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }
 
  if (nextIndex == m_CurrLMap) {
    std::cerr << "HSS::Map::changeToOldLocalMap no point to enter same map\n";
    throw std::exception();
  }

  int oldIndex = m_CurrLMap;
  long oldId = m_LMaps[m_CurrLMap]->getId();
  long nextId = m_LMaps[nextIndex]->getId();

  HSS::MapTransformation *t = 0;
 
  // First check of there already exist a transformation between these
  // two local maps.
  for (unsigned int i = 0; i < m_LMaps[m_CurrLMap]->m_Transfs.size(); i++) {
    if (m_LMaps[m_CurrLMap]->m_Transfs[i]->getKey() ==
        HSS::MapTransformation::calcKey(oldId, nextId)) {
      t = m_LMaps[m_CurrLMap]->m_Transfs[i];
      break;
    }
  }

  Eigen::Vector3d T;
  Eigen::Matrix3d P;
  bool addedTransf = false;

  if (t == 0) {
    addedTransf = true;

    std::cerr << "Need to create a new connection/transformation between these two places\n";
    // Make sure that the scan exists in both maps
    m_LMaps[oldIndex]->addRefScan(xsR, scan);

    // Add the transformation
    t = new HSS::MapTransformation(scan.m_Id, nextId, oldId);
    m_LMaps[oldIndex]->m_Transfs.push_back(t);
    m_LMaps[nextIndex]->m_Transfs.push_back(t);
    m_Transfs.push_back(t);

    Eigen::Matrix3d Ji;
    T = HSS::icompound(m_LMaps[nextIndex]->getGlobalTransfT(),&Ji);
    P = Ji*m_LMaps[nextIndex]->getGlobalTransfP()*Ji.transpose();
    
    Eigen::MatrixXd J;
    T = HSS::compound(T, m_LMaps[oldIndex]->getGlobalTransfT(), &J);
    Eigen::Matrix3d J1(J.block(0,0,3,3));
    Eigen::Matrix3d J2(J.block(0,3,3,3));
    P = J1*P*J1.transpose() + 
      J2*m_LMaps[oldIndex]->getGlobalTransfP()*J2.transpose();

  } else {

    // Get transformation that allows us to transform stuff expressed
    // in the old local map in the frame of the new one
    if (!getTransformationBetweenMaps(nextIndex, oldIndex, t, T, &P)) {
      std::cerr << "HSS::Map::changeToOldLocalMap logic error, transf should exist\n";
      throw std::exception();
    }

  }

  // Estimate position of the robot pose in the new local map using
  // the transformation
  Eigen::MatrixXd J;
  Eigen::Vector3d Xr(HSS::compound(T, m_LMaps[oldIndex]->getXrL(), &J));
  Eigen::Matrix3d Prr;
  Eigen::Matrix3d J1(J.block(0,0,3,3));
  Eigen::Matrix3d J2(J.block(0,3,3,3));
  Prr = J1*P*J1.transpose() + J2*m_LMaps[oldIndex]->getPrrL()*J2.transpose();

  // Add some uncertainty
  Prr *= 2.0;
  for (unsigned int i = 0; i < 3; i++) Prr(i,i) += 0.01;

  // Switch local map
  m_CurrLMap = nextIndex;

  m_LMaps[m_CurrLMap]->resetRobotPoseState(Xr, Prr);

  if (addedTransf) {
    // Make sure that there is a connecting scan in the new map. The we
    // add with 
    m_LMaps[m_CurrLMap]->addRefScan(xsR, scan);
  }
}

bool
Map::getTransformationBetweenMaps(long fromMapId, long toMapId,
                                  Eigen::Vector3d &T,
                                  Eigen::Matrix3d *P)
{
  for (unsigned int i = 0; i < m_Transfs.size(); i++) {

    if (m_Transfs[i]->getKey() ==
        HSS::MapTransformation::calcKey(fromMapId, toMapId)) {

      return getTransformationBetweenMaps(fromMapId, toMapId, 
                                          m_Transfs[i], T, P);

    }

  }

  // If we get here there was no connection between these two maps
  return false;
}

bool
Map::getTransformationBetweenMaps(long fromMapId, long toMapId,
                                  HSS::MapTransformation *t,
                                  Eigen::Vector3d &T,
                                  Eigen::Matrix3d *P)
{
  if (t == 0) {
    std::cerr << "Map::getTransformationBetweenMaps t==0!!!!\n";
    throw std::exception();
  }

  if (t->getKey() == HSS::MapTransformation::calcKey(fromMapId, toMapId)) {

    int mIndex1 = getIndexFromId(fromMapId);
    int mIndex2 = getIndexFromId(toMapId);

    int sIndex1 = m_LMaps[mIndex1]->getScanIndexFromId(t->getScanId());
    int sIndex2 = m_LMaps[mIndex2]->getScanIndexFromId(t->getScanId());

    HSS::MapScan2D *scan1 = m_LMaps[mIndex1]->m_MapScans[sIndex1];
    HSS::MapScan2D *scan2 = m_LMaps[mIndex2]->m_MapScans[sIndex2];

    Eigen::Vector3d pos1 = scan1->getVector3(m_LMaps[mIndex1]->m_X);
    Eigen::Vector3d pos2 = scan2->getVector3(m_LMaps[mIndex2]->m_X);

    Eigen::Matrix3d P1 = scan1->getMatrix3(m_LMaps[mIndex1]->m_P);
    Eigen::Matrix3d P2 = scan2->getMatrix3(m_LMaps[mIndex2]->m_P);

    Eigen::Matrix3d iJ;
    Eigen::Vector3d ipos2 = HSS::icompound(pos2, &iJ);

    Eigen::MatrixXd J;
    T = HSS::compound(pos1, ipos2, &J);

    if (P) {
      Eigen::Matrix3d iP2 = iJ*P2*iJ.transpose();
      Eigen::Matrix3d J1(J.block(0,0,3,3));
      Eigen::Matrix3d J2(J.block(0,3,3,3));
      *P = J1*P1*J1.transpose() + J2*iP2*J2.transpose();
    }

    return true;
  }

  // If we get here the MapTransformation object is not correct for these mapIds
  std::cerr << "Map::getTransformationBetweenMaps transform wrong, connecting "
            << t->getMap1Id() << " and " << t->getMap2Id() << " and not "
            << fromMapId << " and " << toMapId << std::endl;
  throw std::exception();
}

void
Map::updateTransitionInfo(int fromIndex)
{
  std::cerr << "HSS::Map::updateTransitionInfo called\n";

  if (m_LMaps.empty()) {
    std::cerr << "HSS::Map::updateTransitionInfo you need to call init to "
              << "create first local map before anything else\n";
    throw std::exception();
  }
  
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::updateTransitionInfo could not be done since "
              << "m_CurrLMap=" << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }
 
  if (!m_LMaps[m_CurrLMap]->hasRobotPose()) {
    std::cerr << "HSS::Map::updateTransitionInfo error "
              << "localmap " << m_LMaps[m_CurrLMap]->getId()
              << " does not have a robot pose\n";
    throw std::exception();
  }

  // Do a complete breadth first search throw the local maps to find
  // the closest distance, in number of transitions from the current
  // map to all other maps
  setAllLocalMapAuxValues(-1);
  Eigen::Vector3d T;
  T.setZero();
  m_LMapsT.resize(m_LMaps.size());
  wavePropagation(m_CurrLMap, 0, T);

  buildInternals(fromIndex);

  std::cerr << "Result of wave propagation:\n";
  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    std::cerr << "lmap(i=" << i << ", id=" << m_LMaps[i]->getId()
              << ") has cost=" << m_LMaps[i]->getAuxValue()
              << " T=[" << m_LMapsT[i].transpose() << "]\n";
  }
}

void
Map::wavePropagation(int mapIndex, double cost, const Eigen::Vector3d &T)
{
  if (mapIndex < 0 || mapIndex >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::wavePropagation could not be done since "
              << "mapIndex=" << mapIndex << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    throw std::exception();
  }

  m_LMaps[mapIndex]->setAuxValue(cost);
  m_LMapsT[mapIndex] = T;

  double nextCost = cost + 1;

  // Now go through the connected local maps and see if any of them
  // have not been visited before or if we have gotten there with a
  // lower cost than before
  for (unsigned int c = 0; c < m_LMaps[mapIndex]->m_Transfs.size(); c++) {
    HSS::MapTransformation *t = m_LMaps[mapIndex]->m_Transfs[c];    
    int index = getIndexFromId(t->getOtherId(m_LMaps[mapIndex]->getId()));
    if (m_LMaps[index]->getAuxValue() < 0 ||       // Not vistited before
        m_LMaps[index]->getAuxValue() > nextCost) {// Better way to get there

      Eigen::Vector3d newT;
      // Get the transformation between the current and the next map
      getTransformationBetweenMaps(m_LMaps[mapIndex]->getId(), 
                                   m_LMaps[index]->getId(), newT);
      newT = HSS::compound(T, newT);

      wavePropagation(index, nextCost, newT);
    }
  }
}

void
Map::buildInternals(int fromIndex)
{
  if (m_LMaps.empty()) {
    std::cerr << "HSS::Map::updateOtherMapObjects you need to call init to "
              << "create first local map before anything else\n";
    throw std::exception();
  }
  
  if (m_LMapsT.size() != m_LMaps.size()) {
    std::cerr << "HSS::Map::updateOtherMapObjects must make sure to call "
              << "updateTransitionInfo()\n";
    throw std::exception();
  }
  
  m_LMapObjects.clear();
  m_LMapObjects.resize(m_LMaps.size());
  m_LMapsC.clear();
  m_LMapsC.resize(m_LMaps.size());
  m_Dists.clear();
  m_Dists.resize(m_LMaps.size());

  for (unsigned int i = 0; i < m_LMaps.size(); i++) {

    m_LMapsC[i].setZero();

    // Do not add any objects from the current map since this is being
    // changed and we take those directly from this map instead
    if ((int)i == m_CurrLMap) {
      m_Dists[i].state = DIST_INSIDE;
      continue;
    } else if ((int)i == fromIndex) {
      m_Dists[i].state = DIST_LEAVING;
    } else {
      m_Dists[i].state = DIST_OUTSIDE;
    }

    int n = 0;
    for (unsigned int j = 0; j < m_LMaps[i]->m_MapObjects.size(); j++) {

      MapObject *mo = m_LMaps[i]->m_MapObjects[j];

      // Skip the robot pose
      if (mo->getType() == HSS::MapObject::TYPE_ROBOTPOSE) continue;

      OtherMapObject omo(*mo);
      omo.mapIndex = i;
      omo.mapId = m_LMaps[i]->getId();
      omo.xL = HSS::compound(m_LMapsT[i], mo->getVector3(m_LMaps[i]->m_X));

      // Add one more ref.scan to the calc of the center of the map
      if (mo->getType() == HSS::MapObject::TYPE_SCAN) {
        m_LMapsC[i][0] += omo.xL[0];
        m_LMapsC[i][1] += omo.xL[1];
        n++;
      }

      m_LMapObjects[i].push_back(omo);
    }

    if (n > 0) {
      // Calc the average
      m_LMapsC[i][0] /= n;
      m_LMapsC[i][1] /= n;
    } else {
      // This should never happen since we also init a new local map
      // with a reference scan
      std::cerr << "HSS::Map::buildMapObjectList ERROR no ref. scans!!!\n";
      throw std::exception();
    }
  }
}

void 
Map::displayMap(peekabot::GroupProxy &peekabotRoot, bool displayLabels)
{
  if (m_CurrLMap < 0 || m_CurrLMap >= (int)m_LMaps.size()) {
    std::cerr << "HSS::Map::displayMap could not be done since m_CurrLMap="
              << m_CurrLMap << " is outside [0,"
              << (int)m_LMaps.size()-1 << "]\n";
    return;
  }

  for (unsigned int i = 0; i < m_LMaps.size(); i++) {
    m_LMaps[i]->displayMap(peekabotRoot, true, displayLabels);
  }  
}

void
Map::displayLandmarksInSameFrame(peekabot::GroupProxy &peekabotRoot)
{
  peekabot::GroupProxy gp;
  gp.add(peekabotRoot, "landmarks", peekabot::REPLACE_ON_CONFLICT);
  
  // Display all the scan positions in the frame of the current scan
  for (unsigned int i = 0; i < m_LMapsT.size(); i++) {
    for (unsigned int j = 0; j < m_LMaps[i]->m_MapScans.size(); j++) {

      Eigen::Vector3d x(m_LMaps[i]->m_MapScans[j]->getVector3(m_LMaps[i]->m_X));
      
      // Transform into common frame of reference
      x = HSS::compound(m_LMapsT[i], x);
      
      peekabot::CubeProxy cp;
      char name[128];
      sprintf(name, "lmap%02d:scan%02d", i, j);
      cp.add(gp, name);
      cp.set_scale(0.3, 0.1, 0.01);
      cp.set_pose(x[0], x[1], 0, x[2], 0, 0);      
      cp.set_color(0,1,0);
    }
  }

  {
    // Display the robot pose in the local frame
    {
      peekabot::CubeProxy cp;
      char name[128];
      sprintf(name, "lmap%02d:robotpose", m_CurrLMap);
      cp.add(gp, name);
      cp.set_scale(0.5, 0.125, 0);
      Eigen::Vector3d x = m_LMaps[m_CurrLMap]->m_MapObjects[0]->getVector3(m_LMaps[m_CurrLMap]->m_X);
      cp.set_pose(x[0], x[1], 0.01, x[2], 0, 0);
      cp.set_color(0,1,0);
    }

    // Display objects in all maps except the current one
    {
      for (unsigned int i = 0; i < m_LMapObjects.size(); i++) {
        for (unsigned int j = 0; j < m_LMapObjects[i].size(); j++) {

          OtherMapObject &omo = m_LMapObjects[i][j];

          peekabot::CubeProxy cp;
          char name[128];
          sprintf(name, "lmap%02d:%02d:t%d", 
                  omo.mapIndex, omo.getStatePos(), omo.getType());
          cp.add(gp, name);
          cp.set_scale(0.2, 0.075, 0.01);
          cp.set_pose(omo.xL[0], omo.xL[1], 0.01, omo.xL[2], 0, 0);
          cp.set_color(1,0,0);
        }
      }
    }

    // Display objects in current local map
    {
      std::vector<MapObject*>::iterator iter;
      for (iter = m_LMaps[m_CurrLMap]->m_MapObjects.begin();
           iter != m_LMaps[m_CurrLMap]->m_MapObjects.end(); iter++) {

        if ((*iter)->getType() == HSS::MapObject::TYPE_ROBOTPOSE) continue;

        peekabot::CubeProxy cp;
        char name[128];
        sprintf(name, "lmap%02d:%02d:t%d", 
                m_CurrLMap, (*iter)->getStatePos(), (*iter)->getType());
        cp.add(gp, name);
        cp.set_scale(0.2, 0.075, 0.01);
        Eigen::Vector3d x = (*iter)->getVector3(m_LMaps[m_CurrLMap]->m_X);
        cp.set_pose(x[0], x[1], 0.01, x[2], 0, 0);
        cp.set_color(1,0,0);        
      }
    }
  }
}

}; // namespace HSS
