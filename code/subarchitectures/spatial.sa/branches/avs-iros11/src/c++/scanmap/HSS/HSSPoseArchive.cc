//
// = FILENAME
//    PoseArchive.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2006 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "AddressBank/PoseArchive.hh"

#include "Utils/CureDebug.hh"
#include "Utils/HelpFunctions.hh"

namespace Cure {

PoseArchive::PoseArchive(int length)
  :DataSlot(length)
{}

PoseArchive::~PoseArchive()
{}

int 
PoseArchive::addPose(const Cure::Pose3D &p)
{
  m_Mutex.lock();

  // We require that the poses are added with monotonically increasing
  // timestamps
  if (getNumItems() > 0 &&
      p.getTime() < m_LastAddedPose.getTime()) {
    m_Mutex.unlock();
    CureCERR(30) << "New pose with timestamp " << p.getTime() 
                 << " is older than last added ("
                 << m_LastAddedPose.getTime()
                 << "), skipping it\n";
    return 1;
  }

  m_LastAddedPose = p;

  Cure::Pose3D tmp(p); // To get around the fact that DatSlot
                       // does not except const arguments
  write(tmp);
  m_Mutex.unlock();
  return 0;
}

int  
PoseArchive::getInterpolation(const Cure::Timestamp &t, Cure::Pose3D &p)
{
  m_Mutex.lock();
  int err = read(p, t);
  m_Mutex.unlock();

  return err;
}

int  
PoseArchive::getExtrapolation(const Cure::Timestamp &t, Cure::Pose3D &p,
                              const double maxT)
{
  if (maxT <= 0) return getInterpolation(t,p);

  m_Mutex.lock();

  int err = read(p, t);

  // Check if we have to actually perform the extrapolation
  if (err & TIMESTAMP_IN_FUTURE) {
    
    if (DataSlot::getNumItems() < 2) {
      CureCERR(30) << "WARNING: Asking for extrapolation but have only "
                   << getNumItems() << " data point\n";
      m_Mutex.unlock();
      return (err | NO_DATA_AVAILABLE);
    }
  
    Cure::Timestamp eTime(t - m_LastAddedPose.getTime());
    if (eTime.getDouble() > maxT) {
      m_Mutex.unlock();
      CureCERR(30) << "WARNING: Asking for extrapolation "
                   << eTime << "s into the future, only allow "
                   << maxT << "s\n";
      return err;
    }

    // Get hold of the second last item in the Slot
    Cure::Pose3D p1;
    History[(Last - 1 + Size) % Size].setEqual(p1);
    
    err = calcExtrapolation(t, p1, m_LastAddedPose, p);
  }

  m_Mutex.unlock();

  return err;
}

int
PoseArchive::calcExtrapolation(const Cure::Timestamp &t, 
                               const Cure::Pose3D &p1,
                               const Cure::Pose3D &p2,
                               Cure::Pose3D &p)
{
  double p1Ang[3], p2Ang[3];
  p1.getAngles(p1Ang);
  p2.getAngles(p2Ang);
  
  // Estimate the difference (the speed for the 6 axes)
  Cure::Pose3D diffP;
  double diffAng[3];
  diffP.setX(p2.getX() - p1.getX());
  diffP.setY(p2.getY() - p1.getY());
  diffP.setZ(p2.getZ() - p1.getZ());
  diffAng[0] = Cure::HelpFunctions::angleDiffRad(p2Ang[0], p1Ang[0]);
  diffAng[1] = Cure::HelpFunctions::angleDiffRad(p2Ang[1], p1Ang[1]);
  diffAng[2] = Cure::HelpFunctions::angleDiffRad(p2Ang[2], p1Ang[2]);
  diffP.setAngles(diffAng);

  Cure::Timestamp dt12(p2.getTime() - p1.getTime());
  Cure::Timestamp eTime(t - p2.getTime());
  double scale = eTime.getDouble() / dt12.getDouble();
  
  p.setX(p2.getX() + scale * diffP.getX());
  p.setY(p2.getY() + scale * diffP.getY());
  p.setZ(p2.getZ() + scale * diffP.getZ());
  double pAng[3];
  pAng[0] = p2Ang[0] + scale * diffAng[0];
  pAng[1] = p2Ang[1] + scale * diffAng[1];
  pAng[2] = p2Ang[2] + scale * diffAng[2];
  p.setAngles(pAng);
  p.setTime(t);

  // Approximate the covariance to be the same as the last pose in the
  // archive
  p.setCovType(p2.getCovType());
  p.Covariance = p.Covariance;
  
  return 0;
}

int  
PoseArchive::getPoseInterpolation(const Cure::Pose3D &pOld, 
                                  const Cure::Timestamp &tNew,
                                  Cure::Pose3D &pNew)
{
  return getPoseExtrapolation(pOld, tNew, pNew, 0);
}

int  
PoseArchive::getPoseExtrapolation(const Cure::Pose3D &pOld, 
                                  const Cure::Timestamp &tNew,
                                  Cure::Pose3D &pNew,
                                  const double maxT)
{
  // We begin by trying to get the archive pose at the time of the
  // first pose
  Cure::Pose3D archOld, archNew;
  int err = getExtrapolation(pOld.getTime(), archOld, maxT);
  if (err) return err;

  // Then we get the archive pose of the new pose
  err = getExtrapolation(tNew, archNew, maxT);
  if (err) return err;

  // Now calculate the difference in archive pose from the time of the
  // old pose to the new one we look for
  Cure::Pose3D diffArch;
  diffArch.minusPlus(archOld, archNew);
  
  // Now add the archive pose difference to the old pose and voila we
  // have the new pose in the same coordinate system as the old
  // (assuming that the transformation been archive pose and the other
  // pose is fixed).
  pNew.add(pOld, diffArch);
  pNew.setTime(tNew);
  return 0;
}

int
PoseArchive::getTransformationToArchive(const Cure::Pose3D &p2,
                                          Cure::Pose3D &T)
{
  Cure::Pose3D p1;
  Cure::Pose3D p2const(p2);
  int err = getInterpolation(p2const.getTime(), p1);
  if (err) return err;
  T.minusPlus_(p2const, p1);
  return 0;
}

int
PoseArchive::getTransformationFromArchive(const Cure::Pose3D &p2,
                                          Cure::Pose3D &T)
{
  Cure::Pose3D p1;
  Cure::Pose3D p2const(p2);
  int err = getInterpolation(p2const.getTime(), p1);
  if (err) return err;
  T.minusPlus_(p1, p2const);
  return 0;
}

}; // namespace Cure
