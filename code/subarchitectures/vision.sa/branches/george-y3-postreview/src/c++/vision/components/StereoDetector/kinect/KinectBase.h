/**
 * @file KinectBase.h
 * @author Andreas Richtsfeld
 * @date Mai 2011
 * @version 0.1
 * @brief Base class for caculation of Gestalts from kinect data.
 */

#ifndef Z_KINECT_BASE_H
#define Z_KINECT_BASE_H

#include "VisionCore.hh"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"

namespace Z
{

// Enable pruning thresholds for lines
// static const bool SC_USE_LINE_THRESHOLDS = true;

class KinectCore;  // forward declaration necessary

/**
 * @class KinectBase
 * @brief Base class for all kinect principle calculations.
 */
class KinectBase
{
public:
  enum Type
  {
    KINECT_PATCHES,
    KINECT_SEGMENTS,
    KINECT_LINES,
    KINECT_COLLINEARITIES,
    KINECT_CLOSURES,
    KINECT_RECTANGLES,
    KINECT_PCL_MODELS,
    KINECT_PCL_EDGELS,
    KINECT_PCL_SEGMENTS,
    KINECT_PCL_LINES,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };                                          ///< Type of Kinect Gestalts

protected:
  KinectCore *kcore;                          ///< Kinect core
  VisionCore *vcore;                          ///< Left and right vision core
  Type type;                                  ///< KinectBase Type

private:
  bool enabled;                               ///< Enabled / disabled Kinect-Gestalt
  bool masking;                               ///< TODO masking

public:
  KinectBase(KinectCore *kc, VisionCore *vc);
  void EnablePrinciple(bool status);
  bool IsEnabled() {return enabled;}

  static Type EnumType(const char *type_name);
  static const char* TypeName(Type t);
  static const int KinectTypeNameLength(Type t);

  // virtual functions for the stereo classes.
  virtual int Num3DGestalts() = 0;
// //  virtual void Draw(int side, bool masked = false) = 0;//{}				// TODO pure virtual setzen																			/// TODO Sollten alle pure virtual (=0) sein.
//   virtual void DrawMatched(int side, bool single, int id, int detail) = 0;
  virtual void Process() = 0;
  virtual void ClearResults() = 0;
  virtual void Get3DGestalt(Array<double> &values, int id) = 0;

};

}

#endif
