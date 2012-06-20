/**
 * @file KinectBase.cpp
 * @author Andreas Richtsfeld
 * @date Mai 2011
 * @version 0.1
 * @brief Base class for caculation of Gestalts from kinect data.
 */

#include "KinectBase.h"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "KINECT_PATCHES",
  "KINECT_SEGMENTS",
  "KINECT_LINES",
  "KINECT_COLLINEARITIES",
  "KINECT_CLOSURES",
  "KINECT_RECTANGLES",
  "KINECT_PCL_MODELS",
  "KINECT_PCL_EDGELS",
  "KINECT_PCL_SEGMENTS",
  "KINECT_PCL_LINES",
  "UNDEF"
  };

static const int stereo_type_names_length[] = {13, 14, 11, 21, 14, 16, 17, 17, 18, 15, 5};

/**
 * @brief Returns the length of the name of a given stereo type.
 * @param t Stereo type
 * @return Lenght of Gestalt type name.
 */
const int KinectBase::KinectTypeNameLength(Type t)
{
  return stereo_type_names_length[t];
}

/**
 * @brief Returns the name of a given gestalt type.
 * @param t Type of the stereo principle
 */
const char* KinectBase::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Return the enum type of a given gestalt type name.
 * @param type_name Name of the kinect principle.
 */
KinectBase::Type KinectBase::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}


/**
 * @brief Constructor of the kinect base.
 */
KinectBase::KinectBase(KinectCore *kc, VisionCore *vc)
{
  kcore = kc;
  vcore = vc;
  enabled = false;                      // disabled by default
}

/**
 * @brief Enables or disables the stereo principle
 * @param status True to enable, false to disable principle.
 */
void KinectBase::EnablePrinciple(bool status)
{
  enabled = status;
}


}


