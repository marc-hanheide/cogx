//
// = FILENAME
//    HSSMapRobotPose.hh
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

#ifndef HSSMapRobotPose_hh
#define HSSMapRobotPose_hh

#include "HSSMapObject.hh"

namespace HSS {

/**
 * Map objects of type robot pose
 *
 * @author Patric Jensfelt
 * @see
 */
class MapRobotPose : public MapObject
{
public:
  MapRobotPose(int pos)
    :MapObject(MapObject::TYPE_ROBOTPOSE, 3, pos)
  {
    if (pos != 0) {
      std::cerr << "HSS::MapRobotPose(" << pos << ") WARNING pos="
                << pos << "!=0\n";
    }
  }

protected:
};

}; // namespace HSS

#endif // HSSMapRobotPose_hh
