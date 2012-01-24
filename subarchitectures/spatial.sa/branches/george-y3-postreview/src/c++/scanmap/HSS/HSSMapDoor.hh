//
// = FILENAME
//    HSSMapDoor.hh
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

#ifndef HSSMapDoor_hh
#define HSSMapDoor_hh

#include "HSSMapObject.hh"

namespace HSS {

/**
 * Map objects of type door
 *
 * @author Patric Jensfelt
 * @see
 */
class MapDoor : public MapObject
{
public:
  MapDoor(double with)
    :MapObject(MapObject::TYPE_DOOR, 3),
     m_Width(with)
  {}

  MapDoor(int pos, double with)
    :MapObject(MapObject::TYPE_DOOR, 3, pos),
     m_Width(with)
  {}

  double getWidth() const { return m_Width; }
  void setWidth(double w) { m_Width = w; }

  void updateWidth(double meas) 
  {
    double alpha = 0.95;
    m_Width = alpha * m_Width + (1-alpha) * meas;
  }

protected:

  double m_Width;
};

}; // namespace HSS

#endif // HSSMapDoor_hh
