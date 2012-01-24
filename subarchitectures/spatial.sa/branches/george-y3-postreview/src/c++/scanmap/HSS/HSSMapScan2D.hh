//
// = FILENAME
//    HSSMapScan2D.hh
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

#ifndef HSSMapScan2D_hh
#define HSSMapScan2D_hh

#include "HSSMapObject.hh"
#include "HSSCandScan2D.hh"

namespace HSS {

/**
 * Map object of type reference scan
 *
 * @author Patric Jensfelt
 * @see
 */
class MapScan2D : public MapObject, 
                  public CandScan2D
{
public:
  MapScan2D(const CandScan2D &src)
    :MapObject(MapObject::TYPE_SCAN, 3)
  {
    *((CandScan2D*)this) = src;
  }

  MapScan2D(int pos, const CandScan2D &src)
    :MapObject(MapObject::TYPE_SCAN, 3, pos)
  {
    *((CandScan2D*)this) = src;
  }
};

};

#endif // HSSMapScan2D_hh
