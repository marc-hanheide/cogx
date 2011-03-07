//
// = FILENAME
//    HSSMapTransformation.hh
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

#ifndef HSSMapTransformation_hh
#define HSSMapTransformation_hh

#include "Eigen/Core"
#include "HSSutils.hh"

namespace HSS {

class MapTransformation
{
public:

  MapTransformation(long scanId, long map1Id, long map2Id);

  long long getKey() const { return m_Key; }

  static long long calcKey(long map1Id, long map2Id)
  {
    if (map1Id > map2Id) {
      return 10000*map1Id + map2Id;
    } else {
      return 10000*map2Id + map1Id;
    }
  }

  long getScanId() const { return m_ScanId; }
  long getMap1Id() const { return m_Map1Id; }
  long getMap2Id() const { return m_Map2Id; }

  /**
   * This function will return he other map id given one of them for the
   * transformation. Will throw if the provided mapId is none of the
   * ones for this transformation.
   */
  long getOtherId(long mapId) const;

protected:
  long m_ScanId;
  long m_Map1Id;
  long m_Map2Id;
  long long m_Key;
};

}; // namespace HSS

#endif // HSSMapTransformation_hh
