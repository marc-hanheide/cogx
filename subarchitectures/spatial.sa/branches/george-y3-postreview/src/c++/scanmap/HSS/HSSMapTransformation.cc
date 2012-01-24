//
// = FILENAME
//    HSSMapTransformation.cc
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

#include "HSSMapTransformation.hh"

namespace HSS {

MapTransformation::MapTransformation(long scanId, long map1Id, long map2Id)
{
  m_ScanId = scanId;
  m_Map1Id = map1Id;
  m_Map2Id = map2Id;
  m_Key = calcKey(map1Id, map2Id);
}

long
MapTransformation::getOtherId(long mapId) const
{
  if (mapId != m_Map1Id && mapId != m_Map2Id) {
    std::cerr << "HSSMapTransformation::getOtherId mapId="
              << mapId << " is none of ("
              << m_Map1Id << ", " << m_Map2Id << ")\n";
    throw std::exception();
  }

  if (mapId == m_Map1Id) return m_Map2Id;
  else return m_Map1Id;
}
  
}; // namespace HSS

