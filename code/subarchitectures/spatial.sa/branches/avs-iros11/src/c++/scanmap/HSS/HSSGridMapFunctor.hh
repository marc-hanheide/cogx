//
// = FILENAME
//    HSSGridMapFunctor.hh
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

#ifndef HSS_GridMapFunctor_hh
#define HSS_GridMapFunctor_hh

namespace HSS {

/**
 * Helper class that defines the values for unknown, free and occupied
 * in the grid map.
 *
 * @author Patric Jensfelt
 * @see
 */
template <class MAPDATA>
class GridMapFunctor {
public:
  virtual MAPDATA getUnknownValue() const = 0;
  virtual MAPDATA getFreeValue() const = 0;
  virtual MAPDATA getOccupiedValue() const = 0;
  
  virtual bool isUnknownValue(const MAPDATA &d) const 
  { return (d==getUnknownValue()); }

  virtual bool isFreeValue(const MAPDATA &d) const 
  { return (d==getFreeValue()); }

  virtual bool isOccupiedValue(const MAPDATA &d) const 
  { return (d==getOccupiedValue()); }

  virtual double getBelief(const MAPDATA &d) const
  {
    if (d==getOccupiedValue()) return 1;
    else if (d==getFreeValue()) return 0;
    else return 0.5;
  }
};

/**
 * Class that defines the values for unknown, free and occupied for
 * the grid map with char values.
 *
 * @author Patric Jensfelt
 * @see
 */
class CharGridMapFunctor : public GridMapFunctor<char> {
public:
  char getUnknownValue() const { return '?'; }
  char getFreeValue() const { return ' '; }
  char getOccupiedValue() const { return 'X'; }
};

}; // namespace HSS

#endif // HSS_GridMapFunctor_hh
