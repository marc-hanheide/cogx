/**
 * $Id: Gestalt.hh,v 1.22 2007/02/18 18:02:48 mxz Exp mxz $
 * TODO: - function AssignIDs(), call whenever an element was deleted or moved
 */

#ifndef Z_GESTALT_HH
#define Z_GESTALT_HH

#include "Namespace.hh"
#include "Array.hh"

namespace Z
{

class VisionCore;

class Gestalt
{
public:
  enum Type
  {
    SEGMENT,
    LINE,
    ARC,
    PARALLEL_LINE_GROUP,
    CONVEX_ARC_GROUP,
    ELLIPSE,
    COLLINEARITY,
    T_JUNCTION,
    L_JUNCTION,
    A_JUNCTION,
    CLOSURE,
    RECTANGLE,
    FLAP,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:
  VisionCore *core;
  Type type;
  unsigned id;
  unsigned rank;

public:
  double acc;  ///< probability of accidental occurrance
  double sig;  ///< significance
  unsigned masked;  // TODO: might be masked by more gestalts

public:
  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);

  Gestalt(VisionCore *c, Type t);
  virtual ~Gestalt() {}
  Type GetType() const {return type;}
  unsigned ID() const {return id;}
  void SetID(unsigned i) {id = i;}
  unsigned Rank() {return rank;}
  void SetRank(unsigned r) {rank = r;}
  void Mask(unsigned by) {masked = by;}
  bool IsMasked() {return masked != UNDEF_ID;}
  bool IsUnmasked() {return masked == UNDEF_ID;}
  virtual void Draw(int detail = 0) {}
  virtual void DrawInfo() {}
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y) {return false;}
};

}

#endif

