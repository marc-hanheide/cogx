/**
 * $Id: GestaltPrinciple.hh,v 1.21 2007/02/18 18:02:48 mxz Exp mxz $
 */

#ifndef Z_GESTALT_PRINCIPLE_HH
#define Z_GESTALT_PRINCIPLE_HH

#include "Config.hh"
#include "Gestalt.hh"

namespace Z
{

class VisionCore;

class GestaltPrinciple
{
public:
  enum Type
  {
    FORM_SEGMENTS,
    FORM_LINES,
    FORM_ARCS,
    FORM_PARALLEL_LINE_GROUPS,
    FORM_ARC_JUNCTIONS,
    FORM_CONVEX_ARC_GROUPS,
    FORM_ELLIPSES,
    FORM_JUNCTIONS,
    FORM_CLOSURES,
    FORM_RECTANGLES,
    FORM_FLAPS,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

private:
  double runtime;   ///< runtime in [s]

protected:
  VisionCore *core;
  Array<unsigned> next_principles;  ///< next principles, to be informed of new
                                    // gestalts

  void RankGestalts(Gestalt::Type type,
      int(*compar)(const void *, const void *));

public:
  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);

  GestaltPrinciple(VisionCore *vc);
  virtual ~GestaltPrinciple() {}
  virtual void Reset() {};
  virtual void Operate(bool incremental) {};
  virtual bool NeedsOperate() {return true;}
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx) {}
  virtual void Draw(int detail = 0) {}
  double RunTime() {return runtime;}
  void ResetRunTime() {runtime = 0.;}
  void SetRunTime(double t) {runtime = t;}
  void AddRunTime(double t) {runtime += t;}
};

}

#endif

