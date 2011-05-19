/**
 * $Id: EJunction.hh,v 1.0 2007/12/19 13:47:03 mxz Exp mxz $
 */

#ifndef Z_EJUNCTION_HH
#define Z_EJUNCTION_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class EJunction : public Gestalt
{
public:
  unsigned line;			// line
  unsigned ellipse;		// ellipse
  unsigned lineEnd;		// line end (START/END)
  unsigned vertex; 		// vertex of ellipse-junction (LEFT/RIGHT)
  Vector2 isct;				// intersection of search-lines
  double gap[2]; 			// gap line[0], ellipse[1] and intersection

  EJunction(unsigned l, unsigned e, unsigned end, unsigned ver, 
			Vector2 inter, double *g);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  void DrawVotes();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& EJunctions()
{
  return Gestalts(Gestalt::E_JUNCTION);
}
inline EJunction* EJunctions(unsigned id)
{
  return (EJunction*)Gestalts(Gestalt::E_JUNCTION, id);
}
inline unsigned NumEJunctions()
{
  return NumGestalts(Gestalt::E_JUNCTION);
}

}

#endif
