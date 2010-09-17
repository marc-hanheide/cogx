/**
 * @file FormRectangle.hh
 * @author Richtsfeld Andreas
 * @date Februar 2008
 * @version 0.1
 * @brief Header of Gestalt principle class for forming rectangles.
 **/

#ifndef Z_FORM_RECTANGLES_HH
#define Z_FORM_RECTANGLES_HH

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Gestalt principle class FormRectangles
 */
class FormRectangles : public GestaltPrinciple
{
private:

//   class RectCand
//   {
//   public:
//     unsigned base, right, jct[2];
//     RectCand() {}
//     RectCand(unsigned b, unsigned r, unsigned left_jct, unsigned right_jct)
//     {
//       base = b;
//       right = r;
//       jct[LEFT] = left_jct;
//       jct[RIGHT] = right_jct;
//     }
//     RectCand(unsigned r) : right(r) {}
//     bool operator==(const RectCand &rc) {return right == rc.right;}
//   };
// 
//   static const int HASH_SIZE = 29989;  // TODO: this is quite arbitrary
//   unsigned hashtable[HASH_SIZE];
// 
//   // TODO: replace array of arrays by some efficient linked list.
//   Array< Array<RectCand> > u_left;

  void Rank();
  void Mask();
//  unsigned Hash(unsigned lines[4]);
//  void ClearHashTable();
  void CreateQuadrilateral(unsigned clos);
  bool RectangleExists(unsigned jcts[4]);
  bool IsConvexPolygon(unsigned jcts[4]);
  bool IsConvexPolygon(Array<unsigned> jcts);
  double IsRectangle(unsigned ljcts[4]);
//  void HaveNewLJunction(unsigned idx);
//  void StepOperate();

public:
  FormRectangles(Config *cfg);
//  virtual void Reset(const Image *img);
  virtual bool NeedsOperate(); 
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif
