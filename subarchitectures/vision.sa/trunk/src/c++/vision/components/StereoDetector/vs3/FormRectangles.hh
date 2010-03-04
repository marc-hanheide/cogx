/**
 * @file FormRectangle.hh
 * @author Richtsfeld Andreas
 * @date October 2009
 * @version 0.1
 * @brief Header of Gestalt principle class for forming rectangles.
 **/

#ifndef Z_FORM_RECTANGLES_HH
#define Z_FORM_RECTANGLES_HH

#include "GestaltPrinciple.hh"
#include "LJunction.hh"

namespace Z
{

/**
 * @brief Gestalt principle class FormRectangles
 */
class FormRectangles : public GestaltPrinciple
{
private:

	/**
	* @brief TempLine
	*/
	struct TempLine
	{
		Vector2 p;  ///< some point on the line
		Vector2 d;  ///< direction of the line
		TempLine(float px, float py, float dx, float dy) : p(px, py), d(dx, dy) {}
	};

  void Rank();
  void Mask();

  void CreateWithFourLJ(unsigned clos);
  void CreateWithMoreLJLine(unsigned clos);
  void CreateWithMoreLJAngle(unsigned clos);
  void CreateQuadrilateral(unsigned clos);


  bool RectangleExists(unsigned jcts[4]);
  bool IsConvexPolygon(unsigned jcts[4]);
  bool IsConvexPolygon(Array<unsigned> jcts);
  bool IsConvexPolygon(Vector2 isct[4]);
  double IsRectangle(unsigned jcts[4]);
  double IsRectangle(Vector2 isct[4]);


public:
  FormRectangles(VisionCore *vc);
//  virtual void Reset(const Image *img);
  virtual bool NeedsOperate(); 
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif
