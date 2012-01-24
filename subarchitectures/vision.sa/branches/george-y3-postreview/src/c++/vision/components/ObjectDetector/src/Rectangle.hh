/**
 * @file Rectangle.hh
 * @author Richtsfeld Andreas
 * @date Februar 2008
 * @version 0.1
 * @brief Header of class rectangles
 **/

#ifndef Z_RECTANGLE_HH
#define Z_RECTANGLE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

/**
 * @brief Class of Gestalt Rectangle
 */
class Rectangle : public Gestalt
{
private:
  void CalculateSignificance();
	double CalculateSupport();
  void GetRectPixels();
	void GetLinePixels(int x1, int y1, int x2, int y2, unsigned id);
	void SetPixel(int x, int y, unsigned id);
	bool CountSupport(int x, int y, unsigned id);

public:
  unsigned clos;    		/// the underlying closure
  unsigned jcts[4]; 		/// junctions, in counter-clockwise order
  double parallelity; 	/// parallelity of the two opposed edges
	Vector2 direction[2];	/// TODO mean direction of the two line-pairs (1,3 and 2,4)
	double phi[2];				/// mean angle of the two line-pairs
  unsigned nrOfLJcts;		/// number of L-Jcts from the underlying closure
	Vector2 centerPoint;	/// center point of the rectangle (2D mean value of corners)
	double radius; 				/// maximum distance from center-point to a corner-point
	double innerRadius;		/// minimum distance from center-point to middle of lines.

	unsigned width;				/// image width
	unsigned height;			/// image height
	unsigned pixelmass;		///	number of pixels from lines of closure
	unsigned *data;				/// data from CalculateSupport() (Bresenham alg.)
	double pixelsupport; 	/// pixelsupport of the rectangle (in %) (>100 possible?)

  //Rectangle(unsigned c, unsigned js[4]);
  Rectangle(unsigned c, unsigned js[4], double par, unsigned ljct);
	~Rectangle(){ delete data; };
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);

	bool IsInside(unsigned rectangle);
  double SumGaps();
  double Area();
  double Circumference();
};

inline Array<Gestalt*>& Rectangles()
{
  return Gestalts(Gestalt::RECTANGLE);
}
inline Rectangle* Rectangles(unsigned id)
{
  return (Rectangle*)Gestalts(Gestalt::RECTANGLE, id);
}
inline unsigned NumRectangles()
{
  return NumGestalts(Gestalt::RECTANGLE);
}

}

#endif
