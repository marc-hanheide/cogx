/**
 * @file Flap.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Header file of Gestalt Flap
 **/

#ifndef Z_FLAP_HH
#define Z_FLAP_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "FlapProperties.hh"

namespace Z
{

/**
 * @brief Gestalt Flap.
 */
class Flap : public Gestalt
{
public:
	FlapProp fp;											///< FlapProperties (2D/3D data)

  unsigned rects[2];								///< Rectangles of flap
  double meanGap;										///< Mean value of smallest two gaps between two rectangle-corners
  Array<unsigned> sharedLines;			///< SharedLines from the two rectangles
  unsigned innerJcts[4];						///< The inner 4 L-Junctions of the flap
  unsigned outerJcts[4];						///< The outer 4 L-Junctions of the flap
																		// [0,1] from rect[0] clockwise
																		// [2,3] from rect[1] counter clockwise

	Vector2 center;										///< Center point of the flap (mean value of innerJcts->iscts)
	double radius;										///< Maximum radius from center to outerJcts->iscts
	Vector2 rectCenter[2];						///< Center points of the rectangles
	double rectRadius[2];							///< Maximum radius from rectCenter to junction intersections

	/* 6 possibilities for oCase (Anordnung der beiden Rechtecke lt. Flap-Def):
				Right/Left	...	oCase = 1
				Left/Right	...	oCase = 2
				Front/Top		...	oCase = 3
				Top/Front		... oCase = 4
				Left/Top		...	oCase = 5
				Top/Right		...	oCase = 6
	*/
	unsigned oCase;										///< ordering of rectangles (r/l - l/r - f/t - t/f - l/t - t/r <==> 1--6)


  Flap(unsigned r0, unsigned r1, double gap, Array<unsigned> sL, unsigned iJ[4], unsigned oJ[4]);
  void CalcOrientation();
  void CalculateSignificance();
	bool IsInside(unsigned flap);
	void CalculateFlapProperties();

  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Flaps()
{
  return Gestalts(Gestalt::FLAP);
}
inline Flap* Flaps(unsigned id)
{
  return (Flap*)Gestalts(Gestalt::FLAP, id);
}
inline unsigned NumFlaps()
{
  return NumGestalts(Gestalt::FLAP);
}

}

#endif
