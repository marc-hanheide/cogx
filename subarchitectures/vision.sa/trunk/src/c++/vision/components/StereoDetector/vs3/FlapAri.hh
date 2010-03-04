/**
 * @file FlapAri.hh
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Header file of Gestalt FlapAri: Flaps from rectangles.
 **/

#ifndef Z_FLAP_ARI_HH
#define Z_FLAP_ARI_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Rectangle.hh"

namespace Z
{

/**
 * @class FlapAri Flaps, created from rectangles
 * @brief Flaps, created from rectangles.
 */
class FlapAri : public Gestalt
{
public:
//  unsigned rects[2];								///< IDs of rectangles from flap																		///	TODO remove, now rectangles
  Array<Rectangle*> rectangles;			///< Rectangles of the flap
  Array<unsigned> sharedLines;			///< SharedLines from the two rectangles															/// TODO sollte auf Line* geändert werden

  double meanGap;										///< Mean value of smallest two gaps between two rectangle-corners		/// TODO kann bleiben
	Vector2 orderedIsctR0[4];					///< Ordered intersection points of first rectangle (counter clockwise)
	Vector2 orderedIsctR1[4];					///< Ordered intersection points of second rectangle (counter clockwise)

  unsigned innerJcts[4];						///< The inner 4 L-Junctions of the flap															/// TODO keine Junctions => Vector2 isct
  unsigned outerJcts[4];						///< The outer 4 L-Junctions of the flap															/// TODO keine Junctions => Vector2 isct
																		// [0,1] from rect[0] clockwise
																		// [2,3] from rect[1] counter clockwise


	// TODO Center und radius werden verwendet um Flaps zu maskieren.
	Vector2 center;										///< Center point of the flap (mean value of innerJcts->iscts)
	double radius;										///< Maximum radius from center to outerJcts->iscts
	Vector2 rectCenter[2];						///< Center points of the rectangles
	double rectRadius[2];							///< Maximum radius from rectCenter to junction intersections

	// TODO Das hier wird für später für das Tracken von Cubes verwendet, oder? => sollte wegkommen, da es einfach nicht richtig ist, das zu tun.
	/* 6 possibilities for oCase (Anordnung der beiden Rechtecke lt. Flap-Def):
				Right/Left	...	oCase = 1
				Left/Right	...	oCase = 2
				Front/Top		...	oCase = 3
				Top/Front		... oCase = 4
				Left/Top		...	oCase = 5
				Top/Right		...	oCase = 6
	*/
	unsigned oCase;										///< ordering of rectangles (r/l - l/r - f/t - t/f - l/t - t/r <==> 1--6)						/// TODO braucht man bei Stereo hoffentlich nicht mehr!


  FlapAri(VisionCore *c, unsigned r0, unsigned r1, double gap, Array<unsigned> sL, unsigned iJ[4], unsigned oJ[4], Vector2 oIsctR0[4], Vector2 oIsctR1[4]);

  void CalcOrientation();
  void CalculateSignificance();
	bool IsInside(unsigned flap);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};


inline Array<Gestalt*>& FlapsAri(VisionCore *core)
{
  return core->Gestalts(Gestalt::FLAP_ARI);
}
inline FlapAri* FlapsAri(VisionCore *core, unsigned id)
{
  return (FlapAri*)core->Gestalts(Gestalt::FLAP_ARI, id);
}
inline unsigned NumFlapsAri(VisionCore *core)
{
  return core->NumGestalts(Gestalt::FLAP_ARI);
}
}

#endif
