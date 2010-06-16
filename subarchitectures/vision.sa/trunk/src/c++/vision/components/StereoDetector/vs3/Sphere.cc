/**
 * @file Sphere.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Gestalt Sphere
 **/

#include "Draw.hh"
#include "Sphere.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Constructor of class Sphere.
 *	@param e Ellipse
 *	@param rad Radius of the estimated sphere.
 *	@param rat Ratio between the axis of the ellipse.
 */
Sphere::Sphere(VisionCore *vc, Ellipse *e, double rad, double rat) : Gestalt(vc, SPHERE)
{
	ellipse = e;
	radius = rad;

	CalculateSignificance(rat);
}

/**
 *	@brief Draw the Gestalt in the MainWin
 *	@param detail Degree of details to draw
 */
void Sphere::Draw(int detail)
{
	DrawEllipse2D(ellipse->x, ellipse->y, ellipse->a, ellipse->b, ellipse->phi, RGBColor::red);
}

/**
 *	@brief Get information for displaying at InfoDrawArea
 */
const char* Sphere::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s  ellipse: %u\n  radius: %4.1f", Gestalt::GetInfo(), ellipse->ID(), radius);

	return info_text;
}

/**
 *	@brief Returns true if the Gestalt is at this position.
 *	@param x x-Coordinate
 *	@param y y-Coordinate
 *	@return Returns true, if the Gestalt is at this position.
 */
bool Sphere::IsAtPosition(int x, int y)
{
 	if(ellipse->IsAtPosition(x, y)) return true;
  return false;
}

/**
 *	@brief Calcultes the significance for the Gestalt Ball
 *	@param r Ratio between the length of the axis from the ellipse
 */
void Sphere::CalculateSignificance(double r)
{
	double f = fabs(1-r);
	sig = ellipse->sig/(f+1);	// TODO noch genau überlegen
}

}
