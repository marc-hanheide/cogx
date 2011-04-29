/**
 * @file Ball.cc
 * @author Andreas Richtsfeld
 * @date Mon December 29 2008
 * @version 0.1
 * @brief Gestalt Ball
 **/

#include "Draw.hh"
#include "Ball.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Constructor of class Ball.
 *	@param e	Underlying Gestalt: Ellipse.
 *	@param rad Radius of the estimated ball.
 *	@param rat Ration between the axis of the ellipse.
 */
Ball::Ball(unsigned e, double rad, double rat) : Gestalt(BALL)
{
	ellipse = e;
	radius = rad;

	center.x = Ellipses(ellipse)->x;
	center.y = Ellipses(ellipse)->y;

	CalculateSignificance(rat);
}

/**
 *	@brief Draw the Gestalt in the MainWin
 *	@param detail Degree of details to draw
 */
void Ball::Draw(int detail)
{
	Ellipse *e = Ellipses(ellipse);
  DrawEllipse2D(e->x, e->y, e->a, e->b, e->phi, RGBColor::red);
}

/**
 *	@brief Get information for displaying at InfoDrawArea
 */
const char* Ball::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s\nEllipse: %u, radius: %4.3f", Gestalt::GetInfo(), ellipse, radius);

  n += snprintf(info_text + n, info_size - n, "\n");

	return info_text;
}

/**
 *	@brief Returns true if the Gestalt is at this position.
 *	@param x x-Coordinate
 *	@param y y-Coordinate
 *	@return Returns true, if the Gestalt is at this position.
 */
bool Ball::IsAtPosition(int x, int y)
{
	
  return false;
}

/**
 *	@brief Calcultes the significance for the Gestalt Ball
 *	@param r Ratio between the length of the axis from the ellipse
 */
void Ball::CalculateSignificance(double r)
{
	double f = fabs(1-r);
	sig = Ellipses(ellipse)->sig/(f+1);	// TODO noch genauer überlegen
	
}

}
