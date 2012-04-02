/** @file math_helpers.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa

 * @author      Jan Hanzelka - DFKI

 This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#include <smltools/math_helpers.h>

namespace smlearning {

///
///generate random double nr. between min and max
///
double fRand(double min, double max) {
	return min + (max - min)*::rand()/RAND_MAX;
}

///
///calculate position to direct the arm given parameters set in the learning scenario
///
void set_point_coordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical) {
	position.v1 += (spacing*normalVec.v1); 
	position.v2 += (spacing*normalVec.v2); 
	position.v1 += (horizontal*orthogonalVec.v1); 
	position.v2 +=(horizontal*orthogonalVec.v2); 
	position.v3 += vertical; 
}


///
///calls set_point_coordinates for a discrete canonical number of different actions
///
void set_coordinates_into_target(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over) {


	//set it's coordinates into target
	switch (startPosition) {
	case 1: 
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down left (1)");
		break;

	case 2:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down middle (2)");
		break;

	case 3:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down right (3)");
		break;

	case 4:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center left (4)");
		break;

	case 5:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center middle (5)");
		break;

	case 6:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center right (6)");
		break;

	case 7:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up left (7)");
		break;

	case 8:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up middle (8)");
		break;

	case 9:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up right (9)");
		break;

	case 10:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down left (10)");
		break;

	case 11:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down middle (11)");
		break;

	case 12:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down right (12)");
		break;

	case 13:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center left (13)");
		break;

	case 14:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center middle (14)");
		break;

	case 15:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center right (15)");
		break;

	case 16:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up left (16)");
		break;

	case 17:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up middle (17)");
		break;

	case 18:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up right (18)");
		break;

	case 19:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side down left (19)");
		break;

	case 20:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side down right (20)");
		break;

	case 21:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side center left (21)");
		break;

	case 22:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side center right (22)");
		break;

	case 23:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side up left (23)");
		break;

	case 24:
		set_point_coordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side up right (24)");
		break;

	};

}



}; /* namespace smlearning */
