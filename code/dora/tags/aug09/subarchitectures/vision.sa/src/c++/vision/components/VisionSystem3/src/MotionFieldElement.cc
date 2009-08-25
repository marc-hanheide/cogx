/**
 * @file MotionFieldElement.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief The class represents a element of the motion field.
 **/

#include "MotionFieldElement.hh"
#include "Draw.hh"
#include "FormMotionField.hh"
#include "VisionCore.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt Motion_Field_Element
 * @param nId ID of new L-Junction
 * @param oId ID of old L-Junction
 * @param newIsct Intersection point of new L-Junction
 * @param oldIsct Intersection point of new L-Junction
 * @param s Significance value
 */
MotionFieldElement::MotionFieldElement(unsigned nId, unsigned oId, Vector2 newIsct, Vector2 oldIsct, double s) 
: Gestalt(MOTION_FIELD_ELEMENT)
{
// cout << "NewGestalt: MotionFieldElement\n" << endl;
	newID = nId;
	oldID = oId;

	point[0] = oldIsct;
	point[1] = newIsct;

	dir = point[1] - point[0];
	if (dir.Norm() != 0) dir = Normalise(dir);
	else
		printf("MotionFieldElement: Motion vector with zero length.!\n");

	double dx = fabs(point[0].x - point[1].x);
	double dy = fabs(point[0].y - point[1].y);
	double square = dx*dx + dy*dy;
	if (square != 0) length = sqrt(square);
	else printf("MotionFieldElement: Square sum is 0!\n");

	sig = s;
//   CalculateSignificance();
}

/**
 * @brief Draw()
 * @param detail Degree of detail
 */
void MotionFieldElement::Draw(int detail)
{
// cout << "MotionFieldElement::Draw" << endl;

// 	DrawLine2D(point[0].x/3.+320, point[0].y/3.+240, point[1].x/3.+320, point[1].y/3.+240, RGBColor::red);
	DrawLine2D(point[0].x, point[0].y, point[1].x, point[1].y, RGBColor::red);
	Vector2 helper = point[0] - 640*dir;
	DrawArrow(point[0], point[1], RGBColor::red);

// 	if(detail >= 1) DrawLine2D(point[1].x/3.+320, point[1].y/3.+240, helper.x/3.+320, helper.y/3.+240, RGBColor::coral);
	if(detail == 1 || detail ==2) DrawLine2D(point[1].x, point[1].y, helper.x, helper.y, RGBColor::coral);

	if(detail == 2)
	{
		DrawEllipse2D(FOE.x, FOE.y, 3. , 3., 0., RGBColor::white);
		DrawEllipse2D(FOE.x, FOE.y, 100. , 100., 0., RGBColor::white);
		DrawEllipse2D(FOE.x, FOE.y, 200. , 200., 0., RGBColor::white);
		DrawEllipse2D(FOE.x, FOE.y, 300. , 300., 0., RGBColor::white);
		DrawEllipse2D(FOE.x, FOE.y, FOE.x , FOE.x, 0., RGBColor::white);
		DrawEllipse2D(FOE.x, FOE.y, FOE.y , FOE.y, 0., RGBColor::white);

// 		DrawEllipse2D(FOE.x, FOE.y, FOE.x , FOE.x, 0., RGBColor::white);
// 		DrawEllipse2D(FOE.x, FOE.y, FOE.y , FOE.y, 0., RGBColor::white);

		Vector2 dir0; dir0.x = 1.; dir0.y = 0.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = 1.; dir0.y = 1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = 0.; dir0.y = 1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = -1.; dir0.y = 1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = -1.; dir0.y = 0.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = -1.; dir0.y = -1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = 0.; dir0.y = -1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);

		dir0.x = 1.; dir0.y = -1.; dir0.Normalise(); dir0 *= 1000.;
		DrawLine2D(FOE.x, FOE.y, FOE.x + dir0.x, FOE.y + dir0.y, RGBColor::white);
	}

	if(detail >= 3)
	for(double i=32; i<640; i=i+64)		/// TODO use IW und IH
	{
		for(double j=24; j<480; j=j+48)
		{
			Vector2 helper, helper2, dirFOE; 
			double rot=0, distance = 0;

			switch(motionCase) {
				case 1: rot = 0; break;
				case 2: rot = M_PI; break;
				case 3: rot = -M_PI/2.; break;
				case 4: rot = M_PI/2.; break;
				default: rot = 0; break;
			}

			helper.x = i;	helper.y = j;
			dirFOE = helper - FOE;
			dirFOE = Rotate(dirFOE, rot);
			dirFOE = Normalise(dirFOE);
			distance = Length(FOE - helper);
			helper = dirFOE*distance*disMul;
			helper2.x = i; helper2.y =j;
			DrawLine2D(i, j, i + helper.x, j + helper.y, RGBColor::white); 
			DrawArrow(helper2, helper2 + helper, RGBColor::white);
		}
	}
// cout << "MotionFieldElement::Draw end" << endl;
/*cout << "FOE: " << FOE.x << " / " << FOE.y << endl;
cout << "motion case: " << motionCase << endl; 
cout << "disMul: " << disMul << " || sig: " << FOEsig << endl;*/
// printf("direction: %4.2f / %4.2f\n", direction.x, direction.y);

}

/**
 * @brief GetInfo()
 */
const char* MotionFieldElement::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s\nID-new: %u\nID-old: %u\nlength: %5.2f\ndirection: %5.2f\n\nmotion case: %u\nFOE: %4.2f / %4.2f\nDist. Multip.: %5.4f\nMotion Field Significance: %5.4f", Gestalt::GetInfo(), newID, oldID, length, PolarAngle(dir), motionCase, FOE.x, FOE.y, disMul, FOEsig);

	return info_text;
}

/**
 * @brief IsAtPosition()
 */
bool MotionFieldElement::IsAtPosition(int x, int y)
{
  return true;
}

/**
 * @brief CalculateSignificance()
 */
void MotionFieldElement::CalculateSignificance()
{
}

/**
 * @brief Delivers the Focus of Expansion (FOE) point and the other parameter for motion calculation.
 * @param mC Motion case: 1=forward / 2=backward / 3=right / 4=left
 * @param foe Focus of Expension (center) point
 * @param dM Distance multiplicator (dM*distance = Length of motion vector)
 * @param fSig Significance value for the focus of expansion
 */
void MotionFieldElement::SetFOE(int mC, Vector2 foe, double dM, double fSig)
{
	motionCase = mC;
	FOE = foe;
	disMul = dM;
	FOEsig = fSig;
}


}
