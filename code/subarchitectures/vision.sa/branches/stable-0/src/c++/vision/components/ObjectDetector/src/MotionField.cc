/**
 * @file MotionField.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores motion fields.
 **/

#include "MotionField.hh"
#include "Draw.hh"

namespace Z
{

static int CmpMotion(const void *a, const void *b)
{
  if( MotionFields(*(unsigned*)a)->sig > MotionFields(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

MotionField::MotionField(int mC, Vector2 foe, double dM, double fSig, double a)
 : Gestalt(MOTION_FIELD)
{
	motionCase = mC;
	FOE = foe;
	disMul = dM;
	sig = fSig;
	age = a;

// 	CalculateSignificance();
}

void MotionField::Draw(int detail)
{
	if(detail >= 0)
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
}

/** 
 * @brief Get motion vector for a postion p.
 */
void MotionField::GetMotion(Vector2 &m, Vector2 p)
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

			helper.x = p.x;	helper.y = p.y;
			dirFOE = helper - FOE;
			dirFOE = Rotate(dirFOE, rot);
			dirFOE = Normalise(dirFOE);
			distance = Length(FOE - helper);
			helper = dirFOE*distance*disMul;
	m = helper;
}


const char* MotionField::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s\n\nage: %u\n\nmotionCase: %u\nFOE: %4.2f, %4.2f\ndistance mult.: %4.2f", 
		Gestalt::GetInfo(), age, motionCase, FOE.x, FOE.y, disMul);


	return info_text;
}

bool MotionField::IsAtPosition(int x, int y)
{
// 	return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

void MotionField::CalculateSignificance()
{

}

// // // void MotionField::AgeGestalt()
// // // {
// // // 	age++;
// // // 	if (age > 5) clear = true;
// // // cout << "Age Gestalt with id=" << id << " : age=" << age << "  clear=" << clear << endl;
// // // }

}
