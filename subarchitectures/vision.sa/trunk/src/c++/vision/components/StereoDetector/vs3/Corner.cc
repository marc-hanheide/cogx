/**
 * @file FormCorners.hh
 * @author Andreas Richtsfeld
 * @date September 2010
 * @version 0.1
 * @brief Gestalt class of corners.
 */

#include "LJunction.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt corner.
 * @param j Junctions
 * @param l Lines
 * @param np Near points of lines
 */
Corner::Corner(VisionCore *vc, Array<LJunction*> j, Array<Line*> l, Array<unsigned> np) : Gestalt(vc, CORNER)
{
  ljcts = j;
  lines = l;
  near_points = np;
  
  // Add the new created corner to the lines
  for(unsigned i=0; i<lines.Size(); i++)
    lines[i]->corners[near_points[i]].PushBack(this);
  
  if(lines.Size() != 3) 
    printf("Corner::Corner: Warning: Corner with more ore less than 3 arms!\n");
  
// printf("NEW CORNER: ljcts: ");
// for(unsigned i=0; i<ljcts.Size(); i++)
// {
//   printf("%u  ", ljcts[i]->ID());
// }
// printf("\nlines: ");
// for(unsigned i=0; i<lines.Size(); i++)
// {
//   printf("%u@%u  ", lines[i]->ID(), near_points[i]);
// }
// printf("\n");

  CalculateProperties();
}



/*
**	TODO ARI: Umbennen (ist eher ein Update oder so)
**	Add new L-Junction to existing corner
*/
// void Corner::AddLJunctions(Array<unsigned> &lj, Array<unsigned> &li, Array<unsigned> &np)
// {
//   for (unsigned i=0; i<lj.Size(); i++)
//   {
// 	if (!ljcts.Contains(lj[i]))
// 	  ljcts.PushBack(lj[i]);
//   }
//   
//   for (unsigned i=0; i<li.Size(); i++)
//   {
// 	if(!lines.Contains(li[i]))
// 	{
// 	  lines.PushBack(li[i]);
// 	  near_point.PushBack(np[i]);
// 	  Lines(li[i])->corners.PushBack(id);
// 	}
//   }
// }


/**
 * @brief Calculate the intersection point.
 * @param gap The mean gap between the original intersection points and the new calculated intersection point
 * @param
 * @param
 */
void Corner::CalculateProperties()
{
  // get the mean between the intersection points
  Vector2 zw;
  zw.x = 0;
  zw.y = 0;
  unsigned counter = 0;
  gap = 0.;
  for(unsigned i=0; i<ljcts.Size(); i++)
  {
    zw += ljcts[i]->isct;
    counter++;
  }
  isct = zw / (double) counter;
// printf("Corner::CalculateIntersection: intersection: %4.3f - %4.3f\n", isct.x, isct.y);

  // calculate mean gap to the new intersection point!
  zw.x = 0;	zw.y = 0;
  counter = 0;
  for(unsigned i=0; i<ljcts.Size(); i++)
  {
	  Vector2 zwi = ljcts[i]->isct - isct;
// printf("Corner::CalculateIntersection: zwi: %4.3f - %4.3f\n", zwi.x, zwi.y);
// printf("Corner::CalculateIntersection: gap: %4.3f\n", Length(zwi));
	  gap += Length(zwi);
	  counter++;
  }
gap /= (double) counter;

// printf("Corner::CalculateIntersection: mean gap: %4.3f\n", gap);
  
  // calculate angles
  for(unsigned i=0; i<lines.Size(); i++)
  {
	  Vector2 zwi = lines[i]->point[Other(near_points[i])] - isct;
	  double ang = PolarAngle(zwi);
// printf("Corner::CalculateIntersection: angles: %4.0f\n", ang*180/M_PI);
	  angle.PushBack(ang);
  }
}

/*
**	Add a third L-Jct to a existing Corner
*/
// void Corner::Recalc(unsigned lj)
// {
// //  ljct[2] = lj;
// //  sig += 100;
// }


/*
**	Draw
*/
// void Corner::Draw(int detail)
// {
//   for (unsigned i=0; i<ljcts.Size(); i++)
// 	LJunctions(ljcts[i])->Draw(detail+1);
// }


// const char* Corner::GetInfo()
// {
//   const unsigned info_size = 10000;
//   static char info_text[info_size] = "";
//   int n = 0;
//   n += snprintf(info_text, info_size, 
// 		"%s\nL-Jcts: ", Gestalt::GetInfo());
//   for (unsigned i=0; i<ljcts.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n,
//   		"%i ", ljcts[i]);
// 	
//   n += snprintf(info_text + n, info_size - n, "\nLines: ");
//   
//   for (unsigned i=0; i<lines.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n,
//   		"%i ", lines[i]);
// 
//   n += snprintf(info_text + n, info_size - n, "\nnear Point: ");
//   
//   for (unsigned i=0; i<near_point.Size(); i++)
// 	n += snprintf(info_text + n, info_size - n,
//   		"%i ", near_point[i]);
// 
//    return info_text;
// }


// bool Corner::IsAtPosition(int x, int y)
// {
//   for (unsigned i=0; i<lines.Size(); i++)
//   {
// 	if (Lines(lines[i])->IsAtPosition(x, y))
// 	  return true;
//   }
//   return false;
// }


// void Corner::CalculateSignificance(double gap)
// {
// //  double cGap = gap/50.;
// //  sig = 1./(cGap + 0.005);
//   sig += meanLength/(gap + 0.2);
// }

}

