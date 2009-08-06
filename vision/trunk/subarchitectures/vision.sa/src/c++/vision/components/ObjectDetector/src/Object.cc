/**
 * @file Object.cc
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Header file of Gestalt Object (Cube, Cone, Cylinder, Ball, Wall, Exit (and tracked Gestalts))
 **/

#include "Object.hh"
//#include "ObjectTracker.hh"
#include "Cube.hh"
#include "Cone.hh"
#include "Cylinder.hh"
#include "Ball.hh"
#include "Wall.hh"
#include "Exit.hh"
#include "Draw.hh"

namespace Z
{

/**
 *	@brief Cunstructor of class Object for cubes
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param cubeDef Stored cube properties
 */
Object::Object(Gestalt::Type t, unsigned i, CubeDef cubeDef) : Gestalt(OBJECT)
{
  type = t;
	cuD = cubeDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

	sig = cubeDef.sig;

  CalculateSignificance();
}

/**
 *	@brief Cunstructor of class Object for cones
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param coneDef Stored cone properties
 */
Object::Object(Gestalt::Type t, unsigned i, ConeDef coneDef) : Gestalt(OBJECT)
{
  type = t;
	coD = coneDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

  CalculateSignificance();
}

/**
 *	@brief Cunstructor of class Object for cylinders
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param cylDef Stored cylinder properties
 */
Object::Object(Gestalt::Type t, unsigned i, CylDef cylDef) : Gestalt(OBJECT)
{
  type = t;
	cyD = cylDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

  CalculateSignificance();
}

/**
 *	@brief Cunstructor of class Object for balls
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param balDef Stored ball properties
 */
Object::Object(Gestalt::Type t, unsigned i, BalDef balDef) : Gestalt(OBJECT)
{
  type = t;
	baD = balDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

  CalculateSignificance();
}

/**
 *	@brief Cunstructor of class Object for walls
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param walDef Stored wall properties
 */
Object::Object(Gestalt::Type t, unsigned i, WalDef walDef) : Gestalt(OBJECT)
{
  type = t;
	waD = walDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

  CalculateSignificance();
}

/**
 *	@brief Cunstructor of class Object for exits
 *	@param t Gestalt type (Cube, Cone, ...)
 *	@param i Gestalt ID
 *	@param exitDef Stored cube properties
 */
Object::Object(Gestalt::Type t, unsigned i, ExitDef exitDef) : Gestalt(OBJECT)
{
  type = t;
	exD = exitDef;	

	gestalt_id = i;
  tracked_id = UNDEF_ID;

  CalculateSignificance();
}

/**
 *	@brief Mark object as tracked and store the ground center of the tracked object
 *	@param id Gestalt-ID TODO ????
 *	@param trGrCe Ground center of tracked object
 */
void Object::Tracked(unsigned id, Vector2 trGrCe)
{
  tracked_id = id;
	if(type == Gestalt::CUBE) cuD.trackedCubeGroundCenter = trGrCe;
	else if(type == Gestalt::CONE) coD.trackedConeGroundCenter = trGrCe;
	else if(type == Gestalt::CYLINDER) cyD.trackedCylinderGroundCenter = trGrCe;
	else if(type == Gestalt::BALL) baD.trackedBallGroundCenter = trGrCe;
	else if(type == Gestalt::WALL) waD.trackedWallGroundCenter = trGrCe;
	else if(type == Gestalt::EXIT_) exD.trackedExitGroundCenter = trGrCe;
}


/**
 *	@brief Delivering true, if the center of the object is inside the ratius of this object
 *	@param object Object, which could be inside
 *	@return True, if object center is inside the radius of the object
 */
bool Object::IsInside(unsigned object)
{
	// get center and radius of this object
	Vector2 center;
	double radius;
	if(type == Gestalt::CUBE || type == Gestalt::TRACKEDCUBE)
	{
		center = cuD.center;
		radius = cuD.radius;
	}
	else if(type == Gestalt::CONE || type == Gestalt::TRACKEDCONE)
	{
		center = coD.center;
		radius = coD.radius;
	}
	else if(type == Gestalt::CYLINDER || type == Gestalt::TRACKEDCYLINDER)
	{
		center = cyD.center;
		radius = cyD.radius;
	}
	else if(type == Gestalt::BALL || type == Gestalt::TRACKEDBALL)
	{
		center = baD.center;
		radius = baD.radius;
	}
	else if(type == Gestalt::WALL) return false;
	else if(type == Gestalt::EXIT_) return false;

	// get center and radius of ->object
	Vector2 objectCenter;
	if(Objects(object)->type == Gestalt::CUBE || Objects(object)->type == Gestalt::TRACKEDCUBE)
	{
		objectCenter = Objects(object)->cuD.center;
	}
	else if(Objects(object)->type == Gestalt::CONE || Objects(object)->type == Gestalt::TRACKEDCONE)
	{
		objectCenter = Objects(object)->coD.center;
	}
	else if(Objects(object)->type == Gestalt::CYLINDER || Objects(object)->type == Gestalt::TRACKEDCYLINDER)
	{
		objectCenter = Objects(object)->cyD.center;
	}	
	else if(Objects(object)->type == Gestalt::BALL || Objects(object)->type == Gestalt::TRACKEDBALL)
	{
		objectCenter = Objects(object)->baD.center;
	}	

	// return true if objectCenter is inside the radius
  if((center - objectCenter).Length() < radius) return true;
  else return false; 
}

/**
 *	@brief Draw the objects on the main window (overlay)
 *	@param detail Details of the drawings
 */
void Object::Draw(int detail)
{
	RGBColor color;
	RGBColor blue = RGBColor::blue;
	RGBColor red = RGBColor::red;
	RGBColor coral = RGBColor::coral;
	RGBColor green = RGBColor::blue;

	if(detail == 0) color = red;
	if(detail >= 1 && tracked_id == UNDEF_ID) color = red;
	if(detail >= 1 && tracked_id != UNDEF_ID) color = blue;
	if(/*detail >= 1 &&*/ (type == Gestalt::TRACKEDCUBE || type == Gestalt::TRACKEDCONE || 
										 type == Gestalt::TRACKEDCYLINDER || type == Gestalt::TRACKEDBALL)) color = green;

	/// CUBE
  if (type == Gestalt::CUBE)
  {
//     if (tracked_id == UNDEF_ID)
// 		{
// 			Cubes(gestalt_id)->Draw(0);
// 		}
// 		else
// 		{
			DrawLine2D(cuD.corner_points[3][1].x,
			   cuD.corner_points[3][1].y,
			   cuD.corner_points[3][0].x,
			   cuD.corner_points[3][0].y, color);
			DrawLine2D(cuD.corner_points[3][1].x,
			   cuD.corner_points[3][1].y,
			   cuD.corner_points[0][1].x,
			   cuD.corner_points[0][1].y, color);
			DrawLine2D(cuD.corner_points[3][1].x,
			   cuD.corner_points[3][1].y,
			   cuD.corner_points[2][1].x,
			   cuD.corner_points[2][1].y, color);
			for (unsigned i=0; i<4; i++)
			{
	  		DrawLine2D(cuD.corner_points[i][0].x,
						cuD.corner_points[i][0].y,
						cuD.corner_points[(i<3?i+1:0)][0].x,
						cuD.corner_points[(i<3?i+1:0)][0].y, color);
	  
			}
			for (unsigned i=0; i<2; i++)
			{
	  		DrawLine2D(cuD.corner_points[i][1].x,
						cuD.corner_points[i][1].y,
						cuD.corner_points[(i<2?i+1:0)][1].x,
						cuD.corner_points[(i<2?i+1:0)][1].y, color);	
			}
			DrawLine2D(cuD.corner_points[0][0].x,
					cuD.corner_points[0][0].y,
					cuD.corner_points[0][1].x,
					cuD.corner_points[0][1].y, color);
			DrawLine2D(cuD.corner_points[1][0].x,
					cuD.corner_points[1][0].y,
					cuD.corner_points[1][1].x,
					cuD.corner_points[1][1].y, color);
			DrawLine2D(cuD.corner_points[2][0].x,
					cuD.corner_points[2][0].y,
					cuD.corner_points[2][1].x,
					cuD.corner_points[2][1].y, color);

		// draw tracked line
/*			if(detail == 1)
			{
				DrawLine2D(cuD.groundCenter.x, cuD.groundCenter.y, cuD.trackedCubeGroundCenter.x, cuD.trackedCubeGroundCenter.y, color);
			}*/
//     }

		// write coordinates of 3D ground center point
		if(detail == 2)
		{
			DrawEllipse2D(cuD.groundCenter.x, cuD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", cuD.groundCenter3D.x, cuD.groundCenter3D.y);
			DrawText2D(text, cuD.groundCenter.x, cuD.groundCenter.y, color);
		}
		// write coordinates of 3D corner points
		if(detail == 3)
		{
			char text[40];
			snprintf(text, 40, " %3.0f/%3.0f/0", cuD.corner_points3D[0][1].x, cuD.corner_points3D[0][1].y);
			DrawText2D(text, cuD.corner_points[1][0].x, cuD.corner_points[0][1].y, color);
			snprintf(text, 40, " %3.0f/%3.0f/0", cuD.corner_points3D[1][1].x, cuD.corner_points3D[1][1].y);
			DrawText2D(text, cuD.corner_points[1][1].x, cuD.corner_points[1][1].y, color);
			snprintf(text, 40, " %3.0f/%3.0f/0", cuD.corner_points3D[2][1].x, cuD.corner_points3D[2][1].y);
			DrawText2D(text, cuD.corner_points[2][1].x, cuD.corner_points[2][1].y, color);
		}
		// draw 2D cube radius and center
		if(detail == 4)
		{
			DrawEllipse2D(cuD.center.x, cuD.center.y, cuD.radius, cuD.radius, 0., color);
			DrawEllipse2D(cuD.center.x, cuD.center.y, 1, 1 , 0., color);
		}
  }

	/// CONE
  if (type == Gestalt::CONE)
  {
	  if (tracked_id == UNDEF_ID)
	  {
		  Cones(gestalt_id)->Draw(0);
		}
		else
		{
			DrawEllipse2D(coD.x, coD.y, coD.a, coD.b, coD.phi, color);
			DrawLine2D(coD.isct.x, coD.isct.y, coD.vertex[0].x, coD.vertex[0].y, color);
			DrawLine2D(coD.isct.x, coD.isct.y, coD.vertex[1].x, coD.vertex[1].y, color);

			if(detail == 1)
			{
				DrawLine2D(coD.groundCenter.x, coD.groundCenter.y, coD.trackedConeGroundCenter.x, coD.trackedConeGroundCenter.y, color);
			}
		}
		if(detail == 2)
		{
			DrawEllipse2D(coD.groundCenter.x, coD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", coD.groundCenter3D.x, coD.groundCenter3D.y);
			DrawText2D(text, coD.groundCenter.x, coD.groundCenter.y, color);
		}
		if(detail == 3)
		{
			DrawEllipse2D(coD.center.x, coD.center.y, coD.radius, coD.radius, 0., color);
			DrawEllipse2D(coD.center.x, coD.center.y, 1, 1 , 0., color);
		}
	}

	/// CYLINDER
  if (type == Gestalt::CYLINDER)
  {
	  if (tracked_id == UNDEF_ID)
	  {
			Cylinders(gestalt_id)->Draw(0);
		}
		else
		{
			DrawEllipse2D(cyD.x[0], cyD.y[0], cyD.a[0], cyD.b[0], cyD.phi[0], color);
			DrawEllipse2D(cyD.x[1], cyD.y[1], cyD.a[1], cyD.b[1], cyD.phi[1], color);
			if(cyD.equalVertex)
			{
				DrawLine2D(cyD.vertex[0][0].x, cyD.vertex[0][0].y, cyD.vertex[1][0].x, cyD.vertex[1][0].y, color);
				DrawLine2D(cyD.vertex[0][1].x, cyD.vertex[0][1].y, cyD.vertex[1][1].x, cyD.vertex[1][1].y, color);
			}
			else
			{
				DrawLine2D(cyD.vertex[0][0].x, cyD.vertex[0][0].y, cyD.vertex[1][1].x, cyD.vertex[1][1].y, color);
				DrawLine2D(cyD.vertex[0][1].x, cyD.vertex[0][1].y, cyD.vertex[1][0].x, cyD.vertex[1][0].y, color);
			}
			if(detail == 1)
			{
				DrawLine2D(cyD.groundCenter.x, cyD.groundCenter.y, cyD.trackedCylinderGroundCenter.x, cyD.trackedCylinderGroundCenter.y, color);
			}
		}
		if(detail == 2)
		{
			DrawEllipse2D(cyD.groundCenter.x, cyD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", cyD.groundCenter3D.x, cyD.groundCenter3D.y);
			DrawText2D(text, cyD.groundCenter.x, cyD.groundCenter.y, color);
		}
		if(detail == 3)
		{
			DrawEllipse2D(cyD.center.x, cyD.center.y, cyD.radius, cyD.radius, 0., color);
			DrawEllipse2D(cyD.center.x, cyD.center.y, 1, 1 , 0., color);
		}

	}
	
	/// BALL
  if (type == Gestalt::BALL)
  {
	  if (tracked_id == UNDEF_ID)
	  {
			Balls(gestalt_id)->Draw(0);
		}
		else
		{
			/// TODO TODO TODO TODO TODO Draw tracked ball !!!
// 			DrawEllipse2D(cyD.x[0], cyD.y[0], cyD.a[0], cyD.b[0], cyD.phi[0], color);
		}
		if(detail == 2)
		{
// 			Balls(gestalt_id)->Draw(0);
			DrawEllipse2D(baD.center.x, baD.center.y, baD.radius, baD.radius, 0., color);
			DrawEllipse2D(baD.groundCenter.x, baD.groundCenter.y, 2, 2, 0., color);

			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", baD.groundCenter3D.x, baD.groundCenter3D.y);
			DrawText2D(text, baD.groundCenter.x, baD.groundCenter.y, color);
		}
	}
	
	/// WALL
  if (type == Gestalt::WALL)
  {

		if(!waD.corner)
			TransparentQuadrilateral2D(waD.borderPoints[0].x, waD.borderPoints[0].y, waD.borderPoints[1].x, waD.borderPoints[1].y, 
				waD.borderPoints[1].x, waD.borderPoints[1].y-10., waD.borderPoints[0].x, waD.borderPoints[0].y-10., color, true, 64);
		else
		{
			TransparentQuadrilateral2D(waD.borderPoints[0].x, waD.borderPoints[0].y, waD.points[0].x, waD.points[0].y, 
				waD.points[0].x, waD.points[0].y-10., waD.borderPoints[0].x, waD.borderPoints[0].y-10., color, true, 64);
			TransparentQuadrilateral2D(waD.borderPoints[1].x, waD.borderPoints[1].y, waD.points[0].x, waD.points[0].y, 
				waD.points[0].x, waD.points[0].y-10., waD.borderPoints[1].x, waD.borderPoints[1].y-10., color, true, 64);
		}

	  if (tracked_id == UNDEF_ID)
	  {
		  Walls(gestalt_id)->Draw(0);
		}
		else
		{
			DrawLine2D(waD.borderPoints[0].x, waD.borderPoints[0].y, waD.borderPoints[1].x, waD.borderPoints[1].y, color);

			if(detail == 1)
			{
				DrawLine2D(waD.groundCenter.x, waD.groundCenter.y, waD.trackedWallGroundCenter.x, waD.trackedWallGroundCenter.y, color);
			}
		}
		if(detail == 2)
		{
			DrawEllipse2D(waD.groundCenter.x, waD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", waD.groundCenter3D.x, waD.groundCenter3D.y);
			DrawText2D(text, waD.groundCenter.x, waD.groundCenter.y, color);
		}
		if(detail == 3)
		{
//			DrawEllipse2D(waD.center.x, waD.center.y, waD.radius, waD.radius, 0., color);
			DrawEllipse2D(waD.center.x, waD.center.y, 1, 1 , 0., color);
		}
	}

	/// EXIT
  if (type == Gestalt::EXIT_)
  {
	  if (tracked_id == UNDEF_ID)
	  {
		  Exits(gestalt_id)->Draw(0);
		}
		else
		{
			DrawLine2D(exD.exitLinesStart[0].x, exD.exitLinesStart[0].y, exD.exitLinesIntersection[0].x, exD.exitLinesIntersection[0].y, color);
			DrawLine2D(exD.exitLinesStart[1].x, exD.exitLinesStart[1].y, exD.exitLinesIntersection[1].x, exD.exitLinesIntersection[1].y, color);
			DrawLine2D(exD.exitLinesIntersection[0].x, exD.exitLinesIntersection[0].y, exD.exitLinesIntersection[1].x, exD.exitLinesIntersection[1].y, color);

			TransparentQuadrilateral2D(exD.exitLinesStart[0].x, exD.exitLinesStart[0].y, exD.exitLinesStart[1].x, exD.exitLinesStart[1].y, 
				exD.exitLinesIntersection[1].x, exD.exitLinesIntersection[1].y, exD.exitLinesIntersection[0].x, exD.exitLinesIntersection[0].y, RGBColor::green, true, 64);

			if(detail == 1)
			{
				DrawLine2D(exD.groundCenter.x, exD.groundCenter.y, exD.trackedExitGroundCenter.x, exD.trackedExitGroundCenter.y, color);
			}
		}
		if(detail == 2)
		{
			DrawEllipse2D(exD.groundCenter.x, exD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", exD.groundCenter3D.x, exD.groundCenter3D.y);
			DrawText2D(text, exD.groundCenter.x, exD.groundCenter.y, color);
		}
		if(detail == 3)
		{
//			DrawEllipse2D(waD.center.x, waD.center.y, waD.radius, waD.radius, 0., color);
			DrawEllipse2D(exD.center.x, exD.center.y, 1, 1 , 0., color);
		}
	}

	/// TRACKED_CUBE
  if (type == Gestalt::TRACKEDCUBE)
  {
		DrawLine2D(cuD.corner_points[3][1].x,
				cuD.corner_points[3][1].y,
				cuD.corner_points[3][0].x,
				cuD.corner_points[3][0].y, color);
		DrawLine2D(cuD.corner_points[3][1].x,
				cuD.corner_points[3][1].y,
				cuD.corner_points[0][1].x,
				cuD.corner_points[0][1].y, color);
		DrawLine2D(cuD.corner_points[3][1].x,
				cuD.corner_points[3][1].y,
				cuD.corner_points[2][1].x,
				cuD.corner_points[2][1].y, color);
		for (unsigned i=0; i<4; i++)
		{
			DrawLine2D(cuD.corner_points[i][0].x,
					cuD.corner_points[i][0].y,
					cuD.corner_points[(i<3?i+1:0)][0].x,
					cuD.corner_points[(i<3?i+1:0)][0].y, color);
	
		}
		for (unsigned i=0; i<2; i++)
		{
			DrawLine2D(cuD.corner_points[i][1].x,
					cuD.corner_points[i][1].y,
					cuD.corner_points[(i<2?i+1:0)][1].x,
					cuD.corner_points[(i<2?i+1:0)][1].y, color);	
		}
		DrawLine2D(cuD.corner_points[0][0].x,
				cuD.corner_points[0][0].y,
				cuD.corner_points[0][1].x,
				cuD.corner_points[0][1].y, color);
		DrawLine2D(cuD.corner_points[1][0].x,
				cuD.corner_points[1][0].y,
				cuD.corner_points[1][1].x,
				cuD.corner_points[1][1].y, color);
		DrawLine2D(cuD.corner_points[2][0].x,
				cuD.corner_points[2][0].y,
				cuD.corner_points[2][1].x,
				cuD.corner_points[2][1].y, color);

		if(detail == 1)
		{
			DrawLine2D(cuD.groundCenter.x, cuD.groundCenter.y, cuD.trackedCubeGroundCenter.x, cuD.trackedCubeGroundCenter.y, color);
		}
		if(detail == 2)
		{
			DrawEllipse2D(cuD.groundCenter.x, cuD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, "%4.1f/%4.1f/0", cuD.groundCenter3D.x, cuD.groundCenter3D.y);
			DrawText2D(text, cuD.groundCenter.x, cuD.groundCenter.y, color);
		}
		if(detail == 3)
		{
			DrawEllipse2D(cuD.center.x, cuD.center.y, cuD.radius, cuD.radius, 0., color);
			DrawEllipse2D(cuD.center.x, cuD.center.y, 1, 1 , 0., color);
		}
	}

	/// TRACKED_CONE
  if (type == Gestalt::TRACKEDCONE)
	{
		DrawEllipse2D(coD.x, coD.y, coD.a, coD.b, coD.phi, color);
		DrawLine2D(coD.isct.x, coD.isct.y, coD.vertex[0].x, coD.vertex[0].y, color);
		DrawLine2D(coD.isct.x, coD.isct.y, coD.vertex[1].x, coD.vertex[1].y, color);

		if(detail == 1)
		{
			DrawLine2D(coD.groundCenter.x, coD.groundCenter.y, coD.trackedConeGroundCenter.x, coD.trackedConeGroundCenter.y, color);
		}
		if(detail == 2)
		{
			DrawEllipse2D(coD.groundCenter.x, coD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", coD.groundCenter3D.x, coD.groundCenter3D.y);
			DrawText2D(text, coD.groundCenter.x, coD.groundCenter.y, color);
		}
		if(detail == 3)
		{
			DrawEllipse2D(coD.center.x, coD.center.y, coD.radius, coD.radius, 0., color);
			DrawEllipse2D(coD.center.x, coD.center.y, 1, 1 , 0., color);
		}
	}

	/// TRACKED_CYLINDER
  if (type == Gestalt::TRACKEDCYLINDER)
  {
		DrawEllipse2D(cyD.x[0], cyD.y[0], cyD.a[0], cyD.b[0], cyD.phi[0], color);
		DrawEllipse2D(cyD.x[1], cyD.y[1], cyD.a[1], cyD.b[1], cyD.phi[1], color);
		if(cyD.equalVertex)
		{
			DrawLine2D(cyD.vertex[0][0].x, cyD.vertex[0][0].y, cyD.vertex[1][0].x, cyD.vertex[1][0].y, color);
			DrawLine2D(cyD.vertex[0][1].x, cyD.vertex[0][1].y, cyD.vertex[1][1].x, cyD.vertex[1][1].y, color);
		}
		else
		{
			DrawLine2D(cyD.vertex[0][0].x, cyD.vertex[0][0].y, cyD.vertex[1][1].x, cyD.vertex[1][1].y, color);
			DrawLine2D(cyD.vertex[0][1].x, cyD.vertex[0][1].y, cyD.vertex[1][0].x, cyD.vertex[1][0].y, color);
		}
		if(detail == 1)
		{
			DrawLine2D(cyD.groundCenter.x, cyD.groundCenter.y, cyD.trackedCylinderGroundCenter.x, cyD.trackedCylinderGroundCenter.y, color);
		}
		if(detail == 2)
		{
			DrawEllipse2D(cyD.groundCenter.x, cyD.groundCenter.y, 1, 1 , 0., color);
			char text[40];
			snprintf(text, 40, " %4.1f/%4.1f/0", cyD.groundCenter3D.x, cyD.groundCenter3D.y);
			DrawText2D(text, cyD.groundCenter.x, cyD.groundCenter.y, color);
		}
		if(detail == 3)
		{
			DrawEllipse2D(cyD.center.x, cyD.center.y, cyD.radius, cyD.radius, 0., color);
			DrawEllipse2D(cyD.center.x, cyD.center.y, 1, 1 , 0., color);
		}
	}
}

/**
 *	@brief Get information about the object
 *	@return Information about the object, which will be displayed in the Gestalt-window.
 */
const char* Object::GetInfo()
{
  static char gestalt[40] = "";
  switch (type){
    case Gestalt::CUBE: snprintf(gestalt, 40, "Cube"); break;
    case Gestalt::CONE: snprintf(gestalt, 40, "Cone"); break;
    case Gestalt::CYLINDER: snprintf(gestalt, 40, "Cylinder"); break;
    case Gestalt::BALL: snprintf(gestalt, 40, "Ball"); break;
    case Gestalt::WALL: snprintf(gestalt, 40, "Wall"); break;
    case Gestalt::EXIT_: snprintf(gestalt, 40, "Exit"); break;
		case Gestalt::TRACKEDCUBE: snprintf(gestalt, 40, "Tracked Cube"); break;
		case Gestalt::TRACKEDCONE: snprintf(gestalt, 40, "Tracked Cone"); break;
		case Gestalt::TRACKEDCYLINDER: snprintf(gestalt, 40, "Tracked Cylinder"); break;
		case Gestalt::TRACKEDBALL: snprintf(gestalt, 40, "Tracked Ball"); break;
    default: snprintf(gestalt, 40, "unknown"); break;
  }
 
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;

	if(type == Gestalt::CUBE)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\nground center: %3.0f - %3.0f\ntracked ground center: %3.0f - %3.0f\ncenter: %4.3f / %4.3f\nradius: %4.3f\n\nCorner points 2D || 3D:\n%4.0f - %4.0f || %4.0f - %4.0f\n%4.0f - %4.0f || %4.0f - %4.0f\n%4.0f - %4.0f || %4.0f - %4.0f\n%4.0f - %4.0f || %4.0f - %4.0f\n",
      Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, cuD.groundCenter.x, cuD.groundCenter.y, cuD.trackedCubeGroundCenter.x, cuD.trackedCubeGroundCenter.y,
			cuD.center.x, cuD.center.y, cuD.radius, 
			cuD.corner_points[0][1].x, cuD.corner_points[0][1].y, cuD.corner_points3D[0][1].x, cuD.corner_points3D[0][1].y,
			cuD.corner_points[1][1].x, cuD.corner_points[1][1].y, cuD.corner_points3D[1][1].x, cuD.corner_points3D[1][1].y,
			cuD.corner_points[2][1].x, cuD.corner_points[2][1].y, cuD.corner_points3D[2][1].x, cuD.corner_points3D[2][1].y,
			cuD.corner_points[3][1].x, cuD.corner_points[3][1].y, cuD.corner_points3D[3][1].x, cuD.corner_points3D[3][1].y);
	}
	if(type == Gestalt::CONE)
  {
 		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\nground center: %3.0f - %3.0f\ntracked ground center: %3.0f - %3.0f\n\ncenter: %4.3f / %4.3f\nradius: %4.3f", 
			Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, coD.groundCenter.x, coD.groundCenter.y, coD. trackedConeGroundCenter.x, coD.trackedConeGroundCenter.y,
			coD.center.x, coD.center.y, coD.radius);
	}
	if(type == Gestalt::CYLINDER)
  {
 		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\nground center: %3.0f - %3.0f\ntracked ground center: %3.0f - %3.0f\n\ncenter: %4.3f / %4.3f\nradius: %4.3f", 
			Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, cyD.groundCenter.x, cyD.groundCenter.y, cyD.trackedCylinderGroundCenter.x, cyD.trackedCylinderGroundCenter.y,
			cyD.center.x, cyD.center.y, cyD.radius);
	}
	if(type == Gestalt::BALL)
  {
 		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\ncenter: %4.3f / %4.3f\nradius: %4.3f\nground center: %3.0f - %3.0f\n\n groundCenter3D: %4.3f - %4.3f\nradius3D: %4.3f", 
			Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, baD.center.x, 
			baD.center.y, baD.radius, baD.groundCenter.x, baD.groundCenter.y, baD.groundCenter3D.x, baD.groundCenter3D.y, baD.radius3D);
	}
	if(type == Gestalt::WALL)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\nground center: %4.3f - %4.3f\ntracked ground center: %4.3f - %4.3f\n3D border points: %4.3f/%4.3f - %4.3f/%4.3f\n", 
			Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, waD.groundCenter.x, waD.groundCenter.y, waD.trackedWallGroundCenter.x, waD.trackedWallGroundCenter.y,
			waD.borderPoints3D[0].x, waD.borderPoints3D[0].y, waD.borderPoints3D[1].x, waD.borderPoints3D[1].y);
	}
	if(type == Gestalt::EXIT_)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\ngestalt_id: %u\nid_tracked: %u\n\nground center: %4.3f - %4.3f\n\nitsc 0: %4.3f - %4.3f\nitsc 1: %4.3f - %4.3f\n", 
			Gestalt::GetInfo(), gestalt, gestalt_id, tracked_id, exD.groundCenter.x, exD.groundCenter.y, exD.exitLinesIntersection3D[0].x, exD.exitLinesIntersection3D[0].y,
			exD.exitLinesIntersection3D[1].x, exD.exitLinesIntersection3D[1].y);
	}
	if(type == Gestalt::TRACKEDCUBE)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\nid: %u\nground center: %3.0f - %3.0f\ntracked groundCenter: %3.0f - %3.0f\n\ncenter: %4.3f / %4.3f\nradius: %4.3f",
      Gestalt::GetInfo(), gestalt, gestalt_id, cuD.groundCenter.x, cuD.groundCenter.y, cuD.trackedCubeGroundCenter.x, cuD.trackedCubeGroundCenter.y, cuD.center.x, cuD.center.y, cuD.radius);
	}
	if(type == Gestalt::TRACKEDCONE)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\n\ncenter: %4.3f / %4.3f\nradius: %4.3f",
      Gestalt::GetInfo(), gestalt, coD.center.x, coD.center.y, coD.radius);
	}
	if(type == Gestalt::TRACKEDCYLINDER)
  {
		n += snprintf(info_text, info_size, 
	  	"%s\nGestalt: %s\n\ncenter: %4.3f / %4.3f\nradius: %4.3f",
      Gestalt::GetInfo(), gestalt, cyD.center.x, cyD.center.y, cyD.radius);
	}

	return info_text;
}

/**
 *	@brief Check if there is a object at this position
 *	@param x x-Coordinate
 *	@param y y-Coordinate
 *	@return Returns true, if the object is at position x/y
 */
bool Object::IsAtPosition(int x, int y)
{
  return true;
}

/**
 *	@brief Calculate significance of the object
 */
void Object::CalculateSignificance()
{
}

}
