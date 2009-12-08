/**
 * @file StereoCore.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
 */


#include "StereoCore.hh"

//#define ZSC_DEBUG

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);


//-----------------------------------------------------------------//
//---------------------------- Surf3D -----------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor of Stereo Core
 * @param stereocal_file Stereo calibration file
 */
StereoCore::StereoCore(const string &stereocal_file)
    throw(Except)
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    // create vision core, don't specify config file, we'll configure ourselves
    vcore[side] = new VisionCore();

    // hardwire the gestalt principles we need, saves loading a config file
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ARCS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ELLIPSES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS);
  }

	// init stereo camera calibration parameters
  stereo_cam = new StereoCamera();
  stereo_cam->ReadSVSCalib(stereocal_file);

	InitStereoGestalts();
}

/**
 * @brief Destructor of Stereo Core
 */
StereoCore::~StereoCore()
{
  delete vcore;
  delete stereo_cam;
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    delete stereoGestalts[i];
}


void StereoCore::InitStereoGestalts()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    stereoGestalts[i] = 0;

	stereoGestalts[StereoBase::STEREO_ELLIPSE] = new StereoEllipses(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_CLOSURE] = new StereoClosures(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_RECTANGLE] = new StereoRectangles(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_FLAP] = new StereoFlaps(vcore, stereo_cam);
//   stereoGestalts[StereoBase::STEREO_FLAP2] = new StereoFlaps2(vcore, stereo_cam);
//   stereoGestalts[StereoBase::STEREO_CUBE] = new StereoCube(vcore, stereo_cam);

//   for(unsigned i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
//     config.AddItem(GestaltPrinciple::TypeName((GestaltPrinciple::Type)i), "0");
}


/**
 * @brief Clear the vision cores and the used arrays.
 */
void StereoCore::ClearResults()
{
  for(int side = LEFT; side <= RIGHT; side++)
    vcore[side]->ClearGestalts();
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
  	stereoGestalts[i]->ClearResults();
}


/**
 * @brief Set the active draw area to left or right stereo image.
 * @param side Left or right image
 */
void StereoCore::SetActiveDrawAreaSide(int side)
{
	if (side == LEFT) SetActiveDrawArea(img_l);
	else if (side == RIGHT) SetActiveDrawArea(img_r);
}


/**
 * @brief Process stereo image 
 * @param runtime_ms granted runtime in [ms] for each image
 */
void StereoCore::ProcessStereoImage(int runtime_ms, IplImage *iIl, IplImage *iIr)
{
// printf("\nProcess stereo image:\n");
	img_l = iIl;
	img_r = iIr;

  // do monocular processing for each stereo image
  for(int side = LEFT; side <= RIGHT; side++)
  {
    vcore[side]->NewImage(side == LEFT ? img_l : img_r);
    vcore[side]->ProcessImage(runtime_ms);
	}
// printf("\nProcess stereo image 2:\n");

	// do stereo processing
	try 
	{
// printf("\nProcess stereo image 3:\n");
		for(int i = 0; i < StereoBase::MAX_TYPE; i++)
			stereoGestalts[i]->Process();											/// TODO Funktioniert nur, wenn alle Gestalts initialisiert wurden!!!! VORSICHT
// printf("\nProcess stereo image: 4\n");
  }
	catch(Z::Except &e) 
	{
		printf("%s\n", e.what());
	}

// printf("\nProcess stereo image 5:\n");

	/// HACK draw results of the stereo image
// 	DrawStereoResults(StereoBase::STEREO_FLAP);
		SetActiveDrawAreaSide(LEFT);
// 		vcore[LEFT]->DrawGestalts(Gestalt::ARC, 0);
		vcore[LEFT]->DrawGestalts(Gestalt::ELLIPSE, 0);
		SetActiveDrawAreaSide(RIGHT);
// 		vcore[RIGHT]->DrawGestalts(Gestalt::ARC, 0);
		vcore[RIGHT]->DrawGestalts(Gestalt::ELLIPSE, 0);

	/// HACK Print results
	PrintResults();
}



/**
 * @brief Get a stereo object as visual object for the CogX cast-framework.
 * @param type Type of stereo object.
 * @param number ID of the stereo object
 * @param obj Visual object as pointer
 */
void StereoCore::GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj)
{
	stereoGestalts[type]->StereoGestalt2VisualObject(obj, id);
}


/**
 * @brief Draw the results into a iplImage
 * TODO Eigentlich sollte man die linke und rechte gematchte Gestalten in beide einzeichnen und dann anzeigen.
 */
void StereoCore::DrawStereoResults(StereoBase::Type type)
{
	for(int side = LEFT; side <= RIGHT; side++)
	{
		SetActiveDrawAreaSide(side);
		stereoGestalts[type]->Draw(side);
		stereoGestalts[type]->DrawMatched(side);
	}
}



/**
 *  HACK
 * @brief Print the corner points of the calculated flap.
 */
void StereoCore::PrintResults()
{
	// print results of vision core
  printf("Arc: 2D arcs left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::ARC),
			vcore[RIGHT]->NumGestalts(Gestalt::ARC));
  printf("ArcGroup: 2D arc_groups left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::CONVEX_ARC_GROUP),
			vcore[RIGHT]->NumGestalts(Gestalt::CONVEX_ARC_GROUP));
  printf("Ellipse: 2D ellipses left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::ELLIPSE),
			vcore[RIGHT]->NumGestalts(Gestalt::ELLIPSE));
//   printf("StereoClosures:   clos-matches: %d\n", stereoGestalts[StereoBase::STEREO_CLOSURE]->NumStereoMatches());

	// print results of vision core
  printf("Closure: 2D rects left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::CLOSURE),
			vcore[RIGHT]->NumGestalts(Gestalt::CLOSURE));
  printf("StereoClosures:   clos-matches: %d\n", stereoGestalts[StereoBase::STEREO_CLOSURE]->NumStereoMatches());

//   printf("StereoRects: 2D rects left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::RECTANGLE),
// 			vcore[RIGHT]->NumGestalts(Gestalt::RECTANGLE));
//   printf("StereoRects:   rect-matches: %d\n", stereoGestalts[StereoBase::STEREO_RECTANGLE]->NumStereoMatches());

//   printf("StereoFlaps: flaps left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::FLAP), 											// HACK
// 			vcore[RIGHT]->NumGestalts(Gestalt::FLAP));
//   printf("StereoFlaps:   flap-matches: %d\n", stereoGestalts[StereoBase::STEREO_FLAP]->NumStereoMatches()); 																				// HACK


// 	for(unsigned i=0; i<flaps[LEFT].Size(); i++)
// 	{
// 		for(unsigned j=0; j<flaps[LEFT][i].surf[0].pr.size(); j++)
// 			printf("Points left, flap %u, surf 0:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
// 				i, flaps[LEFT][i].surf[0].p[j].x, flaps[LEFT][i].surf[0].p[j].y, flaps[LEFT][i].surf[0].pr[j].x, flaps[LEFT][i].surf[0].pr[j].y);
// 		for(unsigned j=0; j<flaps[LEFT][i].surf[1].pr.size(); j++)
// 			printf("Points left, flap %u, surf 1:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
// 				i, flaps[LEFT][i].surf[1].p[j].x, flaps[LEFT][i].surf[1].p[j].y, flaps[LEFT][i].surf[1].pr[j].x, flaps[LEFT][i].surf[1].pr[j].y);
// 	}
// 	for(unsigned i=0; i<flaps[RIGHT].Size(); i++)
// 	{
// 		for(unsigned j=0; j<flaps[RIGHT][i].surf[0].pr.size(); j++)
// 			printf("Points right, flap %u, surf 0:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
// 				i, flaps[RIGHT][i].surf[0].p[j].x, flaps[RIGHT][i].surf[0].p[j].y, flaps[RIGHT][i].surf[0].pr[j].x, flaps[RIGHT][i].surf[0].pr[j].y);
// 		for(unsigned j=0; j<flaps[RIGHT][i].surf[1].pr.size(); j++)
// 			printf("Points right, flap %u, surf 1:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
// 				i, flaps[RIGHT][i].surf[1].p[j].x, flaps[RIGHT][i].surf[1].p[j].y, flaps[RIGHT][i].surf[1].pr[j].x, flaps[RIGHT][i].surf[1].pr[j].y);
// 	}
}

/**
 * @brief Print the corner points of the calculated flaps.
 */
void StereoCore::PrintRectResults()
{
// 	printf("\nRECTANGLE RESULTS:  \n");
// 	for(unsigned i=0; i<flaps[LEFT].Size(); i++)
// 	{
// 	}
}


}

