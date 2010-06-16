/**
 * @file StereoCore.cc
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
 */

#include "StereoCore.hh"

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);

/**
 * @brief Constructor of Stereo Core
 * @param stereocal_file Stereo calibration file
 */
StereoCore::StereoCore(const string &stereocal_file) throw(Except)
{
	pPara = new PruningParameter;
	pPara->pruning = false;

  for(int side = LEFT; side <= RIGHT; side++)
  {
    // create vision core, don't specify config file, we'll configure ourselves
    vcore[side] = new VisionCore();

    // hardwire the gestalt principles we need, saves loading a config file
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);

		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ARCS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ARC_JUNCTIONS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ELLIPSES);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_SPHERES);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_E_JUNCTIONS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_EXT_ELLIPSES);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CYLINDERS);
		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONES);

		vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS_ARI);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CUBES);

		/// TODO Es fehlt: EXTELLIPSES, CYLINDERS, CONES, SPHERES
  }

	// init stereo camera calibration parameters
  stereo_cam = new StereoCamera();
  if(!stereo_cam->ReadSVSCalib(stereocal_file)) throw (Except(__HERE__, "cannot open calibration file for stereo camera."));

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
    if(stereoGestalts[i]->IsEnabled())
			delete stereoGestalts[i];
}


/**
 * @brief Initialisation of the stereo Gestalt classes.
 */
void StereoCore::InitStereoGestalts()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    stereoGestalts[i] = 0;

	// initialise all principles
	stereoGestalts[StereoBase::STEREO_ELLIPSE] = new StereoEllipses(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_CLOSURE] = new StereoClosures(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_RECTANGLE] = new StereoRectangles(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_FLAP] = new StereoFlaps(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_FLAP_ARI] = new StereoFlapsAri(vcore, stereo_cam);
  stereoGestalts[StereoBase::STEREO_CUBE] = new StereoCubes(vcore, stereo_cam);

	// set principles enabled or disabled
	stereoGestalts[StereoBase::STEREO_ELLIPSE]->EnablePrinciple(false);													/// TODO Ellipses disabled
	stereoGestalts[StereoBase::STEREO_CLOSURE]->EnablePrinciple(true);
	stereoGestalts[StereoBase::STEREO_RECTANGLE]->EnablePrinciple(true);
	stereoGestalts[StereoBase::STEREO_FLAP]->EnablePrinciple(true);
	stereoGestalts[StereoBase::STEREO_FLAP_ARI]->EnablePrinciple(true);
	stereoGestalts[StereoBase::STEREO_CUBE]->EnablePrinciple(true);
}


/**
 * @brief Clear the vision cores and the used stereo Gestalts.
 */
void StereoCore::ClearResults()
{
  for(int side = LEFT; side <= RIGHT; side++)
    vcore[side]->ClearGestalts();
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
  	if(stereoGestalts[i]->IsEnabled())
			stereoGestalts[i]->ClearResults();
}


/**
 * @brief Set the stereo images.
 * @param side Left or right image
 */
void StereoCore::SetImages(IplImage *iIl, IplImage *iIr)
{
	img_l = iIl;
	img_r = iIr;
}


/**
 * @brief Set the active draw area to left or right stereo image.
 * @param side Left or right image of stereo.
 */
void StereoCore::SetActiveDrawAreaSide(int side)
{
	if (side == LEFT) SetActiveDrawArea(img_l);
	else if (side == RIGHT) SetActiveDrawArea(img_r);
}


/**
 * @brief Process stereo image 
 * @param runtime_ms granted runtime in [ms] for each image
 * @param ca Canny alpha value
 * @param co Canny omega value
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * TODO Throw sollte hier implementiert werden und try-catch block weg!
 */
void StereoCore::ProcessStereoImage(int runtime_ms, float ca, float co, IplImage *iIl, IplImage *iIr)
{
// printf("StereoCore::ProcessStereoImage\n");
	SetImages(iIl, iIr);

// printf("StereoCore::ProcessStereoImage 1\n");
  // do monocular processing for each stereo image
  for(int side = LEFT; side <= RIGHT; side++)
  {
    vcore[side]->NewImage(side == LEFT ? img_l : img_r);
    vcore[side]->ProcessImage(runtime_ms, ca, co);
	}

// printf("StereoCore::ProcessStereoImage 2\n");
	// do stereo processing for enabled stereo principles
	try 
	{
		for(int i = 0; i < StereoBase::MAX_TYPE; i++)
		{
			if(stereoGestalts[i]->IsEnabled())
			{
				if(pPara->pruning) 
					stereoGestalts[i]->Process(pPara->offsetX, pPara->offsetY, pPara->scale);
				else
					stereoGestalts[i]->Process();
			}
		}
		printf("StereoCore::ProcessStereoImage: process image ended.\n");
  }
	catch(Z::Except &e) 
	{
		printf("StereoCore::ProcessStereoImage: Error during stereo calculation.\n");
		printf("%s\n", e.what());
	}

	/// HACK Print results
// 	PrintResults();

// printf("StereoCore::ProcessStereoImage end\n");
}


/**
 * @brief Process stereo image 
 * @param runtime_ms granted runtime in [ms] for each image
 * @param ca Canny alpha value
 * @param co Canny omega value
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void StereoCore::ProcessStereoImage(int runtime_ms, float ca, float co, IplImage *iIl, IplImage *iIr, int oX, int oY, int sc)
{
	// set pruning parameters
	pPara->pruning = true;
	pPara->offsetX = oX;
	pPara->offsetY = oY;
	pPara->scale = sc;
	ProcessStereoImage(runtime_ms, ca, co, iIl, iIr);
	pPara->pruning = false;
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
 * @brief Draw the mono results into a iplImage
 * @param type Type of stereo object.
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param masked Draw the masked features.
 * @param single Draw only single Gestalt.
 * @param detail Degree of detail.
 * @return Returns true for success.
 */
bool StereoCore::DrawMonoResults(Gestalt::Type type, IplImage *iIl, IplImage *iIr, bool masked, bool single, int singleSide, int id, int detail)
{
	SetImages(iIl, iIr);

	if(!single)
		for(int side = LEFT; side <= RIGHT; side++)
		{
			SetColor(RGBColor::blue);
			SetActiveDrawAreaSide(side);
			int numGestalts = NumMonoGestalts(type, side);
			if (id > numGestalts) return false;

			// draw all Gestalts
			for(int i=0; i<numGestalts; i++)
			{
				if(masked)
					vcore[side]->Gestalts(type, i)->Draw(detail);	
				else
					if (vcore[side]->Gestalts(type, i)->IsUnmasked())
						vcore[side]->Gestalts(type, i)->Draw(detail);	
			}
		}

	if(single)
	{
		SetColor(RGBColor::red);
		SetActiveDrawAreaSide(singleSide);
		int numGestalts = NumMonoGestalts(type, singleSide);
		if (id >= numGestalts) return false;

		// draw only one gestalt if id is in range of 
		if(id < numGestalts && id >= 0)
		{
			if(masked)
				vcore[singleSide]->Gestalts(type, id)->Draw(detail);	
			else
				if (vcore[singleSide]->Gestalts(type, id)->IsUnmasked())
					vcore[singleSide]->Gestalts(type, id)->Draw(detail);
		}
	}

	return true;
}


/**
 * @brief Draw the stereo results into the stereo iplImages.
 * @param type Type of stereo object.
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param matched Draw the matched features.
 */
void StereoCore::DrawStereoResults(StereoBase::Type type, IplImage *iIl, IplImage *iIr, bool matched, bool showAllStereoMatched)
{
	SetImages(iIl, iIr);

	if(!showAllStereoMatched)
	{
		for(int side = LEFT; side <= RIGHT; side++)
		{
			SetActiveDrawAreaSide(side);
			if(matched) stereoGestalts[type]->DrawMatched(side);
		}
	}
	else // show all stereo matched features
	{
		for(int i=0; i< StereoBase::MAX_TYPE; i++)
		{
			for(int side = LEFT; side <= RIGHT; side++)
			{
				SetActiveDrawAreaSide(side);
				stereoGestalts[i]->DrawMatched(side);
			}
		}
	}
}


/**
 * @brief Draw the region of interest (ROI) into the image
 * @param side To which side of the stereo image pair.
 * @param roi The CvRect, describing the ROI.
 * @param roiScale The scale between the StereoServer and the VideoServer
 * @param iIl Left stereo image
 * @param iIr Right stereo image
 */
void StereoCore::DrawROI(int side, CvRect roi, int roiScale, IplImage *iIl, IplImage *iIr)
{
	SetImages(iIl, iIr);
	SetActiveDrawAreaSide(side);
	DrawRect2D(roi.x*roiScale, roi.y*roiScale, roi.x*roiScale + roi.width*roiScale, roi.y*roiScale + roi.height*roiScale, RGBColor::red);
}


/**
 * 
 * @param side To which side of the stereo image pair.
 * @param offsetX 
 * @param offsetY 
 * @param iIl Left stereo image
 * @param iIr Right stereo image
 */
void StereoCore::DrawPrunedROI(int side, int offsetX, int offsetY, IplImage *iIl, IplImage *iIr)
{
	SetImages(iIl, iIr);
	SetActiveDrawAreaSide(side);
	DrawRect2D(offsetX, offsetY, offsetX + 320, offsetY + 240, RGBColor::blue);	
}

/**
 * @brief Print the statistics from the vision cores.
 */
void StereoCore::PrintVCoreStatistics()
{
	vcore[0]->PrintRunTime();
	vcore[1]->PrintRunTime();
}

/**
 * @brief Returns id of first gestalt at pixel position (x,y).
 * @param side Left / right side of stereo image pair.
 * @param type Gestalt type.
 * @param x x-coordinate in image pixels.
 * @param y y-coordinate in image pixels.
 * @param start_after Choose only Gestalts with id higher than start_after.
 * @param reject_masked Reject masked Gestalts. 
 * @return Returns the ID of the next Gestalt.
 */
unsigned StereoCore::PickGestaltAt(int side, Gestalt::Type type, int x, int y, unsigned start_after, bool reject_masked)
{
	return vcore[side]->PickGestaltAt(type, x, y, start_after, reject_masked);
}


/**
 *  HACK Wieder entfernen
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
//   printf("Closure: 2D rects left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::CLOSURE), vcore[RIGHT]->NumGestalts(Gestalt::CLOSURE));
  printf("StereoClosures:   clos-matches: %d\n", stereoGestalts[StereoBase::STEREO_CLOSURE]->NumStereoMatches());

//   printf("StereoRects: 2D rects left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::RECTANGLE), vcore[RIGHT]->NumGestalts(Gestalt::RECTANGLE));
//   printf("StereoRects:   rect-matches: %d\n", stereoGestalts[StereoBase::STEREO_RECTANGLE]->NumStereoMatches());

//   printf("StereoFlaps: flaps left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::FLAP), vcore[RIGHT]->NumGestalts(Gestalt::FLAP));
  printf("StereoFlaps:   flap-matches: %d\n", stereoGestalts[StereoBase::STEREO_FLAP]->NumStereoMatches()); 

}



}

