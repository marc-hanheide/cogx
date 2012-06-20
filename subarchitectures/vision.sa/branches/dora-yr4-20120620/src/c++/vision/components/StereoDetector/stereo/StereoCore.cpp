/**
 * @file StereoCore.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
 */

#include "StereoCore.h"
#include "StereoLJunctions.h"
#include "StereoCorners.h"
#include "StereoEllipses.h"
#include "StereoLines.h"
#include "StereoClosures.h"
#include "StereoRectangles.h"
#include "StereoFlaps.h"
#include "StereoFlapsAri.h"
#include "StereoCubes.h"

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);

/**
 * @brief Constructor of Stereo Core
 * @param stereocal_file Stereo calibration file
 */
StereoCore::StereoCore(const string &stereocal_file) throw(std::runtime_error)
{
  throw (std::runtime_error("StereoCore::StereoCore: Error: Antiquated constructor: Use initialisation with openCV xml-files."));
  
//   printf("StereoCore::StereoCore: Warning: Antiquated: Use initialisation with openCV xml-files!\n");
//   pPara = new PruningParameter;
//   pPara->pruning = false;
// 
//   for(int side = LEFT; side <= RIGHT; side++)
//   {
//     // create vision core, don't specify config file, we'll configure ourselves
//     vcore[side] = new VisionCore();
// 
//     // hardwire the gestalt principles we need, saves loading a config file
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);
// 
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ARCS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ARC_JUNCTIONS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_ELLIPSES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CIRCLES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_E_JUNCTIONS);                 /// TODO Wenn E-junctions deaktiviert und ellipsen aktiviert, dann array fehler bei search lines im vote image!
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_EXT_ELLIPSES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CYLINDERS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONES);
// 
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CORNERS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS_ARI);
//     vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CUBES);
//   }
// 
//   // init stereo camera calibration parameters
//   stereo_cam = new cast::StereoCamera();
//   if(!stereo_cam->ReadSVSCalib(stereocal_file)) throw (std::runtime_error("StereoCore::StereoCore: Cannot open calibration file for stereo camera."));
// 
//   InitStereoPrinciples();
}

/**
 * @brief Constructor of Stereo Core with openCV calibration files
 * @param stereocal_file Stereo calibration file
 */
StereoCore::StereoCore(const string &stereocal_file_xml_left, const string &stereocal_file_xml_right) throw(std::runtime_error)
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
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CIRCLES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_E_JUNCTIONS);                 /// TODO Wenn E-junctions deaktiviert und ellipsen aktiviert, dann array fehler bei search lines im vote image!
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_EXT_ELLIPSES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CYLINDERS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CONES);

    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CORNERS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS_ARI);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CUBES);
  }

  // init stereo camera calibration parameters
  stereo_cam = new cast::StereoCamera();
  stereo_cam->ReadFromXML(stereocal_file_xml_left, 0, true);  
  stereo_cam->ReadFromXML(stereocal_file_xml_right, 1, true);
  stereo_cam->SetupImageRectification();
  
  InitStereoPrinciples();
}

/**
 * @brief Destructor of Stereo Core
 */
StereoCore::~StereoCore()
{
  delete vcore;
  delete stereo_cam;
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    if(stereoPrinciples[i]->IsEnabled())
      delete stereoPrinciples[i];
}


/**
 * @brief Initialisation of the stereo Gestalt classes.
 */
void StereoCore::InitStereoPrinciples()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    stereoPrinciples[i] = 0;

  // stereoPrinciples all principles
  stereoPrinciples[StereoBase::STEREO_LJUNCTION] = new StereoLJunctions(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_CORNER] = new StereoCorners(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_ELLIPSE] = new StereoEllipses(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_LINE] = new StereoLines(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_CLOSURE] = new StereoClosures(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_RECTANGLE] = new StereoRectangles(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_FLAP] = new StereoFlaps(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_FLAP_ARI] = new StereoFlapsAri(this, vcore, stereo_cam);
  stereoPrinciples[StereoBase::STEREO_CUBE] = new StereoCubes(this, vcore, stereo_cam);

  // set principles enabled or disabled
  stereoPrinciples[StereoBase::STEREO_LJUNCTION]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_CORNER]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_ELLIPSE]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_LINE]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_CLOSURE]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_RECTANGLE]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_FLAP]->EnablePrinciple(false);						/// TODO disabled
  stereoPrinciples[StereoBase::STEREO_FLAP_ARI]->EnablePrinciple(true);
  stereoPrinciples[StereoBase::STEREO_CUBE]->EnablePrinciple(true);
}


/**
 * @brief Create a new 3D Gestalt
 * @param g New 3D Gestalt
 */
void StereoCore::NewGestalt3D(Gestalt3D *g)
{
  Gestalts3D(g->GetType()).PushBack(g);
}
  

/**
 * @brief Clear the vision cores and the used stereo Gestalts.
 */
void StereoCore::ClearResults()
{
  for(int side = LEFT; side <= RIGHT; side++)
    vcore[side]->ClearGestalts();
  for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    if(stereoPrinciples[i]->IsEnabled())
      stereoPrinciples[i]->ClearResults();
  for(int i = 0; i < Gestalt3D::MAX_TYPE; i++)
    stereoGestalts[i].Clear();
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
printf("StereoCore::ProcessStereoImage: Start\n");
	SetImages(iIl, iIr);

  // do monocular processing for each stereo image
	try 
	{
		for(int side = LEFT; side <= RIGHT; side++)
		{
printf("StereoCore::ProcessStereoImage: Start mono\n");
			vcore[side]->NewImage(side == LEFT ? img_l : img_r);
			vcore[side]->ProcessImage(runtime_ms, ca, co);
printf("StereoCore::ProcessStereoImage: Mono finished: %u\n", side);
		}
	}
	catch (exception &e)
  {
		printf("StereoCore::ProcessStereoImage: Unknown exception during processing of stereo images.\n");
    cout << e.what() << endl;
  }
	
printf("StereoCore::ProcessStereoImage: monocular image processed.\n");

  // do stereo processing for enabled stereo principles
  try 
  {
    for(int i = 0; i < StereoBase::MAX_TYPE; i++)
    {
      if(stereoPrinciples[i] != 0)
        if(stereoPrinciples[i]->IsEnabled())
        {
          if(pPara->pruning) 
            stereoPrinciples[i]->Process(pPara->offsetX, pPara->offsetY, pPara->scale);
          else
          {
printf("  StereoCore::Processing stereo principle: %u\n", i);
            stereoPrinciples[i]->Process();
          }
        }
    }
  }
  catch (exception &e)
  {
    printf("StereoCore::ProcessStereoImage: Exception during processing of stereo images");
    std::cout << e.what() << std::endl;
  }

printf("StereoCore::ProcessStereoImage: stereo images processed.\n");

//   PrintResults();			/// HACK Print results

//  PrintJunctions2File();		/// TODO Print junctions to file => For Odense project.
//  PrintCorners2File();		/// TODO Print corners to file => For Odense project.
//   Print3DLines2File();

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
 * @return Returns true for success.
 */
#ifdef HAVE_CAST
bool StereoCore::GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj)
{
  return stereoPrinciples[type]->StereoGestalt2VisualObject(obj, id);
}
#endif


/**
 * @brief Get the name of a stereo type with a fixed character length.  
 * @param type Stereo type
 * @return Returns the information as string.
 */
const char* StereoCore::GetStereoTypeName(StereoBase::Type type)
{
  const unsigned info_size = 1000;
  static char name_text[info_size] = "";
  int n = 0;

  n += snprintf(name_text + n, info_size - n, "%s: ", StereoBase::TypeName(type));

  for(int i=0; i< (20 - StereoBase::StereoTypeNameLength(type)); i++)
    n += snprintf(name_text + n, info_size -n, " ");
  
  return name_text;
}

/**
 * @brief Get the Gestalt list with the number of all found mono and stereo Gestalts.
 * @return Returns the information as character array.
 */
const char* StereoCore::GetGestaltListInfo()
{
  const unsigned info_size = 50000;
  static char info_text[info_size] = "";
  int n = 0;

  n += snprintf(info_text + n, info_size - n, 
    "  GESTALT LIST		LEFT	RIGHT\n  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  for(int i=0; i < Gestalt::MAX_TYPE; i++)
  {
    n += snprintf(info_text + n, info_size - n, "  %s", vcore[0]->GetGestaltTypeName((Gestalt::Type) i));
    n += snprintf(info_text + n, info_size - n, "	%i", NumMonoGestalts((Gestalt::Type) i, LEFT));
    n += snprintf(info_text + n, info_size - n, "	%i\n", NumMonoGestalts((Gestalt::Type) i, RIGHT));
  }
  n += snprintf(info_text + n, info_size - n, "\n  STEREO LIST		STEREO\n  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  for(unsigned i=0; i< StereoBase::MAX_TYPE; i++)
  {
    n += snprintf(info_text + n, info_size - n, "\n  %s", GetStereoTypeName((StereoBase::Type) i));
    n += snprintf(info_text + n, info_size - n, "%i", NumStereoMatches((StereoBase::Type) i));
  }
  return info_text;
}

/**
 * @brief Draw the mono results into a iplImage
 * @param type Type of stereo object.
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param masked Draw the masked features.
 * @param single Draw only single Gestalt.
 * @param id ID of the mono Gestalt
 * @param detail Degree of detail.
 * @return Returns true for success.
 */
bool StereoCore::DrawMonoResults(Gestalt::Type type, IplImage *iIl, IplImage *iIr, bool masked, bool single, 
				 int singleSide, int id, int detail)
{
  SetImages(iIl, iIr);

  if(!single)
  for(int side = LEFT; side <= RIGHT; side++)
  {
    SetColor(RGBColor::red);
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
    SetColor(RGBColor::white);
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
void StereoCore::DrawStereoResults(StereoBase::Type type, IplImage *iIl, IplImage *iIr, 
				 bool showAllStereoMatched, bool single, int id, int detail)
{
  SetImages(iIl, iIr);
  SetColor(RGBColor::blue);
  if(!single)
  {
    if(!showAllStereoMatched)
    {
      for(int side = LEFT; side <= RIGHT; side++)
      {
        SetActiveDrawAreaSide(side);
        stereoPrinciples[type]->DrawMatched(side, single, id, detail);
      }
    }
    else // show all stereo matched features
    {
      for(int i=0; i< StereoBase::MAX_TYPE; i++)
      {
        for(int side = LEFT; side <= RIGHT; side++)
        {
          SetActiveDrawAreaSide(side);
          stereoPrinciples[i]->DrawMatched(side, single, id, detail);
        }
      }
    }
  }
  else
  {
    for(int side = LEFT; side <= RIGHT; side++)
    {
      SetActiveDrawAreaSide(side);
      stereoPrinciples[type]->DrawMatched(side, single, id, detail);
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
  /// print results of vision core
//   printf("Arc: 2D arcs left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::ARC),
// 			vcore[RIGHT]->NumGestalts(Gestalt::ARC));
//   printf("ArcGroup: 2D arc_groups left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::CONVEX_ARC_GROUP),
// 			vcore[RIGHT]->NumGestalts(Gestalt::CONVEX_ARC_GROUP));
//   printf("Ellipse: 2D ellipses left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::ELLIPSE),
// 			vcore[RIGHT]->NumGestalts(Gestalt::ELLIPSE));
// //   printf("StereoClosures:   clos-matches: %d\n", stereoGestalts[StereoBase::STEREO_CLOSURE]->NumStereoMatches());

  /// print results of stereo core
//   printf("Corners: 2D corners left/right: %d %d\n", vcore[LEFT]->NumGestalts(Gestalt::CORNER), vcore[RIGHT]->NumGestalts(Gestalt::CORNER));
//   printf("Stereo:CORNERS: corner-matches: %d\n", stereoPrinciples[StereoBase::STEREO_CORNER]->NumStereoMatches());
//   printf("Stereo:LJUNCTIONS: ljct-matches: %d\n", stereoPrinciples[StereoBase::STEREO_LJUNCTION]->NumStereoMatches());
//   printf("Stereo:LINES: line-matches: %d\n", stereoPrinciples[StereoBase::STEREO_LINE]->NumStereoMatches());
//   printf("Stereo:FLAPS_ARI: flap-matches: %d\n", stereoPrinciples[StereoBase::STEREO_FLAP_ARI]->NumStereoMatches());
}

/**
 * @brief Print the junctions into a file, for reading in the Odense project.
 */
void StereoCore::PrintJunctions2File()
{
printf("StereoCore::PrintJunctions2File()\n");

  FILE *file = fopen("junction_cache_5_L_filer_2.csv", "w");
  fprintf(file,"%u\n", vcore[0]->NumGestalts(Gestalt::L_JUNCTION));					// number of junctions
  fprintf(file,"%u %u\n", vcore[0]->IW(), vcore[0]->IH());									// image width and height
  
  for(unsigned i=0; i<vcore[0]->NumGestalts(Gestalt::L_JUNCTION); i++)
  {
    LJunction *lj = (LJunction*) vcore[0]->Gestalts(Gestalt::L_JUNCTION)[i];
    
    // calculate bin:
    double angle_0 = ScaleAngle_0_2pi(PolarAngle(lj->dir[0]) + M_PI/2.) * 180/M_PI;
    double angle_1 = ScaleAngle_0_2pi(PolarAngle(lj->dir[1]) + M_PI/2.) * 180/M_PI;
    int bin_0 = (int) (64.*angle_0/360.);
    int bin_1 = (int) (64.*angle_1/360.);
    fprintf(file,"%u %u 2 %u %u\n", (int) lj->isct.x, (int) lj->isct.y, bin_0, bin_1);					// image width and height
  }
  
  fprintf(file, "\n");
  fclose(file);
  
  file = fopen("junction_cache_5_R_filer_2.csv", "w");
  fprintf(file,"%u\n", vcore[1]->NumGestalts(Gestalt::L_JUNCTION));					// number of junctions
  fprintf(file,"%u %u\n", vcore[1]->IW(), vcore[1]->IH());									// image width and height
  
  for(unsigned i=0; i<vcore[1]->NumGestalts(Gestalt::L_JUNCTION); i++)
  {
    LJunction *lj = (LJunction*) vcore[1]->Gestalts(Gestalt::L_JUNCTION)[i];
    
    // calculate bin:
    double angle_0 = ScaleAngle_0_2pi(PolarAngle(lj->dir[0]) + M_PI/2.) * 180/M_PI;
    double angle_1 = ScaleAngle_0_2pi(PolarAngle(lj->dir[1]) + M_PI/2.) * 180/M_PI;
    int bin_0 = (int) (64.*angle_0/360.);
    int bin_1 = (int) (64.*angle_1/360.);
    fprintf(file,"%u %u 2 %u %u\n", (int) lj->isct.x, (int) lj->isct.y, bin_0, bin_1);					// image width and height
  }
  
  fprintf(file, "\n");
  std::fclose(file);
}

/**
 * @brief Print the junctions into a file, for reading in the Odense project.
 * TODO Works only with corners with 3 arms!!!
 */
void StereoCore::PrintCorners2File()
{
  printf("StereoCore::PrintCorners2File()\n");

  FILE *file = fopen("junction_cache_5_L_filer_2.csv", "w");
  std::fprintf(file,"%u\n", vcore[0]->NumGestalts(Gestalt::CORNER));		// number of corners
  fprintf(file,"%u %u\n", vcore[0]->IW(), vcore[0]->IH());		// image width and height
  
  for(unsigned i=0; i<vcore[0]->NumGestalts(Gestalt::CORNER); i++)
  {
    Corner *cor = (Corner*) vcore[0]->Gestalts(Gestalt::CORNER)[i];
    
    // calculate bin for the 3 angles:
    if(cor->angle.Size() > 2)
    {
      double angle_0 = ScaleAngle_0_2pi(cor->angle[0] + M_PI/2.) * 180/M_PI;
      double angle_1 = ScaleAngle_0_2pi(cor->angle[1] + M_PI/2.) * 180/M_PI;
      double angle_2 = ScaleAngle_0_2pi(cor->angle[2] + M_PI/2.) * 180/M_PI;

      // 		double angle_1 = ScaleAngle_0_2pi(PolarAngle(lj->dir[1]) + M_PI/2.) * 180/M_PI;
      int bin_0 = (int) (64.*angle_0/360.);
      int bin_1 = (int) (64.*angle_1/360.);
      int bin_2 = (int) (64.*angle_2/360.);

      fprintf(file,"%u %u 3 %u %u %u\n", (int) cor->isct.x, (int) cor->isct.y, bin_0, bin_1, bin_2);	// image width and height
    }
    else
	    printf("StereoCore::PrintCorners2File: ACHTUNG: WENIGER ALS 3 ANGLES IM CORNER!\n");
  }
  fprintf(file, "\n");
  fclose(file);
  
  file = fopen("junction_cache_5_R_filer_2.csv", "w");
  fprintf(file,"%u\n", vcore[1]->NumGestalts(Gestalt::CORNER));		// number of corners
  fprintf(file,"%u %u\n", vcore[1]->IW(), vcore[1]->IH());		// image width and height
  
  for(unsigned i=0; i<vcore[1]->NumGestalts(Gestalt::CORNER); i++)
  {
    Corner *cor = (Corner*) vcore[1]->Gestalts(Gestalt::CORNER)[i];
    
    // calculate bin for the 3 angles:
    if(cor->angle.Size() > 2)
    {
      double angle_0 = ScaleAngle_0_2pi(cor->angle[0] + M_PI/2.) * 180/M_PI;
      double angle_1 = ScaleAngle_0_2pi(cor->angle[1] + M_PI/2.) * 180/M_PI;
      double angle_2 = ScaleAngle_0_2pi(cor->angle[2] + M_PI/2.) * 180/M_PI;

      int bin_0 = (int) (64.*angle_0/360.);
      int bin_1 = (int) (64.*angle_1/360.);
      int bin_2 = (int) (64.*angle_2/360.);
      fprintf(file,"%u %u 3 %u %u %u\n", (int) cor->isct.x, (int) cor->isct.y, bin_0, bin_1, bin_2);	// image width and height
    }
    else
      printf("StereoCore::PrintCorners2File: ACHTUNG: WENIGER ALS 3 ANGLES IM CORNER!\n");
  }

  fprintf(file, "\n");
  std::fclose(file);
}


/**
 * @brief Print the junctions into a file, for reading in the Odense project.
 */
void StereoCore::Print3DLines2File()
{
printf("StereoCore::Print3DLines2File()\n");

  FILE *file = fopen("3Dlines.txt", "w");
  for(unsigned i=0; i < stereoGestalts[Gestalt3D::LINE].Size(); i++)
  {
    Line3D *line = (Line3D*) stereoGestalts[Gestalt3D::LINE][i];

    Line *ll = (Line*) vcore[LEFT]->Gestalts(Gestalt::LINE, line->GetVs3ID(0));

    fprintf(file,"%6.3f - %6.3f - %6.3f - %6.3f - %6.3f - %6.3f - %6.3f - %6.3f - %6.3f - %6.3f\n", 
      line->isct3D[0].p.x, line->isct3D[0].p.y, line->isct3D[0].p.z, line->isct3D[1].p.x, line->isct3D[1].p.y, line->isct3D[1].p.z,
      ll->point[0].x, ll->point[0].y, ll->point[1].x, ll->point[1].y 
    );					// image width and height
  }
  fprintf(file, "\n");
  fclose(file);
}

} 
