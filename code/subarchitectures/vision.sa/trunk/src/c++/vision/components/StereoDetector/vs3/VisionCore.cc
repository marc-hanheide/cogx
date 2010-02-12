/**
 * @file VisionCore.cc
 * @author Andreas Richtsfeld, Michael Zillich
 * @date 2006, 2010
 * @version 0.1
 * @brief Vision Core for perceptual grouping.
 */

#include <time.h>
#include <signal.h>
#include <assert.h>
#include "Except.hh"
#include "Math.hh"
#include "Draw.hh"

#include "FormSegments.hh"
#include "FormLines.hh"
#include "FormArcs.hh"
//#include "FormParallelLineGroups.hh"
#include "FormConvexArcGroups.hh"
#include "FormEllipses.hh"
#include "FormJunctions.hh"
#include "FormArcJunctions.hh"
#include "FormClosures.hh"
#include "FormRectangles.hh"
#include "FormFlaps.hh"
#include "FormFlapsAri.hh"
#include "FormCubes.hh"

#include "Line.hh"
#include "Collinearity.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "Ellipse.hh"
#include "VisionCore.hh"

namespace Z
{

#ifdef DEBUG_CHECKINDEX
/**
 * HACK: this must be a non-inline, non-template function, otherwise the
 * debugger cannot put a breakpoint at throw.
 */
void CheckIndex(unsigned i, unsigned size)
{
  if(i >= size)
    throw Except(__HERE__, "invalid index %u, valid range [0,%u)", i, size);
}
#endif


/**
 * @brief Constructor of class VisionCore.
 * @param config_name Name of config file.
 */
VisionCore::VisionCore(const string &config_name)
{
  InitMath();
  img = 0;
  p_e = 0.;
  p_ee = 0.;
  roi_center = Vector2(0., 0.);
  roi_sigma = 0.;
  use_masking = true;
  InitGestaltPrinciples();
  Configure(config_name);
}

/**
 * @brief Destructor of class VisionCore.
 */
VisionCore::~VisionCore()
{
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    delete principles[i];
  ClearGestalts();
  ExitMath();
}

/**
 * @brief Initialise all known Gestalt principles.
 */
void VisionCore::InitGestaltPrinciples()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    principles[i] = 0;
  principles[GestaltPrinciple::FORM_SEGMENTS] = new FormSegments(this);
  principles[GestaltPrinciple::FORM_LINES] = new FormLines(this);
  principles[GestaltPrinciple::FORM_ARCS] = new FormArcs(this);
  //principles[GestaltPrinciple::FORM_PARALLEL_LINE_GROUPS] = new FormParallelLineGroups(this);
  principles[GestaltPrinciple::FORM_CONVEX_ARC_GROUPS] = new FormConvexArcGroups(this);
  principles[GestaltPrinciple::FORM_ELLIPSES] = new FormEllipses(this);
  principles[GestaltPrinciple::FORM_JUNCTIONS] = new FormJunctions(this);
  principles[GestaltPrinciple::FORM_ARC_JUNCTIONS] = new FormArcJunctions(this);
  principles[GestaltPrinciple::FORM_CLOSURES] = new FormClosures(this);
  principles[GestaltPrinciple::FORM_RECTANGLES] = new FormRectangles(this);
  principles[GestaltPrinciple::FORM_FLAPS] = new FormFlaps(this);
  principles[GestaltPrinciple::FORM_FLAPS_ARI] = new FormFlapsAri(this);
  principles[GestaltPrinciple::FORM_CUBES] = new FormCubes(this);
  for(unsigned i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    config.AddItem(GestaltPrinciple::TypeName((GestaltPrinciple::Type)i), "0");
}


/**
 * @brief Enable a Gestalt principle for calculation.
 * @param p Type of Gestalt principle
 */
void VisionCore::EnableGestaltPrinciple(GestaltPrinciple::Type p)
{
  string type = GestaltPrinciple::TypeName((GestaltPrinciple::Type)p);
  config.items[type] = "1";
}


/**
 * @brief Disable a Gestalt principle for calculation.
 * @param p Type of Gestalt principle
 */
void VisionCore::DisableGestaltPrinciple(GestaltPrinciple::Type p)
{
  string type = GestaltPrinciple::TypeName((GestaltPrinciple::Type)p);
  config.items[type] = "0";
}


/**
 * @brief Returns true if a Gestalt principle is enabled.
 * @param p Type of Gestalt principle.
 * @return Returns true, if Gestalt principle is enabled.
 */
bool VisionCore::IsEnabledGestaltPrinciple(GestaltPrinciple::Type p)
{
  return principles[p] != 0 &&
    config.GetValueInt(GestaltPrinciple::TypeName(p)) == 1;
}


/**
 * @brief Configure the Gestalt principle tree, appropriate to the config-file.
 * @param config_name Name of the config file.
 */
void VisionCore::Configure(const string &config_name)
{
  if(!config_name.empty())
    config.Load(config_name);
}


/**
 * @brief Clear Gestalts: Deletes and clears the Gestalt list (with ranked_gestalts).
 */
void VisionCore::ClearGestalts()
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
  {
    for(unsigned j = 0; j < gestalts[i].Size(); j++)
      delete gestalts[i][j];
    gestalts[i].Clear();
    ranked_gestalts[i].Clear();
  }
}


/**
 * @brief Informs the vision core about a new image and prepares for new processing (clear and reset).
 * @param new_img New openCV ipl-image.
 */
void VisionCore::NewImage(const IplImage *new_img)
{
  img = new_img;
  ClearGestalts();
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
    {
      principles[i]->Reset();
      principles[i]->ResetRunTime();
    }
}

/**
 * @brief Process the current image incrementally for a given amount of time.
 * @param runtime_ms  amount of time to run, in microseconds.
 * TODO try-catch hier weg und nur mehr exception werfen
 */
void VisionCore::ProcessImage(int runtime_ms) //throw Except()
{
  struct timespec start, cur;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  try
  {
    do
    {
      for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
      {
        if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i) && principles[i]->NeedsOperate())
        {
          struct timespec t1, t2;
          clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
          principles[i]->Operate(true);
          clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t2);
          principles[i]->AddRunTime(timespec_diff(&t2, &t1));
        }
      }
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cur);
    } while(timespec_diff(&cur, &start) < (double)runtime_ms/1000.);
  }
  catch(Z::Except &e)
  {
    printf("%s\n", e.what());
  }
}


/**
 * @brief Older, non-incremental (non-anytime) version of ProcessImage
 */
void VisionCore::ProcessImage()
{
  try
  {
    for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    {
      if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
      {
        struct timespec start, end;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
        principles[i]->Operate(false);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
        principles[i]->SetRunTime(timespec_diff(&end, &start));
      }
    }
  }
  catch(Z::Except &e)
  {
    printf("%s\n", e.what());
  }
}

/**
 * @brief Draw the original image and all the gestalts we have.
 */
void VisionCore::Draw(int detail)
{
  DrawImage();
  DrawAllGestalts(detail);
}

/**
 * @brief Draw only the original image.
 */
void VisionCore::DrawImage()
{
  if(img != 0)
    DrawImageRGB24(img->imageData, img->width, img->height);
}

/**
 * @brief Draw all the gestalts we have.
 */
void VisionCore::DrawAllGestalts(int detail)
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
    for(unsigned j = 0; j < gestalts[i].Size(); j++)
      gestalts[i][j]->Draw(detail);
}

/**
 * @brief Draw all gestalts of given type.
 */
void VisionCore::DrawGestalts(Gestalt::Type type, int detail)
{
  for(unsigned j = 0; j < gestalts[type].Size(); j++)
    gestalts[type][j]->Draw(detail);
}

/**
 * @brief Draw just a single gestalt.
 */
void VisionCore::DrawGestalt(Gestalt::Type type, unsigned num, int detail)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->Draw(detail);
}

/**
 * @brief Draw whatever pictorial info a gestalt has to offer.
 */
void VisionCore::DrawGestaltInfo(Gestalt::Type type, unsigned num)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->DrawInfo();
}

/**
 * @brief Draw whatever the given gestalt principle has to draw.
 */
void VisionCore::DrawPrinciple(GestaltPrinciple::Type type, int detail)
{
  if(IsEnabledGestaltPrinciple(type))
    principles[type]->Draw(detail);
}

/**
 * @brief Returns id of first gestalt at pixel position (x,y). \n
 * start_after can be used to skip the first gestalts. So all gestalts at x,y can be selected consecutively. \n
 * If mask is set to true, only unmasked gestalts will be returned.
 * @param type Gestalt type
 * @param x x-coordinate
 * @param y y-coordinate
 * @param start_after Deliver only return value, if id is greater than "start_after"
 * @param reject_masked If true, only unmasked Gestalts will be returned.
 * @return Return id of next Gestalt, or UNDEF_ID (-1), if nothing found.
 */
unsigned VisionCore::PickGestaltAt(Gestalt::Type type, int x, int y, unsigned start_after, bool reject_masked)
{
  unsigned start = (start_after == UNDEF_ID ? 0 : start_after + 1);
  for(unsigned j = start; j < gestalts[type].Size(); j++)
    if( gestalts[type][j]->IsAtPosition(x, y) &&
        (!reject_masked || !gestalts[type][j]->IsMasked()) )
      return j;
  return UNDEF_ID;
}

/**
 * @brief Accumulated runtime of active Gestalt principles.
 */
double VisionCore::RunTime()
{
  double sum = 0.;
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
      sum += principles[i]->RunTime();
  return sum;
}

/**
 * @brief Add a new Gestalt (of any type) to the system.
 * @param g  new Gestalt
 * @param inform  if true, inform other parts of the system of this new Gestalt, \n
 *                otherwise add quietly. Default is true.
 */
void VisionCore::NewGestalt(Gestalt *g, bool inform)
{
  Gestalts(g->GetType()).PushBack(g);
  RankedGestalts(g->GetType()).PushBack(g);
  if(inform)
    InformNewGestalt(g->GetType(), g->ID());
}

/**
 * @brief InformNewGestalt
 * Once a new Gestalt is created, inform all those interested.
 * TODO: let Principles register themselves here.
 * TODO: Kann man abfragen, ob initialisiert? => Funktioniert nicht, wenn man oben nicht initialisiert hat.
 */
void VisionCore::InformNewGestalt(Gestalt::Type type, unsigned id)
{
  switch(type)
  {
    case Gestalt::ARC:
      if(VisionCore::config.GetValueInt("FORM_CONVEX_ARC_GROUPS") == 1)
        Principles(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS)->InformNewGestalt(type, id);
      break;
    case Gestalt::A_JUNCTION:
      if(VisionCore::config.GetValueInt("FORM_CONVEX_ARC_GROUPS") == 1)
        Principles(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS)->InformNewGestalt(type, id);
      break;
    case Gestalt::CONVEX_ARC_GROUP:
      if(VisionCore::config.GetValueInt("FORM_ELLIPSES") == 1)
        Principles(GestaltPrinciple::FORM_ELLIPSES)->InformNewGestalt(type, id);
      break;
    case Gestalt::L_JUNCTION:
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      break;
    case Gestalt::COLLINEARITY:
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      break;
    case Gestalt::CLOSURE:
      if(VisionCore::config.GetValueInt("FORM_RECTANGLES") == 1)
        Principles(GestaltPrinciple::FORM_RECTANGLES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_FLAPS") == 1)
        Principles(GestaltPrinciple::FORM_FLAPS)->InformNewGestalt(type, id);
      break;
    case Gestalt::RECTANGLE:
      if(VisionCore::config.GetValueInt("FORM_FLAPS_ARI") == 1)
        Principles(GestaltPrinciple::FORM_FLAPS_ARI)->InformNewGestalt(type, id);
      break;
    case Gestalt::FLAP_ARI:
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      break;
    default:
      break;
  }
}


/**
 * @brief SetROI: Set the region of interest for faster growing search lines.
 * Growing of search lines with a gaussian function.
 */
void VisionCore::SetROI(const Vector2 center, double sigma)
{
  roi_center = center;
  roi_sigma = sigma;
}

}

