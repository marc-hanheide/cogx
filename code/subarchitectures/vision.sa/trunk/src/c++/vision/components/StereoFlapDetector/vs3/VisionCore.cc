/**
 * $Id: VisionCore.cc,v 1.44 2007/07/27 17:01:25 mxz Exp mxz $
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
//#include "FormRectangles.hh"
#include "FormFlaps.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "LJunction.hh"
#include "TJunction.hh"
#include "Closure.hh"
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

Statistics::Statistics()
{
  time = 0.;
  pix_cnt = 0;
  hop_cnt = 0;
  jcts = 0;
  clos = 0;
  attempt_path_search = 0;
  path_search = 0;
  path_max_visited = 0;
  path_avg_visited = 0;
  new_clos = 0;
  avg_clos_sig = 0.;
}

Statistics::~Statistics()
{
  /*FILE *file = fopen("stats.dat", "w");
  fprintf(file,
    "# pixcnt: total voting pixels drawn\n"
    "# hopcnt: voting pixel hops from line to line\n"
    "# jcts, clos: junctions and closures\n"
    "# attsrch: attempted closure searches, might be denied if isolated jct\n"
    "# search: performed path searches\n"
    "# maxvis: maximum number of visited nodes in any path search\n"
    "# avgvis: average number of visited nodes in path searches\n"
    "# avgcsig: average sig. of newly created closures in this time interval\n#\n"
    "#    time  pixcnt  hopcnt    jcts    clos attsrch  search  maxvis  avgvis"
    " avgcsig\n");
  for(unsigned i = 0; i < time_series.size(); i++)
    fprintf(file, "%9.6f  %6d  %6d  %6d  %6d  %6d  %6d  %6d  %6.1f  %6.3f\n",
      time_series[i],
      pix_cnt_series[i],
      hop_cnt_series[i],
      jcts_series[i],
      clos_series[i],
      attempt_path_search_series[i],
      path_search_series[i],
      path_max_visited_series[i],
      path_avg_visited_series[i],
      avg_clos_sig_series[i]);
  fclose(file);*/
}

void Statistics::PushEntry()
{
  time_series.push_back(time);
  pix_cnt_series.push_back(pix_cnt);
  hop_cnt_series.push_back(hop_cnt);
  jcts_series.push_back(jcts);
  clos_series.push_back(clos);
  attempt_path_search_series.push_back(attempt_path_search);
  path_search_series.push_back(path_search);
  path_max_visited_series.push_back(path_max_visited);
  path_avg_visited_series.push_back(path_avg_visited);
  avg_clos_sig_series.push_back(avg_clos_sig);
  // reset averge sig of new closures
  new_clos = 0;
  avg_clos_sig = 0.;
}


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

VisionCore::~VisionCore()
{
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    delete principles[i];
  ClearGestalts();
  ExitMath();
}

void VisionCore::InitGestaltPrinciples()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    principles[i] = 0;
  principles[GestaltPrinciple::FORM_SEGMENTS] = new FormSegments(this);
  principles[GestaltPrinciple::FORM_LINES] = new FormLines(this);
  principles[GestaltPrinciple::FORM_ARCS] = new FormArcs(this);
  //principles[GestaltPrinciple::FORM_PARALLEL_LINE_GROUPS] =
  //  new FormParallelLineGroups(this);
  principles[GestaltPrinciple::FORM_CONVEX_ARC_GROUPS] =
    new FormConvexArcGroups(this);
  principles[GestaltPrinciple::FORM_ELLIPSES] = new FormEllipses(this);
  principles[GestaltPrinciple::FORM_JUNCTIONS] = new FormJunctions(this);
  principles[GestaltPrinciple::FORM_ARC_JUNCTIONS] = new FormArcJunctions(this);
  principles[GestaltPrinciple::FORM_CLOSURES] = new FormClosures(this);
  //principles[GestaltPrinciple::FORM_RECTANGLES] = new FormRectangles(this);
  principles[GestaltPrinciple::FORM_FLAPS] = new FormFlaps(this);
  for(unsigned i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    config.AddItem(GestaltPrinciple::TypeName((GestaltPrinciple::Type)i), "0");
}

void VisionCore::EnableGestaltPrinciple(GestaltPrinciple::Type p)
{
  string type = GestaltPrinciple::TypeName((GestaltPrinciple::Type)p);
  config.items[type] = "1";
}

void VisionCore::DisableGestaltPrinciple(GestaltPrinciple::Type p)
{
  string type = GestaltPrinciple::TypeName((GestaltPrinciple::Type)p);
  config.items[type] = "0";
}

bool VisionCore::IsEnabledGestaltPrinciple(GestaltPrinciple::Type p)
{
  return principles[p] != 0 &&
    config.GetValueInt(GestaltPrinciple::TypeName(p)) == 1;
}

void VisionCore::Configure(const string &config_name)
{
  if(!config_name.empty())
    config.Load(config_name);
  /*printf("--- configuration: ------------------------\n");
  for(map<string, string>::iterator i = config.items.begin();
      i != config.items.end(); ++i)
    printf("%s = %s\n", i->first.c_str(), i->second.c_str());
  printf("------------------------------------------\n");*/
}

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
 * Process the current image incrementally for a given amount of time.
 * @param runtime_ms  amount of time to run, in microseconds.
 */
void VisionCore::ProcessImage(int runtime_ms)
{
  struct timespec start, cur;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  try
  {
    do
    {
      for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
      {
        if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i) &&
           principles[i]->NeedsOperate())
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
    stats.time = principles[GestaltPrinciple::FORM_JUNCTIONS]->RunTime();
    stats.jcts = NumCollinearities(this) + NumLJunctions(this) +
      NumTJunctions(this);
    stats.clos = NumClosures(this);
    stats.PushEntry();
  }
  catch(Z::Except &e)
  {
    printf("%s\n", e.what());
  }
}

/**
 * Older, non-incremental (non-anytime) version of ProcessImage
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
 * Draw the original image and all the gestalts we have.
 */
void VisionCore::Draw(int detail)
{
  DrawImage();
  DrawAllGestalts(detail);
}

/**
 * Draw only the original image.
 */
void VisionCore::DrawImage()
{
  if(img != 0)
    DrawImageRGB24(img->imageData, img->width, img->height);
}

/**
 * Draw all the gestalts we have.
 */
void VisionCore::DrawAllGestalts(int detail)
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
    for(unsigned j = 0; j < gestalts[i].Size(); j++)
      gestalts[i][j]->Draw(detail);
}

/**
 * Draw all gestalts of given type.
 */
void VisionCore::DrawGestalts(Gestalt::Type type, int detail)
{
  for(unsigned j = 0; j < gestalts[type].Size(); j++)
    gestalts[type][j]->Draw(detail);
}

/**
 * Draw just a single gestalt.
 */
void VisionCore::DrawGestalt(Gestalt::Type type, unsigned num, int detail)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->Draw(detail);
}

/**
 * Draw whatever pictorial info a gestalt has to offer.
 */
void VisionCore::DrawGestaltInfo(Gestalt::Type type, unsigned num)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->DrawInfo();
}

/**
 * Draw whatever the given gestalt principle has to draw.
 */
void VisionCore::DrawPrinciple(GestaltPrinciple::Type type, int detail)
{
  if(IsEnabledGestaltPrinciple(type))
    principles[type]->Draw(detail);
}

/**
 * Returns id of first gestalt at pixel position (x,y).
 * start_after can be used to skip the first gestalts. So all gestalts at x,y
 * can be selected consecutively.
 * If mask is set to true, only unmasked gestalts will be returned.
 */
unsigned VisionCore::PickGestaltAt(Gestalt::Type type, int x, int y,
    unsigned start_after, bool reject_masked)
{
  unsigned start = (start_after == UNDEF_ID ? 0 : start_after + 1);
  for(unsigned j = start; j < gestalts[type].Size(); j++)
    if( gestalts[type][j]->IsAtPosition(x, y) &&
        (!reject_masked || !gestalts[type][j]->IsMasked()) )
      return j;
  return UNDEF_ID;
}

/**
 * Accumulated runtime of active Gestalt principles.
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
 * Add a new Gestalt (of any type) to the system.
 * @param g  new Gestalt
 * @param inform  if true, inform other parts of the system of this new Gestalt,
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
 * Once a new Gestalt is created, inform all those interested.
 * TODO: let Principles register themselves here.
 */
void VisionCore::InformNewGestalt(Gestalt::Type type, unsigned id)
{
  switch(type)
  {
    case Gestalt::ARC:
      if(VisionCore::config.GetValueInt("FORM_CONVEX_ARC_GROUPS") == 1)
        Principles(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS)->InformNewGestalt(
            type, id);
      break;
    case Gestalt::A_JUNCTION:
      if(VisionCore::config.GetValueInt("FORM_CONVEX_ARC_GROUPS") == 1)
        Principles(GestaltPrinciple::FORM_CONVEX_ARC_GROUPS)->InformNewGestalt(
            type, id);
      break;
    case Gestalt::CONVEX_ARC_GROUP:
      if(VisionCore::config.GetValueInt("FORM_ELLIPSES") == 1)
        Principles(GestaltPrinciple::FORM_ELLIPSES)->InformNewGestalt(
            type, id);
      break;
    case Gestalt::L_JUNCTION:
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      break;
    case Gestalt::COLLINEARITY:
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      break;
    case Gestalt::CLOSURE:
      if(VisionCore::config.GetValueInt("FORM_FLAPS") == 1)
        Principles(GestaltPrinciple::FORM_FLAPS)->InformNewGestalt(type, id);
      break;
    /* HACK: FormRectangles not implemented yet
    case Gestalt::CLOSURE:
      if(VisionCore::config.GetValueInt("FORM_RECTANGLES") == 1)
        Principles(GestaltPrinciple::FORM_RECTANGLES)->InformNewGestalt(type, id);
      break;
     */
    default:
      break;
  }
}

void VisionCore::SetROI(const Vector2 center, double sigma)
{
  roi_center = center;
  roi_sigma = sigma;
}

}

