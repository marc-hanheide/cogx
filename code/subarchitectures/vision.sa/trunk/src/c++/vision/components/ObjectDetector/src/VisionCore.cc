/**
 * @file VisionCore.cc
 * @author Zillich
 * @date March 2007
 * @version 0.1
 * @brief Core of all vision calculations
 **/

#include "VisionCore.hh"

#include <time.h>
#include <signal.h>
#include <assert.h>
#include "Except.hh"
#include "Math.hh"
// #ifdef HAVE_IMLIB
// #include "FileVideo.hh"
// #endif
//#include "OpenCvVideo.hh"
#include "OpenCvImgSeqVideo.hh"
#include "OpenCvLiveVideo.hh"
#include "OpenCvBufferVideo.hh"

// #include "BufferVideo.hh"
#include "Draw.hh"

// Form and Track headers
#include "FormSegments.hh"
#include "FormLines.hh"
#include "FormArcs.hh"
#include "FormParallelLineGroups.hh"
#include "FormConvexArcGroups.hh"
#include "FormEllipses.hh"
#include "FormBalls.hh"
#include "FormJunctions.hh"
#include "FormExtEllipses.hh"
#include "FormCylinders.hh"
#include "FormCones.hh"
#include "FormCorners.hh"
#include "FormClosures.hh"
#include "FormExtClosures.hh"
#include "FormRectangles.hh"
#include "FormExtRectangles.hh"
#include "FormFlaps.hh"
#include "FormCubes.hh"
#include "FormWallLines.hh"
#include "FormWalls.hh"
#include "FormExits.hh"
#include "FormBestResults.hh"
#include "FormMotionField.hh"
#include "TrackRectangles.hh"
#include "TrackFlaps.hh"
#include "TrackCubes.hh"

// Gestalts
#include "Segment.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "LJunction.hh"
#include "TJunction.hh"		// TODO
#include "Ellipse.hh"
#include "Closure.hh"
#include "ExtRectangle.hh"
#include "ObjectTracker.hh"
#include "Flap.hh"
#include "Object.hh"

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


// ----------------------------------------------------------------------
// Statistics
// ----------------------------------------------------------------------

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
  FILE *file = fopen("stats.dat", "w");
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
  fclose(file);
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

// ----------------------------------------------------------------------
// Vision Core
// ----------------------------------------------------------------------

extern void SetActiveDrawArea(IplImage *iI);

Config VisionCore::config;
Image *VisionCore::img = 0;
Video *VisionCore::video = 0;
// IceInterface *VisionCore::ice = 0;
GestaltPrinciple* VisionCore::principles[GestaltPrinciple::MAX_TYPE];
Array<Gestalt*> VisionCore::gestalts[Gestalt::MAX_TYPE];
Array<unsigned> VisionCore::ranked_gestalts[Gestalt::MAX_TYPE];

double VisionCore::p_e = 0.;
double VisionCore::p_ee = 0.;

double VisionCore::camIntrinsic[4];
double VisionCore::camDistortion[4];
double VisionCore::camExtrinsic[12];

Statistics VisionCore::stats;


VisionCore::VisionCore(VideoType vid_type, const char *config_name, int cameraSource)
{
  // initialise math stuff (tables etc.)
  InitMath();
  switch(vid_type)
  {
    case VIDEO_TYPE_IMG_SEQ:
      video = new OpenCvImgSeqVideo();
      break;
    case VIDEO_TYPE_LIVE:
      video = new OpenCvLiveVideo(640, 480);
      break;
    case VIDEO_TYPE_BUFFER:
      video = new OpenCvBufferVideo();
      break;
		default: 
  		throw Except(__HERE__, "unknown video type");
		break;
  }
  img = 0;

//   wmap = 0;
  InitGestaltPrinciples();
  Configure(config_name);

// 	ariCamModel = new mx::CCameraModel();

// 	ice = new IceInterface();
}

VisionCore::VisionCore(Video *vid, const char *config_name, int cameraSource)
{
  // initialise math stuff (tables etc.)
  InitMath();
  video = vid;
  img = 0;
//   wmap = 0;
  InitGestaltPrinciples();
  Configure(config_name);

// 	ariCamModel = new mx::CCameraModel();

// 	ice = new IceInterface();
}

VisionCore::~VisionCore()
{
  delete video;
  delete img;

// 	delete ariCamModel;

// 	delete ice;

// 	delete wmap;

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
  principles[GestaltPrinciple::FORM_SEGMENTS] = new FormSegments(&config);
  principles[GestaltPrinciple::FORM_LINES] = new FormLines(&config);
  principles[GestaltPrinciple::FORM_ARCS] = new FormArcs(&config);
  //principles[GestaltPrinciple::FORM_PARALLEL_LINE_GROUPS] = new FormParallelLineGroups(&config);
  principles[GestaltPrinciple::FORM_CONVEX_ARC_GROUPS] = new FormConvexArcGroups(&config);
  principles[GestaltPrinciple::FORM_ELLIPSES] = new FormEllipses(&config);
	principles[GestaltPrinciple::FORM_BALLS] = new FormBalls(&config);
  principles[GestaltPrinciple::FORM_JUNCTIONS] = new FormJunctions(&config);
  principles[GestaltPrinciple::FORM_EXTELLIPSES] = new FormExtEllipses(&config);
  principles[GestaltPrinciple::FORM_CYLINDERS] = new FormCylinders(&config);
  principles[GestaltPrinciple::FORM_CONES] = new FormCones(&config);
  principles[GestaltPrinciple::FORM_CORNERS] = new FormCorners(&config);
  principles[GestaltPrinciple::FORM_CLOSURES] = new FormClosures(&config);
  principles[GestaltPrinciple::FORM_EXTCLOSURES] = new FormExtClosures(&config);
  principles[GestaltPrinciple::FORM_RECTANGLES] = new FormRectangles(&config);
  principles[GestaltPrinciple::FORM_EXTRECTANGLES] = new FormExtRectangles(&config);
  principles[GestaltPrinciple::FORM_FLAPS] = new FormFlaps(&config);
  principles[GestaltPrinciple::FORM_CUBES] = new FormCubes(&config);
  principles[GestaltPrinciple::FORM_WALLLINES] = new FormWallLines(&config);
  principles[GestaltPrinciple::FORM_WALLS] = new FormWalls(&config);
  principles[GestaltPrinciple::FORM_EXITS] = new FormExits(&config);
  principles[GestaltPrinciple::FORM_BEST_RESULTS] = new FormBestResults(&config);
  principles[GestaltPrinciple::FORM_MOTION_FIELD] = new FormMotionField(&config);
  principles[GestaltPrinciple::TRACK_RECTANGLES] = new TrackRectangles(&config);
  principles[GestaltPrinciple::TRACK_FLAPS] = new TrackFlaps(&config);
  principles[GestaltPrinciple::TRACK_CUBES] = new TrackCubes(&config);
  principles[GestaltPrinciple::TRACK_OBJECTS] = new ObjectTracker(&config);
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

void VisionCore::Configure(const char *config_name)
{
  if(config_name != 0 && strlen(config_name) > 0)
    config.Load(config_name);
  /*printf("--- configuration: ------------------------\n");
  for(map<string, string>::iterator i = config.items.begin();
      i != config.items.end(); ++i)
    printf("%s = %s\n", i->first.c_str(), i->second.c_str());
  printf("------------------------------------------\n");*/
}

/**
 * @brief Clear the Gestalts for the processing of the next image.
 * Clear only Gestalt, where all elements would be cleared.
 */
void VisionCore::ClearGestalts()
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
  {
    for(int j = gestalts[i].Size()-1; j >= 0 ; j--)
		{	
			delete gestalts[i][j];
		}

		gestalts[i].Clear();
		ranked_gestalts[i].Clear();
  }
}

void VisionCore::NewImage()
{
  delete img;
  const IplImage *ipl = video->CurrentFramePtr();

  img = new Image(ipl->imageData, ipl->width, ipl->height,
      video->BytesPerPixel(), video->ColorFmt(), false, video->BytesPerLine());

	/// Convert from RGB to BGR
// 	IplImage *nipl = cvCreateImageHeader(cvSize(ipl->width, ipl->height), IPL_DEPTH_8U, 3);
// 	nipl->imageData = img->Data();
// 	cvConvertImage( ipl, nipl, CV_CVTIMG_SWAP_RB);
// 	cvReleaseImageHeader(&nipl);

  ClearGestalts();
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
    {
      principles[i]->Reset(img);
      principles[i]->ResetRunTime();
    }
}

void VisionCore::InitBuffer(IplImage *img, bool copy)
{
  if (video->IsBuffer())
  {
    ((OpenCvBufferVideo*)video)->Init(img, copy);
    NewImage();
  }
}

void VisionCore::SetBuffer(IplImage *img)
{
  if (video->IsBuffer())
  {
    ((OpenCvBufferVideo*)video)->SetBuffer(img);
    NewImage();
  }
}

void VisionCore::LoadImage(const char *filename)
{
  if(video->IsLive() || video->IsBuffer())
    return;
  video->AddFrame(filename);
  NewImage();
}

void VisionCore::FirstImage()
{
  if(video->IsLive() || video->IsBuffer())
    return;
  video->MoveToStart();
  NewImage();
}

void VisionCore::LastImage()
{
  if(video->IsLive() || video->IsBuffer())
    return;
  video->MoveToEnd();
  NewImage();
}

bool VisionCore::NextImage()
{
  if(video->Forward())
  {
    NewImage();
    return true;
  }
  else
    return false;
}

bool VisionCore::PrevImage()
{
  if(video->Backward())
  {
    NewImage();
    return true;
  }
  else
    return false;
}

void VisionCore::ClearImages()
{
  if(video->IsLive() || video->IsBuffer() )
    return;
  video->ClearFrames();
  delete img;
  img = 0;
}

unsigned VisionCore::ImageNumber()
{
  return video->FrameCount();
}

const char *VisionCore::ImageName()
{
  return video->FrameName().c_str();
}

bool VisionCore::LiveVideo()
{
  return video->IsLive();
}

/*
static bool TimerExpired(timer_t timer)
{
  struct itimerspec tspec;
  timer_gettime(timer, &tspec);
  return tspec.it_value.tv_sec == 0 && tspec.it_value.tv_nsec == 0;
}
*/

void VisionCore::IceInitialise(const char* host, const char* port)
{
	static bool firstCall = true;
	char* h = const_cast<char*>(host);
 	char* p = const_cast<char*>(port);
	if (firstCall)
	{
// 		if(!ice->Initialise(h, p))
// 			printf("Failed to connect with ice-interface\n");
		firstCall = false;
	}
}

void VisionCore::ProcessImage(int runtime_ms, int ca, int co, bool sendIce)
{
// 	printf("VisionCore::ProcessImage: Process new image: %u - %u - %u\n", runtime_ms, ca, co);
// 	img->SavePPM("zImage.ppm");

	bool tracking = false;							///< enable / disable Gestalt tracking
	bool tracker3D = false;							///< enable 3D GPU tracker

  double rt_ms = (double) runtime_ms/1000.;		/// TODO rt_ms = runtime in Sekunden!
	
  // set parameter for canny edge detector
  Principles(GestaltPrinciple::FORM_SEGMENTS)->SetCanny(ca, co);

  struct timespec start, cur;						// processing time of Gestalt principles
	struct timespec i_start, i_cur;				// processing time of all incremental Gestalt principles
	struct timespec ni_start, ni_cur;			// processing time of all non-incremental Gestalt principles
	static double i_all = 0., ni_all = 0.;	// processing time of all incremental and non-incremental Gestalt principles.

  if(!HaveImage())
    return;

  /*tspec.it_interval.tv_sec = 0;
  tspec.it_interval.tv_nsec = 0;
  tspec.it_value.tv_sec = runtime_ms/1000;
  tspec.it_value.tv_nsec = 1000000*(runtime_ms%1000);
  sev.sigev_notify = SIGEV_NONE;
  sev.sigev_signo = 0;
  sev.sigev_value.sival_int = 0;
  sev.sigev_value.sival_ptr = 0;  // just to make sure
  memset(sev._sigev_un._pad, 0, __SIGEV_PAD_SIZE); // clear union
  sev.sigev_notify_function = 0;
  sev.sigev_notify_attributes = 0;
  timer_create(CLOCK_REALTIME, &sev, &timer);
  timer_settime(timer, 0, &tspec, NULL);*/


  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &i_start);
  try
  {
    //while(!TimerExpired(timer))
    do
    {
      for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
      {
        if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i) && principles[i]->NeedsOperate())
        {
// printf("operate principle: %u\n", i);
          principles[i]->Operate(true);
        }
      }
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cur);
// cout << "runtime: " << timespec_diff(&cur, &start) << endl; 
    } while(timespec_diff(&cur, &start) < rt_ms);
//    stats.time = principles[GestaltPrinciple::FORM_JUNCTIONS]->RunTime();
//    stats.jcts = NumCollinearities() + NumLJunctions() + NumTJunctions();
//    stats.clos = NumClosures();
//    stats.PushEntry();

/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
/// Diese non-incremental Funktionen sollten eigentlich alle automatisch nach der Berechnung aufgerufen werden. 
/// Wenn keine existierten (virtual functions), dann werden diese einfach nicht ausgeführt.
/// Wäre das mit der Reihenfolge der Ausführung möglich?

		// calculate time for non-incremental processing
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &i_cur);
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ni_start);

		// Try to hypothesise balls, when processing time is over 
    if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_BALLS))
      Principles(GestaltPrinciple::FORM_BALLS)->OperateNonIncremental();

    // Try to calculate wall lines, when process time is over
    if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_WALLLINES))
      Principles(GestaltPrinciple::FORM_WALLLINES)->OperateNonIncremental();

    // Try to calculate walls, when process time is over
    if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_WALLS))
      Principles(GestaltPrinciple::FORM_WALLS)->OperateNonIncremental();

    // Try to calculate exits, when process time is over
    if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_EXITS))
      Principles(GestaltPrinciple::FORM_EXITS)->OperateNonIncremental();

		if (tracking)
		{
			// Create the final motion field at the end of the Gestalt principle processing, becor Tracking starts.
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_MOTION_FIELD))
				Principles(GestaltPrinciple::FORM_MOTION_FIELD)->OperateNonIncremental();
	
			// Start tracking of rectangles, when process time is over
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::TRACK_RECTANGLES))
				Principles(GestaltPrinciple::TRACK_RECTANGLES)->OperateNonIncremental();
	
			// Start tracking of flaps, when process time is over
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::TRACK_FLAPS))
				Principles(GestaltPrinciple::TRACK_FLAPS)->OperateNonIncremental();
	
			// Start tracking of cubes, when process time is over
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::TRACK_CUBES))
				Principles(GestaltPrinciple::TRACK_CUBES)->OperateNonIncremental();
	
		}


	// printf("Track_Objects - Start\n");
			// Start tracking of objects, when process time is over
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::TRACK_OBJECTS))
				Principles(GestaltPrinciple::TRACK_OBJECTS)->OperateNonIncremental();
	// printf("Track_Objects - End\n");
			

			// Show the best results of interesting Gestalts
			if(IsEnabledGestaltPrinciple(GestaltPrinciple::FORM_BEST_RESULTS))
				Principles(GestaltPrinciple::FORM_BEST_RESULTS)->OperateNonIncremental();

// 			printf("Processed all Gestalt principles: Number of detected objects: %u\n", NumObjects());
// 			printf("Segments/Lines: %u/%u\n", NumSegments(), NumLines());
// 			printf("Ellipses/L-Junctions: %u/%u\n", NumEllipses(), NumLJunctions());

		// calculate and print processing times
//     clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ni_cur);
// 		double processingTime = timespec_diff(&i_cur, &i_start) + timespec_diff(&ni_cur, &ni_start);
// 		i_all += timespec_diff(&i_cur, &i_start);
// 		ni_all += timespec_diff(&ni_cur, &ni_start);
// 		static unsigned image_nr = 0;
// 		cout << "Calculated image " << image_nr << endl;
// 		cout << "TIME: non-incremental processing time: " << processingTime << " (" << timespec_diff(&i_cur, &i_start) << " / " << timespec_diff(&ni_cur, &ni_start) << ")" << endl;
// 		cout << "WHOLE TIME: non-incremental processing time: " << i_all << " / " << ni_all << endl;
// 		image_nr++;

		/// Toms 3D tracker
//		if (tracker3D)
//		{
//			static Tracker3D tracker;										///< Toms 3D tracker
//			tracker.SetImage((unsigned char*) img->Data(), img->Width(), img->Height(), img->BytesPerPixel()*8);
//			tracker.SetObjectModel();
//			if(tracker.RunSigleImage())
//				printf("VisionCore::ProcessImage: Tracker3D: RunSingelImage sucessfull!\n");
//		}
//
		// delivering information via ice interface
// 		if (sendIce) 
// 			if(ice->SendObjects(true, true, true, true, true, true) < 0)
// 				printf("Could not send message via ice. No connection available.\n");

  }
  catch(Z::Except &e)
  {
    printf("Vision Core: %s\n", e.what());
  }
}


void VisionCore::ProcessImage()
{
  if(!HaveImage())
    return;
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


// ----------------------------------------------------------------------
// Drawing components for qt window.
// ----------------------------------------------------------------------
void VisionCore::Draw(int detail)
{
  DrawImage();
  DrawAllGestalts(detail);
}

void VisionCore::DrawImage()
{
  if(HaveImage())
    DrawImageRGB24(img->Data(), img->Width(), img->Height());
}

void VisionCore::DrawAllGestalts(int detail)
{
  for(int i = 0; i < Gestalt::MAX_TYPE; i++)
    for(unsigned j = 0; j < gestalts[i].Size(); j++)
      gestalts[i][j]->Draw(detail);
}

void VisionCore::DrawGestalts(Gestalt::Type type, int detail)
{
  for(unsigned j = 0; j < gestalts[type].Size(); j++)
    gestalts[type][j]->Draw(detail);
}

void VisionCore::DrawUnmaskedGestalts(Gestalt::Type type, int detail)
{
  for(unsigned j = 0; j < gestalts[type].Size(); j++)
	{
		if (!gestalts[type][j]->IsMasked())
			gestalts[type][j]->Draw(detail);
	}
}

void VisionCore::DrawGestalt(Gestalt::Type type, unsigned num, int detail)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->Draw(detail);
}

void VisionCore::DrawGestaltInfo(Gestalt::Type type, unsigned num)
{
  if(num < gestalts[type].Size())
    gestalts[type][num]->DrawInfo();
}

void VisionCore::DrawPrinciple(GestaltPrinciple::Type type, int detail)
{
  if(IsEnabledGestaltPrinciple(type))
    principles[type]->Draw(detail);
}

// ----------------------------------------------------------------------
// Drawing components for OpenCv IplImage drawings.
// ----------------------------------------------------------------------
void VisionCore::DrawArea(IplImage *iI)
{
	SetActiveDrawArea(iI);
}

/**
 * @brief
 * @return Returns next enabled Gestalt type.
 */
// int VisionCore::IplDrawGestalts(int type, int detail)
// {
// 	while (!IsEnabledGestaltPrinciple((GestaltPrinciple::Type)type)) type++;
// 
// printf("VisionCore::IplDrawGestalts: týpe: %i / detail %i\n", type, detail);
//   for(unsigned j = 0; j < gestalts[type].Size(); j++)
//     gestalts[type][j]->Draw(detail);
// 
// 	return type;
// }


// ----------------------------------------------------------------------
// Other stuff
// ----------------------------------------------------------------------
unsigned VisionCore::PickGestaltAt(Gestalt::Type type, int x, int y,
    unsigned start_after)
{
  unsigned start = (start_after == UNDEF_ID ? 0 : start_after + 1);
  for(unsigned j = start; j < gestalts[type].Size(); j++)
    if(gestalts[type][j]->IsAtPosition(x, y))
      return j;
  return UNDEF_ID;
}

double VisionCore::RunTime()
{
  double sum = 0.;
  for(int i = 0; i < GestaltPrinciple::MAX_TYPE; i++)
    if(IsEnabledGestaltPrinciple((GestaltPrinciple::Type)i))
      sum += principles[i]->RunTime();
  return sum;
}

unsigned NewGestalt(Gestalt *g, bool inform)
{
  if(inform)
    InformNewGestalt(g->GetType(), g->ID());
  return g->ID();
}

void InformNewGestalt(Gestalt::Type type, unsigned id)
{
  switch(type)
  {
  	case Gestalt::EXTELLIPSE:
//	  if(VisionCore::config.GetValueInt("FORM_CYLINDERS") == 1)
//		Principles(GestaltPrinciple::FORM_CYLINDERS)->InformNewGestalt(type, id);
//	  if(VisionCore::config.GetValueInt("FORM_CONES") == 1)
//		Principles(GestaltPrinciple::FORM_CONES)->InformNewGestalt(type, id);
      break;
    case Gestalt::L_JUNCTION:
			if(VisionCore::config.GetValueInt("FORM_MOTION_FIELD") == 1)
				Principles(GestaltPrinciple::FORM_MOTION_FIELD)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CORNERS") == 1)
        Principles(GestaltPrinciple::FORM_CORNERS)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CONES") == 1)
        Principles(GestaltPrinciple::FORM_CONES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      break;
    case Gestalt::E_JUNCTION:
      if(VisionCore::config.GetValueInt("FORM_EXTELLIPSES") == 1)
        Principles(GestaltPrinciple::FORM_EXTELLIPSES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CONES") ==1)
        Principles(GestaltPrinciple::FORM_CONES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CYLINDERS") ==1)
        Principles(GestaltPrinciple::FORM_CYLINDERS)->InformNewGestalt(type, id);
      break;
    case Gestalt::COLLINEARITY:
      if(VisionCore::config.GetValueInt("FORM_CLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_CLOSURES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      break;
    case Gestalt::CLOSURE:
      if(VisionCore::config.GetValueInt("FORM_EXTCLOSURES") == 1)
        Principles(GestaltPrinciple::FORM_EXTCLOSURES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_RECTANGLES") == 1)
        Principles(GestaltPrinciple::FORM_RECTANGLES)->InformNewGestalt(type, id);
      break;
    case Gestalt::RECTANGLE:
      if(VisionCore::config.GetValueInt("FORM_FLAPS") == 1)
        Principles(GestaltPrinciple::FORM_FLAPS)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("FORM_EXTRECTANGLES") == 1)
        Principles(GestaltPrinciple::FORM_EXTRECTANGLES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("TRACK_RECTANGLES") == 1)
        Principles(GestaltPrinciple::TRACK_RECTANGLES)->InformNewGestalt(type, id);
      break;
    case Gestalt::EXTRECTANGLE:
      if(VisionCore::config.GetValueInt("FORM_FLAPS") == 1)
        Principles(GestaltPrinciple::FORM_FLAPS)->InformNewGestalt(type, id);
      break;
    case Gestalt::FLAP:
      if(VisionCore::config.GetValueInt("FORM_CUBES") == 1)
        Principles(GestaltPrinciple::FORM_CUBES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("TRACK_FLAPS") == 1)
        Principles(GestaltPrinciple::TRACK_FLAPS)->InformNewGestalt(type, id);
      break;
		case Gestalt::BALL:
			if(VisionCore::config.GetValueInt("TRACK_OBJECTS") == 1)
        Principles(GestaltPrinciple::TRACK_OBJECTS)->InformNewGestalt(type, id);
      break;
    case Gestalt::CUBE:
      if(VisionCore::config.GetValueInt("TRACK_CUBES") == 1)
        Principles(GestaltPrinciple::TRACK_CUBES)->InformNewGestalt(type, id);
      if(VisionCore::config.GetValueInt("TRACK_OBJECTS") == 1)
        Principles(GestaltPrinciple::TRACK_OBJECTS)->InformNewGestalt(type, id);
      break;
    case Gestalt::CONE:
      if(VisionCore::config.GetValueInt("TRACK_OBJECTS") == 1)
        Principles(GestaltPrinciple::TRACK_OBJECTS)->InformNewGestalt(type, id);
      break;
    case Gestalt::CYLINDER:
      if(VisionCore::config.GetValueInt("TRACK_OBJECTS") == 1)
        Principles(GestaltPrinciple::TRACK_OBJECTS)->InformNewGestalt(type, id);
      break;

    case Gestalt::TRKT_CUBE:
      if(VisionCore::config.GetValueInt("TRACK_OBJECTS") == 1)
        Principles(GestaltPrinciple::TRACK_OBJECTS)->InformNewGestalt(type, id);
      break;

    default:
      break;
  }
}


// ------------------------------------------------------------------
// Get vs3-Gestalts for processing with cast system (cogx project)
// ------------------------------------------------------------------
void VisionCore::GetObject(Gestalt::Type type, unsigned number, CubeDef cd)
{
// 	switch(type)
// 	{
// 		case CUBE:
// 			break;	
// 		default: break;
// 	}
}

bool VisionCore::GetCube(unsigned number, CubeDef &cd, bool &masked)
{
	if(NumObjects() > number)
	{
		if(Objects(number)->type == Gestalt::CUBE && !Objects(number)->IsMasked())
		{
			cd = Objects(number)->cuD;
			masked = false;
			return true;
		}
		else masked = true;
		return true;
	}
	return false;
}


bool VisionCore::GetCylinder(unsigned number, CylDef &cd, bool &masked)
{
	if(NumObjects() > number)
	{
		if(Objects(number)->type == Gestalt::CYLINDER && !Objects(number)->IsMasked())
		{
			cd = Objects(number)->cyD;
			masked = false;
			return true;
		}
		else masked = true;
		return true;
	}
	return false;
}


/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
bool VisionCore::GetFlap(unsigned number, FlapDef &fd, bool &masked)
{
	if(NumFlaps() > number)
	{
		
/// flap ist kein object und hat keine 3D Parameter !!!! 



// 		if(Objects(number)->type == Gestalt::CUBE && !Objects(number)->IsMasked())
// 		{
// 			cd = Objects(number)->cuD;
// 			masked = false;
// 			return true;
// 		}
// 		else masked = true;
// 		return true;
	}
	return false;
}

// void VisionCore::SetROI(const Vector2 center, double sigma)
// {
//   roi_center = center;
//   roi_sigma = sigma;
// }
// 
// /**
//  * @brief Sets a weighting map for line significances, i.e. a possibility to set rois
//  * @brief Attention! It does not take care of image size
//  * @param fimg 8bit IplImage (weights have to be scaled 0..255)
//  */
// void VisionCore::SetWeightMap(IplImage *fimg)
// {
//   if (fimg->nChannels==1 && fimg->depth==IPL_DEPTH_8U)
//     wmap=fimg;
// }
// 
// /**
//  * @brief draw the voting image for debugging
//  */
// void VisionCore::DrawVoteImg(IplImage *img, bool use_colour)
// {
//   ((FormJunctions*)principles[GestaltPrinciple::FORM_JUNCTIONS])->DrawVoteImg(img,use_colour);
// }

	/**
	 * @brief Set camera parameters
	 * @param intrinsic Intrinsic camera parameters: fx, fy, cx, cy
	 * @param distortion Radial and tangential distortion: k1, k2, p1, p2
	 */
	void VisionCore::SetCamParameters(double *intrinsic, double *distortion, double *extrinsic)
	{
		for(unsigned i=0; i<4; i++)
		{
			camIntrinsic[i] = intrinsic[i];
			camDistortion[i] = distortion[i];
// printf("	VisionCore::SetCamParameters():	camIntrinsic %u: %4.3f\n", i, intrinsic[i]);
		}
		for(unsigned i=0; i<12; i++)
		{
			camExtrinsic[i] = extrinsic[i];
// printf("	VisionCore::SetCamParameters():	camExtrinsic %u: %4.3f\n", i, extrinsic[i]);
		}
	}

	/**
	 * @brief Get camera model
	 * @param m_cCamModel Camera model with intrinsic, extrinsic and distortion matrix
	 */
	void VisionCore::GetCamModel(mx::CCameraModel &m_cCamModel)
	{
		// Set camera parameters
		m_cCamModel.SetIntrinsic(camIntrinsic);
		m_cCamModel.SetDistortion(camDistortion);
		m_cCamModel.SetExtrinsic(camExtrinsic);
	}
}

