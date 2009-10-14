/**
 * $Id: VisionCore.hh,v 1.27 2007/03/03 10:12:24 mxz Exp mxz $
 *
 * TODO - for each principle and/or gestalt have a SetupParameters() function
 * TODO   called after getting the image
 * TODO - more gestalt principles, color!
 * TODO - feedback
 * TODO - different gestalt principles produce the same groups
 */

#ifndef Z_VISION_CORE_HH
#define Z_VISION_CORE_HH

#include <vector>
#include <opencv/cv.h>
#include "Namespace.hh"
#include "Array.hh"
#include "Config.hh"
#include "Color.hh"
#include "Vector2.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * Class to collect whatever statistics are interesting at the moment.
 * Basically one big continued hack.
 */
class Statistics
{
private:
  vector<double> time_series;
  vector<int> pix_cnt_series;
  vector<int> hop_cnt_series;
  vector<int> jcts_series;
  vector<int> clos_series;
  vector<int> attempt_path_search_series;
  vector<int> path_search_series;
  vector<int> path_max_visited_series;
  vector<double> path_avg_visited_series;
  vector<double> avg_clos_sig_series;

public:
  double time;
  int pix_cnt;
  int hop_cnt;
  int jcts;
  int clos;
  int attempt_path_search;
  int path_search;
  int path_max_visited;
  double path_avg_visited;
  int new_clos;
  double avg_clos_sig;

  Statistics();
  ~Statistics();
  void PushEntry();
};

/**
 * Class representing the complete vision system.
 */
class VisionCore
{
private:
  Config config;
  const IplImage *img;

private:
  void InitGestaltPrinciples();

public:
  GestaltPrinciple* principles[GestaltPrinciple::MAX_TYPE];
  Array<Gestalt*> gestalts[Gestalt::MAX_TYPE];
  Array<Gestalt*> ranked_gestalts[Gestalt::MAX_TYPE];
  double p_e;    ///< probability of an edgel
  double p_ee;   ///< probability of an edgel given another edgel
  Statistics stats;
  Vector2 roi_center;
  double roi_sigma;
  bool use_masking;

public:
  VisionCore(const string &config_name = "");
  ~VisionCore();
  void Configure(const string &config_name);
  Config& GetConfig() {return config;}
  void NewImage(const IplImage *new_img);
  bool HaveImage() {return img != 0;}
  const IplImage* GetImage() {return img;}
  int ImageArea() {return img->width*img->height;}
  void EnableGestaltPrinciple(GestaltPrinciple::Type p);
  void DisableGestaltPrinciple(GestaltPrinciple::Type p);
  bool IsEnabledGestaltPrinciple(GestaltPrinciple::Type p);
  RGBColor Pixel(int x, int y)
  {
    char *p = &img->imageData[y*img->widthStep + x*img->nChannels];
    return RGBColor(p[0], p[1], p[2]);
  }
  void ProcessImage();
  void ProcessImage(int runtime_ms);
  void ClearGestalts();
  void Draw(int detail = 0);
  void DrawImage();
  void DrawAllGestalts(int detail = 0);
  void DrawGestalts(Gestalt::Type type, int detail = 0);
  void DrawGestalt(Gestalt::Type type, unsigned num, int detail = 0);
  void DrawGestaltInfo(Gestalt::Type type, unsigned num);
  void DrawPrinciple(GestaltPrinciple::Type type, int detail = 0);
  unsigned PickGestaltAt(Gestalt::Type type, int x, int y,
      unsigned start_at, bool reject_masked = false);
  double RunTime();
  void SetROI(const Vector2 center, double sigma);
  void NewGestalt(Gestalt *g, bool inform = true);
  void InformNewGestalt(Gestalt::Type type, unsigned id);

  Array<Gestalt*>* Gestalts() {return gestalts;}
  Array<Gestalt*>& Gestalts(Gestalt::Type type)
  {
    return gestalts[type];
  }
  Gestalt* Gestalts(Gestalt::Type type, unsigned id)
  {
    return gestalts[type][id];
  }

  Array<Gestalt*>& RankedGestalts(Gestalt::Type type)
  {
    return ranked_gestalts[type];
  }
  Gestalt* RankedGestalts(Gestalt::Type type, unsigned id)
  {
    return ranked_gestalts[type][id];
  }

  unsigned NumGestalts(Gestalt::Type type)
  {
    return gestalts[type].Size();
  }

  GestaltPrinciple* Principles(GestaltPrinciple::Type type)
  {
    return principles[type];
  }
};

}

#endif

