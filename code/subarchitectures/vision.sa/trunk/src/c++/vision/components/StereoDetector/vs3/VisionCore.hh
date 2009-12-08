/**
 * @file VisionCore.hh
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Vision Core for perceptual grouping.
 *
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
 * @brief Class representing the perceptual grouping system.
 */
class VisionCore
{
private:
  Config config;																								///< configuration of the perceptual grouping
  const IplImage *img;																					///< iplImage

  GestaltPrinciple* principles[GestaltPrinciple::MAX_TYPE];			///< principle list 
  Array<Gestalt*> gestalts[Gestalt::MAX_TYPE];									///< gestalt list 
  Array<Gestalt*> ranked_gestalts[Gestalt::MAX_TYPE];						///< ranked_gestalt list

  void InitGestaltPrinciples();

public:

	double p_e;																										///< probability of an edgel TODO das sollte hier weg => für sig-Berechnung
	double p_ee;																									///< probability of an edgel given another edgel

  Vector2 roi_center;																						///< center of the region of interest
  double roi_sigma;																							///< sigma of the region of interest
  bool use_masking;																							///< use masking of gestalts?

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
  RGBColor Pixel(int x, int y)																						/// TODO Sollte das hier nicht zu den Draw-Sachen? geht das?
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
  unsigned PickGestaltAt(Gestalt::Type type, int x, int y, unsigned start_at, bool reject_masked = false);
  double RunTime();
  void SetROI(const Vector2 center, double sigma);
  void NewGestalt(Gestalt *g, bool inform = true);
  void InformNewGestalt(Gestalt::Type type, unsigned id);


	/**
	 * @brief Get width of the image.
	 * @return Returns the width of the image in pixel (unsigned).
	 */
  unsigned IW() {return img->width;}

	/**
	 * @brief Get height of the image.
	 * @return Returns the height of the image in pixel (integer).
	 */
  unsigned IH() {return img->height;}

  Array<Gestalt*>* Gestalts() {return gestalts;}
  Array<Gestalt*>& Gestalts(Gestalt::Type type) {return gestalts[type];}
  Gestalt* Gestalts(Gestalt::Type type, unsigned id) {return gestalts[type][id];}
  Array<Gestalt*>& RankedGestalts(Gestalt::Type type) {return ranked_gestalts[type];}
  Gestalt* RankedGestalts(Gestalt::Type type, unsigned id) {return ranked_gestalts[type][id];}
  unsigned NumGestalts(Gestalt::Type type) {return gestalts[type].Size();}
  GestaltPrinciple* Principles(GestaltPrinciple::Type type) {return principles[type];}
};

}

#endif

