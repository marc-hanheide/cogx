/**
 * @file FormSegments.hh
 * @author Zillich
 * @date Februar 2007
 * @version 0.1
 * @brief Form edge segments with Matas canny edge detector.
 **/

#ifndef Z_FORM_SEGMENTS_HH
#define Z_FORM_SEGMENTS_HH

#include "IdImage.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Class of Gestalt principle FormSegments
 */ 
class FormSegments : public GestaltPrinciple
{
private:
  const Image *cur_img;
  float alpha, omega;		// alpha and omega of the canny e.d.
  bool done;

  void CreateSegmentsMatas();
  void DrawToEdgeImage(unsigned seg);
  void Create();
  void Rank();
  void SegmentArcsLines();

public:
  static IdImage *edge_img; ///< id image containing segment ids for each edgel
  static float *dir_img;    ///< image containing gradient orientations
  static int num_edgels;    ///< total number of edgels

  FormSegments(Config *cfg);
  virtual ~FormSegments();
  virtual void Reset(const Image *img);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate() {return !done;}
  virtual void SetCanny(int alpha, int omega);
};

}

#endif
