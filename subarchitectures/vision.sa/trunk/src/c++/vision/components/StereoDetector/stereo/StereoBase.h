/**
 * @file StereoBase.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Base class for stereo caculation of Gestalts, with class Vertex2D, Surf2D, Vertex3D and Surf3D.
 */

#ifndef Z_STEREO_BASE_HH
#define Z_STEREO_BASE_HH

#ifdef HAVE_CAST
  #include <VisionData.hpp>
#endif

#include "StereoTypes.h"
#include "Gestalt3D.h"
#include "Vector.hh"
#include "Edgel.hh"


namespace Z
{

class StereoCore;		// forward declaration necessary

/**
 * @class StereoBase
 * @brief Base class for all stereo matching classes.
 */
class StereoBase
{
public:
  enum Type
  {
    STEREO_LJUNCTION,
    STEREO_CORNER,
    STEREO_ELLIPSE,
    STEREO_LINE,
    STEREO_CLOSURE,
    STEREO_RECTANGLE,
    STEREO_FLAP,
    STEREO_FLAP_ARI,
    STEREO_CUBE,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };						///< Type of stereo Gestalts for matching

  VisionCore *vcore[2];				///< Left and right vision core		/// TODO public??? => change!!!
  StereoCamera *stereo_cam;			///< Stereo camera parameters		/// TODO public??? => change!!!

  struct PruningParameter			///< Parameters, when pruned image will be processed at stereo core
  {
    bool pruning;				///< Pruned image delivered
    int offsetX;				///< Offset x-coordinate
    int offsetY;				///< Offset y-coordinate
    int scale;					///< Scale between original and pruned image
  };
  PruningParameter pPara;			///< Pruning parameters of an image.

protected:
  StereoCore *score;				///< Stereo core
  Type type;					///< StereoBase Type

private:
  bool enabled;					///< Enabled / disabled Stereo-Gestalt
  bool masking;					///< TODO 

public:
  StereoBase(StereoCore *sc);
  void EnablePrinciple(bool status);
  bool IsEnabled() {return enabled;}

  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);

  double MatchingScoreSurf(Surf2D &left_surf, Surf2D &right_surf, unsigned &match_offs);			/// TODO Gehört eigentlich zu den StereoTypes?
  double MatchingScorePoint(Vertex2D &left_point, Vertex2D &right_point);					/// TODO Gehört eigentlich zu den StereoTypes?
	

  // virtual functions for the stereo classes.
  virtual int NumStereoMatches() = 0;
#ifdef HAVE_CAST
  virtual bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id) = 0;
#endif
//  virtual void Draw(int side, bool masked = false) = 0;//{}				// TODO pure virtual setzen																			/// TODO Sollten alle pure virtual (=0) sein.
  virtual void DrawMatched(int side, bool single, int id, int detail) = 0;
  virtual void Process() = 0;
  virtual void Process(int oX, int oY, int sc) = 0;
  virtual void ClearResults() {}						// TODO pure virtual setzen

  virtual void Get3DGestalt(Array<double> &values, int id) {}			// TODO pure virtual setzen ?

};

}

#endif
