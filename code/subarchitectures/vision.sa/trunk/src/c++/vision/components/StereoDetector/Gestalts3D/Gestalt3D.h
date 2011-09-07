/**
 * @file Gestalt3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated Gestalts in 3D.
 */

#ifndef Z_GESTALT3D_HH
#define Z_GESTALT3D_HH

#include "v4r/TomGine/tgTomGineThread.h"

#include "StereoTypes.h"
#include "GraphLink.h"

#include <VisionData.hpp>

namespace Z
{

/**
 * @class Gestalt3D
 * @brief Base class for stereo calculated Gestalts in 3D.
 */
class Gestalt3D
{
public:
  enum Type
  {
    PATCH,
    SEGMENT,
    COLLINEARITY,
    LJUNCTION,
    CORNER,
    LINE,
    CLOSURE,
    RECTANGLE,
    FLAP,
    ELLIPSE,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:

  Type type;                // Type of stereo Gestalt
  unsigned id;              // Unique ID of the stereo Gestalt (with respect to the core)
  
  unsigned vs3ID;           // ID, if we got it from one vs3 Gestalt (Kinect)
  unsigned vs3IDs[2];       // IDs of the left and right vs3 Gestalts				/// TODO für alle 3D Gestalts nachziehen
  double sig;               // Significance value
  unsigned rank;	          // Rank of the the stereo Gestalt

  unsigned nodeID;          // Unique ID of the Gestalt (used for graph building/SVM/Learning)
  unsigned objectLabel;     // Object label from Plane-Popout (ground truth data)
  unsigned graphCutLabel;   // Labels from the graph cut

public:
  static const char* TypeName(Type t);
  static const int TypeNameLength(Type t);
  static Type EnumType(const char *type_name);


public:
  Gestalt3D(Type _type);
  
  Type GetType() const {return type;}
  double GetSignificance() {return sig;}
  unsigned GetVs3ID() {return vs3ID;}
  unsigned GetVs3ID(unsigned side) {return vs3IDs[side];}
  
  void SetID(unsigned _id) {id = _id;}
  unsigned GetID() {return id;}
  
  void SetNodeID(unsigned _id) {nodeID = _id;}
  unsigned GetNodeID() {return nodeID;}
  
  void SetObjectLabel(const unsigned i) {objectLabel = i;}
  unsigned GetObjectLabel() {return objectLabel;}

  void SetGraphCutLabel(const unsigned i) {graphCutLabel = i;}
  unsigned GetGraphCutLabel() {return graphCutLabel;}

  
  virtual bool GetLinks(vector<GraphLink> &links) {return false;}                                                 /// TODO ARI: noch notwendig?
//  virtual void DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool randomColor) {}                            // TODO pure virtual
  virtual void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool use_color = false, float color = 0.0) {}   // 
  virtual void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters camPars) {}              // TODO pure virtual ???

  virtual void PrintGestalt3D() {}                                                                                // TODO pure virtual
};

}

#endif
