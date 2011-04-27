/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_COMPUTE_OBJECT_MODEL_3D_HH
#define P_COMPUTE_OBJECT_MODEL_3D_HH

#include "PNamespace.hh"
#include "Object3D.hh"
#include "Array.hh"
#include "Definitions.hh"
#include "CodebookEntry.hh"
#include "Geometry.hh"
#include "Homography.hh"


namespace P
{

/**
 * Create a plane model
 * Map keypoints to zero centered coordinates and create codebook
 */
class ModelObject3D
{
private:
  void ComputeNewHnorm(P::Array<KeypointDescriptor*> &keys, Matrix &Hnorm);
  void InsertMeanShift(Array<KeypointDescriptor* > &keys, P::Array<CodebookEntry*> &codebook, Matrix &H);


public:
  ModelObject3D();
  ~ModelObject3D();

  void AddToModel(Array<KeypointDescriptor *> &keys, Object3D &obj);
  void SaveModel(const char *filename, Object3D &obj);
  bool LoadModel(const char *filename, Object3D &obj);
};

/*********************** INLINE METHODES **************************/


}

#endif

