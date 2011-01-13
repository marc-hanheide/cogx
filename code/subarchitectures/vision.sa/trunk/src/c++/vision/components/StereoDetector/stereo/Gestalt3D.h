/**
 * @file Gestalt3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated Gestalts in 3D.
 */

#ifndef Z_GESTALT3D_HH
#define Z_GESTALT3D_HH

#include "StereoTypes.h"

namespace Z
{

/**
 * @class Gestalt3D
 * @brief Base class for stereo calculated Gestalts in 3D.
 */
class Gestalt3D
{
public:

private:
  
protected:

public:

  Gestalt3D();

};


/**
 * @brief Class Rectangle3D
 */
class Rectangle3D : public Gestalt3D
{
public:
  Surf3D surf;                                                  ///< 3D surface

//   bool Reconstruct(StereoCamera *stereo_cam, TmpFlap &left, TmpFlap &right);
};

}

#endif
