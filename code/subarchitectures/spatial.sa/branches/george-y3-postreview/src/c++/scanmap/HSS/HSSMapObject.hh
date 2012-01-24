//
// = FILENAME
//    HSSMapObject.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSMapObject_hh
#define HSSMapObject_hh

#include "HSSutils.hh"

namespace HSS {

/**
 * The generic map object
 *
 * @author Patric Jensfelt
 * @see
 */
class MapObject {
public:
  enum {
    TYPE_ROBOTPOSE = 0,
    TYPE_SCAN,
    TYPE_DOOR,
    NUM_TYPES // should always be the last in the list
  };

  MapObject(int type, int size)
    :m_Type(type),
     m_StateSize(size),
     m_StatePos(-1)
  {}

  MapObject(int type, int size, int pos)
    :m_Type(type),
     m_StateSize(size),
     m_StatePos(pos)
  {}

  MapObject(const MapObject &src)
  {    
    *this = src;
  }

  int getType() const { return m_Type; }
  int getStatePos() const { return m_StatePos; }
  int getStateSize() const { return m_StateSize; }
  
  void setStatePos(int p) { m_StatePos = p; }
  int decrementStatePos(int dpos) { m_StatePos -= dpos; return m_StatePos; }
  void setStateSize(int s) { m_StateSize = s; }

  Eigen::Vector3d getVector3(const Eigen::VectorXd &X) const
  {
    Eigen::Vector3d ret;
    for (int i = 0; i < 3; i++) ret[i] = X[getStatePos()+i];
    return ret;
  }
  
  Eigen::Matrix3d getMatrix3(const Eigen::MatrixXd &P) const
  {
    Eigen::Matrix3d ret;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        ret(i,j) = P(getStatePos()+i,getStatePos()+j);
      }
    }
    return ret;
  }
  
  /**
   * Call this function to normalize the state information such as
   * angles. 
   *
   * The default behavior is to normalize the 3rd variable which is
   * assumed to be an angle.
   */
  virtual void normalize(Eigen::VectorXd &X)
  {
    X[getStatePos() + 2] = HSS::pi_to_pi(X[getStatePos() + 2]);
  }

private:
  // Type of map objects
  int m_Type;

  // The number of variables in the state vector
  int m_StateSize;

  // Position in the statevector for this scan
  int m_StatePos;  
};

}; // namespace HSS

#endif // HSSMapObject_hh
