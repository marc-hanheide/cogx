#ifndef Wm5_h
#define Wm5_h

#include <Wm5Vector2.h>
#include <Wm5Vector3.h>
#include <Wm5GVector.h>
#include <Wm5Matrix3.h>
#include <Wm5GMatrix.h>
#include <Wm5Plane3.h>
#include <Wm5Quaternion.h>
#include <map>

namespace m {

  typedef nuklei_wmf::Vector2<double> Vector2;
  typedef nuklei_wmf::Vector3<double> Vector3;
  typedef nuklei_wmf::GVector<double> GVector;
  typedef nuklei_wmf::Quaternion<double> Quaternion;
  typedef nuklei_wmf::Matrix3<double> Matrix3;
  typedef nuklei_wmf::GMatrix<double> GMatrix;
  typedef nuklei_wmf::Plane3<double> Plane3;

  typedef Vector3 Direction;
  typedef Vector3 Location;
  typedef Vector3 Translation;

  typedef Quaternion Rotation;
  typedef Rotation Orientation;

  typedef std::pair< m::Vector3, m::Quaternion > Pose;

  /**
   * @brief Returns @f$ Xy + x @f$.
   */
  inline Vector3 transform(const Vector3& x,
                           const Matrix3& X,
                           const Vector3& y)
  {
    return X*y + x;
  }
    
  /**
   * @brief Returns @f$ Xy + x @f$.
   */
  inline Vector3 transform(const Vector3& x,
                           const Quaternion& X,
                           const Vector3& y)
  {
    return X.Rotate(y) + x;
  }
    
  /**
   * @brief @f$ z = X y + x,  Z = X Y @f$.
   */
  inline void transform(Vector3& z, Matrix3& Z,
                        const Vector3& x, const Matrix3& X,
                        const Vector3& y, const Matrix3& Y)
  {
    z = transform(x, X, y);
    Z = X*Y;
  }
    
  /**
   * @brief @f$ z = X y + x,  Z = X Y @f$.
   */
  inline void transform(Vector3& z, Quaternion& Z,
                        const Vector3& x, const Quaternion& X,
                        const Vector3& y, const Quaternion& Y)
  {
    z = transform(x, X, y);
    Z = X*Y;
  }
    
  /**
   * @brief @f$ z = X y + x,  Z = X Y @f$.
   */
  inline void transform(Vector3& z, Vector3& Z,
                        const Vector3& x, const Matrix3& X,
                        const Vector3& y, const Vector3& Y)
  {
    z = transform(x, X, y);
    Z = transform(Vector3::ZERO, X, Y);
  }
    
  /**
   * @brief @f$ z = X y + x,  Z = X Y @f$.
   */
  inline void transform(Vector3& z, Vector3& Z,
                        const Vector3& x, const Quaternion& X,
                        const Vector3& y, const Vector3& Y)
  {
    z = transform(x, X, y);
    Z = transform(Vector3::ZERO, X, Y);
  }

  /**
   * @brief Returns @f$ X^T (z-x) @f$.
   */
  inline Vector3 project(const Vector3& x,
                         const Matrix3& X,
                         const Vector3& z)
  {
    return X.Transpose() * (z-x);
  }
    
  /**
   * @brief Returns @f$ X^T (z-x) @f$.
   */
  inline Vector3 project(const Vector3& x,
                         const Quaternion& X,
                         const Vector3& z)
  {
    return X.Conjugate().Rotate(z-x);
  }
    
  /**
   * @brief @f$ y = X^T (z-x), Y = X^T Z @f$.
   */
  inline void project(Vector3& y, Matrix3& Y,
                      const Vector3& x, const Matrix3& X,
                      const Vector3& z, const Matrix3& Z)
  {
    y = project(x, X, z);
    Y = X.TransposeTimes(Z);
  }
    
  /**
   * @brief @f$ y = X^T (z-x), Y = X^T Z @f$.
   */
  inline void project(Vector3& y, Quaternion& Y,
                      const Vector3& x, const Quaternion& X,
                      const Vector3& z, const Quaternion& Z)
  {
    y = project(x, X, z);
    Y = X.Conjugate() * Z;
  }
    
  /**
   * @brief @f$ y = X^T (z-x), Y = X^T Z @f$.
   */
  inline void project(Vector3& y, Vector3& Y,
                      const Vector3& x, const Matrix3& X,
                      const Vector3& z, const Vector3& Z)
  {
    y = project(x, X, z);
    Y = project(Vector3::ZERO, X, Z);
  }
    
  /**
   * @brief @f$ y = X^T (z-x), Y = X^T Z @f$.
   */
  inline void project(Vector3& y, Vector3& Y,
                      const Vector3& x, const Quaternion& X,
                      const Vector3& z, const Vector3& Z)
  {
    y = project(x, X, z);
    Y = project(Vector3::ZERO, X, Z);
  }
    
  /**
   * @brief @f$ x = z - Z Y^T y, X = Z Y^T @f$
   */
  inline void transfoTo(Vector3& x, Matrix3& X,
                        const Vector3& y, const Matrix3& Y,
                        const Vector3& z, const Matrix3& Z)
  {
    X = Z.TimesTranspose(Y);
    x = z - X*y;
  }

  /**
   * @brief @f$ x = z - Z Y^T y, X = Z Y^T @f$
   */
  inline void transfoTo(Vector3& x, Quaternion& X,
                        const Vector3& y, const Quaternion& Y,
                        const Vector3& z, const Quaternion& Z)
  {
    X = Z * Y.Conjugate();
    x = z - X.Rotate(y);
  }

  /**
   * @}
   */
    

}


#endif

