/**
 * $Id: Vector2I.cc,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#include "Vector2I.hh"

namespace P 
{

/**
 * Vector2
 */
Vector2I::Vector2I()  //Constructor
{
}

Vector2I::Vector2I(const Vector2I &copyin)  //Copy constructor
{
  x = copyin.x;
  y = copyin.y;
}

Vector2I& Vector2I::operator=(const Vector2I &rhs)
{
  this->x = rhs.x;
  this->y = rhs.y;
  return *this;
}

int Vector2I::operator==(const Vector2I &rhs) const
{
  if(this->x != rhs.x) return 0;
  if(this->y != rhs.y) return 0;
  return 1;
}

int Vector2I::operator<(const Vector2I &rhs) const
{
  if( this->y < rhs.y) return 1;
  return 0;
}

}
