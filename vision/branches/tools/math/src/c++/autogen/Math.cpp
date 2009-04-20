// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `Math.ice'

#include <Math.hpp>
#include <Ice/BasicStream.h>
#include <Ice/Object.h>
#include <IceUtil/Iterator.h>
#include <IceUtil/ScopedArray.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 303
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

bool
cogx::Math::Vector2::operator==(const Vector2& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(x != __rhs.x)
    {
        return false;
    }
    if(y != __rhs.y)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Vector2::operator<(const Vector2& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(x < __rhs.x)
    {
        return true;
    }
    else if(__rhs.x < x)
    {
        return false;
    }
    if(y < __rhs.y)
    {
        return true;
    }
    else if(__rhs.y < y)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Vector2::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
}

void
cogx::Math::Vector2::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
}

bool
cogx::Math::Vector3::operator==(const Vector3& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(x != __rhs.x)
    {
        return false;
    }
    if(y != __rhs.y)
    {
        return false;
    }
    if(z != __rhs.z)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Vector3::operator<(const Vector3& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(x < __rhs.x)
    {
        return true;
    }
    else if(__rhs.x < x)
    {
        return false;
    }
    if(y < __rhs.y)
    {
        return true;
    }
    else if(__rhs.y < y)
    {
        return false;
    }
    if(z < __rhs.z)
    {
        return true;
    }
    else if(__rhs.z < z)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Vector3::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(z);
}

void
cogx::Math::Vector3::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(z);
}

bool
cogx::Math::Matrix33::operator==(const Matrix33& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(m00 != __rhs.m00)
    {
        return false;
    }
    if(m01 != __rhs.m01)
    {
        return false;
    }
    if(m02 != __rhs.m02)
    {
        return false;
    }
    if(m10 != __rhs.m10)
    {
        return false;
    }
    if(m11 != __rhs.m11)
    {
        return false;
    }
    if(m12 != __rhs.m12)
    {
        return false;
    }
    if(m20 != __rhs.m20)
    {
        return false;
    }
    if(m21 != __rhs.m21)
    {
        return false;
    }
    if(m22 != __rhs.m22)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Matrix33::operator<(const Matrix33& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(m00 < __rhs.m00)
    {
        return true;
    }
    else if(__rhs.m00 < m00)
    {
        return false;
    }
    if(m01 < __rhs.m01)
    {
        return true;
    }
    else if(__rhs.m01 < m01)
    {
        return false;
    }
    if(m02 < __rhs.m02)
    {
        return true;
    }
    else if(__rhs.m02 < m02)
    {
        return false;
    }
    if(m10 < __rhs.m10)
    {
        return true;
    }
    else if(__rhs.m10 < m10)
    {
        return false;
    }
    if(m11 < __rhs.m11)
    {
        return true;
    }
    else if(__rhs.m11 < m11)
    {
        return false;
    }
    if(m12 < __rhs.m12)
    {
        return true;
    }
    else if(__rhs.m12 < m12)
    {
        return false;
    }
    if(m20 < __rhs.m20)
    {
        return true;
    }
    else if(__rhs.m20 < m20)
    {
        return false;
    }
    if(m21 < __rhs.m21)
    {
        return true;
    }
    else if(__rhs.m21 < m21)
    {
        return false;
    }
    if(m22 < __rhs.m22)
    {
        return true;
    }
    else if(__rhs.m22 < m22)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Matrix33::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(m00);
    __os->write(m01);
    __os->write(m02);
    __os->write(m10);
    __os->write(m11);
    __os->write(m12);
    __os->write(m20);
    __os->write(m21);
    __os->write(m22);
}

void
cogx::Math::Matrix33::__read(::IceInternal::BasicStream* __is)
{
    __is->read(m00);
    __is->read(m01);
    __is->read(m02);
    __is->read(m10);
    __is->read(m11);
    __is->read(m12);
    __is->read(m20);
    __is->read(m21);
    __is->read(m22);
}

bool
cogx::Math::Rect2::operator==(const Rect2& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(pos != __rhs.pos)
    {
        return false;
    }
    if(width != __rhs.width)
    {
        return false;
    }
    if(height != __rhs.height)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Rect2::operator<(const Rect2& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(pos < __rhs.pos)
    {
        return true;
    }
    else if(__rhs.pos < pos)
    {
        return false;
    }
    if(width < __rhs.width)
    {
        return true;
    }
    else if(__rhs.width < width)
    {
        return false;
    }
    if(height < __rhs.height)
    {
        return true;
    }
    else if(__rhs.height < height)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Rect2::__write(::IceInternal::BasicStream* __os) const
{
    pos.__write(__os);
    __os->write(width);
    __os->write(height);
}

void
cogx::Math::Rect2::__read(::IceInternal::BasicStream* __is)
{
    pos.__read(__is);
    __is->read(width);
    __is->read(height);
}

bool
cogx::Math::Sphere3::operator==(const Sphere3& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(pos != __rhs.pos)
    {
        return false;
    }
    if(rad != __rhs.rad)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Sphere3::operator<(const Sphere3& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(pos < __rhs.pos)
    {
        return true;
    }
    else if(__rhs.pos < pos)
    {
        return false;
    }
    if(rad < __rhs.rad)
    {
        return true;
    }
    else if(__rhs.rad < rad)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Sphere3::__write(::IceInternal::BasicStream* __os) const
{
    pos.__write(__os);
    __os->write(rad);
}

void
cogx::Math::Sphere3::__read(::IceInternal::BasicStream* __is)
{
    pos.__read(__is);
    __is->read(rad);
}

bool
cogx::Math::Pose3::operator==(const Pose3& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(pos != __rhs.pos)
    {
        return false;
    }
    if(rot != __rhs.rot)
    {
        return false;
    }
    return true;
}

bool
cogx::Math::Pose3::operator<(const Pose3& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(pos < __rhs.pos)
    {
        return true;
    }
    else if(__rhs.pos < pos)
    {
        return false;
    }
    if(rot < __rhs.rot)
    {
        return true;
    }
    else if(__rhs.rot < rot)
    {
        return false;
    }
    return false;
}

void
cogx::Math::Pose3::__write(::IceInternal::BasicStream* __os) const
{
    pos.__write(__os);
    rot.__write(__os);
}

void
cogx::Math::Pose3::__read(::IceInternal::BasicStream* __is)
{
    pos.__read(__is);
    rot.__read(__is);
}
