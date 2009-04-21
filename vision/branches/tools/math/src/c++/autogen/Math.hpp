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

#ifndef ___home_tm_Work_03_CogX_tools_math_src_c___autogen_Math_hpp__
#define ___home_tm_Work_03_CogX_tools_math_src_c___autogen_Math_hpp__

#include <Ice/LocalObjectF.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/UndefSysMacros.h>

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

namespace cogx
{

namespace Math
{

struct Vector2
{
    ::Ice::Double x;
    ::Ice::Double y;

    bool operator==(const Vector2&) const;
    bool operator<(const Vector2&) const;
    bool operator!=(const Vector2& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Vector2& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Vector2& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Vector2& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Vector3
{
    ::Ice::Double x;
    ::Ice::Double y;
    ::Ice::Double z;

    bool operator==(const Vector3&) const;
    bool operator<(const Vector3&) const;
    bool operator!=(const Vector3& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Vector3& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Vector3& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Vector3& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Matrix33
{
    ::Ice::Double m00;
    ::Ice::Double m01;
    ::Ice::Double m02;
    ::Ice::Double m10;
    ::Ice::Double m11;
    ::Ice::Double m12;
    ::Ice::Double m20;
    ::Ice::Double m21;
    ::Ice::Double m22;

    bool operator==(const Matrix33&) const;
    bool operator<(const Matrix33&) const;
    bool operator!=(const Matrix33& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Matrix33& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Matrix33& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Matrix33& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Rect2
{
    ::cogx::Math::Vector2 pos;
    ::Ice::Double width;
    ::Ice::Double height;

    bool operator==(const Rect2&) const;
    bool operator<(const Rect2&) const;
    bool operator!=(const Rect2& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Rect2& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Rect2& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Rect2& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Sphere3
{
    ::cogx::Math::Vector3 pos;
    ::Ice::Double rad;

    bool operator==(const Sphere3&) const;
    bool operator<(const Sphere3&) const;
    bool operator!=(const Sphere3& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Sphere3& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Sphere3& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Sphere3& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Pose3
{
    ::cogx::Math::Vector3 pos;
    ::cogx::Math::Matrix33 rot;

    bool operator==(const Pose3&) const;
    bool operator<(const Pose3&) const;
    bool operator!=(const Pose3& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Pose3& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Pose3& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Pose3& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

}

}

#endif
