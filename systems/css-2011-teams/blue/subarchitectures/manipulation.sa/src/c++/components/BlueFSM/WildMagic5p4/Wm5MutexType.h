// Geometric Tools, LLC
// Copyright (c) 1998-2010
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

#ifndef NUKLEI_WMFMUTEXTYPE_H
#define NUKLEI_WMFMUTEXTYPE_H

#include "Wm5CoreLIB.h"

//----------------------------------------------------------------------------
// Mutex type for Linux/Apple.  The file pthread.h exposes only native data
// types, so including it here does not suck in lots of extra stuff.
//----------------------------------------------------------------------------
#include <pthread.h>
namespace nuklei_wmf
{
    typedef struct
    {
        pthread_mutexattr_t Attribute;
        pthread_mutex_t Mutex;
    }
    MutexType;
}

#endif
