
//
// (C) Labor Dr. Steinbichler
//
// $History: sysys.hpp $
// 
// *****************  Version 10  *****************
// User: Af           Date: 20.12.05   Time: 16:06
// Updated in $/NT/DLLs/FRBASICS/INC
// disabled warning C4100: unreferenced formal parameter
// 
// *****************  Version 8  *****************
// User: Af           Date: 18.10.05   Time: 13:20
// Updated in $/NT/DLLs/FRBASICS/INC
// <af> new version 3.0
// 
// *****************  Version 8  *****************
// User: Arm          Date: 20.02.05   Time: 12:49
// Updated in $/NT/DLLs/Frbasics/INC
// 
// *****************  Version 7  *****************
// User: Arm          Date: 13.02.05   Time: 18:34
// Updated in $/NT/DLLs/Frbasics/INC
// 
// *****************  Version 6  *****************
// User: Arm          Date: 23.01.05   Time: 14:57
// Updated in $/NT/DLLs/Frbasics/INC
// 
// *****************  Version 5  *****************
// User: Arm          Date: 20.10.99   Time: 15:22
// Updated in $/NT/DLLs/Frbasics/INC
// library update
// 
// *****************  Version 5  *****************
// User: Hw           Date: 19.12.96   Time: 16:37
// Updated in $/FRBASICS.DLL/INC
// Disabled a further warning.
// 
// *****************  Version 3  *****************
// User: Hw           Date: 18.10.96   Time: 18:25
// Updated in $/FRBASICS.DLL/INC
// Disabled warnings 4251 and 4273 (non dll-interface) for whole project
// 
// *****************  Version 2  *****************
// User: Hw           Date: 12/12/95   Time: 9:25a
// Updated in $/FRBASICS.DLL/INC
// Added file header for keyword expansion.
//

#if !defined _SYSYS_HPP
#define      _SYSYS_HPP

////////////////////////////////////////////////////////////////////////////////
// Opens the namespace for all Steinbichler code
// This macro is called at the beginning of each *.hpp (after includes)
// and the beginning of each *.cpp (after includes)
#define  NAMESPACE_CLASS_BEGIN( Name )                                        \
            namespace Name                                                    \
            {

////////////////////////////////////////////////////////////////////////////////
// Closes the namespace for all Steinbichler code
// This macro is called at the end of each *.hpp (before #endif)
// and the end of each *.cpp
// There is also implemented an error detection for the TRY/CATCH macros
// to avoid missing call of CATCH_END after a catch block
#define  NAMESPACE_CLASS_END()                                                \
            }                                                                 

////////////////////////////////////////////////////////////////////////////////
// Defines the namespace for all Steinbichler code
#define  CzN     CzN

#if defined( _MSC_VER )
   // nonstandard extension used : nameless struct/union
   #pragma warning( disable : 4201 )   // disable in warning level 4
   // warning C4786: '' : identifier was truncated to '255' characters in the browser information 
   #pragma warning( disable : 4786 4996 )    
#endif

#define SUPD 358

#if defined( _WINDOWS ) 
   #include <svwin.h>                  // includes now the MFC stuff
#endif

#if defined( _MSC_VER )
   // nonstandard extension used : nameless struct/union
   #pragma warning( default : 4201 )
#endif

                                       // include frequently used headers here:
#include <memory.h>                    // malloc, memcpy etc.
#include <string.h>

#include <iostream>                  // streams
#include <fstream>

                                       // disable warnings non dll-interface...
#if defined( _MSC_VER )
   // C4251: 'identifier' : class 'type' needs to have dll-interface to be used by clients of class 'type2'
   // C4273: 'identifier' : inconsistent DLL linkage. dllexport assumed
   // C4275: non - DLL-interface classkey 'identifier' used as base for DLL-interface classkey 'identifier'
   // C4996: 'function': was declared deprecated
   // C4571: is generated for every catch(...) block when compiling with /EHs.
   // C4100: 'identifier' : unreferenced formal parameter
   #pragma warning( disable : 4251 4273 4275 4996 4571 4100) 
#endif



#endif
