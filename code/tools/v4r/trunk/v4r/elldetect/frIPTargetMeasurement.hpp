//
// (C) Steinbichler Optotechnik GmbH
//
// $History: frIPTargetMeasurement.hpp $
// 

#ifndef  _FRIPTARGETMEASUREMENT_HPP
#define  _FRIPTARGETMEASUREMENT_HPP

NAMESPACE_CLASS_BEGIN( CzN )

// define for the used calling convention
#define _CC_TOOLS __cdecl

///////////////////////////////////////////////////////////////////////////////
#ifdef _INDLL
                                       // module header used in DLL
                                       // module exists in DLL             
#   ifdef _FRIPTARGETMEASUREMENT
#      if defined WIN                                     // under Win16
#         define _API_IPTARGETMEASUREMENT    _export
#      elif defined WIN32
#         ifdef _MSC_VER                                  // MSVC and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllexport ) 
#         else                                            // other Compiler and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllexport )  // force errors !!!
#         endif
#      else                                               // other OS and DLL
#         define _API_IPTARGETMEASUREMENT    __declspec( dllexport )     // force errors !!!
#      endif
                                       // module header used in another DLL
#   else
#      if defined WIN                                     // under Win16
#         define _API_IPTARGETMEASUREMENT    
#      elif defined WIN32
#         ifdef _MSC_VER                                  // MSVC and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllimport ) 
#         else                                            // other Compiler and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllimport )  // force errors !!!
#         endif
#      else                                               // other OS and DLL
#         define _API_IPTARGETMEASUREMENT    __declspec( dllimport )     // force errors !!!
#      endif
#   endif

#else
                                       // module header used in application
                                       // module exists in DLL
#   ifdef _FRIPTARGETMEASUREMENT                    
#      if defined WIN                                     // under Win16
#         define _API_IPTARGETMEASUREMENT    
#      elif defined WIN32
#         ifdef _MSC_VER                                  // MSVC and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllimport ) 
#         else                                            // other Compiler and Win32 DLL
#            define _API_IPTARGETMEASUREMENT    __declspec( dllimport )  // force errors !!!
#         endif
#      else                                               // other OS and DLL
#         define _API_IPTARGETMEASUREMENT    __declspec( dllimport )     // force errors !!!
#      endif
#   else
                                       // module used in application
                                       // module exists not in DLL             
#      define _API_IPTARGETMEASUREMENT                                    
#   endif

#endif
///////////////////////////////////////////////////////////////////////////////



NAMESPACE_CLASS_END()

#endif









