
#ifndef GLOBALS_YO3IVLM7
#define GLOBALS_YO3IVLM7

#include "Enumerator.h"
#include <string>
#include <map>

namespace matlab {

extern CTypeEnumerator Enumerator;
extern std::map<std::string, int> labelConceptMap; // name -> 1=color, 2=shape
extern std::string ClfStartConfig;
extern void CheckInit();

} // namespace

#endif /* end of include guard: GLOBALS_YO3IVLM7 */
