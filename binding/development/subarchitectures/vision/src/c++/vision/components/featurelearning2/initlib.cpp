#include <vision/matlab/libFeatureLearningCtf.h> // includes mclmcr.h
#include "initlib.h"

static int initialized = 0;
extern "C"
{
   void DoInitializeCosyLib()
   {
      if (initialized) return;
      if (!mclInitializeApplication(NULL,0)) {
         printf("Could not initialize the program properly\n");
         return;
      }
      if (! libFeatureLearningCtfInitialize() ) {
         printf("Could not initialize the libFeatureLearningCtf library properly\n");
         return;
      }
      initialized = 1;
   }
}
