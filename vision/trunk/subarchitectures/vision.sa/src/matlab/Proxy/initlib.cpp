#include "initlib.h"
#include "libFeatureLearningCtf.h"

static int initialized = 0;
static int terminated = 0;
extern "C"
{
   void InitFeatureLearningLib()
   {
      if (terminated) {
         printf("WARNING: Initialize called after Terminate\n");
      }

      if (initialized > 0) {
         initialized++;
         return;
      }

      if (!mclInitializeApplication(NULL,0)) {
         printf("Could not initialize the program properly\n");
         return;
      }
      if (! libFeatureLearningCtfInitialize() ) {
         printf("Could not initialize the libFeatureLearningCtf library properly\n");
         return;
      }
      initialized++;
   }

   void TermFeatureLearningLib()
   {
      try {
         if (initialized <= 0) return;
         if (terminated) {
            printf("WARNING: Terminate was already called.\n");
            return;
         }
         initialized --;
         if (initialized <= 0) {
            //~ mclWaitForFiguresToDie(NULL);
            //~ mclKillAllFigures(NULL); // crashes badly
            printf("calling libFeatureLearningCtfTerminate().\n");
            fflush(stdout);
            libFeatureLearningCtfTerminate();
            printf("calling mclTerminateApplication().\n");
            fflush(stdout);
            mclTerminateApplication();
            terminated++;
         }
      }
      catch (...) {
            printf("WARNING: Unknown Exception in TermFeatureLearningLib.\n");
            fflush(stdout);
            return;
      }
   }
}
