#include "libcosyvision.h" // includes mclmcr.h

void DestroyArray(mxArray* &mxavar)
{
   if (mxavar != NULL) mxDestroyArray(mxavar);
   mxavar = NULL;
}

int main (int argc, char* argv[])
{
   if (!mclInitializeApplication(NULL,0))
   {
      printf("Could not initialize the program properly\n");
      return 1;
   }
   if (! libcosyvisionInitialize() ) {
      printf("Could not initialize the cosyvision library properly\n");
      return 1;
   }

#if CTF_LIB
   mxArray* vmHS = NULL;
   if ( ! mlfVMstart(1, &vmHS))
      return 11;

   printf("mx-vmHS: %lx\n", vmHS);
   DestroyArray(vmHS);
#else
   mwArray vmHS2;
   VMstart(1, vmHS2);
#endif

   mclWaitForFiguresToDie(NULL);
   libcosyvisionTerminate();
   mclTerminateApplication();
   return 0;
}
