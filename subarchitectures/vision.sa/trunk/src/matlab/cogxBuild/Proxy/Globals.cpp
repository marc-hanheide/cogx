
#include "Globals.h"
#include "initlib.h"
#include "MatlabHelper.h"
#include "libVisualLearnerCtf.h"

using namespace std;

namespace matlab {

CTypeEnumerator Enumerator;
std::map<std::string, int> labelConceptMap; // name -> 1=color, 2=shape
std::string ClfStartConfig;

class CInitializer
{
public:
   CInitializer() {
      printf("VisualLearnerProxy initializing\n");
      InitVisualLearnerLib();

      printf("ClfStartConfig='%s'\n", ClfStartConfig.c_str());

      // Load global variables 
      mwArray clfConfig(ClfStartConfig.c_str());
      CLFstart(clfConfig);
   }
   ~CInitializer() {
      printf("VisualLearnerProxy terminating\n");
      TermVisualLearnerLib();
   }

   void initEnumeration() {
      try {
         Enumerator.clear();
         labelConceptMap.clear();
         mwArray avNames, scConcept;
         getGlobalArray(1, avNames, "Coma", "Coma.avNames");
         getGlobalArray(1, scConcept, "Coma", "Coma.SCC");
         mwArray dims = avNames.GetDimensions();
         double dim0 = dims.Get(mwSize(1), 1);
         for (int i = 0; i < dim0; i++) {
            mwString mws = avNames.Get(mwSize(1), i+1).ToString();
            int concept = scConcept.Get(mwSize(2), i+1, 2);
            string label((const char*)mws);
            Enumerator.addMapping(label, i+1);
            labelConceptMap[label] = concept;
            printf(" ... %d .. %s .. c%d \n", i+1, label.c_str(), concept);
         }
      }
      catch (...) {
         printf(" **** FAILED to extract from Matlab: Coma.avNames, Coma.SCC\n");
         Enumerator.clear();
         Enumerator.addMapping("red", 1);
         Enumerator.addMapping("green", 2);
         Enumerator.addMapping("blue", 3);
         Enumerator.addMapping("yellow", 4);
         Enumerator.addMapping("black", 5);
         Enumerator.addMapping("white", 6);
         Enumerator.addMapping("orange", 7);
         Enumerator.addMapping("pink", 8);
         Enumerator.addMapping("compact", 9);
         Enumerator.addMapping("elongated", 10);
      }
   }

} *pInitializer=NULL;

class CTerminator
{
public:
   ~CTerminator() {
      if (pInitializer) delete pInitializer;
      pInitializer = NULL;
   }
} Terminator;

void CheckInit()
{
   if (!pInitializer) {
      pInitializer = new CInitializer();
      pInitializer->initEnumeration();
   }
}

} // namespace
