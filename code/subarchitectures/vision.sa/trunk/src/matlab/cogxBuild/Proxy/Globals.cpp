
#include "Globals.h"
#include "initlib.h"
#include "MatlabHelper.h"
#include "libVisualLearnerCtf.h"
#include "LoggerMacro.h"

using namespace std;

namespace matlab {

CTypeEnumerator Enumerator;
std::map<std::string, int> labelConceptMap; // name -> 1=color, 2=shape
std::string ClfStartConfig;

class CInitializer
{
public:
   CInitializer() {
      INFO("VisualLearnerProxy initializing");
      InitVisualLearnerLib();
      DEBUG("InitVisualLearnerLib compelted.");

      INFO("ClfStartConfig='" << ClfStartConfig << "'");

      // Load global variables 
      mwArray clfConfig(ClfStartConfig.c_str());
      CLFstart(clfConfig);
   }
   ~CInitializer() {
      INFO("VisualLearnerProxy terminating");
      TermVisualLearnerLib();
   }

   void initEnumeration() {
      DEBUG("initializing Enumerator");
#if 0
      // This doesn't work when the model is empty!
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
            INFO(" ... id:" << (i+1) << " .. l:" << label << " .. c:" << concept);
         }
         return;
      }
      catch (...) {
         WARN(" **** FAILED to extract from Matlab: Coma.avNames, Coma.SCC");
      }
#endif
      labelConceptMap.clear();
      labelConceptMap["red"] = 1;
      labelConceptMap["green"] = 1;
      labelConceptMap["blue"] = 1;
      labelConceptMap["yellow"] = 1;
      labelConceptMap["orange"] = 1;
      labelConceptMap["black"] = 1;
      labelConceptMap["white"] = 1;
      labelConceptMap["pink"] = 1;
      labelConceptMap["compact"] = 2;
      labelConceptMap["elongated"] = 2;
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
