/* vim:set fileencoding=utf-8 sw=3 ts=8 et:vim */
/** 
 * @brief Proxy for the VisualLearner component.
 *
 * @author Marko Mahnič
 */
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <map>
#include <VisionData.hpp>

#include "../VisualLearnerProxy.h"

using namespace std;
using namespace VisionData;

namespace matlab {

CTypeEnumerator Enumerator;
std::map<std::string, int> labelConceptMap; // name -> 1=color, 2=shape
std::string ClfStartConfig;

class _InitMe_
{
public:
   _InitMe_() {
      labelConceptMap.clear();
      labelConceptMap["red"] = 1;
      labelConceptMap["green"] = 1;
      labelConceptMap["blue"] = 1;
      labelConceptMap["yellow"] = 1;
      labelConceptMap["black"] = 1;
      labelConceptMap["white"] = 1;
      labelConceptMap["orange"] = 1;
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
} _init_me_;

void VL_LoadAvModels(const char* filename)
{
}

void VL_LoadAvModels_from_configured_dir(const char* filename)
{
}

void VL_setEnumeration(const CTypeEnumerator& typeEnum)
{
   Enumerator = typeEnum;
}

void VL_setClfStartConfig(const std::string& absConfigPath)
{
   ClfStartConfig = absConfigPath;
}

static int count = 0;

void VL_recognise_attributes(const ProtoObject &Object, vector<string> &labels,
      vector<int> &labelConcepts, vector<double> &probs, vector<double> &gains)
{
   labels.clear();
   labelConcepts.clear();
   probs.clear();
   gains.clear();

   string attr;
   if(count %2 == 0) {
       attr = "red";
       labels.push_back(attr);
       labelConcepts.push_back(labelConceptMap[attr]);
       probs.push_back(0.45);
       gains.push_back(0.2);

       attr = "compact";
       labels.push_back(attr);
       labelConcepts.push_back(labelConceptMap[attr]);
       probs.push_back(0.8);
       gains.push_back(0.2);
   } else {
       attr = "blue";
       labels.push_back(attr);
       labelConcepts.push_back(labelConceptMap[attr]);
       probs.push_back(0.99);
       gains.push_back(0.2);

       attr = "compact";
       labels.push_back(attr);
       labelConcepts.push_back(labelConceptMap[attr]);
       probs.push_back(0.55);
       gains.push_back(0.2);
   }
   ++count;
}

void VL_update_model(ProtoObject &Object, std::vector<string>& labels, std::vector<double>& weights)
{
   long cntPlus = 0, cntMinus = 0;
   for (unsigned i = 0; i < labels.size(); i++) {
      if (weights[i] > 0) cntPlus++;
      if (weights[i] < 0) cntMinus++;
   }
}

void VL_introspect(vector<string>& labels, vector<int>& labelConcepts, vector<double>& gains)
{
   labels.clear();
   labelConcepts.clear();
   gains.clear();

   Enumerator.getLabels(labels);
   TStringVector::iterator ilab;
   for (ilab = labels.begin(); ilab != labels.end(); ilab++) {
      labelConcepts.push_back(labelConceptMap[*ilab]);
      gains.push_back(0.2);
   }
}

} // namespace

