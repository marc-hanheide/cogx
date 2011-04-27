// NOTE: Don't reference any CAST structues in this Matlab proxy!
// Otherwise the proxy needs to be rebuilt on every CAST change and
// you need Matlab for that!
// #include <VisionData.hpp>

#ifndef VISUALLEARNERPROXY_YGO3EQK8
#define VISUALLEARNERPROXY_YGO3EQK8

#include "Enumerator.h"
#include <vector>

namespace matlab {

// Recognise attributes in an image stored in a protoobject.
// Returns recognized attributes (label/superConcept/probability) and gains for learning.
extern void VL_recognise_attributes(
      const VisionData::ProtoObject &Object,
      std::vector<std::string> &labels,
      std::vector<int> &labelConcepts,
      std::vector<double> &probs,
      std::vector<double> &gains);

// Update the model.
// The object in an image stored in a protoobject is associated with the labels as
// specified in weights. Weights are +1 for learning and -1 for unlearning
extern void VL_update_model(
      VisionData::ProtoObject &Object,
      std::vector<std::string> &labels,
      std::vector<double> &weights);

// Get the current status of the model. 
// Returns the data for a VisualConceptModelStatus (label/superConcept/gain).
void VL_introspect(
            std::vector<std::string>& labels,
            std::vector<int>& labelConcepts,
            std::vector<double>& gains);

// Load predefined AV Models from a file (Matlab mat file)
extern void VL_LoadAvModels(const char* filename);

// Load another model from the directory configured in CLFStart
extern void VL_LoadAvModels_from_configured_dir(const char* filename);


// Configure the mapping between label names and integers.
extern void VL_setEnumeration(const CTypeEnumerator& typeEnum);

// Which configuration file (Matlab script) should be loaded at startup.
extern void VL_setClfStartConfig(const std::string& absConfigPath);

} // namespace

#endif /* end of include guard: VISUALLEARNERPROXY_YGO3EQK8 */
