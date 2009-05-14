#ifndef CAST_VISION_ONTOLOGY_FACTORY_H_
#define CAST_VISION_ONTOLOGY_FACTORY_H_

#include "VisionOntology.h"

/**
Manages mappings between ontological types and framework datatypes
 */
class VisionOntologyFactory {
  
 public:
  
  VisionOntologyFactory() {};
  virtual ~VisionOntologyFactory();

    static const cast::CASTOntology * getOntology();

 private:  
    static cast::CASTOntology * m_pOntology;

};

#endif
