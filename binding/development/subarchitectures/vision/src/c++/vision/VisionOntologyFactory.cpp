#include "VisionOntologyFactory.hpp"

using namespace cast;

CASTOntology * VisionOntologyFactory::m_pOntology = NULL;

VisionOntologyFactory::~VisionOntologyFactory() {
  delete m_pOntology;
  m_pOntology = NULL;
}

const CASTOntology * VisionOntologyFactory::getOntology() {
  
  if(m_pOntology == NULL) {
    m_pOntology = new VisionOntology();    
  }

  return m_pOntology;

}
