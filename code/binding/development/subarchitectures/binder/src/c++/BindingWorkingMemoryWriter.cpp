#include "BindingWorkingMemoryWriter.hpp"


extern "C" {
  cast::interfaces::CASTComponentPtr 
  newComponent() {
    return new binder::BindingWorkingMemoryWriter();
  }
}



namespace binder {

  BindingWorkingMemoryWriter::BindingWorkingMemoryWriter() {}
  
  BindingWorkingMemoryWriter::~BindingWorkingMemoryWriter() {}
  
}
