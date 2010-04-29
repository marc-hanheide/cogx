#include "HelloWriter.hpp"

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new HelloWriter();
  }
}
void HelloWriter::runComponent() {
  println("Look out world, here I come...");
helloworld::AnnouncementPtr ann = new helloworld::Announcement("james");
  addToWorkingMemory(newDataID(), ann);


}


