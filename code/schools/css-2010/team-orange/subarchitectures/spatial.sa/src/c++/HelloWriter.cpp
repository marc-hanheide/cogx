#include "HelloWriter.hpp"

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new HelloWriter();
  }
}
void HelloWriter::runComponent() {
  println("Look out world, here I come...");
  SpatialData::AnnouncementPtr ann = new SpatialData::Announcement();
  ann->message="This is an announcement";
  ann->songId=1;
  addToWorkingMemory(newDataID(), ann);


}


