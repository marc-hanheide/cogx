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
//  ann->message="Hooray! Hooray! Hooray! I have completed my task! Let's listen to all the records once again!";
//  ann->songId="all";
  ann->message="Human target acquired in place number 1.";
  ann->songId="";
  addToWorkingMemory(newDataID(), ann);


}


