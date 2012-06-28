/**
 * @author Marko Mahnič
 * @date July 2011
 *
 */

#ifndef _VISION_COMMANDER_H_0D0F_
#define _VISION_COMMANDER_H_0D0F_

#include <cast/architecture/WorkingMemoryReaderComponent.hpp>

namespace cogx {

// @author: mmarko
// A helper class base for Working Memory Command Handlers.
// The command status will be written to WM when an instance of the
// notifier is deleted (eg. when going out of scope). The WM entry
// will be if one of fail() or succeed() was called.
// @example
//   void myHandler(WorkingMemoryChange wmc)
//   {
//     class CCmd: public VisionCommandNotifier<Command, CommandPtr>
//     {
//     public:
//       CCmd(const WorkingMemoryAddress& address)
//          : VisionCommandNotifier(address) {}
//       void doFail() { pcmd->status = -1; }
//       void doSucceed() { pcmd->status = 1; }
//     } cmd(this);
//     ...
//     if (ok) cmd.succeed();
//   };
template<class T, class TPtr>
class VisionCommandNotifier
{
private:
  cast::WorkingMemoryReaderComponent* pComponent;
  bool doWrite;
  bool ignoreCalled;
public:
  cast::cdl::WorkingMemoryAddress addr;
  TPtr pcmd;
public:
  bool read(const cast::cdl::WorkingMemoryAddress& address)
  {
    doWrite = false;
    ignoreCalled = false;
    addr = address;
    try {
      pcmd = pComponent->getMemoryEntry<T>(addr);
    }
    catch(cast::DoesNotExistOnWMException){
      pComponent->println("VisionCommandNotifier: Command deleted before read (id=%s).", addr.id.c_str());
      ignore();
      return false;
    }
    return true;
  }
  VisionCommandNotifier(cast::WorkingMemoryReaderComponent* pReader)
  {
    pComponent = pReader;
    ignoreCalled = false;
  }
  ~VisionCommandNotifier()
  {
    if (doWrite) {
      try {
        pComponent->overwriteWorkingMemory(addr, pcmd);
      }
      catch(cast::DoesNotExistOnWMException) {
        pComponent->println("VisionCommandNotifier: Command deleted before overwrite (id=%s).", addr.id.c_str());
      }
      catch(...) {
        pComponent->println("VisionCommandNotifier: Error in overwrite (id=%s).", addr.id.c_str());
      }
    }
    else if (! ignoreCalled) {
      pComponent->error("VisionCommandNotifier destroyed without an overwrite (id=%s)."
           " To fix this: call fail(), succeed() or ignore()", addr.id.c_str());
    }
  }
  void fail() { doWrite = true; ignoreCalled = false; doFail(); }
  void succeed() { doWrite = true; ignoreCalled = false; doSucceed(); }
  void ignore() { doWrite = false; ignoreCalled = true; }
protected:
  virtual void doFail() = 0; // { pcmd->status = VisionData::VCFAILED; }
  virtual void doSucceed() = 0; // { pcmd->status = VisionData::VCSUCCEEDED; }
};

} // namespace

#endif
