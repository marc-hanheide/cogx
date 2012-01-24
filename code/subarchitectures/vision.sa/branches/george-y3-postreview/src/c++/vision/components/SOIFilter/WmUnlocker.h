
#ifndef _SOIFILTER_WMUNLOCKER_H_4E8C4E3D_
#define _SOIFILTER_WMUNLOCKER_H_4E8C4E3D_

#include <cast/architecture/ManagedComponent.hpp>
#include <vector>

namespace cast {

// Removes the successfully locked locks on destruction (eg. exiting scope)
struct WmUnlocker
{
  class LockError: public std::exception
  {
    std::string reason;
  public:
    LockError(const std::string& error = "")
    {
      reason = error;
    }
    ~LockError() throw()
    {
    }
    const char* what() const throw()
    {
      return reason.c_str();
    }
  };

  std::vector<cdl::WorkingMemoryAddress> locks;
  cast::WorkingMemoryAttachedComponent *pc;
  WmUnlocker(cast::WorkingMemoryAttachedComponent* pComponent) {
    pc = pComponent;
  }
  void lock(cdl::WorkingMemoryAddress& addr, cdl::WorkingMemoryPermissions perm) {
    if (!pc->tryLockEntry(addr, perm))
      throw LockError("Trying to lock object: " + addr.id);
    locks.push_back(addr);
  }
  void unlockAll()
  {
    for (int i = 0; i < (int) locks.size(); ++i) {
      try {
        pc->unlockEntry(locks[i]);
      }
      catch(cast::DoesNotExistOnWMException){ }
    }
  }
  ~WmUnlocker() {
    unlockAll();
  }
};

} // namespace

#endif /* _SOIFILTER_WMUNLOCKER_H_4E8C4E3D_ */
