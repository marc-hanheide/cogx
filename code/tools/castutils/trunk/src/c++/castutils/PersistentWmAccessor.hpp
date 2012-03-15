#ifndef _CASTUTILS_PERSISTENTWMACCESSOR_HPP_4F605375_
#define _CASTUTILS_PERSISTENTWMACCESSOR_HPP_4F605375_
/**
 * @author Marko Mahnič
 * @date March 2012
 *
 */

#include <cast/architecture/WorkingMemoryReaderComponent.hpp>
#include <IceUtil/IceUtil.h>
#include <string>

namespace castutils {

// This class is usable when:
//   - there is a persistent WM entry that reflects some state of the system
//   - the stored state is needed occasionally
//   - the order of state changes is not important
//
// It can also be used to transfer a WM entry between threads if the entry
// is not likely to disappear between T1.update() and T2.get(). The methods
// are synchronized with a mutex.
//
// Note: a get() in T2 may block and update() in T1 which could be a WM filter.
// This is not a problem if WM reads are fast.
template<class CastWmEntryT, class CastWmEntryPtrT>
class CPersistentWmAccessor
{
private:
  cast::WorkingMemoryReaderComponent* mpComponent;
  cast::cdl::WorkingMemoryAddress mAddress;
  std::string mType;
  bool mbExists;   // a WM entry was added and not yet deleted
  bool mbChanged;  // the WM entry was updated
  CastWmEntryPtrT mLastValue;
  IceUtil::Mutex mMutex; // TODO: use c++11 mutex
public:
  CPersistentWmAccessor(cast::WorkingMemoryReaderComponent* pCastComponent)
    : mpComponent(pCastComponent), mbExists(false), mbChanged(false)
  {
    assert(pCastComponent != 0);
  }

  bool isValid()
  {
    return mbExists;
  }

  bool isChanged()
  {
    return mbExists && mbChanged;
  }

  void clear()
  {
    IceUtil::Mutex::Lock lock(mMutex);
    mbExists = false;
    mbChanged = false;
    mLastValue = 0;
  }

  void update(const cast::cdl::WorkingMemoryChange& wmc)
  {
    IceUtil::Mutex::Lock lock(mMutex);
    if (mbExists && wmc.address != mAddress) {
      // Don't signal an error. This makes it easier to update multiple instances
      // of the same type in a WM filter.
      return;
    }
    if (!mbExists) {
      if (wmc.operation == cast::cdl::DELETE)
        return;
      mAddress = wmc.address;
      mType = wmc.type;
      mbExists = true;
    }
    else if (wmc.operation == cast::cdl::DELETE) {
      mbExists = false;
    }
    mbChanged = true;
  }

  // TODO: change to bool get(CastWmEntryPtrT&) and don't throw on !mbExists
  CastWmEntryPtrT get()
  {
    IceUtil::Mutex::Lock lock(mMutex);
    if (!mbExists) {
      throw(cast::DoesNotExistOnWMException(
            cast::exceptionMessage(__HERE__,
              "CPersistentWmAccessor: %s WM Address not registered.", mType.c_str()),
            mAddress));
    }
    if (mbChanged) {
      mLastValue = 0;
      mLastValue = mpComponent->getMemoryEntry<CastWmEntryT>(mAddress);
      mbChanged = false;
    }
    return mLastValue;
  }
};

} // namespace

#endif /* _CASTUTILS_PERSISTENTWMACCESSOR_HPP_4F605375_ */
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
