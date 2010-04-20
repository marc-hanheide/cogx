/*
 * Author: Marko Mahnič
 * Created: 2010-03-31
 */

#ifndef OBSERVER_ELHUP7YS
#define OBSERVER_ELHUP7YS

#include "ptrvector.hpp"

// CObserver to be used as a public member. Not to be used as a base class.
// Example:
//    class CLookAtMeObserver { public: void onDataChanged(CLookAtMe* where); };
//    class CLookAtMe {
//    public:
//       CObserver<CLookAtMeObserver> Observers;
//       void changeData() {
//          CLookAtMeObserver* pobs;
//          FOR_EACH(Observers, CLookAtMeObserver, pobs) pobs->onDataChanged(this);
//       }
//    };
template<class T>
class CObserver {
private:
   CPtrVector<T> m_Observers;

public:
   void addObserver(T* pObserver)
   {
      m_Observers.remove(pObserver);
      m_Observers.push_back(pObserver);
   }

   void removeObserver(T* pObserver)
   {
      m_Observers.remove(pObserver);
   }

   void clearObservers(T* pObserver)
   {
      m_Observers.clear();
   }

   void operator+=(T* pObserver)
   {
      addObserver(pObserver);
   }

   void operator-=(T* pObserver)
   {
      removeObserver(pObserver);
   }

   // Although these would better be const_iterator, iterator is required by FOR_EACH
   typename CPtrVector<T>::iterator begin()
   {
      return m_Observers.begin();
   }

   typename CPtrVector<T>::iterator end()
   {
      return m_Observers.end();
   }
};

#endif /* OBSERVER_ELHUP7YS */
