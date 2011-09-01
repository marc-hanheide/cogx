//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYARRAY_HPP
#define  _SYARRAY_HPP


// use this definition if you want to use the Steinbichler CzTArray-Class
//#define USE_STB_CZTARRAY


#include <stdarg.h>
#include "multiplatform.hpp"
#include "syExcept.hpp"


#ifdef USE_STB_CZTARRAY
#include "TArray.hpp"
#endif

NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE Array
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
#ifdef USE_STB_CZTARRAY
template<class CzElem> class CzArray : public CzTArray<typename CzElem>
{
public:
   CzArray(int i=0) { Resize(i); }
   inline void PushBack(const CzElem &el) { Append(el); }
   inline void Erase(unsigned i) {
      while (++i < (unsigned) Size()) {
         Data()[i-1] = Data()[i];
      }
      m_iSize--;//      Resize(Size() - 1);
   }

   ////////////////////////////////////////////////////////////////////////////////
   // Returns true if the given item is in the array, false otherwise.
   // Starts seach at the end.
   inline bool ContainsBackwards(const CzElem &el) const
   {
      // note that size - 1 would be max unsigned int for size == 0
      for (int i = Size(); i >= 1; i--)
         if (Data()[i-1] == el)
            return true;
      return false;
   }
};
#else
template <class CzElem> class CzArray
{
private:
   static const unsigned DEFAULT_SIZE = 10;

   CzElem *array;        ///< actual array
   unsigned size;        ///< used size of array
   unsigned capacity;    ///< allocated size of array

   void CheckIndex(unsigned i) const __THROW_EXCEPT;
   void EnsureCapacity(unsigned need_size) __THROW_EXCEPT;

public:
   CzArray(unsigned s = 0) __THROW_EXCEPT;
   CzArray(const CzArray &a);
   ~CzArray();
   CzArray& operator=(const CzArray &a) {DeepCopy(a); return *this;}
   CzElem& operator[](unsigned i);
   const CzElem& operator[](unsigned i) const;
   bool Empty() {return size == 0;}
   CzElem& First() __THROW_EXCEPT;
   const CzElem& First() const __THROW_EXCEPT;
   CzElem& Last() __THROW_EXCEPT;
   const CzElem& Last() const __THROW_EXCEPT;
   void PushFront(const CzElem &el);
   void PushBack(const CzElem &el);
   void InsertBefore(unsigned i, const CzElem &el);
   void InsertAfter(unsigned i, const CzElem &el);
   void InsertSorted(const CzElem &el, int(*compar)(const void *, const void *));
   unsigned Size() const {return size;}
   void Resize(unsigned new_size);
   void Sort(int(*compar)(const void *, const void *));
   bool Contains(const CzElem &el) const;
   bool ContainsBackwards(const CzElem &el) const;
   unsigned Find(const CzElem &el);
   unsigned Find(unsigned start, const CzElem &el);
   unsigned FindBackwards(const CzElem &el);
   void Swap(unsigned i, unsigned j);
   void Set(const CzElem &el);
   void Clear() {Resize(0);}
   void DeepCopy(const CzArray &a);
   unsigned CircularNext(unsigned i);
   unsigned CircularPrev(unsigned i);
   void Erase(unsigned i);
   void EraseFirst();
   void EraseLast();
   void Reverse();
   bool Intersect(const CzArray &a);
   void* Data();
};
//end class/////////////////////////////////////////////////////////////////////
#endif

NAMESPACE_CLASS_END()

#ifdef USE_STB_CZTARRAY
#else
   #if defined WIN32 || defined WIN64
      #if defined ELLDETECTEXE
         #include "syArray.icpp"
      #else
         #include "../C/syArray.icpp"
      #endif
   #else
      #include "syArray.icpp"
   #endif
#endif


#endif

