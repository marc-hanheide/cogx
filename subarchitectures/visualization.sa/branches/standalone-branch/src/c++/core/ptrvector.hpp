/*
 * Author: Marko Mahnič
 * Updated: 2010-03-11
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef VECTORMACRO_TRJR3BZ4
#define VECTORMACRO_TRJR3BZ4

#include <vector>

// macros that ease the mamagement of vectors of pointers

#define ITERATOR_PTR(avector,variterator) (variterator==avector.end() ? NULL : *variterator)

// TODO: swap avector and variterator in FOR_EACH_XXX: it's more similar to c++0x

// typeof works only in gcc; typeof(x) will become auto in c++0x
#define FOR_EACH(varpointer, avector) \
   for(typeof(avector.begin()) __it##varpointer=avector.begin();\
         varpointer=ITERATOR_PTR(avector, __it##varpointer), __it##varpointer!=avector.end(); \
         __it##varpointer++)

// typeof works only in gcc; typeof(x) will become auto in c++0x
// iterate over maps of pointers
// XXX FOR_EACH_V(prbl: can't break out of the outer loop; to break use __it##varpointer=amap.end())
#define FOR_EACH_V(varpointer, amap) \
   for(typeof(amap.begin()) __it##varpointer=amap.begin(); __it##varpointer != amap.end(); ) \
      for(typeof(__it##varpointer->second) varpointer = __it##varpointer->second; \
          __it##varpointer != amap.end(); \
         (++__it##varpointer != amap.end()) ? varpointer=__it##varpointer->second : varpointer=NULL \
         )

// typeof works only in gcc; typeof(x) will become auto in c++0x
// iterate over keys of maps (yields pointers to keys)
// XXX FOR_EACH_K(prbl: can't break out of the outer loop; to break use __it##varpointer=amap.end())
#define FOR_EACH_K(varpointer, amap) \
   for(typeof(amap.begin()) __it##varpointer=amap.begin(); __it##varpointer != amap.end(); ) \
      for(const typeof(__it##varpointer->first)* varpointer = &__it##varpointer->first; \
          __it##varpointer != amap.end(); \
         (++__it##varpointer != amap.end()) ? varpointer=&__it##varpointer->first : varpointer=NULL \
         )

class _CVoidPtrVectorImpl_ {
public:
   enum _packmode_ {
      packnone = 0, packquick = 1, packsorted = 2
   };
   typedef void (*TElementFun)(void*);
   static void pack(std::vector<void*> *pvect, int packmode);
   static void remove(std::vector<void*> *pvect, void *pelem, int packmode);
   static void remove(std::vector<void*> *pvect, std::vector<void*> &elements, int packmode);
   static void delete_all(std::vector<void*> *pvect, TElementFun pDelete);
   static void delete_items(std::vector<void*> *pvect, std::vector<void*> &elements, TElementFun pDelete, int packmode);
};

template<class T>
class CPtrVector: public std::vector<T*>
{
public:
   enum _packmode_ {
      packnone = 0, packquick = 1, packsorted = 2
   };
public:
   void pack(int packmode=packsorted) {
      _CVoidPtrVectorImpl_::pack(reinterpret_cast< std::vector<void*>* >(this), packmode);
      //return;
      //if (packmode == packnone) return;
      //typename CPtrVector<T>::iterator itobj, itend;
      //if (packmode == packsorted) {
      //  // erase NULL runs, keep the order of elements
      //  for(itobj = this->begin(); itobj != this->end(); itobj++) {
      //     if (*itobj != NULL) continue;
      //     itend = itobj + 1;
      //     while (*itend == NULL && itend != this->end()) itend++;
      //     int n = distance(this->begin(), itobj);
      //     this->erase(itobj, itend);
      //     itobj = this->begin(); // need to restart after erase
      //     advance(itobj, n - 1);
      //  }
      //}
      //else if (packmode == packquick) {
      //  // replace NULL objects with non-NULL objects from the end of vector; erase tail
      //  itend = this->end();
      //  for(itobj = this->begin(); itobj < itend; itobj++) {
      //     if (*itobj != NULL) continue;
      //     itend--;
      //     while (*itend == NULL && itend > itobj) itend--;
      //     if (itend > itobj) *itobj = *itend;
      //  }
      //  this->erase(itend, this->end());
      //}
   }

   void remove(T* element, int packmode=packsorted) {
      _CVoidPtrVectorImpl_::remove(reinterpret_cast< std::vector<void*>* >(this), element, packmode);
      //if (element == NULL) {
      //  pack(packmode);
      //  return;
      //}
      //typename CPtrVector<T>::iterator itobj;
      //for(itobj = this->begin(); itobj != this->end(); itobj++) {
      //  if (*itobj == element) {
      //     if (packmode == packsorted) this->erase(itobj);
      //     else if (packmode == packquick) {
      //        // replace with an object from the end of vector, remove trailing NULLs
      //        typename CPtrVector<T>::iterator itend = this->end();
      //        itend--;
      //        while (*itend == NULL && itend > itobj) itend--;
      //        if (itend > itobj) *itobj = *itend;
      //        this->erase(itend, this->end());
      //     }
      //     else *itobj = NULL; // packnone
      //     break;
      //  }
      //}
   }

   // from this vector remove all &elements and pack the vector
   void remove(typename std::vector<T*> &elements, int packmode=packsorted) {
      _CVoidPtrVectorImpl_::remove(reinterpret_cast< std::vector<void*>* >(this), 
            reinterpret_cast< std::vector<void*>& >(elements), packmode);
      //if (elements.size() < 1) {
      //  pack(packmode);
      //  return;
      //}
      //typename CPtrVector<T>::iterator itobj, itdel;
      //for(itobj = this->begin(); itobj != this->end(); itobj++) {
      //  if (*itobj == NULL) continue;
      //  // TODO: sorted&packed elements, binary search
      //  for (itdel = elements.begin(); itdel != elements.end(); itdel++) {
      //     if (*itdel == *itobj) {
      //        *itobj = NULL;
      //        break;
      //     }
      //  }
      //}
      //pack(packmode);
   }

   void delete_item(T* element, int packmode=packsorted) {
      remove(element, packmode);
      if (element) delete element;
   }

   void delete_items(typename std::vector<T*> &elements, int packmode=packsorted) {
      //_CVoidPtrVectorImpl_::delete_items(this, elements,
      //      (_CVoidPtrVectorImpl_::TElementFun)&_f_delete_item, packmode);
      if (&elements == this) {
        delete_all();
        return;
      }
      remove(elements, packmode);
      typename CPtrVector<T>::iterator itdel;
      for (itdel = elements.begin(); itdel != elements.end(); itdel++) {
         if (*itdel) delete *itdel;
      }
   }

   void delete_all() {
      // _CVoidPtrVectorImpl_::delete_all(this, (_CVoidPtrVectorImpl_::TElementFun)&_delete_item);
      typename CPtrVector<T>::iterator itdel;
      for(itdel = this->begin(); itdel != this->end(); itdel++) {
        if (*itdel) delete *itdel;
      }
      this->clear();
   }

private:
   // TElementFun
   static void _f_delete_item(T* pobj) {
      delete pobj;
   }
};

#endif /* VECTORMACRO_TRJR3BZ4 */
