/*
 * Author: Marko Mahnič
 * Created: 2010-03-11
 */
#include "ptrvector.hpp"
#include <cstdlib>

void _CVoidPtrVectorImpl_::pack(std::vector<void*> *pvect, int packmode)
{
    if (packmode == packnone) return;
    std::vector<void*>::iterator itobj, itend;
    if (packmode == packsorted) {
       // erase NULL runs, keep the order of elements
       for(itobj = pvect->begin(); itobj != pvect->end(); itobj++) {
          if (*itobj != NULL) continue;
          itend = itobj + 1;
          while (*itend == NULL && itend != pvect->end()) itend++;
          int n = distance(pvect->begin(), itobj);
          pvect->erase(itobj, itend);
          itobj = pvect->begin(); // need to restart after erase
          advance(itobj, n - 1);
       }
    }
    else if (packmode == packquick) {
       // replace NULL objects with non-NULL objects from the end of vector; erase tail
       itend = pvect->end();
       for(itobj = pvect->begin(); itobj < itend; itobj++) {
          if (*itobj != NULL) continue;
          itend--;
          while (*itend == NULL && itend > itobj) itend--;
          if (itend > itobj) *itobj = *itend;
       }
       pvect->erase(itend, pvect->end());
    }
}

void _CVoidPtrVectorImpl_::remove(std::vector<void*> *pvect, void* element, int packmode)
{
   if (element == NULL) {
      pack(pvect, packmode);
      return;
   }
   std::vector<void*>::iterator itobj;
   for(itobj = pvect->begin(); itobj != pvect->end(); itobj++) {
      if (*itobj == element) {
         if (packmode == packsorted) pvect->erase(itobj);
         else if (packmode == packquick) {
            // replace with an object from the end of vector, remove trailing NULLs
            std::vector<void*>::iterator itend = pvect->end();
            itend--;
            while (*itend == NULL && itend > itobj) itend--;
            if (itend > itobj) *itobj = *itend;
            pvect->erase(itend, pvect->end());
         }
         else *itobj = NULL; // packnone
         break;
      }
   }
}

void _CVoidPtrVectorImpl_::remove(std::vector<void*> *pvect, std::vector<void*> &elements, int packmode)
{
   if (elements.size() < 1) {
      pack(pvect, packmode);
      return;
   }
   std::vector<void*>::iterator itobj, itdel;
   for(itobj = pvect->begin(); itobj != pvect->end(); itobj++) {
      if (*itobj == NULL) continue;
      // TODO: sorted&packed elements, binary search
      for (itdel = elements.begin(); itdel != elements.end(); itdel++) {
         if (*itdel == *itobj) {
            *itobj = NULL;
            break;
         }
      }
   }
   pack(pvect, packmode);
}

void _CVoidPtrVectorImpl_::delete_all(std::vector<void*> *pvect, TElementFun pDelete)
{
   std::vector<void*>::iterator itdel;
   for(itdel = pvect->begin(); itdel != pvect->end(); itdel++) {
      if (! *itdel) continue;
      if (pDelete) pDelete(*itdel);
      else free(*itdel);
   }
   pvect->clear();
}

void _CVoidPtrVectorImpl_::delete_items(std::vector<void*> *pvect,
      std::vector<void*> &elements, TElementFun pDelete, int packmode)
{
   if (&elements == pvect) {
      delete_all(pvect, pDelete);
      return;
   }
   remove(pvect, elements, packmode);
   std::vector<void*>::iterator itdel;
   for (itdel = elements.begin(); itdel != elements.end(); itdel++) {
      if (! *itdel) continue;
      if (pDelete) pDelete(*itdel);
      else free(*itdel);
   }
}
