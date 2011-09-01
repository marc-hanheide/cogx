//
// (C) Labor Dr. Steinbichler
//

#ifndef  _TARRAY_HPP
#define  _TARRAY_HPP


#include "syArray.hpp"
#include "syEllipse.hpp"


NAMESPACE_CLASS_BEGIN( RTE )

// the array supports iterators:
//    
//    for (CzTArray<T>::Iterator it = array.First(); it.IsValid(); it++)
//       T t = *it;
//
// of course index operations are also available.
// 
// some more features of the array:
// - sorting
// - splitting, 
// - setting/retrieving head/tail/subarrays
// - concatenation
// - serialization from/to CArchive (header file "afx.h" is required!)
// - ownership-mechanism which allows to control the creation and deletion of the data from outside.
//   o TransferOwnershipTo() combines TakeOwnership() and LoseOwnership() to
//     make another array handle the data.
//   o TakeOwnership() uses the data without copying it or creating anything
//     new. the given pointer is handled as if the array would have created the
//     data by itself.
//   o LoseOwnership() makes a resize to 0 without destroying the data.
// 
// classes that shall be used by this array must provide
// - a public Default-Constructor
// - a public Assignment-Operator

//typedef CzArray<class CzElem> CzTArray<class CzElem>

template <class CzElem> 
class CzTArray
{
   CzArray<CzElem> arr;
public:

   void Append (const CzElem& elem) { arr.PushBack(elem); }
   void Clear () { arr.Clear(); }
   unsigned Size() { return arr.Size(); }

   CzElem& operator[](unsigned i) { return arr[i]; }
};

NAMESPACE_CLASS_END()

#endif

