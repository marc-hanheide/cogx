/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 * 
 */
#ifndef STL__DEREF_TOOLS_HH
#define STL__DEREF_TOOLS_HH

#include <iostream>

#include "utilities.hh"

using std::ostream;

#define C__PTR_ANNOTATION(T) T*
#define CXX__PTR_ANNOTATION(T) std::tr1::shared_ptr<T>

#define C__CONST_PTR_ANNOTATION(T) const C__PTR_ANNOTATION(T)
#define CXX__CONST_PTR_ANNOTATION(T) const CXX__PTR_ANNOTATION(T)


#define DEREF__HASH(NAME, PTR_ANNOTATION) template<typename T>        \
    struct NAME {                                                     \
        inline std::size_t operator()(PTR_ANNOTATION(T) t) const      \
        {                                                             \
            VERBOSER(12, "Hash is :: "<<hasher(*t)<<std::endl);       \
            return hasher(*t);                                        \
        }                                                             \
    private:                                                          \
        typedef boost::hash<T> Hasher;                                \
        static Hasher hasher;                                         \
    }                                                                 \

DEREF__HASH(C__deref__hash, C__CONST_PTR_ANNOTATION);
DEREF__HASH(CXX__deref__hash, CXX__CONST_PTR_ANNOTATION);

/* Adaptable binary predicate along the lines of \class{std::equal_to}
 * only this dereferences its arguments that are forced to be
 * pointers.*/
#define DEREF__EQUAL_TO(NAME, PTR_ANNOTATION) template<typename T>      \
    struct NAME {                                                       \
        inline bool operator()(PTR_ANNOTATION(T) t1,             \
                                      PTR_ANNOTATION(T) t2) const       \
        {                                                               \
            return (*t1) == (*t2);                                      \
        }                                                               \
    }                                                                   \

DEREF__EQUAL_TO(C__deref__equal_to, C__CONST_PTR_ANNOTATION);
DEREF__EQUAL_TO(CXX__deref__equal_to, CXX__CONST_PTR_ANNOTATION);


template<typename T>
class CXX__deref__shared_ptr
{
public:

    explicit CXX__deref__shared_ptr(const CXX__PTR_ANNOTATION(T)&);
    explicit CXX__deref__shared_ptr();
    
    template<typename TT>
    explicit CXX__deref__shared_ptr(const CXX__deref__shared_ptr<TT>& in)
    {
        assert(in.use_count());
        assert(in.test_cast<T>());

        auto tmp = in.cxx_get<T>();
        contents = CXX__PTR_ANNOTATION(T)(tmp);//in.cxx_get<T>());
    }
    
//     template<typename TT>
//     explicit CXX__deref__shared_ptr(const CXX__PTR_ANNOTATION(TT)& in)
//     {
//         assert(in.use_count());
//         assert(dynamic_cast<const T*>(in.get()));

//         contents = std::tr1::dynamic_pointer_cast<T>(in);//CXX__PTR_ANNOTATION(T)(tmp);//in.cxx_get<T>());
//     }


    long use_count()const{return contents.use_count();};
    
    CXX__PTR_ANNOTATION(T) cxx_get() const {return contents;} ;
    template<typename TT>
    CXX__PTR_ANNOTATION(TT) cxx_get() const {return std::tr1::dynamic_pointer_cast<TT>(contents);} ;

    T& operator*() const {return *contents;};
    T* get() const {return contents.get();};
    
    T* operator->() const {return contents.get();};
//     const T* operator->() const {return contents.get();};
    
    bool operator==(const CXX__deref__shared_ptr&) const;
    bool operator!=(const CXX__deref__shared_ptr&) const;
    bool operator<(const CXX__deref__shared_ptr&) const;
    std::size_t hash_value() const;
    ostream& operator<<(ostream&) const;
    
    template<typename TT> bool test_cast() const {return dynamic_cast<TT*>(contents.get());};
    template<typename TT> C__PTR_ANNOTATION(TT) do_cast() const
    {assert(test_cast<TT>());return dynamic_cast<C__PTR_ANNOTATION(TT)>(contents.get());};
    template<typename TT> TT do_cast_and_copy() const
    {assert(test_cast<TT>());return *dynamic_cast<C__PTR_ANNOTATION(TT)>(contents.get());};
    
private:
    CXX__PTR_ANNOTATION(T) contents;
};



// template<typename T, typename TT>
// CXX__deref__shared_ptr<T>::CXX__deref__shared_ptr<TT>(const CXX__deref__shared_ptr<TT>& in)


template<typename T>
CXX__deref__shared_ptr<T>::CXX__deref__shared_ptr(const CXX__PTR_ANNOTATION(T)&in)
    :contents(in)
{}

template<typename T>
CXX__deref__shared_ptr<T>::CXX__deref__shared_ptr()
    :contents()//static_cast<T*>(NULL))
{}

template<typename T>
std::size_t hash_value(const CXX__deref__shared_ptr<T>&in){return in.hash_value();};

// template<typename T>
// CXX__PTR_ANNOTATION(T) CXX__deref__shared_ptr<T>::get() const
// {
//     return contents;
// }

template<typename T>
bool CXX__deref__shared_ptr<T>::operator==(const CXX__deref__shared_ptr&in) const
{
    return ((*contents) == (*in.contents));
}

template<typename T>
bool CXX__deref__shared_ptr<T>::operator!=(const CXX__deref__shared_ptr&in) const
{
    return !operator==(in);
}

template<typename T>
bool CXX__deref__shared_ptr<T>::operator<(const CXX__deref__shared_ptr&in) const
{
    return ((*contents) < (*in.contents));
}

template<typename T>
std::size_t CXX__deref__shared_ptr<T>::hash_value() const
{
    return contents->hash_value();
}

template<typename T>
ostream& CXX__deref__shared_ptr<T>::operator<<(ostream& o) const
{
    return o<<*(contents.get());
}



template<typename T>
class C__deref__shared_ptr
{
public:
    explicit C__deref__shared_ptr(const C__PTR_ANNOTATION(T)&);
    
    C__PTR_ANNOTATION(T) get() const;
    bool operator==(const C__deref__shared_ptr&) const;
    bool operator<(const C__deref__shared_ptr&) const;
    std::size_t hash_value() const;
    ostream& operator<<(ostream&) const;
    template<typename TT> bool test_cast() const {return dynamic_cast<TT*>(contents);};
    template<typename TT> C__PTR_ANNOTATION(TT) do_cast() const {return dynamic_cast<TT*>(contents);};
private:
    C__PTR_ANNOTATION(T) contents;
};

template<typename T>
std::size_t hash_value(const C__deref__shared_ptr<T>&in){return in.hash_value();};


template<typename T>
C__deref__shared_ptr<T>::C__deref__shared_ptr(const C__PTR_ANNOTATION(T)&in)
    :contents(in)
{}


template<typename T>
C__PTR_ANNOTATION(T) C__deref__shared_ptr<T>::get()  const
{
    return contents;
}

template<typename T>
bool C__deref__shared_ptr<T>::operator==(const C__deref__shared_ptr&in) const
{
    return ((*contents) == (*in.contents));
}

template<typename T>
bool C__deref__shared_ptr<T>::operator<(const C__deref__shared_ptr&in) const
{
    return ((*contents) < (*in.contents));
}

template<typename T>
std::size_t C__deref__shared_ptr<T>::hash_value() const
{
    return contents->hash_value();
}


template<typename T>
ostream& C__deref__shared_ptr<T>::operator<<(ostream& o) const
{
    return o<<*contents;
}


// template<typename T> class deref__equal_to : public CXX__deref__equal_to<T>{};
// template<typename T> class deref__shared_ptr : public CXX__deref__shared_ptr<T>{};

template<typename T>
class NOT_NECESSARY
{
public:
    
};

// template<T>
// typename CXX__deref__equal_to<T> deref__equal_to<T>;
// typedef CXX__deref__shared_ptr deref__shared_ptr;

namespace std
{
    template<typename T>
    ostream& operator<<(ostream&o, const CXX__deref__shared_ptr<T>&in){return in.operator<<(o);};
    template<typename T>
    ostream& operator<<(ostream&o, const C__deref__shared_ptr<T>&in){return in.operator<<(o);};
}


#endif
