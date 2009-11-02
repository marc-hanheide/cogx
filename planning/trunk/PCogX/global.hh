#ifndef GLOBAL_HH
#define GLOBAL_HH


/*---------------------------------

Usual suspects C++::1998

---------------------------------*/

/* cross-vendor C++ ABI in use by GCC. Essentially, you are going to
 * want to be able to do the following...
 *
 * int main()
 * {
 *   int     status;
 *  char   *realname;
 *  * 
 *   // exception classes not in <stdexcept>, thrown by the implementation
 *   // instead of the user
 *  std::bad_exception  e;
 *  realname = abi::__cxa_demangle(e.what(), 0, 0, &status);
 *  std::cout << e.what() << "\t=> " << realname << "\t: " << status << '\n';
 *  free(realname);
 * * 
 * * 
 *  // typeid
 *  bar<empty,17>          u;
 *  const std::type_info  &ti = typeid(u);
 * * 
 *  realname = abi::__cxa_demangle(ti.name(), 0, 0, &status);
 *  std::cout << ti.name() << "\t=> " << realname << "\t: " << status << '\n';
 *  free(realname);
 * * 
 *  return 0;
 * }
 */
#include <cxxabi.h>

/*IO, string, files*/
#include<string>
#include<iostream>
#include<iomanip>
#include<sstream>
#include<fstream>

/*Storage*/
#include<vector>
#include<map>
#include<set>


/*Sorting, and set operations*/
#include <algorithm> // std::swap
#include <utility>  // std::move, std::forward
#include <iterator> // 


/* TR1 storage. */
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <tr1/memory>
#include <tr1/tuple>
#include <tr1/type_traits>

/*---------------------------------

Usual suspects C

---------------------------------*/
#include<cstddef>
#include<cstdio>
#include<cmath>
#include<cstdlib>
//#include<cmalloc>
#include<cassert>
#include<cctype>
#include<csignal>
#include<cstdarg>
#include<cstddef>
#include<cstring>



/*---------------------------------

Usual suspects (C++) BOOST

---------------------------------*/
#include<boost/functional/hash.hpp>


/*---------------------------------

Old fashioned debugging

---------------------------------*/
#define DEBUG_LEVEL 0
#define DEBUG_GET_CHAR(Y) {if(Y > DEBUG_LEVEL) {char ch; std::cin>>ch;} }

#define VERBOSE(X) {std:cerr<<"INFO :: "<<X<<std::endl;}

#define VERBOSER(Y, X) {if(Y > DEBUG_LEVEL) \
                            std::cerr<<"INFO ("<<Y<<") -- "     \
                                     <<__PRETTY_FUNCTION__<<"-- :: \n**\t"<<X<<std::endl; \
    }                                                                   \
        
/*-------------------------------*/
/*---------------------------------

Macros for fatal and lower-level errors.

---------------------------------*/
#define UNRECOVERABLE_ERROR(X) {std::cerr<<"UNRECOVERABLE ERROR ::  -- " \
                                         <<__PRETTY_FUNCTION__<<" -- "<<X;assert(0);exit(0);}

#define QUERY_UNRECOVERABLE_ERROR(Q,X) {if(Q)UNRECOVERABLE_ERROR(X);}

#define WARNING(X) {std::cerr<<"WARNING ::  -- " \
                             <<__PRETTY_FUNCTION__<<" -- "<<X<<std::endl;}


#define QUERY_WARNING(Q,X) {if(Q)WARNING(X);}
/*-------------------------------*/

#define TYPE_NAMED__FROM_PARENT(Parent__TYPE, Child__MEMBER) typedef typename Parent__TYPE::Child__MEMBER Child__MEMBER
#define TYPE__FROM_PARENT(Parent__TYPE, Child__MEMBER) typedef Parent__TYPE::Child__MEMBER Child__MEMBER


#define UNIMPLEMENTED {UNRECOVERABLE_ERROR("NOT IMPLEMENTED!!!");} // = 0


#define C__PTR_ANNOTATION(T) T*
#define CXX__PTR_ANNOTATION(T) std::shared_ptr<T>

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
        inline std::size_t operator()(PTR_ANNOTATION(T) t1,             \
                                      PTR_ANNOTATION(T) t2) const       \
        {                                                               \
            return (*t1) == (*t2);                                      \
        }                                                               \
    }                                                                   \



DEREF__EQUAL_TO(C__deref__equal_to, C__CONST_PTR_ANNOTATION);
DEREF__EQUAL_TO(CXX__deref__equal_to, CXX__CONST_PTR_ANNOTATION);

/* Any set-based Container -- traversable or otherwise --
 * implicitely has a range and a domain. Elements in the range
 * are the indices to elements in the set. Elements in the
 * domain are members of the set/Container. This is an
 * implementation of a contain that makes these ideas
 * explicit.*/
template<typename T
         , typename T_Hash = boost::hash<T>
         , typename T_Equality = std::equal_to<T> >
class Traversable_Container_of_Unique_Elements
    :public std::vector<T>
{
public:
#define INVARIANTS__Traversable_Container_of_Unique_Elements {assert(map__T_to_size_type.size() == this->size());}
    
    /* If the type of the template equality test -- i.e.,
     * \template{T_Equality} -- is a function pointer -- e.g.,
     * bool(*)(const T&, const T&) -- then an instance must be
     * constructed with a function pointer.
     * 
     * Also, if the type of the template hash function -- i.e.,
     * \template{T_Hash} -- is a function pointer -- e.g., size_t
     * (*)(const T&) -- then an instance must be constructed with a
     * function pointer.*/
    explicit Traversable_Container_of_Unique_Elements(const T_Hash& t_Hash = T_Hash(),
                                                      const T_Equality& t_Equality = T_Equality())
        :map__T_to_size_type(11/* FIX :: Manifest constant. Soo dodgy,
                                * and yet... Will resort to library
                                * eventually.*/,
                             t_Hash,
                             t_Equality)
    {
        
    }
    
    TYPE_NAMED__FROM_PARENT(std::vector<T>, value_type);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, pointer);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, reference);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, const_reference);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, size_type);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, difference_type);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, iterator);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, const_iterator);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, reverse_iterator);
    TYPE_NAMED__FROM_PARENT(std::vector<T>, const_reverse_iterator);

    typedef typename std::tr1::unordered_map<T, size_type, T_Hash, T_Equality> Map__T_to_size_type;

    size_type get_domain_element(const T& t) const {
        INVARIANTS__Traversable_Container_of_Unique_Elements;
        return map__T_to_size_type[t];
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }
            
    reference get_range_element(size_type index){
        INVARIANTS__Traversable_Container_of_Unique_Elements;
        return (*this)[index];
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }
            
    void insert(const_reference t)
    {
        INVARIANTS__Traversable_Container_of_Unique_Elements;
        if(map__T_to_size_type.find(t) == map__T_to_size_type.end()){
            push_back(t);
        }
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }
            
    const_reference operator[](size_type index)
    {
        INVARIANTS__Traversable_Container_of_Unique_Elements;
        assert(index < std::vector<T>::size());
        return std::vector<T>::operator[](index);
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }
            
    const_reference operator[](size_type index) const
    {
        INVARIANTS__Traversable_Container_of_Unique_Elements;
        assert(index < std::vector<T>::size());
        return std::vector<T>::operator[](index);
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }
            
    /* Gives a unique ID for \argument{t}.*/
    size_type operator[](const_reference t) const
    {
        if(map__T_to_size_type.find(t) != map__T_to_size_type.end()){
            return map__T_to_size_type[t];
        } else {
            WARNING("Asked for ID of :: "<<t<<"; Not a member of this container.");
            WARNING("Resolution gives invalid index.");
                    
            return std::vector<T>::size() + 1;
        }
    }

    inline iterator begin(){return std::vector<T>::begin();};
    inline iterator end(){return std::vector<T>::end();};
    inline const_iterator begin() const {return std::vector<T>::begin();};
    inline const_iterator end() const {return std::vector<T>::end();};
    
    inline reverse_iterator rend(){return std::vector<T>::rend();};
    inline const_reverse_iterator rend() const{return std::vector<T>::rend();};
    inline reverse_iterator rbegin(){return std::vector<T>::rbegin();};
    inline const_reverse_iterator rbegin() const{return std::vector<T>::rbegin();};
    
    
    iterator find(reference t)
    {
        typename Map__T_to_size_type::iterator pair__element_index =  map__T_to_size_type.find(t);

        /* If \argument{t} is an element that is not in the
         * container.*/
        if(map__T_to_size_type.end() == pair__element_index){
            return end();//std::vector<T>::size() + 1;   
        } else {
            return std::vector<T>::begin() + map__T_to_size_type->second;
        }
    }
    
                
    void push_back(const T& t){
        INVARIANTS__Traversable_Container_of_Unique_Elements;
                
        if(map__T_to_size_type.find(t) == map__T_to_size_type.end()){
            std::vector<T>::push_back(t);
            map__T_to_size_type[t] = std::vector<T>::size() - 1;
        } else {
            WARNING("Expecting insertion of new item.");
        }
                
        INVARIANTS__Traversable_Container_of_Unique_Elements;
    }

    std::size_t hash_value() const
    {
       return boost::hash_range(begin(), end());
        
    }
    
private:
    Map__T_to_size_type map__T_to_size_type;
};

template<typename T>
std::size_t hash_value(const Traversable_Container_of_Unique_Elements<T>& list)
{
    return list.hash_value();
}

namespace std
{
    template <class Elem
              , class Traits
              , typename T
              , typename T_Equality >
    std::basic_ostream<Elem, Traits>&
    operator<<(std::basic_ostream<Elem, Traits>& o,
               const Traversable_Container_of_Unique_Elements<T, T_Equality>& list)
    {
        for(auto i = list.begin()
                ; i != list.end()
                ; ){
            o<<*i;
        
            i++;
            if(list.end() != i){
                o<<", ";
            } else {
                o<<" ";
            }    
        }
    
        return o;
    }
}


class HasStringRepresentation;

namespace std
{
    std::ostream& operator<<(std::ostream&, const HasStringRepresentation&);
}

class HasStringRepresentation
{
public:
    friend std::ostream& std::operator<<(std::ostream&, const HasStringRepresentation&);
    
    explicit HasStringRepresentation(const std::string& = std::string(""), bool = false);
    explicit HasStringRepresentation(std::string&& = std::string(""), bool = false);
    explicit HasStringRepresentation(const HasStringRepresentation& hsr);
    explicit HasStringRepresentation(HasStringRepresentation&& hsr);
    explicit HasStringRepresentation(const HasStringRepresentation&& hsr) = delete;
    
    virtual ~HasStringRepresentation();
    
    /* Assignment using move semantics.*/
    HasStringRepresentation& operator=(HasStringRepresentation&& hsr);
    
    /* Assignment using copy semantics.*/
    HasStringRepresentation& operator=(const HasStringRepresentation& hsr);
    
    
    /*String based, so we may-as-well have an ordering relation based
     * on the string representation. It is up to the children to make
     * this relation more sensible for their purpose.*/
    virtual bool operator<(const HasStringRepresentation&) const;
    virtual bool operator==(const HasStringRepresentation&) const;

    /* Innequality based on the equality text in
     * \method{this->operator==()}.*/
    virtual bool operator!=(const HasStringRepresentation& hsr) const
    {return !operator==(hsr);};

    /* Hashing based on the string representation of the object.*/
    virtual size_t hash_value() const;
protected:
    virtual void computeAsString(const std::string& str = std::string("")) const
    {
	asString = str;
	computedAsString = true;
    }
    
    /*What does this look like as a string.*/
    mutable std::string asString;

    /*Have we already computed what this looks like as a string.*/
    mutable bool computedAsString;

    /*Function computes the hash of a string.*/
    boost::hash<std::string> hasher;
};

std::size_t hash_value(const HasStringRepresentation&);

#endif
