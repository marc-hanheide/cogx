// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef GLOBAL_HH
#define GLOBAL_HH

//   From the ZLIB example for deflate and and inflate pipes:
//   --------------------------------------------------------
//   QUOTE : This is an ugly hack required to avoid corruption of the input and
//   QUOTE : output data on Windows/MS-DOS systems. Without this, those systems
//   QUOTE : would assume that the input and output files are text, and try to
//   QUOTE : convert the end-of-line characters from one standard to
//   QUOTE : another. That would corrupt binary data, and in particular would
//   QUOTE : render the compressed data unusable. This sets the input and output
//   QUOTE : to binary which suppresses the end-of-line
//   QUOTE : conversions. SET_BINARY_MODE() will be used later on stdin and
//   QUOTE : stdout, at the beginning of main().
#if defined(MSDOS) || defined(OS2) || defined(WIN32) || defined(__CYGWIN__)
#  include <fcntl.h>
#  include <io.h>
#  define SET_BINARY_MODE(file) setmode(fileno(file), O_BINARY)
#else
#  define SET_BINARY_MODE(file)
#endif


/*---------------------------------

  Usual suspects C++::1998

  ---------------------------------*/

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
#include<algorithm>

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

  Usual suspects C++::TR1

  ---------------------------------*/
#ifndef NATHAN

#include<tr1/unordered_map>
#include<tr1/unordered_set>
//#include<tr1/functional>
//#include<boost/functional/hash.hpp>

#endif



/*---------------------------------

  Old fashioned debugging

  ---------------------------------*/
#define DEBUG_LEVEL 30
#define DEBUG_GET_CHAR(Y) {if(Y > DEBUG_LEVEL) {char ch; cin>>ch;} }

#define VERBOSE(X) {cerr<<"INFO :: "<<X<<endl;}
#define VERBOSER(Y, X) {if(Y > DEBUG_LEVEL)cerr<<"INFO ("<<Y<<") :: "<<X;}

/*---------------------------------

  Macros for fatal and lower-level errors.

  ---------------------------------*/
#define UNRECOVERABLE_ERROR(X) {cerr<<"UNRECOVERABLE ERROR :: "<<X;assert(0);exit(0);}

#define QUERY_UNRECOVERABLE_ERROR(Q,X) {if(Q)UNRECOVERABLE_ERROR(X);}

#define WARNING(X) {}//{cerr<<"WARNING :: "<<X<<endl;}

/*Short typename for a "long unsigned int".*/
typedef long unsigned int lui;


typedef std::tr1::unordered_set<uint, std::tr1::hash<uint> > SetOfUnsignedInts;
typedef std::tr1::unordered_map<uint, uint, std::tr1::hash<uint> > MapIntToInt;
typedef std::tr1::unordered_map<uint, SetOfUnsignedInts, std::tr1::hash<uint> > MapIntToInts;
typedef std::tr1::unordered_map<std::string, uint, std::tr1::hash<std::string> > MapStringToInt;
typedef std::tr1::unordered_map<std::string, std::pair<uint,uint>, std::tr1::hash<std::string> > MapStringToPairInt;

typedef std::tr1::unordered_set<std::string, std::tr1::hash<std::string> > SetOfStrings;

/*Include for boost hashing et al.*/
#include<boost/functional/hash.hpp>

/*We are generally using the standard library namespace.*/
using namespace std;

/* For a hashed collection of pointers -- where hashes are to be derived
 * from the elements pointed to.*/
template<typename T> 
struct deref_hash {
    inline std::size_t operator()(const T* t) const
    {
	VERBOSER(12, "Hash is :: "<<hasher(*t)<<endl);
	return hasher(*t);
    }
private:
    typedef boost::hash<T> Hasher;
    Hasher hasher;
    //typename HASH_BASE::hash<T> hasher;
};

/*Adaptable binary predicate along the lines of \class{std::equal_to}
 *only this dereferences its arguments that are forced to be
 *pointers.*/
template<typename T>
struct deref_equal_to
{
public:
    bool operator()(const T* t1, const T* t2) const { return *t1 == *t2;};
};

/*Free memory used at X and then assign it to NULL.*/
#define DELETE(X) {delete X; X = 0;}

/*DEPRECATED.*/
namespace MemoryManagement{
    extern vector<void*> toDelete;
}

/*DEPRECATED.*/
#define FAKE_DELETE(X){DELETE(X);}//MemoryManagement::toDelete.push_back(static_cast<void*>(X));}

/*DEPRECATED.*/
#define FREE_MEMORY { \
	for(vector<void*>::iterator p = MemoryManagement::toDelete.begin();\
	    p != MemoryManagement::toDelete.end(); p++){		\
	    delete *p; \
	}	       \
    } \

/*If ELEM can be pushed to an ostream, then we can get its string
 * (STR) representation.*/
#define GET_STRING(STR, ELEM){			\
	ostringstream oss;			\
	oss<<ELEM;				\
	STR = oss.str();			\
    }

/*Sometimes a class base is a template parameter. If you don't want to
 * add functionality to a class via its base, then simply pass the
 * \class{NotApplicable} as the template argument.*/
class NotApplicable{};

/* I have added this class because for some reason the PDDL people had
 * decided to parse integers and floating point numbers using one rule
 * (yacc see \module{pddl+}), and I needed a type to store the data
 * parsed by that rule. */
class IntAndDouble
{
public:
    IntAndDouble(int i);
    IntAndDouble(double d);

    int getVal(int = 0) const;
    double getVal(double = 0.0) const;

    bool isInt() const;
    bool isDouble() const;
    
private:
    /*Number as int.*/
    int i;

    /*Number as double.*/
    double d;
    
    /*Was the constructor argument an \type{int}.*/
    bool wasInt;
    
    /*Was the constructor argument a \type{double}.*/
    bool wasDouble;
};

/*Many classes have a representation as a string. To get a string
 * representation of an object that has such a thing, using the
 * \class{ostringstream}, or \class{istringstream} operators.*/
class HasStringRepresentation
{
public:
    friend ostream& operator<<(ostream&, const HasStringRepresentation&);
    HasStringRepresentation():asString(""),computedAsString(false){};
    virtual ~HasStringRepresentation(){};

    HasStringRepresentation& operator=(const HasStringRepresentation& hsr){
	asString = hsr.asString;
	computedAsString = hsr.computedAsString;
	return *this;
    };
    
    /*String based, so we may-as-well have an ordering relation based
     * on the string representation. It is up to the children to make
     * this relation more sensible for their purpose.*/
    virtual bool operator<(const HasStringRepresentation&) const;
    virtual bool operator==(const HasStringRepresentation&) const;
    
    virtual bool operator!=(const HasStringRepresentation& hsr) const
    {return !operator==(hsr);};
protected:
    virtual void computeAsString(const string& str) const
    {
	asString = str;
	computedAsString = true;
    }
    
    /*What does this look like as a string.*/
    mutable string asString;

    /*Have we already computed what this looks like as a string.*/
    mutable bool computedAsString;
};

ostream& operator<<(ostream&, const HasStringRepresentation&);

/*The following definition is included ONLY FOR COMPILE TIME DEBUGGING
 * purposes. The G++ compiler error when you try and print something
 * that can not be printed is _difficult_ to debug with. Hence this
 * definition is included so that you can explicitly define a stream
 * operator for the type causing problems and see exactly why the cast
 * is not working.*/
#define CAN_PRINT(X) ostream& operator<<(ostream& o, const X& x) {  \
	return o<<dynamic_cast<const HasStringRepresentation&>(x); \
    }								  \


template<typename T1, typename T2, typename T3>
class Triple
{
public:
    Triple(const T1& first, const T2& second, const T3& third)
	:first(first),second(second),third(third){};
    T1 first;
    T2 second;
    T3 third;


    template<typename T11, typename T22, typename T33>
    inline bool operator<(const Triple<T11, T22, T33>& otherTriple) const
    {
	if(first < otherTriple.first) {
	    return true;
	} else if (first == otherTriple.first) {
	    if(second < otherTriple.second){
		return true;
	    } else if (second == otherTriple.second) {
		if(third < otherTriple.third){
		    return true;
		}
	    }
	}

	return false;
    }

    
    template<typename T11, typename T22, typename T33>
    inline bool operator==(const Triple<T11, T22, T33>& otherTriple) const
    {
	return (first == otherTriple.first &&
		second == otherTriple.second &&
		third == otherTriple.third);
    }
    
    
    inline size_t hash_value() const {
        std::size_t seed = 0;
        boost::hash_combine(seed, first);
        boost::hash_combine(seed, second);
        boost::hash_combine(seed, third);
	return seed;
    }  
};

template<typename T1, typename T2, typename T3>
inline std::size_t hash_value(const Triple<T1, T2, T3>& triple)
{
    return triple.hash_value();
}

typedef Triple<uint, uint, uint> TripleInt;

template<typename T>
class Copyable : public T
{
public:
    Copyable(const T& t):T(t){};
    virtual ~Copyable(){};
    
    virtual Copyable<T>* copy() const = 0;
};

#define TYPED_STRING(NAME) class NAME : public Copyable<string> {	\
    public:						\
    NAME(const NAME& name):Copyable<string>(name.c_str()){}; \
    NAME(const char* name = ""):Copyable<string>(name){}; \
    NAME(const string& name):Copyable<string>(name){}; \
    Copyable<string>* copy() const {return new NAME(*this);} ;	\
    }; \



template<typename T>
class print_elements : public unary_function<T, void>
{
public:
    print_elements(ostream& out, int maxCount)
	: out(out),
	  count(0),
	  maxCount(maxCount){};
    
    void operator() (const T& x) {
	out << x ;

	if(count < maxCount - 1){    
	    out<< ' ';
	}
	
	count++;
    }
private:
    /*Writing to.*/
    ostream& out;
    /*Written so far.*/
    int count;
    /*Total to write.*/
    int maxCount;
};


/*Sometimes a container of pointers needs to be _copied_ into a
 * container of non-pointers and vise-versa. */
template<typename T1, typename T2>
class morph_elements : public unary_function<T1, T2>
{
public:
    /*Allocates spaces for returned value.*/
    T2 operator() (const T1* t1) {
	return T2(*t1);
    }
    T2* operator() (const T1& t1) {
	return new T2(t1);
    }
};



/*If you have an array of pointers*/
template<typename T>
class print_pointers : public unary_function<T, void>
{
public:
    print_pointers(ostream& out, int maxCount)
	: out(out),
	  count(0),
	  maxCount(maxCount){};
    
    void operator() (const T* const x) {
	out << *x ;

	if(count < maxCount - 1){    
	    out<< ' ';
	}
	
	count++;
    }
private:
    /*Writing to.*/
    ostream& out;
    /*Written so far.*/
    int count;
    /*Total to write.*/
    int maxCount;
};

/*If you have an array of pointers*/
template<typename T>
class delete_pointers : public unary_function<T, void>
{
public:
    delete_pointers(){};
    
    void operator() (T* x) {
	FAKE_DELETE(x);
    }
};

template<typename T1, typename T2>
class assert_dynamic_type : public unary_function<T2, void>
{
public:
    assert_dynamic_type(){};
    void operator()(const T2* x)
    {
	assert(dynamic_cast<const T1*>(x));
    }
    void operator()(const T2& x)
    {
	assert(dynamic_cast<const T1&>(x));
    }  
};

template<typename T1, typename T2>
ostream& operator<<(ostream& o, const pair<T1, T2>& p)
{
    return o<<p.first<<" - "<<p.second<<endl;
}


template<typename T>
ostream& operator<<(ostream& o
		    , const vector<T>& input)
{
    for_each(input.begin(), input.end(), print_elements<T>(o, input.size()));
    return o;
}

template<typename T>
ostream& operator<<(ostream&o, const vector<T*>& input)
{
    for_each(input.begin(), input.end(), print_pointers<T>(o, input.size()));
    return o;
}

template<typename T1, typename T2>
ostream& operator<<(ostream& o
		    , const map<T1, T2>& input)
{
    for_each(input.begin(), input.end(), print_elements<pair<T1, T2> >(o, input.size()));
    return o;
}


template<typename T>
ostream& operator<<(ostream& o
		    , const std::tr1::unordered_set<T, std::tr1::hash<T> >& input)
{
    for_each(input.begin(), input.end(), print_elements<T>(o, input.size()));
    return o;
}

template<typename T>
ostream& operator<<(ostream& o
		    , const std::set<T>& input)
{
    for_each(input.begin(), input.end(), print_elements<T>(o, input.size()));
    return o;
}


template<typename T1, typename T2>
ostream& operator<<(ostream& o
		    , const std::tr1::unordered_map<T1, T2, std::tr1::hash<T1> >& input)
{
    for_each(input.begin(), input.end(), print_elements<pair<T1, T2> >(o, input.size()));
    return o;
}


#include <zlib.h>

class Compression
{
public:
    /* Returns the number of elements in \argument{output} that were
     * not used.*/
    static int compress(unsigned char* input, uint input_size,
                         unsigned char* output, uint output_size,
                        int level = Z_DEFAULT_COMPRESSION);

    
    /* Returns the number of elements in \argument{output} that were
     * not used.*/
    static int decompress(unsigned char* input, uint input_size,
                          unsigned char* output, uint output_size);
};

template <typename T>
T *cxx_realloc(T *array, size_t old_size, size_t new_size)
{
    T *temp = new T[new_size];
    
    delete [] array;
    
    return copy(array, array + old_size, temp);
}

#endif
