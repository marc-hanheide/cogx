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

#ifndef STL__TYPED_THING_HH
#define STL__TYPED_THING_HH

#include "global.hh"


class basic_type;

namespace std
{
    std::ostream& operator<<(std::ostream&, const basic_type&);
}

template<int type_name, typename... T>
class type_wrapper;

namespace std
{    
    template<int type_name, typename... T>
    ostream& operator<<(std::ostream&, const type_wrapper<type_name, T...>&);
}


#include "stl__deref_tools.hh"
#include "stl__tuple_hash.hh"


#include "stl__tuple_printing.hh"



#define SIMPLE_WRAPPED_THING(TYPE_NAME, WRAPPER) class TYPE_NAME: public WRAPPER {}

#define WRAPPED_STRING(TYPE_ID, TYPE_NAME) SIMPLE_WRAPPED_THING(TYPE_NAME, has_string_name<TYPE_ID>)

#define WRAPPED_DOUBLE(TYPE_ID, TYPE_NAME) SIMPLE_WRAPPED_THING(TYPE_NAME, has_double_value<TYPE_ID>)

#define SIMPLE_WRAPPED_THING__WITH_DECLARATIONS(TYPE_NAME, WRAPPER, DECLARATIONS) class TYPE_NAME: public WRAPPER {DECLARATIONS}

#define WRAPPED_STRING__WITH_DECLARATIONS(TYPE_ID, TYPE_NAME, DECLARATIONS) SIMPLE_WRAPPED_THING__WITH_DECLARATIONS(TYPE_NAME, has_string_name<TYPE_ID>, DECLARATIONS)

#define WRAPPED_DOUBLE__WITH_DECLARATIONS(TYPE_ID, TYPE_NAME, DECLARATIONS) SIMPLE_WRAPPED_THING__WITH_DECLARATIONS(TYPE_NAME, has_double_value<TYPE_ID>, DECLARATIONS)

#define NEW_WRAPPED(TYPE_NAME, OBJECT_NAME, CONTENTS...)                \
    TYPE_NAME OBJECT_NAME = TYPE_NAME();                                \
    OBJECT_NAME.configure(CONTENTS);{}					\


#define NEW_WRAPPED_POINTER(TYPE_NAME, OBJECT_NAME, CONTENTS...)        \
    CXX__PTR_ANNOTATION(TYPE_NAME) OBJECT_NAME(new TYPE_NAME());        \
    OBJECT_NAME->configure(CONTENTS);{}					\


#define NEW_WRAPPED_deref_POINTER(TYPE_NAME, OBJECT_NAME, CONTENTS...)  \
    CXX__deref__shared_ptr<basic_type>                                  \
    OBJECT_NAME(CXX__PTR_ANNOTATION(basic_type)(                        \
                    static_cast<TYPE_NAME*>(0)));                       \
    {                                                                   \
        NEW_WRAPPED_POINTER(TYPE_NAME, _OBJECT_NAME, CONTENTS);         \
        OBJECT_NAME = CXX__deref__shared_ptr<basic_type>(_OBJECT_NAME); \
    }                                                                   \

#define NEW_referenced_WRAPPED(SIZE_T, TYPE_NAME, OBJECT_NAME, CONTENTS...) \
    TYPE_NAME OBJECT_NAME = TYPE_NAME();                                    \
    OBJECT_NAME.set__runtime_Thread(SIZE_T);                                \
    OBJECT_NAME.configure(CONTENTS);{} 					    \
    

#define NEW_referenced_WRAPPED_POINTER(SIZE_T, TYPE_NAME, OBJECT_NAME, CONTENTS...) \
    CXX__PTR_ANNOTATION(TYPE_NAME) OBJECT_NAME(new TYPE_NAME());                    \
    OBJECT_NAME->set__runtime_Thread(SIZE_T);                                       \
    OBJECT_NAME->configure(CONTENTS);                                               \


#define NEW_referenced_WRAPPED_deref_POINTER(SIZE_T, TYPE_NAME, OBJECT_NAME, CONTENTS...)  \
    CXX__deref__shared_ptr<basic_type>                                                     \
    OBJECT_NAME(CXX__PTR_ANNOTATION(basic_type)(                                           \
                    static_cast<TYPE_NAME*>(0)));                                          \
    {                                                                                      \
        NEW_referenced_WRAPPED_POINTER(SIZE_T, TYPE_NAME, _OBJECT_NAME, CONTENTS);         \
        OBJECT_NAME = CXX__deref__shared_ptr<basic_type>(_OBJECT_NAME);                    \
    }                                                                                      \


#define NEW_object_referenced_WRAPPED(TYPE_NAME, OBJECT_NAME, CONTENTS...)                                       \
    assert(sizeof(size_t) == sizeof(void*));                                                                     \
    NEW_referenced_WRAPPED(reinterpret_cast<basic_type::Runtime_Thread>(this), TYPE_NAME, OBJECT_NAME, CONTENTS) \


#define NEW_object_referenced_WRAPPED_POINTER(TYPE_NAME, OBJECT_NAME, CONTENTS...)                                       \
    assert(sizeof(size_t) == sizeof(void*));                                                                             \
    NEW_referenced_WRAPPED_POINTER(reinterpret_cast<basic_type::Runtime_Thread>(this), TYPE_NAME, OBJECT_NAME, CONTENTS) \


#define NEW_object_referenced_WRAPPED_deref_POINTER(TYPE_NAME, OBJECT_NAME, CONTENTS...)                                       \
    assert(sizeof(size_t) == sizeof(void*));                                                                                   \
    NEW_referenced_WRAPPED_deref_POINTER(reinterpret_cast<basic_type::Runtime_Thread>(this), TYPE_NAME, OBJECT_NAME, CONTENTS) \


class basic_type
{
public:
    typedef std::size_t Runtime_Thread;
    
    basic_type(Runtime_Thread runtime_Thread = 0):runtime_Thread(runtime_Thread){};

    virtual ~basic_type(){};
    
    virtual std::ostream& operator<<(std::ostream&) const;
    virtual bool operator<(const basic_type&) const;
    virtual bool operator==(const basic_type&in) const;
    virtual std::size_t hash_value() const = 0;
    virtual int get__type_name() const = 0;
    virtual std::size_t get__id() const = 0;
protected:
    Runtime_Thread runtime_Thread;

private:
    std::string as_string() const;
};

typedef std::set<CXX__deref__shared_ptr<basic_type>> basic_types;
typedef std::vector<CXX__deref__shared_ptr<basic_type>> basic_types__vector;

std::size_t hash_value(const basic_type&);


template<int type_name, typename... T>
class type_wrapper : public basic_type
{
public:
    typedef std::tr1::tuple<T...> Contents;
    typedef type_wrapper<type_name, T...> THIS;
    typedef std::size_t ID_TYPE;

    ~type_wrapper(){};

    template<uint i, typename Modifier>
    void modify(Modifier modifier)
    {
        assert(id < traversable_Collection->size());
        modifier(std::tr1::get<i>(*traversable_Collection)[id]);
    }
    
    /* Should only be called once.*/
    THIS& configure(const T&... wrapped_contents)
    {
        Contents contents(wrapped_contents...);
        return configure(contents);
    }

    bool is_configured() const {return already_called_configure;};
    
    THIS& configure(const Contents& wrapped_contents)
    {
        if(!already_called_configure){
//             std::cerr<<"ONE::  "<<wrapped_contents<<std::endl;
            already_called_configure = true;
            _configure(wrapped_contents);

//             std::cerr<<"TWO::  "<<*this<<std::endl;
        } else {
            WARNING("Attempted to configure object :: "<<id<<std::endl
                    <<"With contents :: "<<contents()<<std::endl
                    <<"TWICE. We are ignoring the second call to \function{configure}." );
        }
    }
    
    const Contents& contents() const
    {   
        QUERY_WARNING(
            id >= traversable_Collection->size(),
            "Requesting object associated with :: "<<id<<" yet not such object exists." );
        
        return (id < traversable_Collection->size()?((*traversable_Collection)[id]):fake_contents);
    }

    Contents contents() 
    {
        QUERY_WARNING(
            id >= traversable_Collection->size(),
            "Requesting object associated with :: "<<id<<" yet not such object exists." );
        
        return ((id < traversable_Collection->size())?(*traversable_Collection)[id]:fake_contents);
    }

    int get__type_name() const {return type_name;};
    
    std::ostream& operator<<(std::ostream& o) const
    {
        return o<<contents();
    }

    std::size_t hash_value() const
    {
        std::size_t seed = boost::hash_value(contents());
        boost::hash_combine(seed, type_name);
        boost::hash_combine(seed, runtime_Thread);
        boost::hash_combine(seed, already_called_configure);
        
        return seed;
    }

    bool operator<(const THIS&in) const{
        if(runtime_Thread < in.runtime_Thread)return true;
        if(runtime_Thread > in.runtime_Thread)return false;
        
        if(already_called_configure){
            if(in.already_called_configure){
                 return id < in.id;
            } else {
                return true;
            }
        } else {
            if(in.already_called_configure){
                return false;
            } else {
                 return id < in.id;
            }
        }
    }
    
    bool operator==(const THIS&in) const{
        return (in.already_called_configure == already_called_configure) &&
            (id == in.id) &&
            (runtime_Thread == in.runtime_Thread);
    }
    
    ID_TYPE get__id() const {return id;}; 
    
    typedef std::vector<Contents > Traversable_Collection;
    typedef std::map<Contents, ID_TYPE> Searchable_Collection;
    
    typedef std::map<decltype(type_wrapper::runtime_Thread)
        , CXX__PTR_ANNOTATION(Traversable_Collection)> Indexed__Traversable_Collection;
    typedef std::map<decltype(type_wrapper::runtime_Thread)
        , CXX__PTR_ANNOTATION(Searchable_Collection)> Indexed__Searchable_Collection;
        
    static Indexed__Traversable_Collection indexed__Traversable_Collection;
    static Indexed__Searchable_Collection indexed__Searchable_Collection;
    
    CXX__PTR_ANNOTATION(Traversable_Collection) traversable_Collection;
    CXX__PTR_ANNOTATION(Searchable_Collection) searchable_Collection;
    
    type_wrapper(decltype(type_wrapper::runtime_Thread) _runtime_thread = 0)
        :basic_type(_runtime_thread),
         already_called_configure(false),
         id(0)
    {
        assert(_runtime_thread == runtime_Thread);
        set__runtime_Thread(runtime_Thread);
//         auto _traversable_Collection = indexed__Traversable_Collection.find(runtime_Thread);
//         auto _searchable_Collection = indexed__Searchable_Collection.find(runtime_Thread);

//         if(_traversable_Collection == indexed__Traversable_Collection.end()){
//             indexed__Traversable_Collection[runtime_Thread]
//                 = CXX__PTR_ANNOTATION(Traversable_Collection)(new Traversable_Collection());
            
//             _traversable_Collection = indexed__Traversable_Collection.find(runtime_Thread);
//         }
//         if(_searchable_Collection == indexed__Searchable_Collection.end()){
//             indexed__Searchable_Collection[runtime_Thread] 
//                 = CXX__PTR_ANNOTATION(Searchable_Collection)(new Searchable_Collection());
            
//             _searchable_Collection = indexed__Searchable_Collection.find(runtime_Thread);   
//         }

//         traversable_Collection = indexed__Traversable_Collection[runtime_Thread];
//         searchable_Collection = indexed__Searchable_Collection[runtime_Thread];
    };

    void set__runtime_Thread(Runtime_Thread in)
    {
        runtime_Thread = in;
        auto _traversable_Collection = indexed__Traversable_Collection.find(runtime_Thread);
        auto _searchable_Collection = indexed__Searchable_Collection.find(runtime_Thread);

        if(_traversable_Collection == indexed__Traversable_Collection.end()){
            indexed__Traversable_Collection[runtime_Thread]
                = CXX__PTR_ANNOTATION(Traversable_Collection)(new Traversable_Collection());
            
            _traversable_Collection = indexed__Traversable_Collection.find(runtime_Thread);
        }
        if(_searchable_Collection == indexed__Searchable_Collection.end()){
            indexed__Searchable_Collection[runtime_Thread] 
                = CXX__PTR_ANNOTATION(Searchable_Collection)(new Searchable_Collection());
            
            _searchable_Collection = indexed__Searchable_Collection.find(runtime_Thread);   
        }

        traversable_Collection = indexed__Traversable_Collection[runtime_Thread];
        searchable_Collection = indexed__Searchable_Collection[runtime_Thread]; 
    }
    
private:
    void _configure(const Contents& wrapped_contents)
    {
        auto map_iterator = searchable_Collection->find(wrapped_contents);
            
        if(map_iterator
           == searchable_Collection->end()){
//             std::cerr<<"NEW ENTRY..."<<wrapped_contents<<std::endl;
            traversable_Collection->push_back(wrapped_contents);
            size_t index = traversable_Collection->size() - 1;
            assert((*traversable_Collection)[index] == wrapped_contents);
            (*searchable_Collection)[wrapped_contents] = index;

//             std::cerr<<(*traversable_Collection)[index]<<std::endl;
//             {char ch; std::cin>>ch;};
            
            map_iterator = searchable_Collection->find(wrapped_contents);
            assert(map_iterator != searchable_Collection->end());
        } else {
//             std::cerr<<"OLD ENTRY..."<<std::endl;
        }
        

        id = map_iterator->second;
        assert(id < traversable_Collection->size());
    }

    
    /* Has the \method{configure} already been called. */
    bool already_called_configure;

    /*(see \member{contents()} and \member{contents() const})*/
    Contents fake_contents;
    
    /*....*/
    ID_TYPE id;
};


template<int type_name>
class has_string_name : public type_wrapper<type_name, std::string>
{
public:
    typedef type_wrapper<type_name, std::string> Parent;
    const std::string& get__name() const {return std::tr1::get<0>(Parent::contents());}
};

template<int type_name>
class has_double_value : public type_wrapper<type_name, double>
{
public:
    typedef type_wrapper<type_name, double> Parent;
    double get__value() const {return std::tr1::get<0>(Parent::contents());}
};


template<int type_name, typename... T>
typename type_wrapper<type_name, T...>::Indexed__Traversable_Collection type_wrapper<type_name, T...>::indexed__Traversable_Collection;

template<int type_name, typename... T>
typename type_wrapper<type_name, T...>::Indexed__Searchable_Collection type_wrapper<type_name, T...>::indexed__Searchable_Collection;

namespace std
{    
    template<int type_name, typename... T>
    std::ostream& operator<<(std::ostream& o, const type_wrapper<type_name, T...>& thing)
    {
        return thing.operator<<(o);
    }
    
}



#endif
