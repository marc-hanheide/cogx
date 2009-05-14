/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef REMOTE_DATA_TRANSLATORS_H_
#define REMOTE_DATA_TRANSLATORS_H_

#include "includes.hpp"

//hpp interface version

//this doesn't do anything except allow many translators to be stored in a single collection
class AbstractRemoteDataTranslator {
 public:
  AbstractRemoteDataTranslator(){};
  virtual ~AbstractRemoteDataTranslator(){};
};

/**
 * Interface that defines an object that is able to translate between
 * a specific datatype and its resulting CORBA::Any form.
 */
template <class T>
class RemoteDataTranslator: public AbstractRemoteDataTranslator {
public:
  
  /**
   * Empty virtual destructor.
   */
  virtual ~RemoteDataTranslator(){};
 
  
  /*
   * Translate from _data into _any. This assumes that data pointed to
   * by _data will not be used subsequently.
   */
  virtual void translate(T *& _data, CORBA::Any &_any) const = 0;

  /**
   * Translate from _any into a pointer to new memory containing the
   * appropriate datatype.
o   */
  virtual T* translate(const CORBA::Any &_any) const = 0;
};


//generic version
template <class T>
class GenericRemoteDataTranslator: public RemoteDataTranslator<T> {
public:

  virtual ~GenericRemoteDataTranslator(){};
  
  //translate to and from a std std::string
  virtual void translate(T *&_data, CORBA::Any &_any) const {
    //cout<<"generic version from Any"<<std::endl;
    //doesn't consume!
    _any<<=(*_data);
  }
  
  virtual T* translate(const CORBA::Any &_any) const {
    //  cout<<"generic version to Any"<<std::endl;
    
    T data;
    if(_any >>= data) {
    
    //invoke copy constructor to create pointer.
    return new T(data);
    }
    else {
      std::cerr<<"GenericRemoteDataTranslator: Extraction from Any failed"<<std::endl;
      return NULL;
    }
    
  }

};



//generic version
template <class T>
class GenericConsumingTranslator: public RemoteDataTranslator<T> {
public:

  virtual ~GenericConsumingTranslator(){};
  
  //translate to and from a std std::string
  virtual void translate(T *&_data, CORBA::Any &_any) const {
    //cout<<"generic consuming version to Any"<<std::endl;

    //non consuming,,,
    //_any<<=(*_data);
    
    //consuming
    _any<<=_data;

    //can't touch that memory any longer
    _data = NULL;
  }



  T * translate(const CORBA::Any & _any) const {

    //cout<<"T version from Any"<<std::endl;
    //cout<<_any.type()->kind()<<std::endl;
    //cout.flush();

    const T *i = NULL;
    if(_any >>= i) { // OK, extract T... but Any owns the memory
    
      //cout<<"Extraction done"<<std::endl;
      //cout.flush();
      
      //this copies the memory, so we're ok
      return new T(*i);
    }
    else {
      std::cerr<<"Extraction failed"<<std::endl;
      return NULL;
    }
    
  }

};


//std::std::string version
template <> class GenericRemoteDataTranslator<std::string>: public RemoteDataTranslator<std::string>  {
 public:
virtual ~GenericRemoteDataTranslator(){};

  //translate to and from a std std::string
  void translate(std::string *&_data, CORBA::Any &_any) const {
    //cout<<"std::string version to Any"<<std::endl;
    _any<<=_data->c_str();
  }
  
  //translate to and from a std std::string
  std::string * translate(const CORBA::Any &_any) const {
    //cout<<"std::string version from Any"<<std::endl;
    
    const char * msg; // Note const char *, not
    if(_any >>= msg) { // OK, extract std::string... but Any owns the memory

      //this copies the const char *, so we're ok
      return new std::string(msg);
    
    }
    else {
      std::cerr<<"Extraction from Any failed"<<std::endl;
      return NULL;
    }

    
  }
  
  
  
};

template <class T>
class GenericObjectDataTranslator: public RemoteDataTranslator<T> {
public:

  virtual ~GenericObjectDataTranslator(){};
  
  //translate to and from a std std::string
  virtual void translate(T *&_data, CORBA::Any &_any) const {

    //std::cerr<<"GenericObjectDataTranslator::translate"<<std::endl;
    
    //  consuming translation
    _any<<=_data;
    
    //can't touch that memory any longer
    _data = NULL;

  }
  
  virtual T* translate(const CORBA::Any &_any) const {
    //  cout<<"generic version to Any"<<std::endl;
    
    const T data;
    if(_any >>= data) {
          //invoke copy constructor to create pointer.
      return new T(data);
    }
    else {
      std::cerr<<"Extraction from Any failed"<<std::endl;
      return NULL;
    }
    
  }

};



//std::std::string version
template <> class GenericRemoteDataTranslator<CORBA::Boolean>: public RemoteDataTranslator<CORBA::Boolean>  {
 public:
virtual ~GenericRemoteDataTranslator(){};
 
//translate to and from a std std::string
 void translate(CORBA::Boolean *&_data, CORBA::Any &_any) const {
   _any<<=CORBA::Any::from_boolean(*_data);
 }
  

 CORBA::Boolean * translate(const CORBA::Any &_any) const {
   CORBA::Boolean b;
   _any >>= CORBA::Any::to_boolean(b);
   return new CORBA::Boolean(b);
 }
   
};

//std::std::string version
template <> class GenericRemoteDataTranslator<CORBA::Char>: public RemoteDataTranslator<CORBA::Char>  {
 public:
virtual ~GenericRemoteDataTranslator(){};
 
//translate to and from a std std::string
 void translate(CORBA::Char *&_data, CORBA::Any &_any) const {
   _any<<=CORBA::Any::from_char(*_data);
 }
  
 CORBA::Char * translate(const CORBA::Any &_any) const {
   CORBA::Char c;
   _any >>= CORBA::Any::to_char(c);
   return new CORBA::Char(c);
 }
   
};

//std::std::string version
template <> class GenericRemoteDataTranslator<int>: public RemoteDataTranslator<int>  {
public:
  //GenericRemoteDataTranslator(){std::cout<<"int created"<<std::endl;};
  virtual ~GenericRemoteDataTranslator(){};
  
  //translate to and from a std std::string
  void translate(int *&_data, CORBA::Any &_any) const {
    //std::cout<<"int looks like: "<<*_data<<std::endl;
    _any<<=(CORBA::Long)(*_data);
  }
  
  int * translate(const CORBA::Any &_any) const {
    CORBA::Long c;
    _any >>= c;
    return new int(c);
  }
  
};

//std::std::string version
template <> class GenericConsumingTranslator<int>: public RemoteDataTranslator<int>  {
public:
  //GenericRemoteDataTranslator(){std::cout<<"int created"<<std::endl;};
  virtual ~GenericConsumingTranslator(){};
  
  //translate to and from a std std::string
  void translate(int *&_data, CORBA::Any &_any) const {
    //std::cout<<"int looks like: "<<*_data<<std::endl;
    _any<<=(CORBA::Long)(*_data);
    //now delete because we consume
    delete _data;
    _data = NULL;
  }
  
  int * translate(const CORBA::Any &_any) const {
    CORBA::Long c;
    _any >>= c;
    return new int(c);
  }
  
};

//std::std::string version
template <> class GenericConsumingTranslator<std::string>: public RemoteDataTranslator<std::string>  {
public:
  virtual ~GenericConsumingTranslator(){};
  
  //translate to and from a std std::string
  void translate(std::string *&_data, CORBA::Any &_any) const {
    //cout<<"std::string version to Any"<<std::endl;
    _any<<=_data->c_str();
  }
  
  //translate to and from a std std::string
  std::string * translate(const CORBA::Any &_any) const {
    //cout<<"std::string version from Any"<<std::endl;
    
    const char * msg; // Note const char *, not
    if(_any >>= msg) { // OK, extract std::string... but Any owns the memory
      
      //this copies the const char *, so we're ok
      return new std::string(msg);
      
    }
    else {
      std::cerr<<"Extraction from Any failed"<<std::endl;
      return NULL;
    }
  }
};

//std::std::string version
template <> class GenericConsumingTranslator<bool>: public RemoteDataTranslator<bool>  {
public:
  virtual ~GenericConsumingTranslator(){};
 
// //translate to and from a std std::string
//  void translate(CORBA::Boolean *&_data, CORBA::Any &_any) const {
//    _any<<=CORBA::Any::from_boolean(*_data);
//    //TODO does this consume???
//  }
  
//  CORBA::Boolean * translate(const CORBA::Any &_any) const {
//    CORBA::Boolean b;
//    _any >>= CORBA::Any::to_boolean(b);
//    return new CORBA::Boolean(b);
//  }


  //translate to and from a std std::string
  void translate(bool *&_data, CORBA::Any &_any) const {
    _any<<=CORBA::Any::from_boolean(*_data);
    //TODO does this consume???
  }
  
  bool * translate(const CORBA::Any &_any) const {
    CORBA::Boolean b;
    _any >>= CORBA::Any::to_boolean(b);
    return new CORBA::Boolean(b);
  }
  
  
};




//sequence version
template <class S>
class GenericRemoteSequenceTranslator: public RemoteDataTranslator<S>  {
 public:

  virtual ~GenericRemoteSequenceTranslator(){};

  //translate to and from a std LongSeq
  void translate(S *&_data, CORBA::Any &_any) const {
    //consuming
    _any<<=_data;

    //can't touch that memory any longer
    _data = NULL;
    
  }
  
  //translate to and from a std S
  S * translate(const CORBA::Any &_any) const {
    //cout<<"S version from Any"<<std::endl;

    

    const S *i;
    if(_any >>= i) {// OK, extract S... but Any owns the memory
      //this copies the const char *, so we're ok
      return new S(*i);
    }
    else {
      std::cerr<<"GenericRemoteSequenceTranslator: Sequence extraction from Any failed"<<std::endl;
      std::cerr<<_any.type()<<std::endl;
      std::cerr<<_any.type()->name()<<std::endl;
      std::cerr<<_any.type()->id()<<std::endl;
      return NULL;
    }
  }
  
};


#endif

