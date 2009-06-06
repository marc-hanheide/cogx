#ifndef CAST_COMPONENT_FACTORY_HPP_
#define CAST_COMPONENT_FACTORY_HPP_

#include <cast/server/ComponentCreator.hpp>

#include <cast/slice/CDL.hpp>


namespace cast {


  class CASTComponentFactory : 
    public virtual cast::interfaces::ComponentFactory {

    typedef StringMap<DynamicComponentCreator *>::map ComponentCreatorMap;
    ComponentCreatorMap m_creators;
    
    
  public:
    virtual ~CASTComponentFactory(){}
    
    virtual ::cast::interfaces::CASTComponentPrx newComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt)
            throw (ComponentCreationException);

    virtual ::cast::interfaces::ManagedComponentPrx newManagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt)
            throw (ComponentCreationException);

    virtual ::cast::interfaces::UnmanagedComponentPrx newUnmanagedComponent(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt)
            throw (ComponentCreationException);

    virtual ::cast::interfaces::WorkingMemoryPrx newWorkingMemory(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt) 
            throw (ComponentCreationException);
    
    virtual ::cast::interfaces::TaskManagerPrx newTaskManager(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt)
      throw (ComponentCreationException);

//     virtual 
//     void remove(const std::string & _id,
// 		const std::string & _type,
// 		const Ice::Current & _crt);

  private:

    cast::interfaces::CASTComponentPrx createBase(const ::std::string& id, const ::std::string& type, const ::Ice::Current & _crt) 
      throw (ComponentCreationException);

  };

};


#endif


