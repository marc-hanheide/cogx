/**
 * Author: Nikolaus Demmel
 * Date:  23.03.2012
 **/

#ifndef __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__
#define __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__

#include <cast/architecture/ManagedComponent.hpp>


// FIXME: add something like this to cast utils instead of having it here
namespace cast {

/**
 *   Provide the name of the dynamic type of an object
 */  
template <class T>
const std::string& dynamicTypeName(const IceInternal::Handle<T> object) {
  return object->ice_id();
}

}



/**
 * The PlacePropertySaver component can save observerd place properties to a
 * file either periodically or at the end of execution. It can also load this
 * information back into the working memory at the beginning of a new run. It
 * should save all information about observations such that conceptual/Observer
 * can recreate the conceptual world state. If compnents that provide these
 * place-properties to the working memory fail if there are already such
 * properties loaded at start-up time by PlacePropertySaver, it is considered a
 * bug in the providing component. (An example would be a component adding
 * object oservations too often, because it keeps track of the number in the
 * internal state of itself and does not recognize object observations added by
 * PlacePropertyManager. This would the be a bug in the object observation
 * component).
 *
 * For serialization the Ice-generated serializers are used. This means the
 * result is a non-human-readable byte string. We might want to write a better
 * text based and human-readable output format that would aid debugging.
 */
class PlacePropertySaver: public cast::ManagedComponent 
{
public:
  
  PlacePropertySaver();

  virtual ~PlacePropertySaver();

  virtual void configure(const std::map<std::string,std::string> &config);

  virtual void runComponent();

  virtual void start();

  virtual void stop();

private:
  // TODO: These config parse helpers could be added to CASTComponent

  /** Parse a string option.
   *
   * The default value is returned if the option was not passed.
   */
  std::string parseOption(
      const std::string name, 
      const std::string defaultValue,
      const std::map<std::string,std::string> &config);

  /** Parse a flag (i.e. boolean) option.
   *
   * Default value is 'false'. It is returned if the option was not passed. If
   * the option was passed, but the value passed is maleformed (i.e. not "true"
   * or "false"), print error and return default value.
   */
  bool parseFlagOption(
      const std::string name, 
      const std::map<std::string,std::string> &config);
  
  /** Parse an option and perform a lexical cast.
   *
   * The default value is returned if the option was not passed. If the option
   * was passed, but the lexical cast throws an exception, print error and
   * return default value.
   */
  template<class T>
  T parseOptionLexicalCast(
      const std::string name,
      const T defaultValue,
      const std::map<std::string,std::string> &config);

  /** Parse a path option and resolve the path.
   *
   * If the option was not passed, the default value is resolved and
   * returned. If in that case the default value can not be resolved, print an
   * error and return an empty string. If the option was passed, but the value
   * could not be resolved as a path, print error and return default value, else
   * return the resolved path.
   */ 
  std::string parsePathOption(
      const std::string name, 
      const std::string defaultValue,
      const std::map<std::string,std::string> &config);

private:
  // FIXME: this should not be here, but in WorkingMemoryWriterComponent

  /**
   * Add new data to working memory. The data will be stored with the given
   * id. Determines the dynamic type of _data instead of using the static
   * type.
   * 
   * @param _id
   *            The id the data will be stored with.
   * @param _subarchitectureID
   *            The subarchitecture to write to.
   * @param _data
   *            The data itself. Must be a ref-counted pointer to an instance of an Ice class.
   * @throws AlreadyExistsOnWMException
   *             If an entry exists at the given id.
   */
  template <class T>
  void addToWorkingMemoryDynamicType(const std::string &_id, 
                                     const std::string &_subarch,
                                     IceInternal::Handle<T>  _data) 
      throw (cast::AlreadyExistsOnWMException, cast::UnknownSubarchitectureException) { 
    
    assert(!_id.empty());//id must not be empty
    assert(!_subarch.empty());//subarch must not be empty
    assert(_data);//data must not be null
    
    //UPGRADE not sure what to do with this now
    //checkPrivileges(_subarch);
    
    std::string type(dynamicTypeName(_data));
    
    //logMemoryAdd(_id, _subarch, type);
    
    //#bug 52, testcase 2: If we are already versioning this data
    //it's ok. This means we had previously written to this address.
    
    int versionWhichWillEndUpOnWM = 0;
    
    //if not versioned, start doing so
    if(!isVersioned(_id)) {
      debug("is not versioned: %s",_id.c_str());
      startVersioning(_id);
    }
    //if it's already versioned, then we need to update our numbers      
    else {
      debug("re-adding to working memory with id %s",_id.c_str());	
      //get the last known version number, which we store + 1
      versionWhichWillEndUpOnWM = getVersionNumber(_id,_subarch) + 1;
      storeVersionNumber(_id, versionWhichWillEndUpOnWM);
    }

    // FIXME: private
    //    if(m_copyOnWrite) {
          m_workingMemory->addToWorkingMemory(_id,_subarch,type,getComponentID(),_data->ice_clone()); 
    //    }
    //    else {
    //      m_workingMemory->addToWorkingMemory(_id,_subarch,type,getComponentID(),_data); 
    //    }
    
    // FIXME: private
    //logAdd(_id, _subarch, type, versionWhichWillEndUpOnWM);
	  
    
  }

private:

  /** Read all place properties from working memory and save them to disk. */
  void savePlaceProperties();

  /** Read all place properties from disk and put them into working memory. */
  void loadPlaceProperties();

private:
  
  /** Working memory id where place properties are found */
  static const std::string cWorkingMemoryID;

private:

  /** Flags saying if we want to save at all. */
  bool _doSave;

  /** Filename where properties are serialized to. */
  std::string _saveFileName;

  /** Interval between saving all place properties in milliseconds. */
  unsigned int _saveInterval;

  /** If false data is only saved once at the end, else it is also saved all the
   * time. */
  bool _saveContinuously;

  /** Flags saying if we want to load at all. */
  bool _doLoad;

  /** Filename where properties are loaded from. Can be the same as savefile. */
  std::string _loadFileName;

  /** Time to wait before starting to load properties. In milliseconds.
   *
   * This can be used to make sure starting all components has setteled down.
   */
  unsigned int _waitBeforeLoading;

  /** Time to wait adding loaded properties to working memory. In milliseconds.
   *
   * This can be used to make sure other components are not overwhelmed by
   * adding too many properties too fast.
   */  
  unsigned int _waitBetweenLoading;
};

#endif
