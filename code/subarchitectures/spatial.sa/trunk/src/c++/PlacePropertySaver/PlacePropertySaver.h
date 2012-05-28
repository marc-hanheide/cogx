/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   23.03.2012
 **/

#ifndef __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__
#define __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__

#include <cast/architecture/ManagedComponent.hpp>

#include <castutils/OptionParserMixin.hpp>

// FIXME: add something like this to cast utils instead of having it here
namespace cast {

/**
 *   Provide the name of the dynamic type of an object
 */
template<class T>
const std::string& dynamicTypeName(const IceInternal::Handle<T> object) {
  return object->ice_id();
}

} // namespace cast


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
 *
 * Configuration:
 *
 * --save
 * Boolean to indicate if we want to save at all.
 *
 * --save-file-name
 * File name where the properties are saved to.
 *
 * --save-continuously
 * 
 * Boolean to indicate if we shoudl, when saving, save all the time. Otherwise
 * we only when component is stopped.
 * 
 * --save-interval
 * Interval in milliseconds of how long we should wait between saves when saving
 * continously.
 *
 * --load
 * Booloan to indicate if we want to load at all.
 *
 * --load-file-name
 * File name where the properties are loaded from. If this file is not there,
 * loading is automatically deactivated, even if "--load true" was supplied
 *
 * --wait-before-loading
 * Time in milliseconds that we should wait additionally before loading anything.
 *
 * --wait-between-loading
 * Time in milliseconds that we should wait between each property being loaded.
 *
 * --wait-for-map-load-status
 * If loading is activated (and the file was found), wait for the map load status
 * in WM to indicate the places being loaded first and also update this status
 * once we are finished loading stuff.
 */
class PlacePropertySaver: public castutils::OptionParserMixinBaseclass<
    cast::ManagedComponent> {
public:

  PlacePropertySaver();

  virtual ~PlacePropertySaver();

  virtual void configure(const std::map<std::string, std::string> &config);

  virtual void runComponent();

  virtual void start();

  virtual void stop();

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
  template<class T>
  void addToWorkingMemoryDynamicType(const std::string &_id,
      const std::string &_subarch, IceInternal::Handle<T> _data)
      throw (cast::AlreadyExistsOnWMException,
      cast::UnknownSubarchitectureException) {

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
    if (!isVersioned(_id)) {
      debug("is not versioned: %s", _id.c_str());
      startVersioning(_id);
    }
    //if it's already versioned, then we need to update our numbers      
    else {
      debug("re-adding to working memory with id %s", _id.c_str());
      //get the last known version number, which we store + 1
      versionWhichWillEndUpOnWM = getVersionNumber(_id, _subarch) + 1;
      storeVersionNumber(_id, versionWhichWillEndUpOnWM);
    }

    // FIXME: private
    //    if(m_copyOnWrite) {
    m_workingMemory->addToWorkingMemory(_id, _subarch, type, getComponentID(),
        _data->ice_clone());
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

  /** Wait for the map load status to become appropriate. */
  void waitForMapLoadStatus();

  /** Callbacks */
  void mapLoadStatusAdded(const cast::cdl::WorkingMemoryChange &wmc);
  void mapLoadStatusOverwritten(const cast::cdl::WorkingMemoryChange &wmc);
  void mapLoadStatusDeleted(const cast::cdl::WorkingMemoryChange &wmc);

  /** Returns true if the map load status in working memory is such that we can
   * start loading. */
  bool checkWMMapLoadStatus();

  /** Indicates in the working memory that the place properties are loaded */
  void updateWMMapLoadStatus();

  // TODO: Add this somewhere in utilities
  /** Sleep component but only in small intervals to stay responsive to stop.
   *
   * Time is given in milliseconds. Returns false if sleep was abortet due to
   * the component being shut down, true otherwise.
   */
  bool sleepComponentResponsive(unsigned long millis);

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

  /** Should we wait for the MapLoadStatus in working memory before loading */
  bool _waitForMapLoadStatus;

  /** Working memory address of the map load status. */
  cast::cdl::WorkingMemoryAddress _mapLoadStatusAddress;

  /** Is the map load status such that we can go ahead with loading? */
  bool _mapLoadStatusOk;

  /** Time to wait before starting to load properties. In milliseconds.
   *
   * This can be used to make sure starting all components has setteled down. If
   * _waitForMapLoadStatus is true, the time is wait additionally after the
   * status becomes such that we could load. Note, however, that if use
   * _waitForMapLoadStatus, additional waiting time is probably not neccessary.
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
