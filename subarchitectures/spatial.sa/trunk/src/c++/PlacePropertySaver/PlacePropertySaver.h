/**
 * Author: Nikolaus Demmel
 * Date:  23.03.2012
 **/

#ifndef __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__
#define __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__

#include <cast/architecture/ManagedComponent.hpp>

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

  /** Read all place properties from working memory and save them to disk. */
  void savePlaceProperties();

  /** Read all place properties from disk and put them into working memory. */
  void loadPlaceProperties() {} ;

private:
  
  /** Working memory id where place properties are found */
  static const std::string cWorkingMemoryID;

private:

  /** Flags saying if we want to save at all. */
  bool _doSave;

  /** Flags saying if we want to load at all. */
  bool _doLoad;

  /** Filename where properties are serialized to. */
  std::string _saveFileName;

  /** Filename where properties are loaded from. Can be the same as savefile. */
  std::string _loadFileName;

  /** Interval between saving all place properties in milliseconds. */
  unsigned long _saveInterval;

  /** If false data is only saved once at the end, else it is also saved all the
   * time. */
  bool _saveContinuously;
  
};

#endif
