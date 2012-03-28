
#ifndef __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__
#define __SPATIAL_SA_PLACE_PROPERTY_SAVER_H__

#include <cast/architecture/ManagedComponent.hpp>

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
