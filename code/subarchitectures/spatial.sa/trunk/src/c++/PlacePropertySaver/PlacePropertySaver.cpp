/**
 * Author: Nikolaus Demmel
 * Date:  23.03.2012
 **/

#include "PlacePropertySaver.h"
#include <SpatialProperties.hpp>

#include <Ice/Initialize.h>
#include <Ice/LocalException.h>

#include <fstream> 
#include <boost/shared_array.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace boost;
using namespace cast;
using boost::posix_time::ptime;
using boost::posix_time::microsec_clock;
using boost::posix_time::to_simple_string;


// ------------------------------------------------------
extern "C" 
{
  cast::interfaces::CASTComponentPtr newComponent()
  {
    return new PlacePropertySaver();
  }
}


// ------------------------------------------------------
const string PlacePropertySaver::cWorkingMemoryID("spatial.sa");


// ------------------------------------------------------
PlacePropertySaver::PlacePropertySaver()
{
  debug("Created.");
}


// ------------------------------------------------------
PlacePropertySaver::~PlacePropertySaver()
{
  debug("Destroyed.");
}


// ------------------------------------------------------
string PlacePropertySaver::parseOption(
    const string name, 
    const string defaultValue,
    const map<string,string> &config)
{
  map<string, string>::const_iterator it = config.find(name);

  if (it != config.end())
    return it->second;

  return defaultValue; // default if not given
}


// ------------------------------------------------------
bool PlacePropertySaver::parseFlagOption(
    const string name, 
    const map<string,string> &config)
{
  map<string, string>::const_iterator it = config.find(name);

  if (it != config.end()) 
  {
    if (it->second == "true")
    {
      return true;
    }
    else if (it->second == "false")
    {
      return false;
    }
    else
    {
      error("Invalid value '%s' for flag option '%s'. "
            "Should be one of {'true', 'false'}", 
            it->second.c_str(), name.c_str());

      return false; // default if malformed
    }
  }

  return false; // default if not given
}


// ------------------------------------------------------
template<class T>
T PlacePropertySaver::parseOptionLexicalCast(
    const std::string name,
    const T defaultValue,
    const std::map<std::string,std::string> &config)
{
  map<string, string>::const_iterator it = config.find(name);
  
  if (it != config.end())
  {
    try
    {
      return boost::lexical_cast<T>(it->second);
    }
    catch(boost::bad_lexical_cast &)
    {
      error("Value '%s' of option '%s' is not of type '%s'", 
            it->second.c_str(), name.c_str(), typeid(T).name());

      return defaultValue; // default if malformed
    }
  }

  return defaultValue; // default if not given
}


// ------------------------------------------------------
string PlacePropertySaver::parsePathOption(
    const string name, 
    const string defaultValue,
    const map<string,string> &config)
{
  string tmp = parseOption("--save-file-name", defaultValue, config);
  shared_array<char> path(realpath(tmp.c_str(), 0));
  if (path)
  {
    return path.get();
  }
  else
  {
    error("Value '%s' of option '%s' could not be resolved as a valid path",
          tmp.c_str(), name.c_str());
    return "";
  }
}


// ------------------------------------------------------
void PlacePropertySaver::configure(const map<string,string> &config)
{

  _doSave = parseFlagOption("--save", config);

  if (_doSave)
  {
    // parse saving related options if we actually want to save

    _saveFileName =
        parsePathOption("--save-file-name", "place_properties.bin", config);

    _saveInterval = 
        parseOptionLexicalCast<unsigned int>("--save-interval", 1000, config);

    _saveContinuously = 
        parseFlagOption("--save-continuously", config);
  }


  _doLoad = parseFlagOption("--load", config);

  if (_doLoad)
  {
    // parse loading related options if we actually want to load

    _loadFileName = 
        parsePathOption("--load-file-name" , "place_properties.bin", config);

    _waitBeforeLoading =
        parseOptionLexicalCast<unsigned int>(
            "--wait-before-loading", 3000, config);
    
    _waitBetweenLoading =
        parseOptionLexicalCast<unsigned int>(
            "--wait-between-loading", 1, config);
  }

  log("Configuration:");
  log("-> saving: %s", (_doSave ? "true" : "false"));
  log("-> save file name: %s", _saveFileName.c_str());
  log("-> save interval: %dms", _saveInterval);
  log("-> continuous saving: %s", (_saveContinuously ? "true" : "false"));
  log("-> loading: %s", (_doLoad ? "true" : "false"));
  log("-> load file name: %s", _loadFileName.c_str());
  log("-> wait before loading: %dms", _waitBeforeLoading);
  log("-> wait between loading: %dms", _waitBetweenLoading);

  debug("Configured.");
  
}


// ------------------------------------------------------
void PlacePropertySaver::start()
{
  debug("Started.");
}


// ------------------------------------------------------
void PlacePropertySaver::stop()
{
  if(_doSave)
  {
    // Save once (more) at the end
    // FIXME: Make sure this is does not interfere with the cast shutdown

    debug("Component stopped running. Save one more time.");

    savePlaceProperties();
  }

  debug("Stopped.");
}


// ------------------------------------------------------
void PlacePropertySaver::runComponent()
{

  // If enabled, first load existing place properties from disk, then, if
  // enabled, start saving properties continuously to disk.

  if (_doLoad)
  {
    loadPlaceProperties();
  }

  if (_doSave)
  {      
    if (_saveContinuously) 
    {
      // Sleep at most 100ms at a time to be responsive to "stop" events. We
      // will round up _saveInterval to 100ms precision.
      unsigned int sleeptime = 100;
      unsigned int sleepiterations = _saveInterval / sleeptime;
      if (sleepiterations % sleeptime != 0) 
        ++sleepiterations; // make sure we round up

      unsigned int count = 0;
      while (isRunning())
      {
        sleepComponent(sleeptime);
        ++count;
        if (count >= sleepiterations)
        {
          count = 0;
          if (isRunning())
            savePlaceProperties();
        }
      } 
    }
    else
    {
      // Do nothing. We save once in the "stop" method.
    }
  }

}


// ------------------------------------------------------
void PlacePropertySaver::savePlaceProperties()
{

  ptime start_t(microsec_clock::local_time());

  // Map of subarchitecture name to a vector of place properties (or other
  // related WM entries like ObjectSearchResult).
  map<string, vector<Ice::ObjectPtr> > output;
  int count = 0;
  
  string subarch;


  // --- CATEGORICAL ---
  subarch = "categorical.sa";
  debug("Looking in %s.", subarch.c_str());

  // shape
  {
    typedef SpatialProperties::RoomShapePlaceProperty prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d shape place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // size
  {
    typedef SpatialProperties::RoomSizePlaceProperty prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d size place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // appearance
  {
    typedef SpatialProperties::RoomAppearancePlaceProperty prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d appearance place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }


  // --- SPATIAL ---
  subarch = "spatial.sa";
  debug("Looking in %s.", subarch.c_str());

  // human assertions
  {
    typedef SpatialProperties::RoomHumanAssertionPlaceProperty prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d human assertions place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // object search result
  {
    typedef SpatialData::ObjectSearchResult prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d object search results.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // object place property
  {
    typedef SpatialProperties::ObjectPlaceProperty prop_t;
    typedef shared_ptr<CASTData<prop_t> > data_ptr_t;

    vector<data_ptr_t> result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d object place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const data_ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }


  debug("Saving %d WM-entries in total.", count);
  
  
  // FIXME @demmeln 22.03.2012: Do we need to filter out inferred place
  // properties? Apparently it does not hurt just save / load them anyway, but
  // this has the potential to bit us in the future.

  
  Ice::OutputStreamPtr os = Ice::createOutputStream(getCommunicator());

  os->write(output);

  vector<Ice::Byte> data;
  os->writePendingObjects(); // @demmeln 22.03.2012: is this needed? Ice
                             // Documentation is not shedding light on this.
                             // @demmeln 24.03.2012: I guess this is necessary
                             // alter all. It seems this does something like
                             // save objects that Handles point to. This is
                             // related to loadPendingObjects().
  os->finished(data);
  
  ofstream fs(_saveFileName.c_str());
  if(!fs) 
  {
    error("Could not open file '%d' while saving place properties.",
          _saveFileName.c_str());
    return;
  }

  debug("Writing to file '%s'.", _saveFileName.c_str());

  fs.write(reinterpret_cast<char *>(data.data()), data.size()); 
  
  fs.close();

  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Done saving %d objects. Time taken: %s.", count, duration.c_str());

}


// ------------------------------------------------------
void PlacePropertySaver::loadPlaceProperties()
{
  
  debug("Waiting %dms before loading place properties", _waitBeforeLoading);
  
  sleepComponent(_waitBeforeLoading);

  ptime start_t(microsec_clock::local_time());

  debug("Loading place properties from file '%s'.", _loadFileName.c_str());

  ifstream fs(_loadFileName.c_str());
  if(!fs)
  {
    error("Could not open file '%d' while loading place properties.",
          _loadFileName.c_str());
    return;
  }

  vector<Ice::Byte> data((istreambuf_iterator<char>(fs)), 
                         istreambuf_iterator<char>());

  fs.close();

  Ice::InputStreamPtr is = Ice::createInputStream(getCommunicator(), data);
  typedef map<string, vector<Ice::ObjectPtr> > input_t;
  input_t input;

  try
  {
    is->read(input);
    is->readPendingObjects();
  }
  catch (Ice::UnmarshalOutOfBoundsException &)
  {
    error("Error while parsing file '%s' for loading saved place properties.",
          _loadFileName.c_str());
    return;
  }

  int count = 0;
  
  BOOST_FOREACH(input_t::value_type value, input)
  {
    const string &subarch = value.first;
    const vector<Ice::ObjectPtr> &objects = value.second;

    debug("Loading %d objects into wm '%s'.", objects.size(), subarch.c_str());

    count += objects.size();

    BOOST_FOREACH(const Ice::ObjectPtr &o, objects)
    {
      if(_waitBetweenLoading > 0)
        sleepComponent(_waitBetweenLoading);

      debug("Adding loaded property of type '%s' to working memory.",
            typeid(*o).name());
      
      // FIXME: do this with functionality provided by upstream CAST
      addToWorkingMemoryDynamicType(newDataID(), subarch, o);
    }
  }


  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Done loading %d objects. Time taken: %s.", count, duration.c_str());

}

