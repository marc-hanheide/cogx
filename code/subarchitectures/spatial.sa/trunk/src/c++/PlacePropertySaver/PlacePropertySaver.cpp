/**
 * Author: Nikolaus Demmel
 * Date:  23.03.2012
 **/

#include "PlacePropertySaver.h"
#include <SpatialProperties.hpp>

#include <Ice/Initialize.h>

#include <fstream> 
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace boost;
using namespace cast;
using boost::posix_time::ptime;
using boost::posix_time::microsec_clock;
using boost::posix_time::to_simple_string;


// Some type definitions
template <typename T>
struct Data {
  typedef shared_ptr<CASTData<T> > ptr_t;
  typedef vector<ptr_t> ptrs_t;
};
typedef vector<Ice::ObjectPtr> IceObjectPtrs;


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
void PlacePropertySaver::configure(const map<string,string> &config)
{
  map<string, string>::const_iterator it;

  _saveFileName = "place_properties.bin";
  it = config.find("--save-file-name");
  if (it != config.end())
    _saveFileName = it->second;

  _loadFileName = "place_properties.bin";
  it = config.find("--load-file-name");
  if (it != config.end())
    _loadFileName = it->second;

  _saveInterval = 1000;
  it = config.find("--save-interval");
  if (it != config.end())
  {
    try
    {
      _saveInterval = boost::lexical_cast<unsigned long>(it->second);
    }
    catch(boost::bad_lexical_cast &)
    {
      error("Value '%s' of argument '--save-interval' is "
            "not of type 'unsigned int'", it->second.c_str());
    }
  }

  _saveContinuously = false;
  it = config.find("--save-continuously");
  if (it != config.end()) {
    if (it->second == "true")
      _saveContinuously = true;
    else if (it->second == "false")
      _saveContinuously = false;
    else
      error("Invalid value '%s' for argument '--save-coninuously'. "
            "Should be one of {'true', 'false'}", it->second.c_str());
  }

  _doSave = false;
  it = config.find("--save");
  if (it != config.end()) {
    if (it->second == "true")
      _doSave = true;
    else if (it->second == "false")
      _doSave = false;
    else
      error("Invalid value '%s' for argument '--save'. "
            "Should be one of {'true', 'false'}", it->second.c_str());
  }

  _doLoad = false;
  it = config.find("--load");
  if (it != config.end()) {
    if (it->second == "true")
      _doLoad = true;
    else if (it->second == "false")
      _doLoad = false;
    else
      error("Invalid value '%s' for argument '--load'. "
            "Should be one of {'true', 'false'}", it->second.c_str());
  }

  log("Configuration:");
  log("-> saving: %s", (_doSave ? "true" : "false"));
  log("-> loading: %s",(_doLoad ? "true" : "false"));
  log("-> save file name: %s", _saveFileName.c_str());
  log("-> load file name: %s", _loadFileName.c_str());
  log("-> save interval: %dms", _saveInterval);
  log("-> continuous saving: %s", (_saveContinuously ? "true" : "false"));

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
  debug("Stopped.");
}


// ------------------------------------------------------
void PlacePropertySaver::runComponent()
{

  if (_doLoad)
  {
    loadPlaceProperties();
  }

  if (_doSave)
  {
    while (isRunning())
    {
      if (_saveContinuously) 
      {
        sleepComponent(_saveInterval);
        savePlaceProperties();
      } 
      else 
      {
        // just wait until the component is stopped
        sleepComponent(100);
      } 
    }

    debug("Component stopped running. Save one more time.");

    // Save once (more) at the end
    // FIXME: Make sure this is does not interfere with the cast shutdown
    savePlaceProperties();
  }

}


// ------------------------------------------------------
void PlacePropertySaver::savePlaceProperties()
{
  // TODO: save stuff from categorical and dont forget about search result
  // TODO: check out workingmemory display for XML backend

  ptime start_t(microsec_clock::local_time());

  // Map of subarchitecture name to a vector of outputs of
  // getWorkingMemoryEntries(...)
  map<string, IceObjectPtrs> output;
  int count = 0;
  
  string subarch;


  // --- CATEGORICAL ---
  subarch = "categorical.sa";
  debug("Looking in %s.", subarch.c_str());

  // shape
  {
    typedef SpatialProperties::RoomShapePlaceProperty prop_t;
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d shape place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // size
  {
    typedef SpatialProperties::RoomSizePlaceProperty prop_t;
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d size place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // appearance
  {
    typedef SpatialProperties::RoomAppearancePlaceProperty prop_t;
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d appearance place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
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
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d human assertions place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // object search result
  {
    typedef SpatialData::ObjectSearchResult prop_t;
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d object search results.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }

  // object place property
  {
    typedef SpatialProperties::ObjectPlaceProperty prop_t;
    Data<prop_t>::ptrs_t result;
    getWorkingMemoryEntries<prop_t>(subarch, 0, result);

    debug("Found %d object place properties.", result.size());
    count += result.size();

    BOOST_FOREACH(const Data<prop_t>::ptr_t &p, result)
    {
      output[subarch].push_back(p->getData());
    }
  }


  debug("Saving %d WM-entries in total.", count);
  
  
  // FIXME @demmeln 22.03.2012: Do we need to filter out inferred place
  // properties? Apparently it does not hurt just save / load them anyway.

  
  Ice::OutputStreamPtr os = Ice::createOutputStream(getCommunicator());

  os->write(output);

  vector<Ice::Byte> data;
  os->writePendingObjects(); // @demmeln 22.03.2012: is this needed?
  os->finished(data);
  
  ofstream fs(_saveFileName.c_str());
  if(!fs) {
    error("Could not open file '%d'.", _saveFileName.c_str());
    return;
  }

  debug("Writing to file '%s'.", _saveFileName.c_str());

  fs.write(reinterpret_cast<char *>(data.data()), data.size()); 
  
  fs.close();

  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Done saving %d objects. Time taken: %s.", count, duration.c_str());

}
