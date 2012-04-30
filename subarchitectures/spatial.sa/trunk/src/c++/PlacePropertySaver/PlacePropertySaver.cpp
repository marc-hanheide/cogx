/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   23.03.2012
 **/

#include "PlacePropertySaver.h"
#include <SpatialProperties.hpp>

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Ice/Initialize.h>
#include <Ice/LocalException.h>

#include <fstream> 
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

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

	_mapLoadStatusOk = false;
}


// ------------------------------------------------------
PlacePropertySaver::~PlacePropertySaver()
{
  debug("Destroyed.");
}


// ------------------------------------------------------
void PlacePropertySaver::configure(const map<string,string> &config)
{
  
  setConfig(config);

  _doSave = parseOptionFlag("--save");

  if (_doSave)
  {
    // parse saving related options if we actually want to save

    _saveFileName = parseOptionPath("--save-file-name", "place_properties.bin");

    _saveInterval = 
        parseOptionLexicalCast<unsigned int>("--save-interval", 1000);

    _saveContinuously = parseOptionFlag("--save-continuously");
  }


  _doLoad = parseOptionFlag("--load");

  if (_doLoad)
  {
    // parse loading related options if we actually want to load

    _loadFileName = parseOptionPath("--load-file-name", "place_properties.bin");

    _waitBeforeLoading =
        parseOptionLexicalCast<unsigned int>("--wait-before-loading", 3000);
    
    _waitBetweenLoading =
        parseOptionLexicalCast<unsigned int>("--wait-between-loading", 1);
  }

  _waitForMapLoadStatus = parseOptionFlag("--wait-for-map-load-status");

  log("Configuration:");
  log("-> saving: %s", (_doSave ? "true" : "false"));
  log("-> save file name: %s", _saveFileName.c_str());
  log("-> save interval: %dms", _saveInterval);
  log("-> continuous saving: %s", (_saveContinuously ? "true" : "false"));
  log("-> loading: %s", (_doLoad ? "true" : "false"));
  log("-> load file name: %s", _loadFileName.c_str());
  log("-> wait before loading: %dms", _waitBeforeLoading);
  log("-> wait between loading: %dms", _waitBetweenLoading);
  log("-> wait for map load status: %s", 
      (_waitForMapLoadStatus ? "true" : "false"));

  debug("Configured.");
  
}


// ------------------------------------------------------
void PlacePropertySaver::start()
{
  addChangeFilter(
      createGlobalTypeFilter<SpatialData::MapLoadStatus>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlacePropertySaver>(
          this, &PlacePropertySaver::mapLoadStatusAdded));
  addChangeFilter(
      createGlobalTypeFilter<SpatialData::MapLoadStatus>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlacePropertySaver>(
          this, &PlacePropertySaver::mapLoadStatusOverwritten));
  addChangeFilter(
      createGlobalTypeFilter<SpatialData::MapLoadStatus>(cdl::DELETE),
      new MemberFunctionChangeReceiver<PlacePropertySaver>(
          this, &PlacePropertySaver::mapLoadStatusDeleted));

  debug("Started.");
}


// ------------------------------------------------------
void PlacePropertySaver::stop()
{
  if(_doSave)
  {
    // Save once (more) at the end

    debug("Component stopped running. Save one more time.");

    savePlaceProperties();
  }

  debug("Stopped.");
}


// ------------------------------------------------------
void PlacePropertySaver::runComponent()
{
  debug("Running.");

  if (!boost::filesystem::exists(_loadFileName))
  {
    log("Did not find file '%s' that was configured to load the place "
        "properties from. Will neither load nor update map load status.",
        _loadFileName.c_str());
    _waitForMapLoadStatus = false;
    _doLoad = false;
  }

  // 1) If enabled, wait for map status.
  if (_waitForMapLoadStatus && isRunning())
  {
    waitForMapLoadStatus();
  }


  // 2) If enabled, load existing place properties from disk.
  if (_doLoad && isRunning())
  {
    loadPlaceProperties();
  }

  if (isRunning())
  {
    // update map load status even if loading is disabled to that map loading
    // procedure does not get stuck even if we don't load place properties.
    updateWMMapLoadStatus();
  }

  // 3) If enabled, start saving properties continuously to disk.
  if (_doSave && isRunning())
  {      
    if (_saveContinuously) 
    {
      while (isRunning())
      {
        if (!sleepComponentResponsive(_saveInterval))
          return;
        savePlaceProperties();
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


  try
  {

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
  }
  catch(CASTException &e)
  {
    log("Caught exception at %s. Message: %s", __HERE__, e.what());
  }

  debug("Saving %d WM-entries in total.", count);
  
  

  Ice::OutputStreamPtr os;
  vector<Ice::Byte> data;

  try
  {
    os = Ice::createOutputStream(getCommunicator());

    os->write(output);

    os->writePendingObjects(); /* @demmeln 22.03.2012: is this needed? Ice
                                * Documentation is not shedding light on this.
                                * @demmeln 24.03.2012: I guess this is necessary
                                * alter all. It seems this does something like
                                * save objects that Handles point to. This is
                                * related to loadPendingObjects(). */
    os->finished(data);
  }
  catch (Ice::MarshalException &e)
  {
    error("Marshal error while saving place properties: %s", e.what());
    return;
  }
  catch (std::exception &e)
  {
    error("General error while saving place properties: %s", e.what());
    return;
  }


  try
  {
    ofstream fs(_saveFileName.c_str(), ios::binary | ios::out);
    if(!fs) 
    {
      error("Could not open file '%d' while saving place properties.",
            _saveFileName.c_str());
      return;
    }

    debug("Writing to file '%s'.", _saveFileName.c_str());

  // NOTE @demmeln 26.03.2012: What I want is simply writing the
  // vector<Ice::Byte>, which at least on my machine is the same as
  // vector<unsigned char>, and write it to a file and then later read it back
  // unchanged. I hope reinterpret cast is ok here. C++ experts are welcome to
  // confirm and remove this lengthy comment. It says the following at [1]:
  //
  // "5. Any pointer to object of type T1 can be converted to pointer to object
  // of another type T2. If T2's alignment is not stricter than T1's, conversion
  // of the resulting pointer back to its original type yields the original
  // value, otherwise the resulting pointer cannot be dereferenced safely,
  // except where allowed by the type aliasing rules (see below)"
  //
  // I would guess that char and unsigned char are guaranteed to have the same
  // alignment (although I can't find a reference for that right now). In any
  // case [1] goes on to say:
  //
  // "When a pointer or reference to object of type T1 is reintrepret_cast (or
  // C-style cast) to a pointer or reference to object of a different type T2,
  // the cast always succeeds, but the resulting pointer or reference may only
  // be accessed if one of the following is true:
  //
  // [...]
  // 
  // * T2 is char or unsigned char"
  //
  // Judging from that writing and reading a usinged char array to and from an
  // fstream is always ok. See also the fs.read(...) call in
  // readPlaceProperties().
  // 
  // [1] http://en.cppreference.com/w/cpp/language/reinterpret_cast
    
    fs.write(reinterpret_cast<char *>(data.data()), data.size()); 
  
    fs.close();
  }
  catch (std::exception &e)
  {
    error("Exception while saving data to file: %s", e.what());
  }

  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Done saving %d objects. Time taken: %s.", count, duration.c_str());

}


// ------------------------------------------------------
void PlacePropertySaver::loadPlaceProperties()
{
  
  debug("Waiting %dms before loading place properties", _waitBeforeLoading);
  
  if (_waitBeforeLoading > 0)
    if (!sleepComponentResponsive(_waitBeforeLoading))
      return;

  ptime start_t(microsec_clock::local_time());

  debug("Loading place properties from file '%s'.", _loadFileName.c_str());


  vector<Ice::Byte> data;

  try
  {
    ifstream fs(_loadFileName.c_str(), ios::binary | ios::in);
    if(!fs)
    {
      error("Could not open file '%d' while loading place properties.",
            _loadFileName.c_str());
      return;
    }



  // Read data from file.
  //
  // NOTE @demmeln 26.03.2012: I had a little trouble making sure that the type
  // conversions writing/reading to from a file are kosher and platform
  // independet. My initial implementation was the following, which seemed to
  // work fine and does not require an reinterpret_cast:
  // 
  // vector<Ice::Byte> data((istreambuf_iterator<char>(fs)), 
  //                        istreambuf_iterator<char>());
  //
  // Not sure if I would need a "fs >> noskipws" to be sure it will not skrew up
  // something, which is suggested by this alternative [1]:
  //
  // fs >> std::noskipws;
  // std::copy((istreambuf_iterator<char>(fs)), 
  //           istreambuf_iterator<char>()
  //           std::back_inserter(data));
  //
  // The above method could be augmented with a call to data.reserve().
  //
  // After some investigation I would assume that the following implemention is
  // potentiall more efficient. Also it mirrows the way I write the file. See
  // the lengthy comment before fs.write(...) in savePlaceProperties. Comments
  // by C++ experts to shed some light on the correctness and performance are
  // very welcome.
  //
  // [1] http://stackoverflow.com/a/7690162

    // get length of file:
    fs.seekg (0, ios::end);
    int file_length = fs.tellg();
    fs.seekg (0, ios::beg);

    // read the data
    data.resize(file_length);
    fs.read(reinterpret_cast<char *>(data.data()), file_length);

    fs.close();
  }
  catch (std::exception &e)
  {
    error("Exception while saving data to file: %s", e.what());
  }



  Ice::InputStreamPtr is = Ice::createInputStream(getCommunicator(), data);
  typedef map<string, vector<Ice::ObjectPtr> > input_t;
  input_t input;


  try
  {
    is->read(input);
    is->readPendingObjects();
  }
  catch (Ice::MarshalException &e)
  {
    error("Marshal error while parsing file '%s' for loading saved place "
          "properties: %s", _loadFileName.c_str(), e.what());
    return;
  }
  catch (std::exception &e)
  {
    error("General error while parsing file '%s' for loading saved place "
          "properties: %s", _loadFileName.c_str(), e.what());
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
        if(!sleepComponentResponsive(_waitBetweenLoading))
          return;

      debug("Adding loaded property of type '%s' to working memory.",
            o->ice_id().c_str());
      
      // FIXME: do this with functionality provided by upstream CAST
      addToWorkingMemoryDynamicType(newDataID(), subarch, o);
    }
  }


  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Done loading %d objects. Time taken: %s.", count, duration.c_str());

}


// ------------------------------------------------------
void PlacePropertySaver::waitForMapLoadStatus()
{
  // TODO: Maybe use mutex / condition signal instead of bool flag / polling

  ptime start_t(microsec_clock::local_time());

  int count = 0;

  while(isRunning() && !_mapLoadStatusOk)
  {
    sleepComponent(100);
    ++count;
    if (count % 100 == 0)
    {
      error("Waited another 10s for map load status.");
    }
  }

  ptime end_t(microsec_clock::local_time());

  string duration = to_simple_string(end_t - start_t);

  log("Time waited for MapLoadStatus: %s.", duration.c_str());

}


// ------------------------------------------------------
bool PlacePropertySaver::checkWMMapLoadStatus()
{

  try {
    SpatialData::MapLoadStatusPtr statusStruct = 
        getMemoryEntry<SpatialData::MapLoadStatus>(_mapLoadStatusAddress);
    return statusStruct->placesWritten;
  }
  catch (DoesNotExistOnWMException) {
    getLogger()->warn("MapLoadStatus struct disappeared from WM! "
                      "Map loading procedure may get stuck.");
  }
  catch(CASTException &e)
  {
    log("Caught exception at %s. Message: %s", __HERE__, e.what());
  }

  return false;

}


// ------------------------------------------------------
void PlacePropertySaver::mapLoadStatusAdded(const cdl::WorkingMemoryChange &wmc)
{
  debug("MapLoadStatus added to working memory");

  // Should be added only once. If added again, overwrite, but signal an error.

  // If the map status indicates that places have been loaded, we set
  // _mapLoadStatusOk to true to indicate to the main thread that it is now ok
  // to load the place properties

  if (_mapLoadStatusAddress.id != "")
    error("Map load status was added multiple times to the WM!");

  _mapLoadStatusAddress = wmc.address;

  if (checkWMMapLoadStatus())
    _mapLoadStatusOk = true;

}


// ------------------------------------------------------
void PlacePropertySaver::mapLoadStatusOverwritten(const cdl::WorkingMemoryChange &wmc)
{
  debug("MapLoadStatus overwritten in working memory");

  // If the map status indicates that places have been loaded, we set
  // _mapLoadStatusOk to true to indicate to the main thread that it is now ok
  // to load the place properties

  if (_mapLoadStatusAddress != wmc.address)
  {
    error("Getting overwrite for different MapLoadStatus. "
          "There should only be one.");
    return;
  }

  if (checkWMMapLoadStatus())
    _mapLoadStatusOk = true;

}


// ------------------------------------------------------
void PlacePropertySaver::mapLoadStatusDeleted(const cdl::WorkingMemoryChange &wmc)
{
  debug("MapLoadStatus deleted from working memory");

  if (wmc.address == _mapLoadStatusAddress)
    _mapLoadStatusAddress = cast::cdl::WorkingMemoryAddress(); // reset

}


// ------------------------------------------------------
void PlacePropertySaver::updateWMMapLoadStatus()
{

  if (_mapLoadStatusAddress.id == "")
  {
    log("Wanted to update map load status, but it is not in WM.");
    return;
  }
  else
  {
    try
    {
      lockEntry(_mapLoadStatusAddress, cdl::LOCKEDOD);
      SpatialData::MapLoadStatusPtr statusStruct = 
          getMemoryEntry<SpatialData::MapLoadStatus>(_mapLoadStatusAddress);
      statusStruct->categoryDataWritten = true;
      overwriteWorkingMemory<SpatialData::MapLoadStatus>
          (_mapLoadStatusAddress, statusStruct);
      unlockEntry(_mapLoadStatusAddress);
    }
    catch (DoesNotExistOnWMException) 
    {
      getLogger()->warn("MapLoadStatus struct disappeared from WM! "
                        "Map loading procedure may get stuck.");
    }
    catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.what());
		}

  } 
}


// ------------------------------------------------------
bool PlacePropertySaver::sleepComponentResponsive(unsigned long millis)
{
  // how much time do we want to sleep at most between checking for status
  const unsigned long sleeptime = 100;

  // If we would want to do it properly, we would clock the actual time the
  // sleeping has been going on, since the intermediate wakeups also take some
  // time. However we only wake up every 'sleeptime' milliseconds and it is not
  // vital that the total sleep time is very acurate.

  unsigned int sleepiterations = millis / sleeptime;
  unsigned int extra = millis % sleeptime;

  unsigned int count = 0;
  while (isRunning())
  {
    if (count == sleepiterations)
    {
      sleepComponent(extra);
      break;
    }
    sleepComponent(sleeptime);
    ++count;
  } 

  return isRunning();
}
