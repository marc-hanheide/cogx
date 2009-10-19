/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * A component the filters out persistent ProtoObjects.
 */

#ifndef OBJECT_ANALYZER_H
#define OBJECT_ANALYZER_H

#define UPD_THR_DEFAULT 5

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <../../VisionUtils.h>

#include <VisionData.hpp>


namespace cast
{

class ObjectAnalyzer : public ManagedComponent
{
  private:

	/**
	 * Time and update thresholds
	 *(part of the ROI persistency criteria)
	 */
	int updateThr;
	bool doDisplay;

	/**
	 * status of ProtoObject persistency
	 */
	enum ProtoObjectStatus {
	  STABLE,
	  DELETED };

	/** 
	 * ProtoObject data, contains also data used to evaluate ProtoObject persistency
	 */	
	struct ProtoObjectData {
	  cdl::WorkingMemoryAddress addr;
	  ProtoObjectStatus status;
	  std::string visualObjId;
	  cdl::CASTTime addedTime;
	  cdl::CASTTime lastUpdateTime;
	  cdl::CASTTime deleteTime;
	};

	std::map<std::string, ProtoObjectData> ProtoObjectMap;

	std::queue<std::string> objToAdd;
	std::queue<std::string> objToDelete;

	boost::interprocess::named_semaphore* queuesNotEmpty;

	/**
	 * callback function called whenever a new ProtoObject appears
	 */
	void newProtoObject(const cdl::WorkingMemoryChange & _wmc);

	/**
	 * callback function called whenever a ProtoObject changes
	 */
	void updatedProtoObject(const cdl::WorkingMemoryChange & _wmc);

	/**
	 * callback function called whenever a ProtoObject is deleted
	 */
	void deletedProtoObject(const cdl::WorkingMemoryChange & _wmc);

	/**
	 * Recognize object attrubutes using VisualLearner
	 */
	void start_VL_RecognitionTask(const VisionData::ProtoObjectPtr& pproto, const cdl::WorkingMemoryAddress &addr);
	void onChangeRecognitionTask(const cdl::WorkingMemoryChange & _wmc);

  protected:
	/**
	 * called by the framework to configure our component
	 */
	virtual void configure(const std::map<std::string,std::string> & _config);
	/**
	 * called by the framework after configuration, before run loop
	 */
	virtual void start();
	/**
	 * called by the framework to start compnent run loop
	 */
	virtual void runComponent();

  public:
	virtual ~ObjectAnalyzer() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim */
