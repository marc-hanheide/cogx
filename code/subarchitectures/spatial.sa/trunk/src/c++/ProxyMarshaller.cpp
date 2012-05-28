#include "ProxyMarshaller.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <BinderEssentials.hpp>

using namespace std;
using namespace boost;
using namespace cast;
using namespace spatial;
using namespace Ice;
using namespace binder::autogen::core;
using namespace binder::autogen::featvalues;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new ProxyMarshaller();
}
}

ProxyMarshaller::ProxyMarshaller() {
}

ProxyMarshaller::~ProxyMarshaller() {
}

void ProxyMarshaller::start() {
	addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cast::cdl::ADD),
			new cast::MemberFunctionChangeReceiver<ProxyMarshaller>(this,
					&ProxyMarshaller::newPlace));

	addChangeFilter(createLocalTypeFilter<SpatialData::Place> (
			cast::cdl::OVERWRITE), new cast::MemberFunctionChangeReceiver<
			ProxyMarshaller>(this, &ProxyMarshaller::changedPlace));

	addChangeFilter(
			createLocalTypeFilter<SpatialData::Place> (cast::cdl::DELETE),
			new cast::MemberFunctionChangeReceiver<ProxyMarshaller>(this,
					&ProxyMarshaller::deletedPlace));
}

void ProxyMarshaller::stop() {
}

void ProxyMarshaller::runComponent() {
}

void ProxyMarshaller::configure(
		const std::map<std::string, std::string>& _config) {
	log("Configure entered");
	BindingWorkingMemoryWriter::configure(_config);

	Marshalling::MarshallerPtr servant = new MarshallingServer(this);
	registerIceServer<Marshalling::Marshaller, Marshalling::Marshaller> (servant);
}

void ProxyMarshaller::MarshallingServer::addProxy(const string & type,
		const string & UID, double probExists,
		const cast::cdl::WorkingMemoryPointerPtr & origin,
		const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->addProxy(type, UID, probExists, origin);
	m_pOwner->unlockComponent();
}

bool ProxyMarshaller::MarshallingServer::addRelation(
		const string & relationType, const string & relationUID,
		const string & sourceType, const string &sourceUID,
		const string & targetType, const string &targetUID, double probExists,
		const cast::cdl::WorkingMemoryPointerPtr & origin,
		const Ice::Current &_context) {
	ProxyMarshaller::RelationCandidate cand(relationType, relationUID,
			sourceType, sourceUID, targetType, targetUID, probExists, origin);

	m_pOwner->lockComponent();
	int status = m_pOwner->addRelation(cand);
	m_pOwner->unlockComponent();

	if (status) {
		return false;
	} else {
		return true;
	}
}

void ProxyMarshaller::addProxy(const string & type, const string & UID,
		double probExists, const cast::cdl::WorkingMemoryPointerPtr & origin) {
	map<string, InternalProxy> &typeMap = m_proxyTypeMap[type];

	map<string, InternalProxy>::iterator it = typeMap.find(UID);

	if (it == typeMap.end()) {
		// create new proxy
		log("creating a new proxy.");

		typeMap[UID].proxy = createNewProxy(origin, probExists);
		typeMap[UID].isRelation = false;
		//typeMap[UID].proxy->entityID = newDataID();
		//typeMap[UID].proxy->subarchId = getSubarchitectureID();
		//typeMap[UID].proxy->probExists = probExists;
		//typeMap[UID].proxy->distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);
		typeMap[UID].onBinder = false; //Not represented on binder yet

		//updateInternalProxy(typeMap[UID]); //Don't upload it until some features are added

		// Check all previously failed relations to see if they work now

		for (list<RelationCandidate>::iterator rIt = m_queuedRelations.begin(); rIt
				!= m_queuedRelations.end();) {
			// Try adding the pending relation
			if (addRelation(*rIt) == 0) {
				// Add all the Features pending for this Relation
				//FIXME: It won't work without this - but it crashes with it... no clue why
				//for(vector<binder::autogen::core::FeaturePtr>::iterator fIt =
				//rIt->addedFeatures.begin(); fIt != rIt->addedFeatures.end(); fIt++) {
				//addFeature(rIt->relationType, rIt->relationUID, *fIt);
				//}
				//commitFeatures(rIt->relationType, rIt->relationUID);

				// Erase the candidate from the pending list
				rIt = m_queuedRelations.erase(rIt);
			} else {
				rIt++;
			}
		}
	} else {
		// The proxy already existed.
		log("Warning: Proxy already existed creating normal proxy!");
	}
}

int ProxyMarshaller::addRelation(const RelationCandidate &cand) {
	const string & relationType = cand.relationType;
	const string & relationUID = cand.relationUID;
	const string & sourceType = cand.sourceType;
	const string &sourceUID = cand.sourceUID;
	const string & targetType = cand.targetType;
	const string &targetUID = cand.targetUID;
	double probExists = cand.probExists;
	const cast::cdl::WorkingMemoryPointerPtr origin = cand.origin;

	log("addRelation: %s, %s, %s, %s, %s, %s", relationType.c_str(),
			relationUID.c_str(), sourceType.c_str(), sourceUID.c_str(),
			targetType.c_str(), targetUID.c_str());

	map<string, InternalProxy> &relTypeMap = m_proxyTypeMap[relationType];

	map<string, InternalProxy>::iterator it = relTypeMap.find(relationUID);

	if (it == relTypeMap.end()) {
		// No such relation exists; potentially create a new one
		// First, check that the source and target proxies exist, though.
		map<string, InternalProxy> &sourceTypeMap = m_proxyTypeMap[sourceType];

		map<string, InternalProxy>::iterator sourceIt = sourceTypeMap.find(
				sourceUID);

		if (sourceIt == sourceTypeMap.end()) {
			log("Could not create relation proxy; source didn't exist! Queuing...");

			return -1;
		} else {
			map<string, InternalProxy> &targetTypeMap = m_proxyTypeMap[targetType];

			map<string, InternalProxy>::iterator targetIt = targetTypeMap.find(
					targetUID);
			if (targetIt == targetTypeMap.end()) {
				log("Could not create relation proxy; target didn't exist! Queuing...");

				return -1;
			} else {
				// create new proxy
				log("creating a new relation proxy.");

				FeatureValues sources;
				AddressValuePtr source = new AddressValue(1, getCASTTime(),
						sourceIt->second.proxy->entityID);
				sources.push_back(source);

				FeatureValues targets;
				AddressValuePtr target = new AddressValue(1, getCASTTime(),
						targetIt->second.proxy->entityID);
				targets.push_back(target);

				relTypeMap[relationUID].proxy = createNewRelationProxy(origin,
						probExists, sources, targets);
				relTypeMap[relationUID].isRelation = true;
				//relTypeMap[relationUID].proxy->entityID = newDataID();
				//relTypeMap[relationUID].proxy->subarchId = getSubarchitectureID();
				//relTypeMap[relationUID].proxy->probExists = probExists;
				//relTypeMap[relationUID].proxy->distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(_newPlaceProxy);
				relTypeMap[relationUID].onBinder = false; //Not represented on binder yet

				//updateInternalProxy(relTypeMap[relationUID]); //Don't upload it until some features are added
			}
		}
	} else {
		// The proxy already existed.
		log("Warning: Proxy already existed creating relation proxy!");
	}
	return 0;
}

void ProxyMarshaller::MarshallingServer::deleteProxy(const string & type,
		const string & UID, const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->deleteProxy(type, UID);
	m_pOwner->unlockComponent();
}

void ProxyMarshaller::deleteProxy(const string & type, const string & UID) {
	//Check pending relations; delete from there if applicable
	for (list<RelationCandidate>::iterator rIt = m_queuedRelations.begin(); rIt
			!= m_queuedRelations.end();) {
		if (rIt->relationType == type && rIt->relationUID == UID) {
			m_queuedRelations.erase(rIt);
			return;
		} else {
			rIt++;
		}
	}

	//TODO: Make sure connected relations are not left on the Binder!
	if (m_proxyTypeMap.find(type) != m_proxyTypeMap.end()) {
		map<string, InternalProxy> &typeMap = m_proxyTypeMap[type];
		if (typeMap.find(UID) != typeMap.end()) {
			InternalProxy &intProxy = typeMap[UID];
			if (intProxy.onBinder) {
				// Delete the proxy from the Binder
				deleteEntityInWM(intProxy.proxy);
			}
			typeMap.erase(typeMap.find(UID));
		} else {
			log("deleteProxy: Proxy type %s, UID %s not found!", type.c_str(),
					UID.c_str());
		}
	} else {
		log("deleteProxy: Proxy type %s not found!", type.c_str(), UID.c_str());
	}
}

void ProxyMarshaller::MarshallingServer::addFeature(const string & proxyType,
		const string & proxyUID, const binder::autogen::core::FeaturePtr &feature,
		const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->addFeature(proxyType, proxyUID, feature);
	m_pOwner->unlockComponent();
}

void ProxyMarshaller::addFeature(const string & proxyType,
		const string & proxyUID, const binder::autogen::core::FeaturePtr feature) {
	//Check pending relations; add there if applicable
	for (list<RelationCandidate>::iterator rIt = m_queuedRelations.begin(); rIt
			!= m_queuedRelations.end(); rIt++) {
		if (rIt->relationType == proxyType && rIt->relationUID == proxyUID) {
			rIt->addedFeatures.push_back(feature);
			return;
		}
	}

	if (m_proxyTypeMap.find(proxyType) != m_proxyTypeMap.end()) {
		map<string, InternalProxy> &typeMap = m_proxyTypeMap[proxyType];
		if (typeMap.find(proxyUID) != typeMap.end()) {
			InternalProxy &intProxy = typeMap[proxyUID];
			vector<FeaturePtr>::iterator it = intProxy.proxy->features.begin();
			//      for (; it != intProxy.proxy->features.end();
			//	  it++) {
			//	if ((*it)->featlabel == feature->featlabel) {
			//	  break;
			//	}
			//      }
			//      if (it != intProxy.proxy->features.end()) {
			//	log("addFeature: Feature %s already exists in proxy type %s, UID %s!",
			//	    feature->featlabel.c_str(), proxyType.c_str(), proxyUID.c_str());
			//      }
			//      else {
			// Add the feature
			intProxy.proxy->features.push_back(feature);

			//updateInternalProxy(typeMap[proxyUID]);
			//      }
		} else {
			log("addFeature: Proxy type %s, UID %s not found!", proxyType.c_str(),
					proxyUID.c_str());
		}
	} else {
		log("addFeature: Proxy type %s not found!", proxyType.c_str(),
				proxyUID.c_str());
	}
}

void ProxyMarshaller::MarshallingServer::deleteFeature(
		const string & proxyType, const string & proxyUID,
		const string & featlabel, const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->deleteFeature(proxyType, proxyUID, featlabel);
	m_pOwner->unlockComponent();
}

void ProxyMarshaller::MarshallingServer::commitFeatures(
		const string &proxyType, const string &proxyUID,
		const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->commitFeatures(proxyType, proxyUID);
	m_pOwner->unlockComponent();
}

void ProxyMarshaller::commitFeatures(const string & type, const string & UID) {
	if (m_proxyTypeMap.find(type) != m_proxyTypeMap.end()) {
		map<string, InternalProxy> &typeMap = m_proxyTypeMap[type];
		if (typeMap.find(UID) != typeMap.end()) {
			InternalProxy &intProxy = typeMap[UID];
			updateInternalProxy(intProxy);
		} else {
			log("commitFeatures: Proxy type %s, UID %s not found!", type.c_str(),
					UID.c_str());
		}
	} else {
		log("commitFeatures: Proxy type %s not found!", type.c_str(), UID.c_str());
	}
}

void ProxyMarshaller::deleteFeature(const string & proxyType,
		const string & proxyUID, const string & featlabel) {
	if (m_proxyTypeMap.find(proxyType) != m_proxyTypeMap.end()) {
		map<string, InternalProxy> &typeMap = m_proxyTypeMap[proxyType];
		if (typeMap.find(proxyUID) != typeMap.end()) {
			InternalProxy &intProxy = typeMap[proxyUID];
			vector<FeaturePtr>::iterator it = intProxy.proxy->features.begin();
			for (; it != intProxy.proxy->features.end();) {
				if ((*it)->featlabel == featlabel) {
					it = intProxy.proxy->features.erase(it);
					log("Erasing feature %s; %i remaining", featlabel.c_str(),
							intProxy.proxy->features.size());
				} else {
					it++;
				}
			}
			if (it == intProxy.proxy->features.end()) {
				log("deleteFeature: Feature %s not found in proxy type %s, UID %s!",
						featlabel.c_str(), proxyType.c_str(), proxyUID.c_str());
			}
		} else {
			log("deleteFeature: Proxy type %s, UID %s not found!", proxyType.c_str(),
					proxyUID.c_str());
		}
	} else {
		log("deleteFeature: Proxy type %s not found!", proxyType.c_str(),
				proxyUID.c_str());
	}
}

void ProxyMarshaller::MarshallingServer::publishProxy(const string & proxyType,
		const string & proxyUID, const Ice::Current &_context) {
	m_pOwner->lockComponent();
	m_pOwner->publishProxy(proxyType, proxyUID);
	m_pOwner->unlockComponent();
}

void ProxyMarshaller::publishProxy(const string & proxyType,
		const string & proxyUID) {
	// All proxies are always published so this does nothing
}

void ProxyMarshaller::updateInternalProxy(InternalProxy &intProxy) {
	// Current function: Always maintain all proxies on the Binder

	// Check if proxy exists on Binder
	if (intProxy.onBinder) {
		// Proxy exists on Binder; overwrite its Features
		log("Proxy existed on Binder; overwrite its features");
		log("Proxy has %i features.", intProxy.proxy->features.size());
		overwriteProxyInWM(intProxy.proxy);
	} else {
		// Proxy is new; add it to Binder
		log("Proxy new on Binder; create it");
		log("Proxy has %i features.", intProxy.proxy->features.size());
		addProxyToWM(intProxy.proxy);
		intProxy.onBinder = true;
	}
}

void ProxyMarshaller::newPlace(const cast::cdl::WorkingMemoryChange &_wmc) {
	lockEntry(_wmc.address.id, cdl::LOCKEDOD);
	SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place> (
			_wmc.address);
	if (place != 0) {
		stringstream ss;
		ss << place->id;
		log("Adding proxy (place, %s)", ss.str().c_str());
		addProxy("place", ss.str(), 1.0, createWorkingMemoryPointer(_wmc.address,
				_wmc.type));
		m_PlaceAddressToIDMap[_wmc.address.id] = place->id;

		// Add place id feature
		log("Adding place_id feature");
		FeaturePtr feature = createFeature("place_id");
		feature->alternativeValues.push_back(createStringValue(ss.str(), 1));
		//binder::autogen::featvalues::IntegerValue(1, place->id));
		addFeature("place", ss.str(), feature);

		// Add Placeholder feature
		if (place->status == SpatialData::PLACEHOLDER) {
			log("Adding explored feature");
			feature = createFeature("explored");
			feature->alternativeValues.push_back(createStringValue("false", 1));
			//binder::autogen::featvalues::IntegerValue(1,1));
			addFeature("place", ss.str(), feature);
		} else {
			log("Adding non-explored feature");
			feature = createFeature("explored");
			feature->alternativeValues.push_back(createStringValue("true", 1));
			//binder::autogen::featvalues::IntegerValue(1,1));
			addFeature("place", ss.str(), feature);
		}
		commitFeatures("place", ss.str());
	}
	unlockEntry(_wmc.address.id);
}

void ProxyMarshaller::changedPlace(const cast::cdl::WorkingMemoryChange &_wmc) {
	lockEntry(_wmc.address.id, cdl::LOCKEDOD);
	SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place> (
			_wmc.address);
	if (place != 0) {
		stringstream ss;
		ss << place->id;
		log("Changed place changes proxy (place, %s)", ss.str().c_str());

		// Add Placeholder feature
		if (place->status == SpatialData::PLACEHOLDER) {
			deleteFeature("place", ss.str(), "explored");
			FeaturePtr feature = createFeature("explored");
			feature->alternativeValues.push_back(createStringValue("false", 1));
			//binder::autogen::featvalues::IntegerValue(1,1));
			addFeature("place", ss.str(), feature);
		} else {
			deleteFeature("place", ss.str(), "explored");
			FeaturePtr feature = createFeature("explored");
			feature->alternativeValues.push_back(createStringValue("true", 1));
			//binder::autogen::featvalues::IntegerValue(1,1));
			addFeature("place", ss.str(), feature);
		}
		commitFeatures("place", ss.str());
	}
	unlockEntry(_wmc.address.id);
}

void ProxyMarshaller::deletedPlace(const cast::cdl::WorkingMemoryChange &_wmc) {
	map<string, int>::iterator it = m_PlaceAddressToIDMap.find(_wmc.address.id);

	if (it != m_PlaceAddressToIDMap.end()) {
		stringstream ss;
		ss << it->second;
		log("Deleting proxy for (place, %s)", ss.str().c_str());
		deleteProxy("place", ss.str());
	}
}

void ProxyMarshaller::updateLocalForegrounding() {
	// Check for any "robot" proxies, and foreground all Places
	// that are locations of such robots
	if (m_proxyTypeMap.find("robot") != m_proxyTypeMap.end()) {
		map<string, InternalProxy> &robotProxies = m_proxyTypeMap["robot"];
		// Foreground the current Places of all robots
		//    for (map<string, InternalProxy>::iterator it = robotProxies.begin();
		//	it != robotProxies.end(); it++) {
		//      InternalProxy &robotPrx = it->second;
		//
		//      for (vector<binder::autogen::core::FeaturePtr>::iterator it2 =
		//	  robotPrx.proxy->features.begin();
		//	  it2 != robotPrx.proxy->features.end(); it2++) {
		//	if ((*it2)->featlabel == "position") {
		//	  string
		//	}
		//      }
		//
		//    }

		// Find all local foregrounded Places that are no longer
		// supported by a robot, and background them.

	} else {
		log("Found no robot proxy!");
	}
}
