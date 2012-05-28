//
// = Filename
//   ProxyMarshaller.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef ProxyMarshaller_hpp
#define ProxyMarshaller_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <BinderEssentials.hpp>
#include <BindingWorkingMemoryWriter.hpp>
#include <Marshalling.hpp>
#include <string>
#include <map>

using namespace std;
using namespace binder::autogen::core;
using namespace cast;

namespace spatial {
class ProxyMarshaller: public binder::BindingWorkingMemoryWriter {
	class MarshallingServer: public Marshalling::Marshaller {
		virtual void addProxy(const string & type, const string & UID,
				double probExists, const cast::cdl::WorkingMemoryPointerPtr & origin,
				const Ice::Current &_context);
		virtual bool addRelation(const string & relationType,
				const string & relationUID, const string &sourceType,
				const string &sourceUID, const string &targetType,
				const string &targetUID, double probExists,
				const cast::cdl::WorkingMemoryPointerPtr & origin,
				const Ice::Current &_context);
		void deleteProxy(const string &typ, const string &UID,
				const Ice::Current &_context);
		virtual void addFeature(const string & proxyType, const string & proxyUID,
				const binder::autogen::core::FeaturePtr &feature,
				const Ice::Current &_context);
		virtual void deleteFeature(const string & proxyType,
				const string & proxyUID, const string & featLabel,
				const Ice::Current &_context);
		void commitFeatures(const string &typ, const string &UID,
				const Ice::Current &_context);
		virtual void publishProxy(const string & type, const string & UID,
				const Ice::Current &_context);
		ProxyMarshaller *m_pOwner;
		MarshallingServer(ProxyMarshaller *owner) :
			m_pOwner(owner) {
		}
		friend class ProxyMarshaller;
	};
	friend class MarshallingServer;

	class RelationCandidate {
	public:
		RelationCandidate(const string &rt, const string &ru, const string &st,
				const string &su, const string &tt, const string &tu, double p,
				const cast::cdl::WorkingMemoryPointerPtr o) :
			relationType(rt), relationUID(ru), sourceType(st), sourceUID(su),
					targetType(tt), targetUID(tu), probExists(p), origin(o) {
		}

		string relationType;
		string relationUID;
		string sourceType;
		string sourceUID;
		string targetType;
		string targetUID;
		double probExists;
		const cast::cdl::WorkingMemoryPointerPtr origin;
		vector<binder::autogen::core::FeaturePtr> addedFeatures;
	};

	struct InternalProxy {
		ProxyPtr proxy;
		bool onBinder;
		bool isRelation;
	};

public:
	/**
	 * Constructor
	 */
	ProxyMarshaller();

	/**
	 * Destructor
	 */
	virtual ~ProxyMarshaller();

	virtual void start();
	virtual void stop();
	virtual void runComponent();
	virtual void configure(const std::map<std::string, std::string>& _config);

	void addProxy(const string & type, const string & UID, double probExists,
			const cast::cdl::WorkingMemoryPointerPtr & origin);
	int addRelation(const RelationCandidate &cand);
	void deleteProxy(const string &typ, const string &UID);
	void addFeature(const string & proxyType, const string & proxyUID,
			const binder::autogen::core::FeaturePtr feature);
	void deleteFeature(const string & proxyType, const string & proxyUID,
			const string & featLabel);
	void commitFeatures(const string &typ, const string &UID);
	void publishProxy(const string & type, const string & UID);

	void newPlace(const cdl::WorkingMemoryChange &_wmc);
	void changedPlace(const cdl::WorkingMemoryChange &_wmc);
	void deletedPlace(const cdl::WorkingMemoryChange &_wmc);

	void updateInternalProxy(InternalProxy &intProxy);
protected:
	// Implementation-specific methods

	void updateLocalForegrounding(); // Foregrounding of the Place the robot is in
	void updateMotivePlaces(); // Foregrounding of Places Motivation is thinking of
	void updatePaths(); //Foregrounding of paths relevant to 
	//Places already foregrounded

private:
	// Main storage structure: A map over proxy types, each with
	// a map over UIDs
	map<string, map<string, InternalProxy> > m_proxyTypeMap;

	string m_bindingSA;

	map<string, int> m_PlaceAddressToIDMap;

	// Places that are foregrounded because of their local importance
	set<InternalProxy> m_localContextPlaces;

	// Relations that were not added properly; waiting for their source/target
	list<RelationCandidate> m_queuedRelations;
};
}
;

#endif
