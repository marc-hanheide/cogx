//
// = FILENAME
//    ObjectRelationManager.hpp
//
// = FUNCTION
//    Testing and visualization of code relating to object relation computations
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2010 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef ObjectRelationTester_hpp
#define ObjectRelationTester_hpp

#include <cast/architecture/ManagedComponent.hpp>

#include <peekabot.hh>
#include "RelationEvaluation.hpp"
#include "DensitySampling.hpp"

namespace spatial {
class ObjectRelationTester: public cast::ManagedComponent {
public:

	ObjectRelationTester();
	virtual ~ObjectRelationTester();

	virtual void runComponent();
	virtual void start();

protected:
	RelationEvaluator m_evaluator;

	bool m_bTestOnness;
	bool m_bSampleOnness;
	bool m_bTestInness;
	bool m_bSampleInness;
	bool m_bTestInference;
	bool m_bDemoSampling;
	bool m_bNoPTZ;
	bool m_bNoVision;
	bool m_bShowPoses;

	vector<spatial::Object *> m_testObjects;
	vector<peekabot::GroupProxy> m_testObjectProxies;

	peekabot::PeekabotClient m_PeekabotClient;
	peekabot::GroupProxy m_relationTester;
	std::string m_PbHost;
	int m_PbPort;
	int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

	void connectPeekabot();

	virtual void configure(const std::map<std::string, std::string>& _config);

	void addProxy(const spatial::Object* obj, const string &label);
	void updatePosesFromPB();

	DensitySampler m_sampler;

	void sampleOnnessForObject(const spatial::Object *objectS,
			spatial::Object *objectO);
	void sampleRecursively(const vector<spatial::SpatialRelationType> &types,
			const vector<spatial::Object*> &objects, unsigned int nSamplesPerStep,
			unsigned int nMaxSamples, vector<vector<Pose3> > &outPoints,
			spatial::Object *supportObject, unsigned int currentLevel = UINT_MAX
	//    , const vector<Vector3> &triangle
			);
};
}
;
#endif //ObjectRelationTester_hpp
