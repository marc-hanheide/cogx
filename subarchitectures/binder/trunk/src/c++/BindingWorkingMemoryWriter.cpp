#include "BindingWorkingMemoryWriter.hpp"

namespace binder {
  using namespace autogen::core;
  using namespace autogen::specialentities;
  using namespace autogen::featvalues;
  using namespace cast;
  using namespace cast::cdl;


   // TODO: 	Check the c++ code is running and up to date

  void BindingWorkingMemoryWriter::configure(const std::map<std::string, std::string>& _config)
  {
    //ManagedComponent::configure(_config);

     if (_config.find("-bsa") != _config.end()) {
       m_bindingSA=_config.find("-bsa")->second;
     } else if (_config.find("--bsa") != _config.end()) {
       m_bindingSA=_config.find("--bsa")->second;
     } else {
       m_bindingSA="binder";
     }
  }


  WorkingMemoryPointerPtr BindingWorkingMemoryWriter::createWorkingMemoryPointer (const std::string & subarchId,
									const std::string &  localDataId, const std::string &  localDataType) {

    WorkingMemoryPointerPtr origin = new WorkingMemoryPointer();
    origin->address.subarchitecture = subarchId;
    origin->address.id = localDataId;
    origin->type = localDataType;
    return origin;
  }


  ProxyPtr BindingWorkingMemoryWriter::createNewProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
						       float probExists) {

    ProxyPtr newProxy = new Proxy();

    newProxy->entityID = newDataID();
    newProxy->origin = origin;
    newProxy->probExists = probExists;

    return newProxy;
  }

  RelationProxyPtr BindingWorkingMemoryWriter::createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
							       float probExists,
							       const autogen::core::FeatureValues source,
							       const autogen::core::FeatureValues target) {

    RelationProxyPtr newProxy = new RelationProxy();

    newProxy->entityID = newDataID();
    newProxy->origin = origin;
    newProxy->probExists = probExists;

    newProxy->source = new Feature();
    newProxy->source->featlabel = "source";
    newProxy->source->alternativeValues = source;

    newProxy->target = new Feature();
    newProxy->target->featlabel = "target";
    newProxy->target->alternativeValues = target;

    return newProxy;
  }


  RelationProxyPtr BindingWorkingMemoryWriter::createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
							       float probExists,
							       const FeaturesList & features,
							       const autogen::core::FeatureValues source,
							       const autogen::core::FeatureValues target) {

    RelationProxyPtr newProxy = createNewRelationProxy(origin, probExists, source, target);

    newProxy->features = features;

    return newProxy;
  }

  ProxyPtr BindingWorkingMemoryWriter::createNewProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
						       float probExists,
						       const FeaturesList & features) {

    ProxyPtr newProxy = createNewProxy(origin, probExists);

    newProxy->features = features;

    return newProxy;
  }




  void BindingWorkingMemoryWriter::addFeatureToProxy(ProxyPtr proxy,
						     FeaturePtr feat) {
    proxy->features.push_back(feat);
  }


  StringValuePtr BindingWorkingMemoryWriter::createStringValue(const std::string & val,
							       float prob) {
    StringValuePtr stringVal = new StringValue();
    stringVal->val = val;
    stringVal->independentProb = prob;
    return stringVal;
  }



  IntegerValuePtr BindingWorkingMemoryWriter::createIntegerValue(int val, float prob) {
    IntegerValuePtr intVal = new IntegerValue();
    intVal->val = val;
    intVal->independentProb = prob;
    return intVal;
  }



  BooleanValuePtr BindingWorkingMemoryWriter::createBooleanValue(bool val, float prob) {
	  BooleanValuePtr boolVal = new BooleanValue();
	  boolVal->val = val;
	  boolVal->independentProb = prob;
    return boolVal;
  }


  UnknownValuePtr BindingWorkingMemoryWriter::createUnknownValue(float prob) {
	  UnknownValuePtr uval = new UnknownValue();
	  uval->independentProb = prob;
    return uval;
  }


  FeaturePtr BindingWorkingMemoryWriter::createFeature(const std::string & featlabel) {
    FeaturePtr feat = new Feature();
    feat->featlabel = featlabel;
    return feat;
  }


  FeaturePtr BindingWorkingMemoryWriter::createFeatureWithUniqueFeatureValue
  (const std::string & featlabel, FeatureValuePtr featvalue) {

    FeaturePtr feat = createFeature(featlabel);
    feat->alternativeValues.push_back(featvalue);
    return feat;
  }


  void BindingWorkingMemoryWriter::addFeatureValueToFeature(FeaturePtr feat,
							    FeatureValuePtr featval) {
    feat->alternativeValues.push_back(featval);
  }

  FeaturePtr BindingWorkingMemoryWriter::createFeatureWithAlternativeFeatureValues
  (const std::string & featlabel, const FeatureValues & featvalues) {

    FeaturePtr feat = createFeature(featlabel);
    feat->alternativeValues = featvalues;
    return feat;
  }

  void BindingWorkingMemoryWriter::addProxyToWM(ProxyPtr proxy) {

    try {
      addToWorkingMemory(proxy->entityID, m_bindingSA, proxy);
      storeOriginInfo(proxy);
      log("new Proxy succesfully added to the binder working memory");
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::addProxyToWM: %s", e.message.c_str());
    }
  }



  void BindingWorkingMemoryWriter::overwriteProxyInWM(ProxyPtr proxy) {

    try {
      overwriteWorkingMemory(proxy->entityID, m_bindingSA, proxy);
      log("existing Proxy succesfully modified in the binder working memory");

    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::overwriteProxyInWM: %s", e.message.c_str());
    }
  }



  void BindingWorkingMemoryWriter::deleteEntityInWM(ProxyPtr proxy) {

    try {
      removeOriginInfo(proxy);
      deleteFromWorkingMemory(proxy->entityID, m_bindingSA);
      log("existing Proxy succesfully modified in the binder working memory");

    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::deleteEntityInWM: %s", e.message.c_str());
    }
  }


  void BindingWorkingMemoryWriter::deleteEntityInWM(std::string _id) {

    try {
      ProxyPtr proxy = getMemoryEntry<Proxy>(_id, m_bindingSA);
      deleteEntityInWM(proxy);
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::deleteEntityInWM: %s", e.message.c_str());
    }
  }

  void BindingWorkingMemoryWriter::storeOriginInfo(autogen::core::ProxyPtr _proxy) {
    OriginMapPtr om;
    bool firstEntry = false;
    if (m_originMapID.size()  == 0) {
      m_originMapID = newDataID();
      om = new OriginMap();
      om->componentID = getComponentID();
      firstEntry = true;
    } else {
      om = getMemoryEntry<OriginMap>(m_originMapID);
    }

    if (om->sourceID2ProxyID.find(_proxy->origin->address.id) != om->sourceID2ProxyID.end()) {
      println("WARNING: OriginMap already contained entry for: %s",
	      _proxy->origin->address.id.c_str());
    }

    om->sourceID2ProxyID[_proxy->origin->address.id] =  _proxy->entityID;
    om->proxyID2WMPointer[_proxy->entityID] =  _proxy->origin;

    if (firstEntry) {
      addToWorkingMemory(m_originMapID, om);
    } else {
      overwriteWorkingMemory(m_originMapID, om);
    }

  }


  void BindingWorkingMemoryWriter::removeOriginInfo(autogen::core::ProxyPtr _proxy) {
    OriginMapPtr om = getMemoryEntry<OriginMap>(m_originMapID);

    if (om->sourceID2ProxyID.find(_proxy->origin->address.id) == om->sourceID2ProxyID.end()) {
      println("WARNING: OriginMap did no contain entry for: %s",
	      _proxy->origin->address.id.c_str());
    }
    else {
      om->sourceID2ProxyID.erase(_proxy->origin->address.id);
      om->proxyID2WMPointer.erase(_proxy->entityID);
      overwriteWorkingMemory(m_originMapID, om);
    }

  }


}
