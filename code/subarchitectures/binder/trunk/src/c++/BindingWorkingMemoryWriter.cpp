#include "BindingWorkingMemoryWriter.hpp"

namespace binder {
  using namespace autogen::core;
  using namespace autogen::featvalues;
  using namespace cast;
  using namespace cast::cdl;


   // TODO: 	Check the c++ code is running and up to date


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

  ProxyPtr BindingWorkingMemoryWriter::createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
							       float probExists,
							       const autogen::core::FeatureValues source,
							       const autogen::core::FeatureValues target) {

    ProxyPtr newProxy = new Proxy();

    newProxy->entityID = newDataID();
    newProxy->origin = origin;
    newProxy->probExists = probExists;

    FeaturePtr feat1 = new Feature();
    feat1->featlabel = "source";
    feat1->alternativeValues = source;
    newProxy->features.push_back(feat1);

    FeaturePtr feat2 = new Feature();
    feat2->featlabel = "target";
    feat2->alternativeValues = target;
    newProxy->features.push_back(feat2);

    return newProxy;
  }


  ProxyPtr BindingWorkingMemoryWriter::createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
							       float probExists,
							       const FeaturesList & features,
							       const autogen::core::FeatureValues source,
							       const autogen::core::FeatureValues target) {

    ProxyPtr newProxy = new Proxy();

    newProxy->origin = origin;
    newProxy->entityID = newDataID();
    newProxy->probExists = probExists;

    newProxy->features = features;

    FeaturePtr feat1 = new Feature();
    feat1->featlabel = "source";
    feat1->alternativeValues = source;
    newProxy->features.push_back(feat1);

    FeaturePtr feat2 = new Feature();
    feat2->featlabel = "target";
    feat2->alternativeValues = target;
    newProxy->features.push_back(feat2);

    return newProxy;
  }

  ProxyPtr BindingWorkingMemoryWriter::createNewProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
						       float probExists,
						       const FeaturesList & features) {

    ProxyPtr newProxy = createNewProxy(origin, probExists);
    newProxy->origin = origin;

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
      addToWorkingMemory(proxy->entityID, proxy);
      //storeOriginInfo(proxy);
      log("new Proxy succesfully added to the binder working memory");
    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::addProxyToWM: %s", e.message.c_str());
    }
  }



  void BindingWorkingMemoryWriter::overwriteProxyInWM(ProxyPtr proxy) {

    try {
      overwriteWorkingMemory(proxy->entityID, proxy);
      log("existing Proxy succesfully modified in the binder working memory");

    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::overwriteProxyInWM: %s", e.message.c_str());
    }
  }



  void BindingWorkingMemoryWriter::deleteEntityInWM(ProxyPtr proxy) {

    try {
      //removeOriginInfo(proxy);
      deleteFromWorkingMemory(proxy->entityID);
      log("existing Proxy succesfully modified in the binder working memory");

    }
    catch (cast::CASTException &e) {
      println("EXCEPTION in BindingWorkingMemoryWriter::deleteEntityInWM: %s", e.message.c_str());
    }
  }

//   void BindingWorkingMemoryWriter::storeOriginInfo(autogen::core::ProxyPtr _proxy) {
//     OriginMapPtr om;
//     bool firstEntry = false;
//     if (m_originMapID.size()  == 0) {
//       m_originMapID = newDataID();
//       om = new OriginMap();
//       om->componentID = getComponentID();
//       firstEntry = true;
//     } else {
//       om = getMemoryEntry<OriginMap>(m_originMapID);
//     }

//     if (om->sourceID2ProxyID.find(_proxy->origin->localDataId) != om->sourceID2ProxyID.end()) {
//       println("WARNING: OriginMap already contained entry for: %s",
// 	      _proxy->origin->localDataId.c_str());
//     }

//     om->sourceID2ProxyID[_proxy->origin->localDataId] =  _proxy->entityID;

//     if (firstEntry) {
//       addToWorkingMemory(m_originMapID, om);
//     } else {
//       overwriteWorkingMemory(m_originMapID, om);
//     }

//   }


//   void BindingWorkingMemoryWriter::removeOriginInfo(autogen::core::ProxyPtr _proxy) {
//     OriginMapPtr om = getMemoryEntry<OriginMap>(m_originMapID);

//     if (om->sourceID2ProxyID.find(_proxy->origin->localDataId) == om->sourceID2ProxyID.end()) {
//       println("WARNING: OriginMap did no contain entry for: %s",
// 	      _proxy->origin->localDataId.c_str());
//     }
//     else {
//       om->sourceID2ProxyID.erase(_proxy->origin->localDataId);
//       overwriteWorkingMemory(m_originMapID, om);
//     }

//   }


}
