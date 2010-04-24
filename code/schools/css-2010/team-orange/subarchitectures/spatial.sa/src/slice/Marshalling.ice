#ifndef MARSHALLING_ICE
#define MARSHALLING_ICE

#include <cast/slice/CDL.ice>
#include <BinderEssentials.ice>
/**
 * Defines interface for writing proxies and features
 * to the Spatial.SA ProxyMarshaller component
 *
 * @author Kristoffer Sjöö
 * @see
 */

module Marshalling {
  interface Marshaller {
    // Add a proxy to the Marshaller. 
    // type: string describing the "class" of proxy: place, robot, ...
    // UID: string uniquely identifying the instance of the class (
    //   such as Place ID)
    void addProxy(string type, string UID, double probExists, 
		  cast::cdl::WorkingMemoryPointer origin);
    bool addRelation(string relationType, string relationUID, 
	 string sourceType, string sourceUID,
	 string targetType, string targetUID,
	 double probExists, cast::cdl::WorkingMemoryPointer origin);
    // Delete a proxy from the Marshaller (and Binding, if present)
    void deleteProxy(string type, string UID);
    // Add a feature to a proxy in the Marshaller. Changes are not
    // uploaded to the Binder (if applicable) until commitFeatures is
    // called.
    void addFeature(string proxyType, string proxyUID, 
	binder::autogen::core::Feature feature);
    // Delete a feature class from a proxy. All features with a given
    // lavel are deleteed. Changes are not
    // uploaded to the Binder (if applicable) until commitFeatures is
    // called.
    void deleteFeature(string proxyType, string proxyUID,
	string featLabel);
    void commitFeatures(string proxyType, string proxyUID);
  };
};

#endif
