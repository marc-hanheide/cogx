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
    void deleteProxy(string type, string UID);
    void addFeature(string proxyType, string proxyUID, 
	binder::autogen::core::Feature feature);
    void deleteFeature(string proxyType, string proxyUID,
	string featLabel);
  };
};

#endif
