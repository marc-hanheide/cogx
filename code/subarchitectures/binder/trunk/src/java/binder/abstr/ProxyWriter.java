// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package binder.abstr;


import java.util.HashMap;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.OriginMap;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.autogen.specialentities.RelationProxy;
import binder.components.Binder;
import binder.constructors.ProxyConstructor;
import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPointer;
 
/**
 * Abstract class for structuring and inserting proxies into the binder working
 * memory
 * 
 * 
 * TODO: define extensions to handle group proxies
 * 
 * @author Pierre Lison
 * @version 09/09/2009 (started 15/08/2009)
 */

public abstract class ProxyWriter extends ManagedComponent {
 
	
	
	// =================================================================
	// METHODS FOR CREATING NEW PROXIES
	// =================================================================
     
	/**
	 * ID in current SA where origin map is stored.
	 */
	private String m_originMapID;

	
	/**
	 * Construct an WorkingMemoryPointer object (information about the proxy origin:
	 * subarchitecture identifier, local data ID in subarchitecture, and data
	 * type)
	 * 
	 * @param subarchId
	 *            subarchitecture identifier
	 * @param localDataId
	 *            identifier of the data in local subarchitecture
	 * @param localDataType
	 *            type of the data in local subarchitecture
	 * @return a new OriginInfo object
	 */
	public WorkingMemoryPointer createWorkingMemoryPointer (String subarchId, String localDataId,
			String localDataType) {	
		return ProxyConstructor.createWorkingMemoryPointer(subarchId, localDataId, localDataType);
	}

	/**
	 * Create a new proxy given the pointer to the originating object and
	 * the probability of the proxy existence itself (the list of features 
	 * is defined to be empty)
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            probability value for the proxy
	 * @return a new proxy
	 */
	public Proxy createNewProxy(WorkingMemoryPointer origin, float probExists) {
		return ProxyConstructor.createNewProxy(origin, newDataID(), probExists);
	}

	/**
	 * Create a new proxy given the pointer to the originating object, the
	 * probability of the proxy, and a list of features
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param features
	 *            the features
	 * @return the created proxy
	 */
	public Proxy createNewProxy(WorkingMemoryPointer origin, float probExists,
			Feature[] features) {

		return ProxyConstructor.createNewProxy(origin, newDataID(), probExists, features);
	}

	/**
	 * Create a new relation proxy given the pointer to the originating object,
	 * the probability of the proxy, and the source and target proxies
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param sources
	 *            the addresses of the source proxies
	 * @param targets
	 *            the addresses of the target proxies
	 * @return the new relation proxy
	 */
	public RelationProxy createNewRelationProxy(WorkingMemoryPointer origin,
			float probExists, AddressValue[] sources, AddressValue[] targets) {

		return ProxyConstructor.createNewRelationProxy(origin, newDataID(), probExists, sources, targets);
	}

	/**
	 * Create a new relation proxy given the pointer to the originating object,
	 * the probability of the proxy, the list of features for
	 * the relation, and the source and target proxies
	 * 
	 * @param origin
	 *            pointer to the object in the local working memory
	 * @param probExists
	 *            the probability of the proxy
	 * @param features
	 *            the features
	 * @param sources
	 *            the addresses of the source proxies
	 * @param targets
	 *            the addresses of the target proxies
	 * @return the new relation proxy
	 */
	public RelationProxy createNewRelationProxy(WorkingMemoryPointer origin,
			float probExists, Feature[] features, AddressValue[] sources,
			AddressValue[] targets) {
		
		return ProxyConstructor.createNewRelationProxy(origin, newDataID(), probExists, features, sources, targets);
	}



	// =================================================================
	// METHODS FOR CREATING AND INSERTING NEW FEATURES
	// =================================================================

	/**
	 * Add a new feature to the proxy (and regenerate the probability
	 * distribution, given this new information)
	 * 
	 * @param proxy
	 *            the proxy
	 * @param feat
	 *            the feature to add
	 */

	public void addFeatureToProxy(Proxy proxy, Feature feat) {

		ProxyConstructor.addFeatureToProxy(proxy, feat);

	}

	/**
	 * Create a new StringValue given a string and a probability
	 * 
	 * @param val
	 *            the string
	 * @param prob
	 *            the probability value
	 * @return the StringValue
	 */

	public StringValue createStringValue(String val, float prob) {
		StringValue v = ProxyConstructor.createStringValue(val, prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}

	/**
	 * Create a new AddressValue given a string and a probability
	 * 
	 * @param address
	 *            the address (as a string)
	 * @param prob
	 *            the probability value
	 * @return the AddressValue
	 */

	public AddressValue createAddressValue(String address, float prob) {
		AddressValue v = ProxyConstructor.createAddressValue(address, prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}

	/**
	 * Create a new AddressValue given a string and a probability
	 * 
	 * @param integer
	 *            the integer
	 * @param prob
	 *            the probability value
	 * @return the IntegerValue
	 */

	public IntegerValue createIntegerValue(int integer, float prob) {
		IntegerValue v =  ProxyConstructor.createIntegerValue(integer, prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}

	/**
	 * Create a new BooleanValue given a boolean and a probability
	 * 
	 * @param val
	 *            the boolean
	 * @param prob
	 *            the probability value
	 * @return the BooleanValue
	 */

	public BooleanValue createBooleanValue(boolean val, float prob) {
		BooleanValue v =  ProxyConstructor.createBooleanValue(val, prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}

	
	/**
	 * Create a new FloatValue given a float and a probability
	 * 
	 * @param floatv
	 *            the float
	 * @param prob
	 *            the probability value
	 * @return the BooleanValue
	 */
	
	public FloatValue createFloatValue(float floatv, float prob) {
		FloatValue v =  ProxyConstructor.createFloatValue(floatv, prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}
	
	
	public UnknownValue createUnknownValue(float prob) {
		UnknownValue v = ProxyConstructor.createUnknownValue(prob);
		ProxyConstructor.setTimeStamp(v, getCASTTime());
		return v;
	}
	
	/**
	 * Create a new feature, without feature values
	 * 
	 * @param featlabel
	 *            the feature label
	 * @return the new feature
	 */

	public Feature createFeature(String featlabel) {
		return ProxyConstructor.createFeature(featlabel);
	}

	/**
	 * Create a new feature with a unique feature value
	 * 
	 * @param featlabel
	 *            the feature label
	 * @param featvalue
	 *            the feature value
	 * @return the new feature
	 */

	public Feature createFeatureWithUniqueFeatureValue(String featlabel,
			FeatureValue featvalue) {
		return ProxyConstructor.createFeatureWithUniqueFeatureValue(featlabel, featvalue);
	}

	/**
	 * Add a new feature value to an existing feature
	 * 
	 * @param feat
	 *            the feature
	 * @param featval
	 *            the feature value
	 */

	public void addFeatureValueToFeature(Feature feat, FeatureValue featval) {
		ProxyConstructor.addFeatureValueToFeature(feat, featval);
	}

	/**
	 * Create a new feature containing several alternative feature values
	 * 
	 * @param featlabel
	 *            the feature label
	 * @param featvalues
	 *            the array of feature values
	 * @return the feature
	 */

	public Feature createFeatureWithAlternativeFeatureValues(String featlabel,
			FeatureValue[] featvalues) {
		return ProxyConstructor.createFeatureWithAlternativeFeatureValues(featlabel, featvalues);
	}



		
	// =================================================================
	// METHODS FOR INSERTING/MODIFYING/DELETING PROXIES IN THE WM
	// =================================================================

	/**
	 * Insert the proxy in the binder working memory
	 * 
	 * @param proxy
	 *            the proxy
	 */

	protected void addProxyToWM(Proxy proxy) {

		try {
			addToWorkingMemory(proxy.entityID, Binder.BINDER_SA, proxy);
			storeOriginInfo(proxy);
			log("new Proxy succesfully added to the binder working memory");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Overwrite an existing proxy with a new one (the new proxy needs to have
	 * the same entityID has the existing one)
	 * 
	 * @param proxy
	 *            the new proxy
	 */

	protected void overwriteProxyInWM(Proxy proxy) {
 
		try {
			overwriteWorkingMemory(proxy.entityID, Binder.BINDER_SA, proxy);
			log("existing Proxy succesfully modified in the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the proxy does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Delete an existing proxy
	 * 
	 * @param proxy
	 *            the proxy to delete
	 */

	protected void deleteEntityInWM(Proxy proxy) {

		try {
			removeOriginInfo(proxy);
			deleteFromWorkingMemory(proxy.entityID, Binder.BINDER_SA);
			log("existing Proxy succesfully deleted from the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the proxy does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	} 
	
	/**
	 * Delete an existing proxy
	 * 
	 * @param proxy
	 *            the proxy to delete
	 */

	protected void deleteEntityInWM(String proxyId) {

		try {
//			removeOriginInfo(proxy);
			deleteFromWorkingMemory(proxyId, Binder.BINDER_SA);
			log("existing Proxy succesfully deleted from the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the proxy does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	} 
	
	/**
	 * Stores a mapping from the source to the proxy which is created from it.
	 * 
	 * @param _proxy
	 * @throws DoesNotExistOnWMException
	 * @throws PermissionException
	 * @throws ConsistencyException
	 * @throws AlreadyExistsOnWMException
	 */
	
	private void storeOriginInfo(Proxy _proxy)
			throws DoesNotExistOnWMException, ConsistencyException, UnknownSubarchitectureException, 
			PermissionException, AlreadyExistsOnWMException {
		OriginMap om = null;
		boolean firstEntry = false;
		if (m_originMapID == null) {
			m_originMapID = newDataID();
			om = new OriginMap(getComponentID(), new HashMap<String, String>(), new HashMap<String, WorkingMemoryPointer>());
			firstEntry = true;
		} else {
			om = getMemoryEntry(m_originMapID, Binder.BINDER_SA, OriginMap.class);
		}

		if (om.sourceID2ProxyID.containsKey(_proxy.origin.address.id)) {
			println("WARNING: OriginMap already contained entry for: "
					+ _proxy.origin.address.id);
		}

		om.sourceID2ProxyID.put(_proxy.origin.address.id, _proxy.entityID);
		om.proxyID2WMPointer.put(_proxy.entityID, _proxy.origin);

		
		if (firstEntry) {
			addToWorkingMemory(m_originMapID, Binder.BINDER_SA, om);
		} else {
			overwriteWorkingMemory(m_originMapID, Binder.BINDER_SA, om);
		}

	}
	 
	
	/**
	 * Removes source id mapping from WM map.
	 * 
	 * @param _proxy
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 */
	
	private void removeOriginInfo(Proxy _proxy)
			throws DoesNotExistOnWMException, ConsistencyException,UnknownSubarchitectureException, 
			PermissionException {
		OriginMap om = getMemoryEntry(m_originMapID, Binder.BINDER_SA, OriginMap.class);

		if (!om.sourceID2ProxyID.containsKey(_proxy.origin.address.id)) {
			println("WARNING: OriginMap did not contain entry for: "
					+ _proxy.origin.address.id);
		} else {
			om.sourceID2ProxyID.remove(_proxy.origin.address.id);
			om.proxyID2WMPointer.remove(_proxy.entityID);
			overwriteWorkingMemory(m_originMapID, Binder.BINDER_SA, om);
		}
	}
	
	

}
