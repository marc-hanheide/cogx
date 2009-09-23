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

package binder.utils;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.Vector;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.discrete.DiscreteProbabilityAssignment;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.UnknownValue;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;

public class BinderUtils {

	public static RelationUnion convertIntoRelationUnion (Union bunion) {
		
		RelationUnion runion = new RelationUnion();
		runion.entityID = bunion.entityID;
		runion.features = bunion.features;
		runion.distribution = bunion.distribution;
		runion.includedProxies = bunion.includedProxies;
		runion.probExists = bunion.probExists;
		runion.timeStamp = bunion.timeStamp;
		
		return runion;
	}
	
	public static boolean induceRelationUnion(Vector<PerceivedEntity> entities) {

		Vector<Proxy> includedProxies = getProxies(entities);
		for (Enumeration<Proxy> e = includedProxies.elements() ; e.hasMoreElements() ;) {
			Proxy proxy = e.nextElement();
			if (! (proxy instanceof RelationProxy)) {
				return false;
			}
		}

		return true;
	}
	


	public static Feature[] addUnknownFeatureValues (Feature[] features) {

		for (int i = 0 ; i < features.length ; i++) {

			float totalProb = 0.0f;
			Vector<FeatureValue> values = new Vector<FeatureValue>();
			for (int j= 0 ; j < features[i].alternativeValues.length ; j++) {
				values.add(features[i].alternativeValues[j]);
				totalProb += features[i].alternativeValues[j].independentProb;
			}
		//	if (totalProb < 0.6f && features.length < 3) {
				features[i].alternativeValues = new FeatureValue[values.size() + 1];
				for (int j = 0 ; j < values.size(); j++ ) {
					features[i].alternativeValues[j] = values.elementAt(j);
				}
				features[i].alternativeValues[values.size()] = 
					new UnknownValue((1.0f - totalProb), features[i].alternativeValues[0].timeStamp);
		//	}
		}

		return features;
	}



	public static ArrayList<Union> getUnions(Vector<UnionConfiguration> configs) {
		
		ArrayList<Union> unions = new ArrayList<Union>();
		
		for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			
			UnionConfiguration config = e.nextElement();
			for (int i = 0; i < config.includedUnions.length ; i++) {
				
				Union union = config.includedUnions[i];
				if (!isInList(unions, union)) {
					unions.add(union);
				}
			}
		}

		
		return unions;
	}
	
	
	
	private static boolean isInList (ArrayList<Union> unions, Union union) {
		for (Iterator<Union> i = unions.iterator(); i.hasNext() ;) {
			Union u = i.next();
			if (u.entityID.equals(union.entityID)  && u.includedProxies.length == union.includedProxies.length && u.timeStamp == union.timeStamp) {
				return true;
			}
		}
		return false;
	}
	
	
	 public static void normaliseConfigProbabilities (Vector<UnionConfiguration> configs) {
		 
		 double sum = 0.0f;
		 for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			 	UnionConfiguration config = e.nextElement();
			 	sum += config.configProb;
			}
	
		 double alpha = 1.0f / sum;
		 for (Enumeration<UnionConfiguration> e = configs.elements(); e.hasMoreElements() ; ) {
			 	UnionConfiguration config = e.nextElement();
			 	config.configProb = alpha * config.configProb;
			}		 
	 }
	 
	 
	 
	 public static void addProbExistsToUnions (Vector<UnionConfiguration> configs) {
		 
		 ArrayList<Union> unions = getUnions (configs);
		 
		 setProbExistUnions(unions, configs);
	 }
	
	

	 
		public static void setProbExistUnions 
			(ArrayList<Union> unions, Vector<UnionConfiguration> configs) {
		
			for (Iterator<Union> e = unions.iterator() ; e.hasNext() ; ) {
				Union u = e.next();
				u.probExists = 0.0f;
				for (Enumeration<UnionConfiguration> f = configs.elements() ; f.hasMoreElements() ;) {
					UnionConfiguration config = f.nextElement();
					if (isUnionInConfig(config, u)) {
						u.probExists += config.configProb;
					}
				}
			}
		}
		

		
		private static boolean isUnionInConfig(UnionConfiguration config, Union union) {
			for (int i = 0 ; i < config.includedUnions.length ; i++) {
				if (config.includedUnions[i].equals(union)) {
					return true;
				}
			}
			return false;
		} 
		
	 
	/**
	 * Get all the proxies included in a set of perceptual entities (in case the entity
	 * is a proxy, it is simply added, and in case it is an union, the set of all included
	 * proxies is added to the resulting set)
	 * @param includedEntities the set of perceptual entities
	 * @return the resulting set of proxies
	 */
	public static Vector<Proxy> getProxies (Vector<PerceivedEntity> includedEntities) {
		Vector<Proxy> includedProxies = new Vector<Proxy>();
		for (Enumeration<PerceivedEntity> en = includedEntities.elements() ; en.hasMoreElements() ;) {
			PerceivedEntity entity = en.nextElement();
			if (entity instanceof Proxy) {
				includedProxies.add((Proxy)entity);
			}
			else if (entity instanceof Union) {
				Union includedUnion = (Union)entity;
				for (int i = 0 ; i < includedUnion.includedProxies.length ; i++) {
					includedProxies.add(includedUnion.includedProxies[i]);
				}
			}
		}
		return includedProxies;
	}
	
	

	public static Proxy completeProxy (Proxy proxy, boolean addUnknowns) {
	// If necessary, add unknown values
	if (addUnknowns && !FeatureValueUtils.hasUnknownValues(proxy.features)) {
		proxy.features = 
			BinderUtils.addUnknownFeatureValues(proxy.features);
	}
	
	// if the probability distribution of the updated proxy is unavailable, regenerate it
	if (proxy.distribution == null) {
		proxy.distribution = 
			ProbabilityUtils.generateProbabilityDistribution(proxy);
	}

	return proxy;
}


	public static String getPrettyPrintProbabilityDistribution
	(DiscreteProbabilityDistribution distrib) {
	String text = "";

	if (distrib.assignments != null) {
	for (int i = 0; i < distrib.assignments.length; i++) {
		DiscreteProbabilityAssignment assignment = distrib.assignments[i];
		text += getPrettyPrintProbabilityAssignment(assignment) + "\n";
	}
	}
	return text;
}

public static String getPrettyPrintProbabilityAssignment
	(DiscreteProbabilityAssignment assignment) {

	String text = "P ( " ;
	for (int j = 0; j < assignment.featurepairs.length ; j++) {
		text += assignment.featurepairs[j].featlabel + " = " + 
			FeatureValueUtils.toString(assignment.featurepairs[j].featvalue) ;
		if (j < (assignment.featurepairs.length - 1)) {
			text += ", ";
		}
	}
	text += " ) = " + assignment.prob ;
	return text;
}
}
