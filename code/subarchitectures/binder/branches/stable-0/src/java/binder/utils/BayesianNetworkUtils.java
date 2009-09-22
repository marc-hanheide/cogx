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

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Enumeration;
import java.util.Vector;

import cast.cdl.CASTTime;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.featvalues.UnknownValue;
import binder.bayesiannetwork.configparser.BNConfigParser;

public class BayesianNetworkUtils {

	public static boolean logging = false;
	
	public static BayesianNetwork constructNetwork(String configFile) {
	
		BayesianNetwork network = new BayesianNetwork();
		
		String text = getText(configFile);
		byte[] stringBytes = text.getBytes();
		ByteArrayInputStream bais = new ByteArrayInputStream(stringBytes);
		BNConfigParser parser = new BNConfigParser(bais);
		try {
	    network = parser.Configuration();
	    
	    for (int i = 0; i < network.nodes.length ; i ++) {
	    	int nbvalues = network.nodes[i].feat.alternativeValues.length;
	    	FeatureValue[] newfeatvalues = new FeatureValue[nbvalues + 1];
	    	for (int j = 0; j < nbvalues; j++ )  {
	    		newfeatvalues[j] = network.nodes[i].feat.alternativeValues[j];
	    	}
	    	newfeatvalues[nbvalues] = new UnknownValue(1.0f/nbvalues, new CASTTime());
	    	network.nodes[i].feat.alternativeValues = newfeatvalues;
	    }
	    
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	//	network = fillPriorProbabilities(network);

		return network;
	}
	
	
	public static Vector<BayesianNetworkEdge> getIncomingEdges(BayesianNetwork network, BayesianNetworkNode node)  {
		
		Vector<BayesianNetworkEdge> incoming = new Vector<BayesianNetworkEdge>();
		
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.incomingNode.equals(node)) {
				incoming.add(edge);
			}
		}
		
		return incoming;
	}
	
	public static Vector<BayesianNetworkEdge> getOutgoingEdges(BayesianNetwork network, BayesianNetworkNode node)  {
		
		Vector<BayesianNetworkEdge> outgoing = new Vector<BayesianNetworkEdge>();
		
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.outgoingNode.equals(node)) {
				outgoing.add(edge);
			}
		}
		
		return outgoing;
	}
	

	public static Vector<BayesianNetworkEdge> getEdgesBetweenNodes(BayesianNetwork network, BayesianNetworkNode node1, BayesianNetworkNode node2)  {
		
		Vector<BayesianNetworkEdge> between = new Vector<BayesianNetworkEdge>();
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.incomingNode.equals(node2) && edge.outgoingNode.equals(node1)) {
				between.add(edge);
			}
		}
		
		return between;
	}


	public static FeatureValueCorrelation getCorrelationsBetweenFVs (BayesianNetwork network, FeatureValuePair pair1, FeatureValuePair pair2) {
		BayesianNetworkNode node1 = getNodeWithFeature(network, pair1.featlabel);
		BayesianNetworkNode node2 = getNodeWithFeature(network, pair2.featlabel);
		Vector<BayesianNetworkEdge> between = getEdgesBetweenNodes(network, node1, node2);

		for (Enumeration<BayesianNetworkEdge> e = between.elements(); e.hasMoreElements(); ) {
			BayesianNetworkEdge edge = e.nextElement();
			for (int i = 0; i < edge.correlations.length; i++) {
				FeatureValueCorrelation corr = edge.correlations[i];
				FeatureValue val1a = pair1.featvalue;
				FeatureValue val1b = corr.value1;
				FeatureValue val2a = pair2.featvalue;
				FeatureValue val2b = corr.value2;
				if (FeatureValueUtils.haveEqualValue(val1a, val1b) &&
						FeatureValueUtils.haveEqualValue(val2a, val2b)) {
					log("correlation found between ("+ pair1.featlabel+"="+ 
							FeatureValueUtils.toString(val1a)+") and (" + pair2.featlabel +
							"=" + FeatureValueUtils.toString(val2a) + "): " + corr.condProb);
					return corr;
				}
			}
		}
		return null;
	}

	public static BayesianNetworkNode getNodeWithFeature (BayesianNetwork network, Feature feat) {

		for (int i = 0; i < network.nodes.length; i++) {
			BayesianNetworkNode node = network.nodes[i];
			if (node.feat.featlabel.equals(feat.featlabel))  {
				return node;
			}
		}
		return null;
	}
	
	public static BayesianNetworkNode getNodeWithFeature (BayesianNetwork network, String featlabel) {
		
		for (int i = 0; i < network.nodes.length; i++) {
			BayesianNetworkNode node = network.nodes[i];
			if (node.feat.featlabel.equals(featlabel))  {
				return node;
			}
		}
		return null;
	}
	
	public static String getText(String aFile) {

		StringBuilder contents = new StringBuilder();
    
    try {

      BufferedReader input =  new BufferedReader(new FileReader(aFile));
      try {
        String line = null; 
    
        while (( line = input.readLine()) != null){
          contents.append(line);
          contents.append(System.getProperty("line.separator"));
        }
      }
      finally {
        input.close();
      }
    }
    catch (IOException ex){
      ex.printStackTrace();
    }
    
    return contents.toString();
	}
	

	private static void log(String s) {
		if (logging) {
			System.out.println("[BayesianNetworkUtils] " + s);
		}
	}
	
	
}
