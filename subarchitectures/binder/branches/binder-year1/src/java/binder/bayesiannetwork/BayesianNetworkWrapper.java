package binder.bayesiannetwork;

import java.io.ByteArrayInputStream;
import java.util.Enumeration;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.core.logging.ComponentLogger;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.distributions.FeatureValuePair;
import binder.bayesiannetwork.configparser.BNConfigParser;
import binder.filtering.EntityFilter;
import binder.utils.FeatureValueUtils;
import binder.utils.GenericUtils;


/**
 * Wrapper library for operations on the bayesian network
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 10/08/2009)
 */

public class BayesianNetworkWrapper {

	// no clue what that is
	private static final long serialVersionUID = 1L;

	// flag to activate logging
	public static boolean LOGGING = false ;
	
	private static Logger logger = ComponentLogger.getLogger(BayesianNetworkWrapper.class);

	
	// The bayesian network wrapped in this class
	BayesianNetwork network;
	
	
	// ================================================================= 
	// CONSTRUCTOR
	// ================================================================= 
	
	
	/**
	 * Construct a new bayesian network based on a configuration text file
	 * 
	 * TODO: get correct prior and conditional probabilities on Bayesian network
	 * 
	 * @param configFile the path to the configuration file
	 */
	
	public BayesianNetworkWrapper (String configFile) {
	 
		// create the bayesian network
		network = new BayesianNetwork() ;
		
		// parse the configuration and assign the result to the bayesian network
		String text = GenericUtils.getText(configFile);
		byte[] stringBytes = text.getBytes();
		ByteArrayInputStream bais = new ByteArrayInputStream(stringBytes);
		BNConfigParser parser = new BNConfigParser(bais);
		try {
	    network = parser.Configuration();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		// add unknown feature values to the bayesian network
	    addUnknownFeatureValuesToBayesianNetwork();    
	
	    // TODO: fill prior and conditional probabilities to the network
	//	network = fillPriorProbabilities(network);

	}

	
	
	// ================================================================= 
	// ACCESS METHODS   
	// ================================================================= 
	
	
	
	/**
	 * Get the list of incoming edges for the node in the bayesian network
	 * 
	 * @param node the node
	 * @return the list of edges
	 */
	public Vector<BayesianNetworkEdge> getIncomingEdges(BayesianNetworkNode node)  {
		
		Vector<BayesianNetworkEdge> incoming = new Vector<BayesianNetworkEdge>();
		
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.incomingNode.equals(node)) {
				incoming.add(edge);
			}
		}
		
		return incoming;
	}
	
	
	/**
	 * Get the list of outgoing edges for the node in the bayesian network
	 * 
	 * @param node the node
	 * @return the list of edges
	 */
	public Vector<BayesianNetworkEdge> getOutgoingEdges(BayesianNetworkNode node)  {
		
		Vector<BayesianNetworkEdge> outgoing = new Vector<BayesianNetworkEdge>();
		
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.outgoingNode.equals(node)) {
				outgoing.add(edge);
			}
		}
		
		return outgoing;
	}
	
	/**
	 * Get the list of edges between two nodes
	 * 
	 * @param node1 the first node
	 * @param node2 the second node
	 * @return the list of edges
	 */
	public Vector<BayesianNetworkEdge> getEdgesBetweenNodes(BayesianNetworkNode node1, BayesianNetworkNode node2)  {
		
		Vector<BayesianNetworkEdge> between = new Vector<BayesianNetworkEdge>();
		for (int i = 0; i< network.edges.length ; i++) {
			BayesianNetworkEdge edge = network.edges[i];
			if (edge.incomingNode.equals(node2) && edge.outgoingNode.equals(node1)) {
				between.add(edge);
			}
		}
		
		return between;
	}


	/**
	 * Get the correlation between two featue-value pairs, based on the bayesian network
	 * 
	 * @param pair1 the first feature value pair
	 * @param pair2 the second feature value pair
	 * @return the extracted correlation
	 */
	
	public FeatureValueCorrelation getCorrelationsBetweenFVs (FeatureValuePair pair1, FeatureValuePair pair2) {
		BayesianNetworkNode node1 = getNodeWithFeature(pair1.featlabel);
		BayesianNetworkNode node2 = getNodeWithFeature(pair2.featlabel);
		Vector<BayesianNetworkEdge> between = getEdgesBetweenNodes(node1, node2);
	
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

	/**
	 * Given a specific feature, find the node in the bayesian network which describes 
	 * its correlations
	 * 
	 * @param feat the feature
	 * @return the node
	 */
	public BayesianNetworkNode getNodeWithFeature (Feature feat) {

		for (int i = 0; i < network.nodes.length; i++) {
			BayesianNetworkNode node = network.nodes[i];
			if (node.feat.featlabel.equals(feat.featlabel))  {
				return node;
			}
		}
		return null;
	}
	
	/**
	 * Given a specific feature label, find the node in the bayesian network which describes 
	 * its correlations
	 * 
	 * @param featlabel the feature label
	 * @return the node
	 */
	public BayesianNetworkNode getNodeWithFeature (String featlabel) {
		
		for (int i = 0; i < network.nodes.length; i++) {
			BayesianNetworkNode node = network.nodes[i];
			if (node.feat.featlabel.equals(featlabel))  {
				return node;
			}
		}
		return null;
	}

	/**
	 * Return the nodes of the network
	 * @return the array of network nodes
	 */
	public BayesianNetworkNode[] getNodes() {
		return network.nodes;
	}
	
	
	/**
	 * Return the edges of the network
	 * @return the array of network edges
	 */
	public BayesianNetworkEdge[] getEdges() {
		return network.edges;
	}
	
	/**
	 * Return the Bayesian Network object
	 */
	public BayesianNetwork getBayesianNetwork() {
		return network;
	}
	
	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 
	
	
	/**
	 * If necessary, add "unknown" feature values to the values in the bayesian network
	 * 
	 */
	
	public void addUnknownFeatureValuesToBayesianNetwork () {
	    for (int i = 0; i < network.nodes.length ; i ++) {
	    	int nbvalues = network.nodes[i].feat.alternativeValues.length;
	    	FeatureValue[] newfeatvalues = new FeatureValue[nbvalues + 1];
	    	for (int j = 0; j < nbvalues; j++ )  {
	    		newfeatvalues[j] = network.nodes[i].feat.alternativeValues[j];
	    	}
	    	newfeatvalues[nbvalues] = FeatureValueUtils.createUnknownValue(1.0f/nbvalues);
	    	network.nodes[i].feat.alternativeValues = newfeatvalues;
	    }
	}
	
	
	/**
	 * logging utility
	 * 
	 * @param s the string to print out
	 */
	
	private static void log(String s) {
		if (LOGGING) {
			logger.debug("[BayesianNetworkWrapper] " + s);
		}
	}
}
