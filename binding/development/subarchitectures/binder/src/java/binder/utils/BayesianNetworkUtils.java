package binder.utils;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.distributions.FeatureValuePair;
import binder.autogen.featvalues.StringValue;
import binder.bayesiannetwork.configparser.BNConfigParser;

public class BayesianNetworkUtils {

	public static String configurationFile = "./subarchitectures/binder/config/bayesiannetwork.txt";

	public static boolean logging = false;
	
	public static BayesianNetwork constructNetwork(String configFile) {
		
		BayesianNetwork network = new BayesianNetwork();
		
		String text = getText(configurationFile);
		byte[] stringBytes = text.getBytes();
		ByteArrayInputStream bais = new ByteArrayInputStream(stringBytes);
		BNConfigParser parser = new BNConfigParser(bais);
		try {
	    network = parser.Configuration();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	//	network = fillPriorProbabilities(network);

		return network;
	}
	
	public static BayesianNetwork constructNetwork() {
		return constructNetwork(configurationFile);
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
				StringValue val1a = (StringValue) pair1.featvalue;
				StringValue val1b = (StringValue) corr.value1;
				StringValue val2a = (StringValue) pair2.featvalue;
				StringValue val2b = (StringValue) corr.value2;
				if (val1a.val.equals(val1b.val) && val2a.val.equals(val2b.val)) {
					log("correlation found between ("+ pair1.featlabel+"="+val1a.val+") and (" + pair2.featlabel + "=" + val2a.val + "): " + corr.condProb);
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
	
	public static void main (String[] args) {
		BayesianNetworkUtils.constructNetwork();
	} 
	
}
