package binder.gui;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.Feature;
import binder.autogen.featvalues.StringValue;
import binder.constructors.ProxyConstructor;
import binder.utils.FeatureValueUtils;
import binder.gui.BayesianNetworkEdgeWrapper;
import binder.gui.BayesianNetworkNodeWrapper;
import binder.gui.ConditionedAlternative;

/**
 * Wrapper class for the Bayesian Network which is an ICE datastructure
 * This class provides methods to extend and modify the network structure and 
 * add features to edges and nodes. Furthermore we use efficient datastructures
 * to maintain the network.
 * 
 * @Author Carsten Ehrler
 * @Version 27/01/2010 (started at 27/01/2010)
 */
public class BayesianNetworkGUIWrapper {
	
	private BayesianNetwork bayesian_network;
	private TreeMap<String, BayesianNetworkNodeWrapper> nodes;
	private TreeSet<BayesianNetworkEdgeWrapper> edges;
	
	public BayesianNetworkNodeWrapper getNode(String name) {
		return this.nodes.get(name); 
	}
	
	public BayesianNetworkEdgeWrapper getEdge(String source, String target) {
		for(BayesianNetworkEdgeWrapper edge : this.nodes.get(source).getEdgesOutgoing()) {
			if(edge.getNodeTarget().getFeatureLabelName().equals(target.toLowerCase())) {
				return edge;
			}
		}
		return null;
	}
	
	public List<String> getNodeNames(){
		return new LinkedList<String>(this.nodes.keySet());
	}
	
	public BayesianNetworkGUIWrapper(BayesianNetwork bayesian_network) {
		assert(bayesian_network) != null;
		this.bayesian_network = bayesian_network;
		this.nodes = new TreeMap<String, BayesianNetworkNodeWrapper>();
		this.edges = new TreeSet<BayesianNetworkEdgeWrapper>();
		
		//TreeMap<BayesianNetworkNode, BayesianNetworkNodeWrapper> nodes_to_wrapped_nodes = new TreeMap<BayesianNetworkNode, BayesianNetworkNodeWrapper>();
		
		// add all nodes
		for(BayesianNetworkNode node : bayesian_network.nodes) {
			BayesianNetworkNodeWrapper wrapped_node = new BayesianNetworkNodeWrapper(node);
			nodes.put(wrapped_node.getFeatureLabelName(), wrapped_node);
			//nodes_to_wrapped_nodes.put(node, wrapped_node);
		}
		
		// add all the edges
		for(BayesianNetworkEdge edge : bayesian_network.edges) {
			// 1) first of all we find the outgoing and incoming wrapped edge
			BayesianNetworkNodeWrapper node_source = nodes.get(edge.outgoingNode.feat.featlabel);
			BayesianNetworkNodeWrapper node_target = nodes.get(edge.incomingNode.feat.featlabel);
			
			// 2) we have to catch all the features conditioned by the edge
			TreeSet<ConditionedAlternative> conditioned_alternatives = new TreeSet<ConditionedAlternative>();
			for (FeatureValueCorrelation alternative : edge.correlations) {
				String feature_alternative_source = FeatureValueUtils.toString(alternative.value1);
				String feature_alternative_target = FeatureValueUtils.toString(alternative.value2);
				assert(node_source.hasFeatureAlternative(feature_alternative_source));
				assert(node_target.hasFeatureAlternative(feature_alternative_target));
				
				conditioned_alternatives.add(new ConditionedAlternative(feature_alternative_source,
						feature_alternative_target,
						alternative.condProb));
			}
			
			// 3) now we can set up the wrapped edge and add it to the object
			BayesianNetworkEdgeWrapper wrapped_edge = new BayesianNetworkEdgeWrapper(node_source, node_target, conditioned_alternatives);
			edges.add(wrapped_edge);
			
			// 4) last we add the edge to the nodes
			node_source.addOutgoingEdge(wrapped_edge);
			node_target.addIncomingEdge(wrapped_edge);
		}
	}
	
	public void addEdge(String source, String target) throws Exception {
		BayesianNetworkNodeWrapper node_source = this.nodes.get(source);
		BayesianNetworkNodeWrapper node_target = this.nodes.get(target);
		
		if(node_source == null) {
			throw new Exception("[ADDEDGE] Source node with name " + source + "does not exist!");
		}
		if(node_target == null) {
			throw new Exception("[ADDEDGE] Targer node with name " + source + "does not exist!");
		}
		
		BayesianNetworkEdgeWrapper edge = new BayesianNetworkEdgeWrapper(node_source, node_target, new TreeSet<ConditionedAlternative>());
		
		edges.add(edge);
		
		node_source.addOutgoingEdge(edge);
		node_target.addIncomingEdge(edge);
	}
	
	public void removeEdge(String source, String target) throws Exception {
		BayesianNetworkNodeWrapper node_source = this.nodes.get(source);
		BayesianNetworkNodeWrapper node_target = this.nodes.get(target);
		
		TreeSet<BayesianNetworkEdgeWrapper> outgoing = node_source.getEdgesOutgoing();
		TreeSet<BayesianNetworkEdgeWrapper> incoming = node_target.getEdgesIncoming();
		
		TreeSet<BayesianNetworkEdgeWrapper> intersection = new TreeSet<BayesianNetworkEdgeWrapper>(outgoing);
		intersection.retainAll(incoming);
		
		if(intersection.size() != 1) {
			System.out.println(intersection.size());
			throw new Exception("[REMOVEEDGE] There are several or no edges to remove");
		}
		
		BayesianNetworkEdgeWrapper edge_to_be_removed = intersection.first();
		
		// remove the edge from the nodes and from the edge tree set
		
		edges.remove(edge_to_be_removed);
		
		node_source.removeOutgoingEdge(edge_to_be_removed);
		node_target.removeIncomingEdge(edge_to_be_removed);
	}
	
	public void addNode(String feature_label) throws Exception {
		if(nodes.containsKey(feature_label)) {
			throw new Exception("[ADDNODE] Node already exists");
		}
		nodes.put(feature_label, new BayesianNetworkNodeWrapper(feature_label));
	}
	
	public void removeNode(String feature_label) throws Exception {
		if(!nodes.containsKey(feature_label)) {
			throw new Exception("[REMOVENDOE] Node does not exist");
		}
		BayesianNetworkNodeWrapper removed_node = nodes.get(feature_label);
		
		TreeSet<BayesianNetworkEdgeWrapper> edges_to_be_removed = new TreeSet<BayesianNetworkEdgeWrapper>();
		
		edges_to_be_removed.addAll(removed_node.getEdgesIncoming());
		edges_to_be_removed.addAll(removed_node.getEdgesOutgoing());
		
		for(BayesianNetworkEdgeWrapper edge : edges_to_be_removed) {
			edges.remove(edge);
			edge.getNodeSource().removeOutgoingEdge(edge);
			edge.getNodeTarget().removeIncomingEdge(edge);
		}
		nodes.remove(feature_label);
	}
	
	public void addFeatureAlternative(String feature_label, String feature_alternative, Float probability) throws Exception {
		if(!nodes.containsKey(feature_label)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node does not exist");
		}
		BayesianNetworkNodeWrapper node = nodes.get(feature_label);
		
		if(node.hasFeatureAlternative(feature_alternative)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node has alternative with same name");
		}
		node.addNewFeatureAlternative(feature_alternative, probability);
	}
	
	public void modifyFeatureAlternative(String feature_label, String feature_alternative, Float probability) throws Exception {
		if(!nodes.containsKey(feature_label)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node does not exist");
		}
		BayesianNetworkNodeWrapper node = nodes.get(feature_label);
		
		if(!node.hasFeatureAlternative(feature_alternative)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node has no alternative with name");
		}
		node.modifyExistingFeatureAlternative(feature_alternative, probability);
	}
	
	public void removeFeatureAlternative(String feature_label, String feature_alternative) throws Exception {
		if(!nodes.containsKey(feature_label)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node does not exist");
		}
		BayesianNetworkNodeWrapper node = nodes.get(feature_label);
		
		if(!node.hasFeatureAlternative(feature_alternative)) {
			throw new Exception("[ADDFEATUREALTERNATIVE] Node has no alternative with name");
		}
		node.removeFeatureAlternative(feature_alternative);
	}
	 
	public void addFeatureConditioned(String source, String target, 
			String alternative, String alternative_conditioned, Float probability) throws Exception {
		BayesianNetworkNodeWrapper node_source = this.nodes.get(source);
		BayesianNetworkNodeWrapper node_target = this.nodes.get(target);
		
		if(!node_source.hasFeatureAlternative(alternative_conditioned)) {
			throw new Exception("[ADDFEATURECONDITIONED] Source has no such alternative");
		}
		
		if(!node_target.hasFeatureAlternative(alternative)) {
			throw new Exception("[ADDFEATURECONDITIONED] Target has no such alternative");
		}
		
		for(BayesianNetworkEdgeWrapper edge : this.edges) {
			if(edge.getNodeSource() == node_source && edge.getNodeTarget() == node_target) {
				edge.addFeatureConditioned(new ConditionedAlternative(alternative_conditioned, alternative, probability));
			} 
		}
	}
	
	public void removeFeatureConditioned(String source, String target, 
			String alternative, String alternative_conditioned) throws Exception {
		BayesianNetworkNodeWrapper node_source = this.nodes.get(source);
		BayesianNetworkNodeWrapper node_target = this.nodes.get(target);
		
		if(!node_source.hasFeatureAlternative(alternative_conditioned)) {
			throw new Exception("[ADDFEATURECONDITIONED] Source has no such alternative");
		}
		
		if(!node_target.hasFeatureAlternative(alternative)) {
			throw new Exception("[ADDFEATURECONDITIONED] Target has no such alternative");
		}
		
		for(BayesianNetworkEdgeWrapper edge : this.edges) {
			if(edge.getNodeSource() == node_source && edge.getNodeTarget() == node_target) {
				edge.removeFeatureConditioned(new ConditionedAlternative(alternative_conditioned, alternative, new Float(0)));
			}
		}
	}
	
	/*
	 * --------------------------------------------------------------------------------------------------
	 */
	
	public List<String> getTargetNames(String name_source) {
		List<String> list = new LinkedList<String>();
		for(BayesianNetworkEdgeWrapper edge : this.nodes.get(name_source).getEdgesOutgoing()) {
			list.add(edge.getNodeTarget().getFeatureLabelName());
		}
		return list;
	}
	
	/*
	 * here we generate a new bayesian network from the wrapped model
	 */
	public BayesianNetwork getBayesianNetwork() {
		Map<BayesianNetworkNodeWrapper, BayesianNetworkNode> wrapped_to_node = 
			new TreeMap<BayesianNetworkNodeWrapper, BayesianNetworkNode>();
		Map<BayesianNetworkNodeWrapper, Map<String, StringValue>> node_to_val = 
			new TreeMap<BayesianNetworkNodeWrapper, Map<String, StringValue>>();
		
		BayesianNetwork network = new BayesianNetwork();
		assert network != null;
		network.nodes = new BayesianNetworkNode[this.nodes.size()];
		network.edges = new BayesianNetworkEdge[this.edges.size()];
		
		int i = 0;
		for(BayesianNetworkNodeWrapper node : this.nodes.values()) {
			BayesianNetworkNode n = new BayesianNetworkNode();
			n.feat = new Feature();
			n.feat.featlabel = node.getFeatureLabelName();
			n.feat.alternativeValues = new StringValue[node.getNumberOfAlternatives()];
			network.nodes[i] = n;
			
			Map<String, StringValue> values = new TreeMap<String, StringValue>();
			
			int feat = 0;
			for(String feature_alternative : node.getAlternativeNames()) {
				StringValue value = ProxyConstructor.createStringValue(feature_alternative, node.getProbabilityOfAlternative(feature_alternative));
				network.nodes[i].feat.alternativeValues[feat] = value;
				values.put(feature_alternative, value);
				feat += 1;
			}
			wrapped_to_node.put(node, n);
			node_to_val.put(node, values);
			
			i += 1;
		}
		
		i = 0;
		for(BayesianNetworkEdgeWrapper edge : this.edges) {
			network.edges[i] = new BayesianNetworkEdge();
			network.edges[i].incomingNode = wrapped_to_node.get(edge.getNodeTarget());
			network.edges[i].outgoingNode = wrapped_to_node.get(edge.getNodeSource());
			network.edges[i].correlations = new FeatureValueCorrelation[edge.getNumberOfAlternatives()];
			
			int feat = 0;
			for(ConditionedAlternative corr : edge.getConditionedAlternatives()) {
				FeatureValueCorrelation f = new FeatureValueCorrelation();
				f.value1 = node_to_val.get(edge.getNodeSource()).get(corr.getAlternativeConditioned());
				f.value2 = node_to_val.get(edge.getNodeTarget()).get(corr.getAlternative());
				f.condProb = corr.getConditionalProbability();
				network.edges[i].correlations[feat] = f;
				feat += 1;
			}
		}
		return network;
	}
	
	public String getSerializedNetwork() {
		StringBuilder output = new StringBuilder();
		
		// part one the nodes:
		output.append("nodes {\n");
		
		for(BayesianNetworkNodeWrapper node : nodes.values()) {
			output.append("\t" + node.getFeatureLabelName() + " {\n");
			
			output.append("\t\tfeaturevalues {\n");
			for(String feature_alterative : node.getAlternativeNames()) {
				output.append("\t\t\t" + feature_alterative + ",\n");
			}
			removeTrailingComma(output);
			output.append("\t\t}\n");
			
			output.append("\t\tfeatureprobs {\n");
			for(String feature_alterative : node.getAlternativeNames()) {
				output.append("\t\t\tP(" + node.getFeatureLabelName() + "=" + 
						feature_alterative + ")=" + 
						node.getProbabilityOfAlternative(feature_alterative) + 
						",\n");
			}
			removeTrailingComma(output);
			output.append("\t\t}\n");
			
			output.append("\t}\n");
		}
		
		output.append("}\n");
		output.append("edges {\n");
		for(BayesianNetworkEdgeWrapper edge : edges) {
			output.append("\t" + edge.getNodeSource().getFeatureLabelName() + 
					" -> " + edge.getNodeTarget().getFeatureLabelName() + " {\n");
			for(ConditionedAlternative corr : edge.getConditionedAlternatives()) {
				output.append("\t\tP(" + edge.getNodeTarget().getFeatureLabelName() +
						"=" + corr.getAlternative() + "|" +
						edge.getNodeSource().getFeatureLabelName() +
						"=" + corr.getAlternativeConditioned() +
						")=" + corr.getConditionalProbability() + ",\n");
			}
			removeTrailingComma(output);
			output.append("\t}\n");
		}
		output.append("}\n");
		return output.toString();
	}

	private void removeTrailingComma(StringBuilder output) {
		if(output.substring(output.length()-2).equals(",\n")) {
			output.replace(output.length()-2, output.length(), "\n");
		}
	}
}
