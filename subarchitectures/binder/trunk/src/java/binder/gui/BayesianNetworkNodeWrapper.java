package binder.gui;

import java.util.LinkedList;
import java.util.List;
import java.util.TreeMap;
import java.util.TreeSet;

import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.core.FeatureValue;
import binder.gui.BayesianNetworkEdgeWrapper;
import binder.utils.FeatureValueUtils;



class BayesianNetworkNodeWrapper implements Comparable<Object> {
	
	public TreeSet<BayesianNetworkEdgeWrapper> getEdgesOutgoing() {
		return edges_outgoing;
	}

	public void removeFeatureAlternative(String featureAlternative) throws Exception{
		if(!alternative_values.containsKey(featureAlternative)) {
			throw new Exception("[REMOVEFEATUREALTERNATIVE] Feature alternative does not exist!");
		}
		this.alternative_values.remove(featureAlternative);
		
		for(BayesianNetworkEdgeWrapper edge : this.getEdgesOutgoing()) {
			for(ConditionedAlternative alt : edge.getConditionedAlternatives()) {
				if(alt.getAlternativeConditioned().equals(featureAlternative)) {
					edge.removeFeatureConditioned(alt);
				}
			}
		}
		for(BayesianNetworkEdgeWrapper edge : this.getEdgesIncoming()) {
			for(ConditionedAlternative alt : edge.getConditionedAlternatives()) {
				if(alt.getAlternative().equals(featureAlternative)) {
					edge.removeFeatureConditioned(alt);
				}
			}
		}
	}

	public void removeIncomingEdge(BayesianNetworkEdgeWrapper edge_to_be_removed) throws Exception {
		if(!this.edges_incoming.remove(edge_to_be_removed)) {
			throw new Exception("[REMOVEINCOMINGEDGE] Edge not in list of incoming edges!");
		}
	}

	public void removeOutgoingEdge(BayesianNetworkEdgeWrapper edge_to_be_removed) throws Exception {
		if(!this.edges_outgoing.remove(edge_to_be_removed)) {
			throw new Exception("[REMOVEoutgoingEDGE] Edge not in list of outgoing edges!");
		}
	}

	public TreeSet<BayesianNetworkEdgeWrapper> getEdgesIncoming() {
		return edges_incoming;
	}

	private String feature_label;
	private TreeMap<String, Float> alternative_values;
	private TreeSet<BayesianNetworkEdgeWrapper> edges_outgoing;
	private TreeSet<BayesianNetworkEdgeWrapper> edges_incoming;
	
	public BayesianNetworkNodeWrapper(BayesianNetworkNode node) {
		feature_label = new String(node.feat.featlabel);
		alternative_values = new TreeMap<String, Float>();
		edges_outgoing = new TreeSet<BayesianNetworkEdgeWrapper>();
		edges_incoming = new TreeSet<BayesianNetworkEdgeWrapper>();
		
		for(FeatureValue feature : node.feat.alternativeValues) {
			alternative_values.put(FeatureValueUtils.toString(feature), feature.independentProb);
		}
	}
	
	public BayesianNetworkNodeWrapper(String feature_label) {
		this.feature_label = feature_label;
		alternative_values = new TreeMap<String, Float>();
		edges_outgoing = new TreeSet<BayesianNetworkEdgeWrapper>();
		edges_incoming = new TreeSet<BayesianNetworkEdgeWrapper>();
	}
	
	public void addOutgoingEdge(BayesianNetworkEdgeWrapper edge) {
		this.edges_outgoing.add(edge);
	}
	
	public void addIncomingEdge(BayesianNetworkEdgeWrapper edge) {
		this.edges_incoming.add(edge);
	}
	
	public String getFeatureLabelName() {
		return this.feature_label;
	}
	
	public boolean hasFeatureAlternative(String alternative) {
		return this.alternative_values.containsKey(alternative.toLowerCase());
	}
	
	public void addNewFeatureAlternative(String alternative_name, Float alternative_prob) throws Exception {
		if(alternative_values.containsKey(alternative_name)) {
			throw new Exception("Feature alternative already exists");
		}
		alternative_values.put(alternative_name, alternative_prob);
	}
	
	public void modifyExistingFeatureAlternative(String alternative_name, Float alternative_prob) throws Exception {
		if(!alternative_values.containsKey(alternative_name)) {
			throw new Exception("Feature alternative does not exist");
		}
		alternative_values.put(alternative_name, alternative_prob);
	}
	
	public void deleteExisitngFeatureALternative(String alternative_name) throws Exception {
		if(!alternative_values.containsKey(alternative_name)) {
			throw new Exception("Fetaure alternative does not exist");
		}
		alternative_values.remove(alternative_name);
	}
	
	public int compareTo(Object compare_to) {
		if(compare_to == null) {
			return -1;
		}
		
		if(compare_to instanceof BayesianNetworkNodeWrapper) {
			BayesianNetworkNodeWrapper that = (BayesianNetworkNodeWrapper) compare_to;
			
			return this.feature_label.compareToIgnoreCase(that.feature_label);
		}
		return -1;
	}

	public Float getProbabilityOfAlternative(String alt) {
		return this.alternative_values.get(alt);
	}

	public List<String> getAlternativeNames() {
		return new LinkedList<String>(this.alternative_values.keySet());
	}

	public int getNumberOfAlternatives() {
		return this.alternative_values.size();
	}
}
