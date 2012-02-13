package binder.gui;

import java.awt.BorderLayout;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.Vector;

import javax.swing.JFrame;


import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.StringValue;
import binder.autogen.core.Union;
import binder.autogen.distributions.FeatureValuePair;
import binder.utils.GradientDescent;
import binder.utils.ProbDistribUtils;
import binder.components.BinderMonitor;

import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.view.mxGraph;

public class BinderMonitorGUI extends JFrame
{

	int DEFAULT_ENTITY_BOX_WIDTH= 220;
	int DEFAULT_ENTITY_BOX_HEIGHT= 80;
	int MIN_FEATURE_BOX_HEIGHT= 6;
	int FEATURELINE_HEIGHT= 12;

	mxGraph graph;
	Object parent;

	int curProxyPosition_X= 50;
	int curProxyPosition_Y = 500;	

	int curUnionPosition_X = 120;
	int curUnionPosition_Y= 150;
	
	ControlPanel controlPanel;
	BinderMonitor bm;
	public boolean LOGGING = false;

	HashMap<String,Object> insertedProxies;
	HashMap<String,Object> insertedUnions;
	
	HashMap<String, String> unionForProxy;
	
	Vector<Object> insertedObjects ;
	
	Vector<Union> relationUnions;

	public BinderMonitorGUI(BinderMonitor bm) {
		this.bm = bm;
		init();
	}


	public void init(){		
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(1000, 700);

		controlPanel = new ControlPanel(bm);
		getContentPane().add(controlPanel, BorderLayout.LINE_END);

		graph = new mxGraph();
		parent = graph.getDefaultParent();
		
		insertedProxies = new HashMap<String, Object>();
		insertedUnions = new HashMap<String, Object>();
		
		unionForProxy = new HashMap<String, String>();
		
		insertedObjects = new Vector<Object>();
		
		relationUnions = new Vector<Union>();
		
		setVisible(true);
	}
	
	

	private int getMaximumLength(Set<String> featureLabels) {
		int max = 0;
		
		log("size: " + featureLabels.size());
		
		for (Iterator<String> e = featureLabels.iterator() ; e.hasNext() ; ) {
			String featLabel = e.next();
			log("featlabel: " + featLabel);
			if (featLabel.length() > max) {
				max = featLabel.length();
			}
		}
		return max;
	}

	private String createSpace(int nbSpaces) {
		String space = "";
		for (int i = 0; i < nbSpaces; i ++) {
			space += " ";
		}
		return space;
	}


	private double roundProb (float prob) {
		return Math.round(prob*100.0) / 100.0;
	}
	
	private String createProxyText(Proxy proxy) {


		HashMap<String,String> AVM = new HashMap<String,String>();
		AVM.put("Proxy ID", proxy.entityID);
		AVM.put("P(exists|z)", ""+roundProb(proxy.probExists));
		AVM.put("Subarchitecture", proxy.subarchId);
		AVM.put("Max prob", ""+ roundProb(GradientDescent.getMaximum(proxy)));
		
		int maxLength = getMaximumLength(AVM.keySet());

		String text = "";
		for (Iterator<String> e = AVM.keySet().iterator() ; e.hasNext() ; ) {
			String key = e.next();
			text += "\n " + key +  ": "  + createSpace(maxLength - key.length()) + AVM.get(key);
		}
		return text;
	}




	private String createUnionText(Union union) {


		HashMap<String,String> AVM = new HashMap<String,String>();
		AVM.put("Union ID", union.entityID);
		AVM.put("P(exists|Z)", ""+roundProb(union.probExists));
		AVM.put("Nb. included proxies", "" + union.includedProxies.length);
		AVM.put("Max prob", ""+ GradientDescent.getMaximum(union));

		int maxLength = getMaximumLength(AVM.keySet());

		String text = "";
		for (Iterator<String> e = AVM.keySet().iterator() ; e.hasNext() ; ) {
			String key = e.next();
			text += "\n " + key +  ": "  + createSpace(maxLength - key.length()) + AVM.get(key);
		}
		return text;
	}




	private int computeHeight(Feature feat) {
		int height = MIN_FEATURE_BOX_HEIGHT;
		for (int i = 0 ; i < feat.alternativeValues.length ; i++) {
			if (!((StringValue)feat.alternativeValues[i]).val.equals("indeterminate")) {
			height += FEATURELINE_HEIGHT;
		}
		}
		return height;
	}

	private int computeHeight(PerceivedEntity entity) {
		int height = DEFAULT_ENTITY_BOX_HEIGHT;
		if (entity.features != null) {
			for (int i = 0; i < entity.features.length; i++) {
				Feature fd = entity.features[i];
				if (!fd.featlabel.equals("source") && !fd.featlabel.equals("target")) {
				height += computeHeight(fd) + 10;
				}
			}
		}
		return height;
	}

	private String createFeatureText(Feature feat) {
		HashMap<String,FeatureValue[]> AVM_f = new HashMap<String,FeatureValue[]>();

		AVM_f.put(feat.featlabel, feat.alternativeValues );

		int maxLength = getMaximumLength(AVM_f.keySet());

		String text = "";
		for (Iterator<String> e = AVM_f.keySet().iterator() ; e.hasNext() ; ) {
			String key = e.next();

			FeatureValue[] featvalues = AVM_f.get(key);

			for (int i = 0; i < featvalues.length ; i ++) {
		
				FeatureValue featvalue = featvalues[i];
				if (!((StringValue)featvalue).val.equals("indeterminate")) {
				double roundedProb = Math.round(featvalue.independentProb*100.0) / 100.0;
				if (i==0) {
					text += " " + key +  ": ";
				}
				else {
					text += "\n " + createSpace(key.length() + 2);
				}
				text += createSpace(maxLength - key.length()) + 
				((StringValue)featvalue).val + " (prob = " + roundedProb + ") " ;
				}
			}
		}
		return text;
	}


	public void addNewProxy(Proxy proxy) {
		addNewProxy(proxy, curProxyPosition_X, curProxyPosition_Y, "");
		curProxyPosition_X += 250;
	}
	
	

	public void addNewProxy(Proxy proxy, int xpos, int ypos, String colour) {

		int width = DEFAULT_ENTITY_BOX_WIDTH;
		int height =  computeHeight(proxy);

		String text = createProxyText(proxy);
		Object vertex = graph.insertVertex(parent, null, text, xpos, ypos, width, height);
		Object[] object_proxy = new Object[1];
		object_proxy[0] = vertex;
		
		graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, colour, object_proxy);
		graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, object_proxy);
		graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_TOP, object_proxy);
		graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", object_proxy);
		graph.setCellStyles(mxConstants.STYLE_SHADOW, "true", object_proxy);
		graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "10", object_proxy);

		insertedProxies.put(proxy.entityID, vertex);
		insertedObjects.add(vertex);
		
		addNewFeatures(proxy.features, vertex, width, colour);

		log("Visual representation of the proxy sucessfully inserted in the GUI");

	}


	public void addNewUnionAndIncludedProxies(Union union) {
	
		log("curUnionPosition_X: " + curUnionPosition_X);
		
		int horizontalIncr = 0;

		for (int i = 0; i < union.includedProxies.length ; i++) {
			log("included proxy: " + i);
			Proxy proxy = union.includedProxies[i];
			if (insertedProxies.containsKey(proxy.entityID)) {
				horizontalIncr += 250;
			}
		}		
		
		for (int i = 0; i < union.includedProxies.length ; i++) {
			log("included proxy: " + i);
			Proxy proxy = union.includedProxies[i];
			if (!insertedProxies.containsKey(proxy.entityID)) {
				
				if (isRelationProxy(proxy)) {
					addNewRelationProxy(proxy, Math.abs(curUnionPosition_X - (DEFAULT_ENTITY_BOX_WIDTH + 100)));
				}
				else {
					addNewProxy(proxy, curUnionPosition_X + horizontalIncr, curProxyPosition_Y, "");
					horizontalIncr += 250;
				}
				
			
			unionForProxy.put(proxy.entityID, union.entityID);
			
			}
		}
		
		if (isRelationUnion(union)) {
			addNewRelationUnion(union);
		}
		else {
		addNewUnion(union);
		}
		
		curUnionPosition_X += horizontalIncr;
	}
	
	
	
	public void addNewRelationUnion (Union union) {
		
		String colour = "#FFFF66";
		log("before adding union");
		addNewUnion(union, Math.abs(curUnionPosition_X - (DEFAULT_ENTITY_BOX_WIDTH + 100)), Math.abs(curUnionPosition_Y -125), colour);
		Object vertex = insertedUnions.get(union.entityID);
		insertSourceAndTargetEdges(union, vertex);
		if (!relationUnions.contains(union)) {
			relationUnions.add(union);
		}
		log("Visual representation of the relation union successfully inserted in the GUI");
	}
	
	
	public void insertSourceAndTargetEdges(Proxy proxy, Object mothervertex) {
		for (int i = 0; i < proxy.features.length; i++) {
			
			Feature feat = proxy.features[i];
		if (feat.featlabel.equals("source")) {
			for (int j = 0; j < feat.alternativeValues.length ; j++) {
				StringValue v = (StringValue) feat.alternativeValues[j];
				if (insertedProxies.containsKey(v.val)) {
					Object daughter_vertex = insertedProxies.get(v.val);
					Object edge = graph.insertEdge(parent, null, "source", mothervertex, daughter_vertex);
					insertedObjects.add(edge);
				}
			}
		}
		
		if (feat.featlabel.equals("target")) {
			for (int j = 0; j < feat.alternativeValues.length ; j++) {
				StringValue v = (StringValue) feat.alternativeValues[j];
				if (insertedProxies.containsKey(v.val)) {
					Object daughter_vertex = insertedProxies.get(v.val);
					Object edge = graph.insertEdge(parent, null, "target", mothervertex, daughter_vertex);
					insertedObjects.add(edge);
				}
			}
		}
		}
	}
	
	
	public void insertSourceAndTargetEdges(Union union, Object mothervertex) {
		for (int i = 0; i < union.features.length; i++) {
			
			Feature feat = union.features[i];
		if (feat.featlabel.equals("source")) {
			for (int j = 0; j < feat.alternativeValues.length ; j++) {
				StringValue v = (StringValue) feat.alternativeValues[j];
				String unionId = unionForProxy.get(v.val);
				if (insertedUnions.containsKey(unionId)) {
					Object daughter_vertex = insertedUnions.get(unionId);
					Object edge = graph.insertEdge(parent, null, "source", mothervertex, daughter_vertex);
					insertedObjects.add(edge);
				}
				else {
					log("WARNING: " + unionId + " not found in the inserted unions");
				}
			}
		}
		
		if (feat.featlabel.equals("target")) {
			for (int j = 0; j < feat.alternativeValues.length ; j++) {
				StringValue v = (StringValue) feat.alternativeValues[j];
				String unionId = unionForProxy.get(v.val);
				if (insertedUnions.containsKey(unionId)) {
					Object daughter_vertex = insertedUnions.get(unionId);
					Object edge = graph.insertEdge(parent, null, "target", mothervertex, daughter_vertex);
					insertedObjects.add(edge);
				}
				else {
					log("WARNING: " + unionId + " not found in the inserted unions");
				}
			}
		}
		}
	}
	
	
	
	public void addNewUnion(Union union) {
		String colour = "#8DD19A";
		addNewUnion(union, curUnionPosition_X, curUnionPosition_Y, colour);
	}
	
	
	public void addNewUnion(Union union, int xpos, int ypos, String colour) {

	//	union.distribution = ProbDistribUtils.normaliseDistribution(union.distribution, 1.0f);
		
		int width = DEFAULT_ENTITY_BOX_WIDTH;
		int height =  computeHeight(union);
				
		String text = createUnionText(union);
		Object union_vertex = graph.insertVertex(parent, null, text, xpos , ypos, width, height);
		Object[] object_union = new Object[1];
		object_union[0] = union_vertex;
		graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, colour, object_union);
		graph.setCellStyles(mxConstants.STYLE_ROUNDED, "true", object_union);
		graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, object_union);
		graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_TOP, object_union);
		graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", object_union);
		graph.setCellStyles(mxConstants.STYLE_SHADOW, "true", object_union);
		graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "10", object_union);

		insertedUnions.put(union.entityID, union_vertex);
	
		insertedObjects.add(union_vertex);
		
		addNewFeatures(union.features, union_vertex, width, colour);

		for (int i = 0; i < union.includedProxies.length ; i++) {
			Proxy p = union.includedProxies[i];
			if (insertedProxies.containsKey(p.entityID)) {
				Object proxy_vertex = insertedProxies.get(p.entityID);
				Object edge = graph.insertEdge(parent, null, "includes", union_vertex, proxy_vertex);
				Object[] object_edges = new Object[1];
				object_edges[0] = edge;
				graph.setCellStyles(mxConstants.STYLE_DASHED, "true", object_edges);
				insertedObjects.add(edge);
			}
		}

		log("Visual representation of the union successfully inserted in the GUI");
	}

	
	
	public void deleteUnion(Union union) {
		if (insertedUnions.containsKey(union.entityID)) {
			Object vertex = insertedUnions.get(union.entityID);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
			if (curUnionPosition_X > (220 * union.includedProxies.length))
				curUnionPosition_X -= (220 * union.includedProxies.length) ;
		}
		insertedUnions.remove(union.entityID);
	}
	
	
	
	public void deleteProxy(Proxy proxy) {
		if (insertedProxies.containsKey(proxy.entityID)) {
			Object vertex = insertedProxies.get(proxy.entityID);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
			insertedProxies.remove(proxy.entityID);
		}
	}

	
	public void updateGUI (Proxy updatedProxy) {
		try {

			graph.getModel().beginUpdate();

			deleteProxy(updatedProxy);

			if (unionForProxy.containsKey(updatedProxy.entityID)) {
				Object unionVertex = insertedUnions.get(unionForProxy.get(updatedProxy.entityID));

				addNewProxy(updatedProxy);

				Object proxy_vertex = insertedProxies.get(updatedProxy.entityID);
				Object edge = graph.insertEdge(parent, null, "includes", unionVertex, proxy_vertex);
				insertedObjects.add(edge);
			}
		}
		finally {
			graph.getModel().endUpdate();
		}

		try {
			mxGraphComponent graphComponent = new mxGraphComponent(graph);
			getContentPane().add(graphComponent);
			setVisible(true);
		}
		catch (Exception e) {		}
	}

	
	
	 public void updateGUI(Vector<Proxy> newProxies, 
			Vector<Union> newUnions, 
			Vector<Proxy> proxiesToDelete, 
			Vector<Union> unionsToDelete) {
		 
		try {
			graph.getModel().beginUpdate();
			
			for (Enumeration<Proxy> e = proxiesToDelete.elements(); e.hasMoreElements();) {
				Proxy proxy = e.nextElement();
					deleteProxy(proxy);
					log("Proxy deleted");
			}
						
			for (Enumeration<Union> e = unionsToDelete.elements(); e.hasMoreElements();) {
				Union union = e.nextElement();
					deleteUnion(union);
					log("Union deleted");
			}
			
			boolean mustRegenerateRelationUnions = false;
			for (Enumeration<Union> e = newUnions.elements(); e.hasMoreElements();) {
				Union union = e.nextElement();
				if (!insertedUnions.containsKey(union.entityID)) {
					log("Adding new union..." + union.entityID);
					for (int i = 0 ; i < union.includedProxies.length ; i++) {
						if (unionForProxy.containsKey(union.includedProxies[i].entityID)) {
							mustRegenerateRelationUnions = true;
						}
						unionForProxy.put(union.includedProxies[i].entityID, union.entityID);
					}
					addNewUnionAndIncludedProxies(union);
				}
			}
		
			if (mustRegenerateRelationUnions) {
			for (Enumeration<Union> e = relationUnions.elements(); e.hasMoreElements();) {
				Union ru = e.nextElement();
				deleteUnion(ru);
				addNewRelationUnion(ru);
			}
			}
		
		}
		
		catch (Exception e) {		}
 
		finally
		{
			graph.getModel().endUpdate();
		}

		try {
		mxGraphComponent graphComponent = new mxGraphComponent(graph);
		getContentPane().add(graphComponent);
		setVisible(true);
		}
		catch (Exception e) {		}
		
	}


	private FeatureValue getMaxFeatureValue (Feature feat) {
		
		float maxProb = 0.0f;
		FeatureValue maxFeatValue = new FeatureValue();
		
		for (int i = 0 ; i < feat.alternativeValues.length ; i++) {
			
			if (feat.alternativeValues[i].independentProb > maxProb) {
				maxProb = feat.alternativeValues[i].independentProb;
				maxFeatValue = feat.alternativeValues[i];
			}
		}
		
		return maxFeatValue;
	}
	
	
	private boolean isRelationProxy (Proxy proxy) {
		
		for (int i = 0; i < proxy.features.length; i++) {
			if (proxy.features[i].featlabel.equals("source")) {
				return true;
				
			}
			if (proxy.features[i].featlabel.equals("target")) {
				return true;
				
			}
		}
		return false;
	}
	
	
	private boolean isRelationUnion (Union union) {
		
		for (int i = 0; i < union.features.length; i++) {
			if (union.features[i].featlabel.equals("source")) {
				return true;
				
			}
			if (union.features[i].featlabel.equals("target")) {
				return true;
				
			}
		}
		return false;
	}
	
	
	public void addNewRelationProxy (Proxy relationProxy, int xpos) {
		
		String colour = "#FFFF99";
		addNewProxy(relationProxy, xpos, Math.abs(curProxyPosition_Y + 200), colour);

		Object vertex = insertedProxies.get(relationProxy.entityID);
		insertSourceAndTargetEdges(relationProxy, vertex);
		log("OK FOR RELATION PROXY");
	}
	
	
	public void addNewFeatures(Feature[] features, Object vertex, int width, String colour) {

		if (features != null) {

			Object[] objects_feats = new Object[features.length];

			int curYPosition = DEFAULT_ENTITY_BOX_HEIGHT;
			for (int i = 0; i < features.length; i++) {
				Feature feat = features[i];
				
			
				if (!feat.featlabel.equals("source") && ! feat.featlabel.equals("target")) {
				
					if (feat.alternativeValues.length == 2) {
						log(((StringValue)feat.alternativeValues[0]).val);
						log("" +((StringValue)feat.alternativeValues[0]).independentProb);
						log(((StringValue)feat.alternativeValues[1]).val);
						log("" + ((StringValue)feat.alternativeValues[1]).independentProb);
					}
				if (!((StringValue)getMaxFeatureValue(feat)).val.equals("indeterminate")) {

					String featureText = createFeatureText(feat);
					log("feature text for " + i + ": " + featureText);
					int featHeight= computeHeight(feat);
					objects_feats[i] = graph.insertVertex(vertex, null, featureText, 6, curYPosition, width - 20, featHeight);
					curYPosition += featHeight + 10;
					insertedObjects.add(objects_feats[i]);
				}
				}
			}

			graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, colour, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_MIDDLE, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", objects_feats);
			graph.setCellStyles(mxConstants.STYLE_SHADOW, "false", objects_feats);
			graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "8", objects_feats);
		}		

		log("Visual representation of features sucessfully inserted");
	}

	private void log(String s) {
		if (LOGGING)
			System.out.println("[BinderMonitorGUI]" + s);
	}
}
