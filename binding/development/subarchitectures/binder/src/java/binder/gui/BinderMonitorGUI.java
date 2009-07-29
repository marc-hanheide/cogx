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
import binder.utils.GradientDescent;
import binder.components.BinderMonitor;

import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.view.mxGraph;

public class BinderMonitorGUI extends JFrame
{

	int DEFAULT_ENTITY_BOX_WIDTH= 200;
	int DEFAULT_ENTITY_BOX_HEIGHT= 80;
	int MIN_FEATURE_BOX_HEIGHT= 8;
	int FEATURELINE_HEIGHT= 12;

	mxGraph graph;
	Object parent;


	int curProxyPosition_X= 50;
	int curProxyPosition_Y = 400;	

	int curUnionPosition_X = 120;
	int curUnionPosition_Y= 100;
	
	ControlPanel controlPanel;
	BinderMonitor bm;

	public boolean LOGGING = false;

	HashMap<Proxy,Object> insertedProxies;
	HashMap<Union,Object> insertedUnions;

	Vector<Object> insertedObjects ;

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
		
		insertedProxies = new HashMap<Proxy, Object>();
		insertedUnions = new HashMap<Union, Object>();

		insertedObjects = new Vector<Object>();
		
		setVisible(true);
	}
	
	
	public void resetPositions() {
		curProxyPosition_X= 100;
		curProxyPosition_Y = 600;	

		curUnionPosition_X = 250;
		curUnionPosition_Y= 300;
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
		AVM.put("Max prob", ""+ roundProb(GradientDescent.getMaximum(proxy.distribution)));
		
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
		AVM.put("Max prob", ""+ GradientDescent.getMaximum(union.distribution));

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
		height += FEATURELINE_HEIGHT * feat.alternativeValues.length;
		return height;
	}

	private int computeHeight(PerceivedEntity entity) {
		int height = DEFAULT_ENTITY_BOX_HEIGHT;
		if (entity.features != null) {
			for (int i = 0; i < entity.features.length; i++) {
				Feature fd = entity.features[i];
				height += computeHeight(fd) + 10;
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
		return text;
	}


	public void addNewProxy(Proxy proxy) {

		int width = DEFAULT_ENTITY_BOX_WIDTH;
		int height =  computeHeight(proxy);

		String text = createProxyText(proxy);
		Object vertex = graph.insertVertex(parent, null, text, curProxyPosition_X, curProxyPosition_Y, width, height);
		Object[] object_proxy = new Object[1];
		object_proxy[0] = vertex;
		graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, object_proxy);
		graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_TOP, object_proxy);
		graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", object_proxy);
		graph.setCellStyles(mxConstants.STYLE_SHADOW, "true", object_proxy);
		graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "10", object_proxy);

		insertedProxies.put(proxy, vertex);
		insertedObjects.add(vertex);
		
		addNewFeatures(proxy.features, vertex, width, "");
		curProxyPosition_X += 250;

		log("Visual representation of the proxy sucessfully inserted in the GUI");

	}


	public void addNewUnion(Union union) {

		int width = DEFAULT_ENTITY_BOX_WIDTH;
		int height =  computeHeight(union);
		String colour = "#8DD19A";

		String text = createUnionText(union);
		Object union_vertex = graph.insertVertex(parent, null, text, curUnionPosition_X, curUnionPosition_Y, width, height);
		Object[] object_union = new Object[1];
		object_union[0] = union_vertex;
		graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, colour, object_union);
		graph.setCellStyles(mxConstants.STYLE_ROUNDED, "true", object_union);
		graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, object_union);
		graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_TOP, object_union);
		graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", object_union);
		graph.setCellStyles(mxConstants.STYLE_SHADOW, "true", object_union);
		graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "10", object_union);

		insertedUnions.put(union, union_vertex);
		
		insertedObjects.add(union_vertex);
		
		addNewFeatures(union.features, union_vertex, width, colour);

		for (int i = 0; i < union.includedProxies.length ; i++) {
			Proxy p = union.includedProxies[i];
			if (insertedProxies.containsKey(p)) {
				Object proxy_vertex = insertedProxies.get(p);
				Object edge = graph.insertEdge(parent, null, "includes", union_vertex, proxy_vertex);
				insertedObjects.add(edge);
			}
		}

		curUnionPosition_X += 220;

		log("Visual representation of the union successfully inserted in the GUI");
	}

	
	
	public void deleteUnion(Union union) {
		if (insertedUnions.containsKey(union)) {
			Object vertex = insertedUnions.get(union);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
			curUnionPosition_X -= 40;
		}
	}
	
	
	
	public void deleteProxy(Proxy proxy) {
		if (insertedUnions.containsKey(proxy)) {
			Object vertex = insertedProxies.get(proxy);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
		}
	}

	public void updateGUI(Vector<Proxy> newProxies, Vector<Union> newUnions, Vector<Proxy> proxiesToDelete, Vector<Union> unionsToDelete) {

		try {
	//		Object[] cellsToRemove = new Object[insertedObjects.size()];
	//		cellsToRemove = insertedObjects.toArray(cellsToRemove);
	//		graph.removeCells(cellsToRemove);
	//		insertedProxies = new HashMap<Proxy,Object>();
	//		insertedObjects = new Vector<Object>();
	//		resetPositions();
			
			graph.getModel().beginUpdate();
			
			
			for (Enumeration<Proxy> e = proxiesToDelete.elements(); e.hasMoreElements();) {
				Proxy proxy = e.nextElement();
					deleteProxy(proxy);
			}
			
			Thread.sleep(20);
			
			for (Enumeration<Union> e = unionsToDelete.elements(); e.hasMoreElements();) {
				Union union = e.nextElement();
					deleteUnion(union);
			}
			
			for (Enumeration<Proxy> e = newProxies.elements(); e.hasMoreElements();) {
				Proxy proxy = e.nextElement();
				if (!insertedProxies.containsKey(proxy)) {
					addNewProxy(proxy);
				}
			}
			
			Thread.sleep(20);
			
			for (Enumeration<Union> e = newUnions.elements(); e.hasMoreElements();) {
				Union union = e.nextElement();
				if (!insertedUnions.containsKey(union)) {
					addNewUnion(union);
				}
			}


		}
		
		catch (Exception e) {
			e.printStackTrace();
		}
 
		finally
		{
			graph.getModel().endUpdate();
		}

		mxGraphComponent graphComponent = new mxGraphComponent(graph);
		getContentPane().add(graphComponent);

		setVisible(true);
	}


	public void addNewFeatures(Feature[] features, Object vertex, int width, String colour) {

		if (features != null) {

			Object[] objects_feats = new Object[features.length];

			int curYPosition = DEFAULT_ENTITY_BOX_HEIGHT;
			for (int i = 0; i < features.length; i++) {
				Feature feat = features[i];
				String featureText = createFeatureText(feat);
				log("feature text for " + i + ": " + featureText);
				int featHeight= computeHeight(feat);
				objects_feats[i] = graph.insertVertex(vertex, null, featureText, 6, curYPosition, width - 20, featHeight);
				curYPosition += featHeight + 10;
				insertedObjects.add(objects_feats[i]);
			}

			graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, colour, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_MIDDLE, objects_feats);
			graph.setCellStyles(mxConstants.STYLE_FONTFAMILY, "Monaco", objects_feats);
			graph.setCellStyles(mxConstants.STYLE_SHADOW, "false", objects_feats);
			graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "9", objects_feats);
		}		

		log("Visual representation of features sucessfully inserted");
	}

	private void log(String s) {
		if (LOGGING)
			System.out.println("[BinderMonitorGUI]" + s);
	}
}

