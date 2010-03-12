package binder.gui;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Set;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;

import org.apache.log4j.Logger;

import binder.abstr.BindingPredictor;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy;
import binder.autogen.specialentities.RelationUnion;
import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.components.BinderMonitor;
import binder.utils.FeatureValueUtils;
import binder.utils.GenericUtils;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;

import com.mxgraph.model.mxGraphModel.mxValueChange;
import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.util.mxEventSource.mxIEventListener;
import com.mxgraph.view.mxGraph;


import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.swing.handler.mxKeyboardHandler;
import com.mxgraph.swing.handler.mxRubberband;
import com.mxgraph.util.mxEvent;
import com.mxgraph.util.mxEventObject;
import com.mxgraph.util.mxUtils;
import com.mxgraph.util.mxEventSource.mxIEventListener;
import com.mxgraph.view.mxGraph;
import com.mxgraph.view.mxMultiplicity;


public class BindingVisualizer {


	// No idea what that is
	private static final long serialVersionUID = 1L;

	private static final long initTimestamp = System.currentTimeMillis();

	int DEFAULT_ENTITY_BOX_WIDTH= 220;
	int DEFAULT_ENTITY_BOX_HEIGHT= 80;
	int MIN_FEATURE_BOX_HEIGHT= 6;
	int FEATURELINE_HEIGHT= 12;

	mxGraph graph;
	Object parent;
	
	BinderGUI frame;

	int curProxyPosition_X= 50;
	int curProxyPosition_Y = 500;	

	int curUnionPosition_X = 120;
	int curUnionPosition_Y= 150;

	BinderMonitor bm;

	public boolean LOGGING = true;

	private static Logger logger = ComponentLogger.getLogger(BindingVisualizer.class);

	HashMap<String, CASTTime> proxyTimestamps;
	
	HashMap<String,Object> insertedCellsForProxies;
	HashMap<String,Object> insertedCellsForUnions;

	Vector<Object> insertedCells ;

	Vector<RelationUnion> relationUnions;

	Object[] unionConfigInfo;

	mxIEventListener listener;


	public BindingVisualizer(BinderMonitor bm) {
		this.bm = bm; 
		this.frame = new BinderGUI(bm);
		init();
	}




	public void selection () {
		String curSelectedEntityId = "";
		Class curSelectedType = null ;
		Object selected = graph.getSelectionCell();
		if (selected != null) {
			log(selected.toString());
			for (Iterator<String> ids = insertedCellsForProxies.keySet().iterator(); ids.hasNext();) {
				String curId = ids.next();
				Object curObj = insertedCellsForProxies.get(curId);
				if (curObj.equals(selected)) {
					curSelectedEntityId = curId;
					curSelectedType = Proxy.class;
				}
			}
			
			for (Iterator<String> ids = insertedCellsForUnions.keySet().iterator(); ids.hasNext();) {
				String curId = ids.next();
				Object curObj = insertedCellsForUnions.get(curId);
				if (curObj.equals(selected)) {
					curSelectedEntityId = curId;
					curSelectedType = Union.class;
				}
			}
		}

		frame.setCurrentSelection(curSelectedEntityId, curSelectedType);
	}


	public void init(){		

		graph = new mxGraph();

		listener = new mxIEventListener()
		{
			public void invoke(Object sender, mxEventObject evt)
			{
				selection();
			}
		};
		graph.getSelectionModel().addListener(mxEvent.CHANGE, listener);
		
		parent = graph.getDefaultParent();

		proxyTimestamps = new HashMap<String, CASTTime>();
		
		insertedCellsForProxies = new HashMap<String, Object>();
		insertedCellsForUnions = new HashMap<String, Object>();

		insertedCells = new Vector<Object>();

		relationUnions = new Vector<RelationUnion>();
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


	private String formatTimpestamp (CASTTime timestamp) {
		return "" + timestamp.s + "." + timestamp.us;
	}


	private String createProxyText(Proxy proxy) {


		HashMap<String,String> AVM = new HashMap<String,String>();
		AVM.put("Proxy ID", proxy.entityID);
		AVM.put("P(exists|z)", ""+roundProb(proxy.probExists));
		AVM.put("Subarchitecture", proxy.origin.address.subarchitecture);
		//	AVM.put("Max prob", ""+ roundProb(GradientDescent.getMaximum(proxy)));
		AVM.put("Timestamp", "+" + formatTimpestamp(proxy.timeStamp) + " s.");

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
		//	AVM.put("Max prob", ""+ GradientDescent.getMaximum(union));
		//	AVM.put("Conf. score", ""+ union.confidenceScore);
		AVM.put("Timestamp", "+" + formatTimpestamp(union.timeStamp) + " s.");

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
			//		if (!FeatureValueUtils.isUnknownValue(feat.alternativeValues[i])) {
			height += FEATURELINE_HEIGHT;
		}
		//	}
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
				//		if (!FeatureValueUtils.isUnknownValue(featvalue)) {
				double roundedProb = Math.round(featvalue.independentProb*100.0) / 100.0;
				if (i==0) {
					text += " " + key +  ": ";
				}
				else {
					text += "\n " + createSpace(key.length() + 2);
				}
				text += createSpace(maxLength - key.length()) + 
				FeatureValueUtils.toString(featvalue) + " (prob = " + roundedProb + ") " ;
				//		}
			}
		}
		return text;
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

		if (containsSaliencyFeat(proxy)) {
			graph.setCellStyles(mxConstants.STYLE_STROKECOLOR, "red", object_proxy);
		}
		
		insertedCellsForProxies.put(proxy.entityID, vertex);
		insertedCells.add(vertex);

		addNewFeatures(proxy.features, vertex, width, colour);

		log("Visual representation of the proxy sucessfully inserted in the GUI");

	}

	public void addNewUnionAndIncludedProxies(Union union) {

		log("curUnionPosition_X: " + curUnionPosition_X);

		int horizontalIncr = 0;

		for (int i = 0; i < union.includedProxies.length ; i++) {
			log("included proxy: " + i);
			Proxy proxy = union.includedProxies[i];
			if (insertedCellsForProxies.containsKey(proxy.entityID)) {
				horizontalIncr += 250;
			}
		}		

		for (int i = 0; i < union.includedProxies.length ; i++) {
			log("included proxy: " + i);
			Proxy proxy = union.includedProxies[i];
			
			if (!proxyTimestamps.containsKey(proxy.entityID)) {

				addNewProxy(proxy, horizontalIncr);
			}
			else {
				CASTTime timestamp1 = proxy.timeStamp;
				CASTTime timestamp2 = proxyTimestamps.get(proxy.entityID);

				if (GenericUtils.isMoreRecent(timestamp1, timestamp2)) {
					
					deleteProxy(proxy);
					addNewProxy(proxy, horizontalIncr);
					
				}
			}
		}

		if (union instanceof RelationUnion) {
			addNewRelationUnion((RelationUnion)union);
		}
		else {
			addNewUnion(union);
		}

		curUnionPosition_X += horizontalIncr;
	}

	
	public void addNewProxy(Proxy proxy, int horizontalIncr) {
		if (proxy instanceof RelationProxy) {
			addNewRelationProxy((RelationProxy) proxy, Math.abs(curUnionPosition_X - (DEFAULT_ENTITY_BOX_WIDTH + 100)));
		}
		else if (proxy instanceof PhantomProxy) {
		if (!BindingPredictor.lastPhantomProxyToDelete) {
				addNewPhantomProxy((PhantomProxy) proxy, curUnionPosition_X + horizontalIncr);
				horizontalIncr += 250;
			}
		}
		else {
			addNewProxy(proxy, curUnionPosition_X + horizontalIncr, curProxyPosition_Y, "");
			horizontalIncr += 250;
		}
		
		proxyTimestamps.put(proxy.entityID, proxy.timeStamp);
	}


	
	
	public void addNewRelationUnion (RelationUnion union) {

		String colour = "#FFFF66";
		log("before adding union");
		addNewUnion(union, Math.abs(curUnionPosition_X - (DEFAULT_ENTITY_BOX_WIDTH + 100)), Math.abs(curUnionPosition_Y -125), colour);
		Object vertex = insertedCellsForUnions.get(union.entityID);
		insertSourceAndTargetEdges(union, vertex);
		if (!relationUnions.contains(union)) {
			relationUnions.add(union);
		}
		log("Visual representation of the relation union successfully inserted in the GUI");
	}


	public void insertSourceAndTargetEdges(RelationProxy proxy, Object mothervertex) {

		if (proxy.source != null) {
			for (int j = 0; j < proxy.source.alternativeValues.length ; j++) {
				AddressValue v = (AddressValue) proxy.source.alternativeValues[j];
				if (insertedCellsForProxies.containsKey(v.val)) {
					Object daughter_vertex = insertedCellsForProxies.get(v.val);
					Object edge = graph.insertEdge(parent, null, "source", mothervertex, daughter_vertex);
					insertedCells.add(edge);
				}
			}
		}

		if (proxy.target != null) {
			for (int j = 0; j < proxy.target.alternativeValues.length ; j++) {
				AddressValue v = (AddressValue) proxy.target.alternativeValues[j];
				if (insertedCellsForProxies.containsKey(v.val)) {
					Object daughter_vertex = insertedCellsForProxies.get(v.val);
					Object edge = graph.insertEdge(parent, null, "target", mothervertex, daughter_vertex);
					insertedCells.add(edge);
				}
			}
		}

	}

	public void insertSourceAndTargetEdges(RelationUnion union, Object mothervertex) {

		if (union.usource != null) {
			for (int j = 0; j < union.usource.alternativeValues.length ; j++) {
				if (union.usource.alternativeValues[j] != null) {
					AddressValue v = (AddressValue) union.usource.alternativeValues[j];
					if (insertedCellsForUnions.containsKey(v.val)) {
						Object daughter_vertex = insertedCellsForUnions.get(v.val);
						Object edge = graph.insertEdge(parent, null, "source", mothervertex, daughter_vertex);
						insertedCells.add(edge);
					}
				}
			}
		}

		if (union.utarget != null) {
			for (int j = 0; j < union.utarget.alternativeValues.length ; j++) {
				if (union.utarget.alternativeValues[j] != null) {
					AddressValue v = (AddressValue) union.utarget.alternativeValues[j];
					if (insertedCellsForUnions.containsKey(v.val)) {
						Object daughter_vertex = insertedCellsForUnions.get(v.val);
						Object edge = graph.insertEdge(parent, null, "target", mothervertex, daughter_vertex);
						insertedCells.add(edge);
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

		//	union.distribution = ProbabilityUtils.normaliseDistribution(union.distribution, 1.0f);

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


		if (containsSaliencyFeat(union)) {
			graph.setCellStyles(mxConstants.STYLE_STROKECOLOR, "red", object_union);
		}
		
		insertedCellsForUnions.put(union.entityID, union_vertex);

		insertedCells.add(union_vertex);

		addNewFeatures(union.features, union_vertex, width, colour);

		for (int i = 0; i < union.includedProxies.length ; i++) {
			Proxy p = union.includedProxies[i];
			if (insertedCellsForProxies.containsKey(p.entityID)) {
				Object proxy_vertex = insertedCellsForProxies.get(p.entityID);
				Object edge = graph.insertEdge(parent, null, "includes", union_vertex, proxy_vertex);
				Object[] object_edges = new Object[1];
				object_edges[0] = edge;
				graph.setCellStyles(mxConstants.STYLE_DASHED, "true", object_edges);
				insertedCells.add(edge);
			}
		}

		log("Visual representation of the union successfully inserted in the GUI");
	}


	private boolean containsSaliencyFeat (PerceivedEntity entity) {
		String feats = "";
		for (int i = 0 ; i < entity.features.length ; i++) {
			feats += createFeatureText(entity.features[i]);
		}
		return feats.contains("high");
	}

	public void deleteUnion(Union union) {
		if (insertedCellsForUnions.containsKey(union.entityID)) {
			Object vertex = insertedCellsForUnions.get(union.entityID);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
			if (curUnionPosition_X > (220 * union.includedProxies.length))
				curUnionPosition_X -= (220 * union.includedProxies.length) ;
		}
		insertedCellsForUnions.remove(union.entityID);
	}



	public void deleteProxy(Proxy proxy) {
		if (insertedCellsForProxies.containsKey(proxy.entityID)) {
			Object vertex = insertedCellsForProxies.get(proxy.entityID);
			Object[] cells = new Object[1];
			cells[0] = vertex;
			graph.getModel().beginUpdate();
			graph.removeCells(cells);
			graph.getModel().endUpdate();
			insertedCellsForProxies.remove(proxy.entityID);
		}
	}


	public void updateGUI (Proxy updatedProxy) {

		//	graph.getModel().removeListener(listener);

		try {

			graph.getModel().beginUpdate();

			deleteProxy(updatedProxy);

			addNewProxy(updatedProxy, 0);

		}
		finally {
			graph.getModel().endUpdate();
		}

		try {
			final mxGraphComponent graphComponent = new mxGraphComponent(graph);

			frame.getContentPane().removeAll();
			frame.getContentPane().add(graphComponent);
			graph.setCellsEditable(false);
			graph.setEdgeLabelsMovable(false);
			graph.setDisconnectOnMove(false);
			graph.setCellsDisconnectable(false);
			graph.setConnectableEdges(false);
			graph.setDropEnabled(false);
			frame.setVisible(true);
		}
		catch (Exception e) {		}

		//		graph.getModel().addListener(mxEvent.SELECT, listener);

	}



	synchronized public void updateGUI(Vector<Proxy> newProxies, 
			Vector<Union> newUnions, 
			Vector<Proxy> proxiesToDelete, 
			Vector<Union> unionsToDelete, UnionConfiguration config) {

		//		 graph.getModel().removeListener(listener);

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
				if (!insertedCellsForUnions.containsKey(union.entityID)) {
					log("Adding new union..." + union.entityID);

					addNewUnionAndIncludedProxies(union);
				}
			}

			// UNSTABLE ?? TESTING!
			for (Enumeration<Proxy> e = newProxies.elements(); e.hasMoreElements();) {
				Proxy proxy = e.nextElement();
				if (!insertedCellsForProxies.containsKey(proxy.entityID)) {
					log("Adding new proxy..." + proxy.entityID);

					addNewProxy(proxy, 0);
				}
			}

			if (mustRegenerateRelationUnions) {
				for (Enumeration<RelationUnion> e = relationUnions.elements(); e.hasMoreElements();) {
					RelationUnion ru = e.nextElement();
					deleteUnion(ru);
					addNewRelationUnion(ru);
				}
			}

		}

		catch (Exception e) {	
			e.printStackTrace();
		}

		finally
		{
			addUnionConfigInfo(config);

			graph.getModel().endUpdate();
		}

		try {
			final mxGraphComponent graphComponent = new mxGraphComponent(graph);
			frame.getContentPane().removeAll();
			frame.getContentPane().add(graphComponent);
			graph.setCellsEditable(false);
			graph.setEdgeLabelsMovable(false);
			graph.setDisconnectOnMove(false);
			graph.setCellsDisconnectable(false);
			graph.setConnectableEdges(false);
			graph.setDropEnabled(false);
			addContextualMenu(graphComponent);
			frame.setVisible(true);
		}
		catch (Exception e) {	
			e.printStackTrace();
		}

		//		graph.getModel().addListener(mxEvent.SELECT, listener);

	}


	private void addUnionConfigInfo(UnionConfiguration config) {
		int rank = bm.alternativeConfigs.alterconfigs.length -
		GenericUtils.getIndex(bm.alternativeConfigs.alterconfigs, config);
		
		if (unionConfigInfo != null) {
			graph.removeCells(unionConfigInfo);
		}
		
		Object configinfo = graph.insertVertex(parent, null, "\n Union configuration: \n Prob:  " +
				roundProb((float)config.configProb) + "\n Rank: " + rank + 
				" (out of " + bm.alternativeConfigs.alterconfigs.length + ")", 10 , 10, 150, 80);
		unionConfigInfo = new Object[1];
		unionConfigInfo[0] = configinfo;
		graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, "white", unionConfigInfo);
		graph.setCellStyles(mxConstants.STYLE_ROUNDED, "false", unionConfigInfo);
		graph.setCellStyles(mxConstants.STYLE_ALIGN, mxConstants.ALIGN_LEFT, unionConfigInfo);
		graph.setCellStyles(mxConstants.STYLE_VERTICAL_ALIGN, mxConstants.ALIGN_TOP, unionConfigInfo);
		graph.setCellStyles(mxConstants.STYLE_SHADOW, "true", unionConfigInfo);
		graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "14", unionConfigInfo);
	}
	

	private void addContextualMenu(final mxGraphComponent graphComponent) {


		//======== popupMenu1 ========
		final JPopupMenu popupMenu1 = new JPopupMenu();
		JMenuItem menuItem20 = new JMenuItem();
		JMenuItem menuItem21 = new JMenuItem();

		//---- menuItem20 ----
		menuItem20.setText("Modify Proxy");
		popupMenu1.add(menuItem20);
		menuItem20.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				frame.menuItem7ActionPerformed(e);
			}
		});

		//---- menuItem21 ----
		menuItem21.setText("Delete Proxy");
		popupMenu1.add(menuItem21);
		menuItem21.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				frame.menuItem8ActionPerformed(e);
			}
		});

		graphComponent.getGraphControl().addMouseListener(new MouseAdapter()
		{

			public void mousePressed(MouseEvent e) {
				if (e.getModifiers() == InputEvent.BUTTON3_MASK)
					maybeShowPopup(e);
			}

			public void mouseReleased(MouseEvent e) {
				if (e.getModifiers() == InputEvent.BUTTON3_MASK)
					maybeShowPopup(e);
			}


			public void maybeShowPopup(MouseEvent e)
			{
				Object cell = graphComponent.getCellAt(e.getX(), e.getY());

				if (cell != null)
				{
					for (Iterator<String> ids = insertedCellsForProxies.keySet().iterator(); ids.hasNext();) {
						String curId = ids.next();
						Object curObj = insertedCellsForProxies.get(curId);
						if (curObj.equals(cell)) {
							frame.curSelectedEntityId = curId;
							frame.curSelectedEntityClass = Proxy.class;
						}
					}
					if (frame.curSelectedEntityClass != null && 
							frame.curSelectedEntityClass.equals(Proxy.class)) {
						popupMenu1.show(e.getComponent(), e.getX(), e.getY());
					}
				}
			}
		});
	}

	public void addNewRelationProxy (RelationProxy relationProxy, int xpos) {

		String colour = "#FFFF99";
		addNewProxy(relationProxy, xpos, Math.abs(curProxyPosition_Y + 200), colour);

		Object vertex = insertedCellsForProxies.get(relationProxy.entityID);
		insertSourceAndTargetEdges(relationProxy, vertex);
		log("OK FOR RELATION PROXY");
	}


	public void addNewPhantomProxy (PhantomProxy phantomProxy, int xpos) {

		String colour = "#FFE4C4";
		addNewProxy(phantomProxy, xpos, curProxyPosition_Y, colour);

		log("OK FOR PHANTOM PROXY");
	}


	public void addNewFeatures(Feature[] features, Object vertex, int width, String colour) {

		if (features != null) {

			Object[] objects_feats = new Object[features.length];

			int curYPosition = DEFAULT_ENTITY_BOX_HEIGHT;
			for (int i = 0; i < features.length; i++) {
				Feature feat = features[i];


				//		if (!FeatureValueUtils.hasUnknownValue(FeatureValueUtils.getMaxFeatureValue(feat))) {

				String featureText = createFeatureText(feat);
				log("feature text for " + i + ": " + featureText);
				int featHeight= computeHeight(feat);
				objects_feats[i] = graph.insertVertex(vertex, null, featureText, 6, curYPosition, width - 20, featHeight);
				curYPosition += featHeight + 10;
				insertedCells.add(objects_feats[i]);
				//		}
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
		if (LOGGING) {
			//		System.out.println("[BindingVisualizer]" + s);
			logger.debug("[BindingVisualizer]" + s);
		}
	}
}
