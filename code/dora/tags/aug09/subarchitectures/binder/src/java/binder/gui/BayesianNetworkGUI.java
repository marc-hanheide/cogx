package binder.gui;

import java.awt.BorderLayout;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Random;
import java.util.Vector;

import javax.swing.JFrame;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.featvalues.StringValue;
import binder.utils.BayesianNetworkUtils;

import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.view.mxGraph;

public class BayesianNetworkGUI extends JFrame {

	int DEFAULT_NODE_BOX_WIDTH= 180;
	int DEFAULT_NODE_BOX_HEIGHT= 30;
	
	int LINE_HEIGHT= 16;
	
	mxGraph graph;
	Object parent;
	
	HashMap<String,Object> insertedNodes;
	
	
	Random randomGenerator;
	
	public BayesianNetworkGUI () {
		randomGenerator = new Random();
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(1400, 1000);

		graph = new mxGraph();
		parent = graph.getDefaultParent();
	    		
		insertedNodes = new HashMap<String,Object>();
		
		setVisible(true);
	}
	
	
	public String createNodeText(BayesianNetworkNode node) {
		String text = "";
		text += node.feat.featlabel + " = \n";
		for (int i = 0; i < node.feat.alternativeValues.length; i ++) {
			StringValue sv = (StringValue) node.feat.alternativeValues[i];
			text += sv.val;
			if (sv.independentProb > 0.0f) {
				text += " (prob=" + sv.independentProb + ")";
			}
			if (i < node.feat.alternativeValues.length -1)
				text += " \\/";
			text +=  "\n";
		}
		return text;
	}
	
	
	
	public String createEdgeText(BayesianNetworkEdge edge) {
		String text = "";
		for (int i = 0; i < edge.correlations.length ; i++) {
			FeatureValueCorrelation corr = edge.correlations[i];
			text += "P ( " + ((StringValue)corr.value1).val + " | " + ((StringValue)corr.value2).val + " ) = " + corr.condProb + "\n";
		}
		return text;
	}
	
	
	private int computeNodeHeight(BayesianNetworkNode node) {
		return DEFAULT_NODE_BOX_HEIGHT + (LINE_HEIGHT * (node.feat.alternativeValues.length + 1));
	}
	
	
	public void drawBayesianNodes(BayesianNetworkNode[] nodes) {

		try {
			graph.getModel().beginUpdate();
			Vector<Integer> lastPos_X = new Vector<Integer>();
			Vector<Integer> lastPos_Y = new Vector<Integer>();
			for (int i = 0; i < nodes.length; i++) {
				BayesianNetworkNode node = nodes[i];
				int curPos_X = randomGenerator.nextInt(800) + 100;
				int curPos_Y = randomGenerator.nextInt(600) + 100;
		//		log("node " + i);
				boolean correctPos = false;
				while (!correctPos) {
					correctPos = true;
					
					for (Enumeration<Integer> e = lastPos_X.elements(); e.hasMoreElements(); ) {
						Integer pos_X = e.nextElement();
						if (Math.abs(curPos_X - pos_X) < 100) {
							correctPos = false;
							curPos_X= randomGenerator.nextInt(1200) + 50;
						}
					}
					for (Enumeration<Integer> e = lastPos_Y.elements(); e.hasMoreElements(); ) {
						Integer pos_Y = e.nextElement();
						if (Math.abs(curPos_Y - pos_Y) < 100) {
							correctPos = false;
							curPos_Y = randomGenerator.nextInt(800) + 50;
						}
					}
				}
				
				int height = computeNodeHeight(node);
				
				String text = createNodeText(node);
				Object vertex = graph.insertVertex(parent, null, text, curPos_X, curPos_Y, DEFAULT_NODE_BOX_WIDTH, height);
				insertedNodes.put(node.feat.featlabel, vertex);
				Object[] verteces = new Object[1];
				verteces[0] = vertex;
				graph.setCellStyles(mxConstants.STYLE_FONTSIZE, "12", verteces);
				graph.setCellStyles(mxConstants.STYLE_FONTSTYLE, "" + mxConstants.FONT_BOLD, verteces);
				graph.setCellStyles(mxConstants.STYLE_ROUNDED, "true", verteces);
				graph.setCellStyles(mxConstants.STYLE_FILLCOLOR, "#90EE90", verteces);
				lastPos_X.add(curPos_X);
				lastPos_Y.add(curPos_Y);
			}
		}

		finally
		{
			graph.getModel().endUpdate();
		}
	}
	
	
	
	
	public void drawBayesianEdges(BayesianNetworkEdge[] edges) {

		try {
			graph.getModel().beginUpdate();
			for (int i = 0; i < edges.length; i++) {
				BayesianNetworkEdge edge = edges[i];
				
			
				String text = createEdgeText(edge);
				
				if (insertedNodes.containsKey(edge.incomingNode) && insertedNodes.containsKey(edge.outgoingNode)) {
					graph.insertEdge(parent, null, text, insertedNodes.get(edge.incomingNode), insertedNodes.get(edge.outgoingNode));
				}
				else {
					log("WARNING: node is not present!");
				}
			}
		}

		finally
		{
			graph.getModel().endUpdate();
		}
	}
	
	
	
	
	public void drawBayesianNetwork(BayesianNetwork network) {

		drawBayesianNodes(network.nodes);
		drawBayesianEdges(network.edges);
		mxGraphComponent graphComponent = new mxGraphComponent(graph);
		getContentPane().add(graphComponent);

		log("Visual representation of the proxy sucessfully inserted in the GUI");


		setVisible(true);
	}


	public static void main (String[] args) {
		BayesianNetworkGUI gui = new BayesianNetworkGUI();
		BayesianNetwork network = BayesianNetworkUtils.constructNetwork();
		gui.drawBayesianNetwork(network);
	}
	
	

	private void log(String s) {
		System.out.println("[BayesianNetworkGUI]" + s);
	}
}
