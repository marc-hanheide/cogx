/*rerueraraalse
 * Created by JFormDesigner on Wed Jan 13 15:19:54 CET 2010
 */

package binder.gui;

import cast.core.CASTData;
import cast.core.logging.ComponentLogger;
import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.util.mxEvent;
import com.mxgraph.util.mxEventObject;
import com.mxgraph.util.mxEventSource.mxIEventListener;
import com.mxgraph.view.mxGraph;
import com.mxgraph.layout.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.HashMap;
import java.util.Random;
import javax.swing.*;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import org.jfree.ui.tabbedui.VerticalLayout;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.bayesiannetworks.BayesianNetworkEdge;
import binder.autogen.bayesiannetworks.BayesianNetworkNode;
import binder.autogen.bayesiannetworks.FeatureValueCorrelation;
import binder.autogen.core.FeatureValue;
import binder.components.Binder;
import binder.components.BinderMonitor;
import binder.utils.FeatureValueUtils;
import binder.gui.BayesianEdgeTable;
import binder.gui.BayesianNodeTable;


/**
 * @author Carsten Ehrler
 */
public class BayesianNetworkVisualizationGUI extends JFrame
{	
	// this is used for serialization purposes, but don't know about 
	// its implifications
	private static final long serialVersionUID = 1L;
	
	// we need the binding monitor in order to access the working memory
	private BinderMonitor binder_monitor;
	
	// this bayesian network is displayed and modified
	private BayesianNetwork bayesian_network;
	
	// the graph and its parent
	private mxGraph graph;
	private Object graph_parent;
	
	// parameters for the size of the nodes and edges in the graph representation
	private final static int NODE_HEIGHT = 40;
	private final static int NODE_WIDTH = 150;
	
	// these maps keep track of the correspondence between edges and nodes in
	// the network and their counterparts in the graphical representation 
	private HashMap<BayesianNetworkNode, Object> map_node_mxnode;
	private HashMap<Object, BayesianNetworkNode> map_mxnode_node;
	private HashMap<Object, BayesianNetworkEdge> map_mxedge_edge;
	
	private static Logger logger = ComponentLogger.getLogger(BayesianNetworkGUI.class);
	
	// for now we do note need this
	private Random randomGenerator;
	
	public BayesianNetworkVisualizationGUI(BinderMonitor binder_monitor) {
		graph = new mxGraph();
		graph_parent = graph.getCurrentRoot();
		initComponents();
		randomGenerator = new Random();
		
		this.binder_monitor = binder_monitor;
		
		// now we retrieve the bayesian network from the bindermonitor
		try {
			CASTData<BayesianNetwork>[] network = binder_monitor.getWorkingMemoryEntries(Binder.BINDER_SA, BayesianNetwork.class);
			this.bayesian_network = network[0].getData();
		}
		catch (Exception e) {
			log(e.toString());
		}
		
		setSize(800, 600);
		
		map_node_mxnode = new HashMap<BayesianNetworkNode, Object>();	
		map_mxnode_node = new HashMap<Object, BayesianNetworkNode>();
		map_mxedge_edge = new HashMap<Object, BayesianNetworkEdge>();
		
		setVisible(true);
	}
	
	private void initComponents() {
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		
		/*
		 * 
		graph.getSelectionModel().addListener(mxEvent.MARK, new mxIEventListener() {
			public void invoke(Object sender, mxEventObject event) {
				label1.setText("TEST" + graph.getSelectionCell().toString());
			}
		});
		*/
		
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - C E
		menuBar1 = new JMenuBar();
		menu1 = new JMenu();
		menuItem1 = new JMenuItem();
		menuItem2 = new JMenuItem();
		menu2 = new JMenu();
		panel1 = new JPanel();
		label1 = new JLabel();
		table1 = new JTable();

		//======== this ========
		setTitle("Bayesian Graphical Model");
		Container contentPane = getContentPane();
		contentPane.setLayout(new BorderLayout());

		//======== menuBar1 ========
		{

			//======== menu1 ========
			{
				menu1.setText("File");
				menu1.setEnabled(false);

				//---- menuItem1 ----
				menuItem1.setText("Load");
				menu1.add(menuItem1);

				//---- menuItem2 ----
				menuItem2.setText("Save");
				menu1.add(menuItem2);
			}
			menuBar1.add(menu1);

			//======== menu2 ========
			{
				menu2.setText("Edit");
			}
			menuBar1.add(menu2);
		}
		setJMenuBar(menuBar1);

		//======== panel1 ========
		{

			// JFormDesigner evaluation mark
			panel1.setBorder(new javax.swing.border.CompoundBorder(
				new javax.swing.border.TitledBorder(new javax.swing.border.EmptyBorder(0, 0, 0, 0),
					"JFormDesigner Evaluation", javax.swing.border.TitledBorder.CENTER,
					javax.swing.border.TitledBorder.BOTTOM, new java.awt.Font("Dialog", java.awt.Font.BOLD, 12),
					java.awt.Color.red), panel1.getBorder())); panel1.addPropertyChangeListener(new java.beans.PropertyChangeListener(){public void propertyChange(java.beans.PropertyChangeEvent e){if("border".equals(e.getPropertyName()))throw new RuntimeException();}});

			panel1.setLayout(new VerticalLayout());

			//---- label1 ----
			label1.setText("Hallllloooooo");
			panel1.add(label1);
			panel1.add(table1);
		}
		contentPane.add(panel1, BorderLayout.WEST);
		pack();
		setLocationRelativeTo(getOwner());
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
	}
	
	private void drawBayesianEdges() {
		for(BayesianNetworkEdge edge : bayesian_network.edges) {
			Object node_start = this.map_node_mxnode.get(edge.outgoingNode);
			Object node_end = this.map_node_mxnode.get(edge.incomingNode);
			Object nx_edge = graph.insertEdge(graph_parent, null, null, node_start, node_end);
			this.map_mxedge_edge.put(nx_edge, edge);
			
		}
	}
	
	private void drawBayesianNodes() {
		// TODO: we have to generate the description for the node
		int pos_x = 0;
		int pos_y = 0;
		
		for(BayesianNetworkNode node : bayesian_network.nodes) {
			String text = new String(node.feat.featlabel);
			Object inserted_node = graph.insertVertex(graph_parent, null, text, pos_x, pos_y, NODE_WIDTH, NODE_HEIGHT);
			this.map_node_mxnode.put(node, inserted_node);
			this.map_mxnode_node.put(inserted_node, node);
		}
	}
	
	public void drawBayesianNetwork() {
		this.setVisible(false);
		try {
			graph.removeCells();
			graph.getModel().beginUpdate();
			drawBayesianNodes();
			drawBayesianEdges();
			
			// compute a hierarchical layout of the graph
			// TODO: we should do a hierarchical layout here
			mxGraphLayout layout = new mxCircleLayout(graph);
			layout.execute(graph.getCurrentRoot());
		}
		finally {
			graph.getModel().endUpdate();
		}
		
		setGraphParameters(graph);
		
		mxGraphComponent graph_component = new mxGraphComponent(graph);
		
		addListenerToGraph(graph_component);
		
		this.getContentPane().add(graph_component, BorderLayout.CENTER);
		this.pack();
		this.setVisible(true);
	}
	
	private void addListenerToGraph(mxGraphComponent graphComponent) {
		// TODO Auto-generated method stub
		graphComponent.getGraphControl().addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				Object selected = graph.getSelectionCell();
				if (map_mxedge_edge.containsKey(selected)) {
					label1.setText("Edge selected");
					JTable table = new JTable(new BayesianEdgeTable(map_mxedge_edge.get(selected)));
					panel1.remove(table1);
					table1 = table;
					panel1.add(table1);
				}
				else if (map_mxnode_node.containsKey(selected)) {
					label1.setText("Node selected");
					JTable table = new JTable(new BayesianNodeTable(map_mxnode_node.get(selected)));
					panel1.remove(table1);
					table1 = table;
					panel1.add(table1);
				}
			}
		});
	}

	private void setGraphParameters(mxGraph graph) {
		graph.setCellsEditable(false);
		graph.setEdgeLabelsMovable(false);
		graph.setDisconnectOnMove(false);
		graph.setCellsDisconnectable(false);
		graph.setConnectableEdges(false);
		graph.setDropEnabled(false);
		graph.setAllowDanglingEdges(false);
		graph.setAutoSizeCells(true);
		graph.setCellsResizable(false);
	}

	private void log(String s) {
		logger.debug("[BayesianNetworkVisualizerGUI]" +  s);
	}

	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - C E
	private JMenuBar menuBar1;
	private JMenu menu1;
	private JMenuItem menuItem1;
	private JMenuItem menuItem2;
	private JMenu menu2;
	private JPanel panel1;
	private JLabel label1;
	private JTable table1;
	// JFormDesigner - End of variables declaration  //GEN-END:variables
}
