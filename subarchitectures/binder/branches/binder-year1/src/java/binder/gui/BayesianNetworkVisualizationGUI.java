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
import com.mxgraph.layout.mxGraphLayout;
import com.mxgraph.layout.hierarchical.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

import javax.swing.*;
import javax.swing.event.TableModelEvent;
import javax.swing.event.TableModelListener;
import javax.swing.table.TableColumn;
import javax.swing.table.TableModel;

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
import binder.gui.BayesianNetworkEdgeWrapper;
import binder.gui.BayesianNetworkNodeWrapper;


/**
 * @author Carsten Ehrler
 */
public class BayesianNetworkVisualizationGUI extends JFrame
{	
	private final class NodeTableModelListener implements TableModelListener {
		public void tableChanged(TableModelEvent e) {
			int row = e.getFirstRow();
			int column = e.getColumn();
			TableModel model = (TableModel)e.getSource();
			String columnName = model.getColumnName(0);
			Object data = model.getValueAt(row, column);
			System.out.println("row: " + row + "col: " + column + columnName);
			
			try {
				System.out.println(data);
				float probability = Float.parseFloat(data.toString());
				bn_wrapper.modifyFeatureAlternative(columnName, model.getValueAt(row, 0).toString(), probability);
			} catch (Exception ex) {
				System.out.println(ex.toString());
			}
		}
	}
	
	// this is used for serialization purposes, but don't know about 
	// its implifications
	private static final long serialVersionUID = 1L;
	
	// we need the binding monitor in order to access the working memory
	private BinderMonitor binder_monitor;
	
	// this bayesian network is displayed and modified
	private BayesianNetwork bayesian_network;
	private BayesianNetworkGUIWrapper bn_wrapper;
	
	// the graph and its parent
	private mxGraph graph;
	private Object graph_parent;
	
	// parameters for the size of the nodes and edges in the graph representation
	private final static int NODE_HEIGHT = 20;
	private final static int NODE_WIDTH = 80;
	
	// these maps keep track of the correspondence between edges and nodes in
	// the network and their counterparts in the graphical representation 
	private HashMap<Object, BayesianNetworkEdgeWrapper> map_mxedge_edge;
	private HashMap<BayesianNetworkEdgeWrapper, Object> map_edge_mxnode;
	
	private static Logger logger = ComponentLogger.getLogger(BayesianNetworkVisualizationGUI.class);
	
	// for now we do note need this
	private Random randomGenerator;

	private HashMap<String, Object> map_name_mxnode;
	private HashMap<Object, String> map_mxnode_name;

	private JButton rem_button;

	private String bayesian_network_id;
	
	public BayesianNetworkVisualizationGUI(BinderMonitor binder_monitor) {
		graph = new mxGraph();
		graph_parent = graph.getDefaultParent();
		initComponents();
		randomGenerator = new Random();
		
		this.binder_monitor = binder_monitor;
		
		// now we retrieve the bayesian network from the bindermonitor
		try {
			CASTData<BayesianNetwork>[] network = binder_monitor.getWorkingMemoryEntries(Binder.BINDER_SA, BayesianNetwork.class);
			this.bayesian_network = network[0].getData();
			this.bayesian_network_id = network[0].getID();
		}
		catch (Exception e) {
			log(e.toString());
		}
		
		assert (this.bayesian_network) != null;
		
		this.bn_wrapper = new BayesianNetworkGUIWrapper(this.bayesian_network);
		
		setSize(800, 600);
		setLocation(200,200);
		
		map_mxedge_edge = new HashMap<Object, BayesianNetworkEdgeWrapper>();
		map_edge_mxnode = new HashMap<BayesianNetworkEdgeWrapper, Object>();
		map_name_mxnode = new HashMap<String, Object>();
		map_mxnode_name = new HashMap<Object, String>();
		
		setVisible(true);
	}
	
	private void initComponents() {
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		
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
				menu1.setEnabled(true);

				//---- menuItem1 ----
				menuItem1.setText("Load");
				menuItem1.setEnabled(false);
				menu1.add(menuItem1);

				//---- menuItem2 ----
				menuItem2.setText("Save");
				menu1.add(menuItem2);
			}
			menuBar1.add(menu1);

			//======== menu2 ========
			{
				menu2.setText("Edit");
				menu2.setEnabled(false);
			}
			menuBar1.add(menu2);
		}
		setJMenuBar(menuBar1);

		//======== panel1 ========
		{

			panel1.setLayout(new VerticalLayout());

			//---- label1 ----
			label1.setText("Hallllloooooo");
		}
		contentPane.add(panel1, BorderLayout.WEST);
		pack();
		setLocationRelativeTo(getOwner());
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
		
		new_node_button = new JButton("Add variable");
		new_edge_button = new JButton("Add dependency");
		rem_button = new JButton("Remove Selected");
		draw_button = new JButton("Redraw bayesian network");
		send_button = new JButton("Send changes to WM");

		
		panel1.add(new_node_button);
		panel1.add(new_edge_button);
		panel1.add(rem_button);
		panel1.add(new JSeparator(JSeparator.HORIZONTAL));
		panel1.add(send_button);
		panel1.add(draw_button);
		panel1.add(new JSeparator(JSeparator.HORIZONTAL));
		panel1.add(label1);
		panel1.add(table1);
		panel1.setPreferredSize(new Dimension(300, panel1.getPreferredSize().height));
		
		new_node_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventAddNewNode();
			}
		});
		
		new_edge_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventAddNewEdge();
			}
		});
		
		rem_button.addActionListener( new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventRemoveNodeOrEdge();
			}
		});
		
		draw_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				drawBayesianNetwork();
			}
		});
		
		send_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				BayesianNetwork network = bn_wrapper.getBayesianNetwork();
				try {
					binder_monitor.addToWorkingMemory(bayesian_network_id, network);
				} catch(Exception ex) {
					log(ex.toString());
				}
			}
		});
		
		menuItem2.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				JFileChooser fc = new JFileChooser();
				int returnVal = fc.showSaveDialog(null);
				
				if (returnVal == JFileChooser.APPROVE_OPTION) {
					try {
					File file = fc.getSelectedFile();
					FileWriter outfile = new FileWriter(file);
					outfile.write(bn_wrapper.getSerializedNetwork());
					outfile.close();
					} catch(java.io.IOException ex) {
						JOptionPane.showMessageDialog(null, ex.toString());
					}
				}
			} 
		});
	}
	
	private void EventRemoveNodeOrEdge() {
		Object selected = graph.getSelectionCell();
		if(this.map_mxnode_name.containsKey(selected)) {
			// we remove a node from the graph
			try {
				this.bn_wrapper.removeNode(this.map_mxnode_name.remove(selected));
				drawBayesianNetwork();
			} catch(Exception e) {
				JOptionPane.showMessageDialog(null, e.toString());
			}
		}
		else if(this.map_mxedge_edge.containsKey(selected)) {
			// we remove an edge from the graph
			String source = map_mxedge_edge.get(selected).getNodeSource().getFeatureLabelName();
			String target = map_mxedge_edge.get(selected).getNodeTarget().getFeatureLabelName();
			try {
				this.bn_wrapper.removeEdge(source, target);
				drawBayesianNetwork();
			} catch(Exception e) {
				JOptionPane.showMessageDialog(null, e.toString());
			}
		}
		// otherwise we do nothing
	}

	private void EventAddNewNode() {
		String feature_label = ((String)JOptionPane.showInputDialog(this, "Name of new Variable")).toLowerCase();
		try {
			this.bn_wrapper.addNode(feature_label);
		}
		catch (Exception e) {
			return;
		}
		try {
			graph.getModel().beginUpdate();
			Object inserted_node = graph.insertVertex(graph_parent, null, feature_label, 0, 0, NODE_WIDTH, NODE_HEIGHT);
			this.map_name_mxnode.put(feature_label, inserted_node);
			this.map_mxnode_name.put(inserted_node, feature_label);
		}
		finally {
			graph.getModel().endUpdate();
		}
		this.validate();
	}
	
	private void EventAddNewEdge() {
		Object[] nodes = this.bn_wrapper.getNodeNames().toArray();
		
		String source = ((String)JOptionPane.showInputDialog(null, "Select source", "Add edge", JOptionPane.INFORMATION_MESSAGE, null, nodes, nodes[0].toString())).toLowerCase();
		String target = ((String)JOptionPane.showInputDialog(null, "Select target", "Add edge", JOptionPane.INFORMATION_MESSAGE, null, nodes, nodes[0].toString())).toLowerCase();
		
		try {
			this.bn_wrapper.addEdge(source, target);
		}
		catch(Exception e) {
			JOptionPane.showMessageDialog(null, e.toString(), "Add edge", JOptionPane.ERROR_MESSAGE);
			return;
		}
		
		Object mxsource = this.map_name_mxnode.get(source);
		Object mxtarget = this.map_name_mxnode.get(target);
		
		try {
			graph.getModel().beginUpdate();
			
			Object nx_edge = graph.insertEdge(graph_parent, null, null, mxsource, mxtarget);
			this.map_mxedge_edge.put(nx_edge, this.bn_wrapper.getEdge(source, target));
			this.map_edge_mxnode.put(this.bn_wrapper.getEdge(source, target), nx_edge);
		}
		finally {
			graph.getModel().endUpdate();
		}
		this.validate();
	}

	private void drawBayesianEdges() {
		for(String name_source : bn_wrapper.getNodeNames()) {
			for(String name_target : bn_wrapper.getTargetNames(name_source)) {
				Object node_source = this.map_name_mxnode.get(name_source);
				Object node_target = this.map_name_mxnode.get(name_target);
				Object nx_edge = graph.insertEdge(graph_parent, null, null, node_source, node_target);
				this.map_mxedge_edge.put(nx_edge, this.bn_wrapper.getEdge(name_source, name_target));
				this.map_edge_mxnode.put(this.bn_wrapper.getEdge(name_source, name_target), nx_edge);
			}
		}
	}
	
	private void drawBayesianNodes() {
		// TODO: we have to generate the description for the node
		int pos_x = 0;
		int pos_y = 0;
		
		for(String text : bn_wrapper.getNodeNames()) {
			Object inserted_node = graph.insertVertex(graph_parent, null, text, pos_x, pos_y, NODE_WIDTH, NODE_HEIGHT);
			this.map_name_mxnode.put(text, inserted_node);
			this.map_mxnode_name.put(inserted_node, text);
		}
	}
	
	public void drawBayesianNetwork() {
		//this.setVisible(false);
		this.map_mxedge_edge.clear();
		this.map_mxnode_name.clear();
		this.map_name_mxnode.clear();
		
		if(graph_component != null) {
			this.getContentPane().remove(graph_component);
		}
		
		try {
			graph = new mxGraph();
			graph_parent = graph.getDefaultParent();
			graph.getModel().beginUpdate();
			drawBayesianNodes();
			drawBayesianEdges();
			
			// compute a hierarchical layout of the graph
			mxGraphLayout layout = new mxHierarchicalLayout(graph);
			layout.execute(graph_parent);
		}
		finally {
			graph.getModel().endUpdate();
		}
		
		setGraphParameters(graph);
		
		graph_component = new mxGraphComponent(graph);
		
		addListenerToGraph(graph_component);
		
		this.setSize(800, 600);
		this.getContentPane().add(graph_component, BorderLayout.CENTER);
		this.setVisible(true);
	}
	
	private void addListenerToGraph(final mxGraphComponent graphComponent) {
		
		graph.getModel().addListener(mxEvent.SELECT, new mxIEventListener()
		{
			public void invoke(Object sender, mxEventObject evt)
			{
				System.out.println(sender.toString() + evt.toString());
			}
		});
		
		graphComponent.getGraphControl().addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent e) {
				Object selected = graph.getSelectionCell();
				if(selected != null) {
					if (map_mxedge_edge.containsKey(selected)) {
						displayTableForEdgeSelected(selected);
					}
					else if (map_mxnode_name.containsKey(selected)) {
						displayTableForNodeSelection(selected);
					}
				}
				else {
					label1.setText("Nothing selected");
					if(table1 != null) {
						panel1.remove(table1);
						panel1.remove(del_button);
						panel1.remove(add_button);
					}
					table1 = null;
					del_button = null;
					add_button = null;
				}
			}
			
			public void mousePressed(MouseEvent e) {
				int x = e.getX();
				int y = e.getY();
				source_for_new_edge = graphComponent.getCellAt(x, y);
			}
			
			public void mouseReleased(MouseEvent e) {
				int x = e.getX();
				int y = e.getY();
				Object target_for_new_edge = graphComponent.getCellAt(x, y);
				
				System.out.println("We selected: " + map_mxnode_name.get(source_for_new_edge) + map_mxnode_name.get(target_for_new_edge));
				
				source_for_new_edge = null;
			}
		});
		
	}
	
	private void addComboBoxListenersEdge(BayesianNetworkEdgeWrapper selectedEdge, JTable table) {
		TableColumn column1 = table.getColumnModel().getColumn(0);
		TableColumn column2 = table.getColumnModel().getColumn(1);
		
		JComboBox comboBox1 = new JComboBox();
		for(String alternative : selectedEdge.getNodeTarget().getAlternativeNames()){
			comboBox1.addItem(alternative);
		}
		
		column1.setCellEditor(new DefaultCellEditor(comboBox1));
		
		JComboBox comboBox2 = new JComboBox();
		for(String alternative : selectedEdge.getNodeSource().getAlternativeNames()){
			comboBox2.addItem(alternative);
		}
		
		column2.setCellEditor(new DefaultCellEditor(comboBox2));
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

	private void displayTableForNodeSelection(Object selected) {
		if(add_button != null) {
			panel1.remove(add_button);
		}
		add_button = new JButton("Add feature alternative");
		
		if(del_button != null) {
			panel1.remove(del_button);
		}
		del_button = new JButton("Delete feature alternative");
		
		if(table1 != null) {
			panel1.remove(table1);
		}
		System.out.println("THERE WE GO");
		BayesianNetworkNodeWrapper selected_node = bn_wrapper.getNode(map_mxnode_name.get(selected));
		label1.setText("Node selected with Feature: " + selected_node.getFeatureLabelName());
		table1 = new JTable(new BayesianNodeTable(selected_node));
		table1.setSelectionMode( ListSelectionModel.SINGLE_SELECTION);
		table1.getModel().addTableModelListener(new NodeTableModelListener());
		
		add_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventNodeNewFeatureAlternative();
			}
		});
		
		del_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventNodeDelFeatureAlternative();
			}
		});

		panel1.add(table1);
		panel1.add(add_button);
		panel1.add(del_button);
		this.validate();
	}
	
	private void EventNodeNewFeatureAlternative() {
		String s = (String)JOptionPane.showInputDialog("Feature alternative:");
		if(s != null && s != "") {
			String feature_label = table1.getColumnName(0);
			try {
				this.bn_wrapper.addFeatureAlternative(feature_label, s.toLowerCase(), new Float(0.));
			}
			catch (Exception e) {
				JOptionPane.showMessageDialog(this, "Alternative already exists", "Update Error", JOptionPane.ERROR_MESSAGE);
			}
			this.displayTableForNodeSelection(this.map_name_mxnode.get(feature_label));
			this.validate();
		}
	}
	
	private void EventNodeDelFeatureAlternative() {
		int selected_row = table1.getSelectedRow();
		if(selected_row == -1) {
			return;
		}
		String feature_alternative = (String)table1.getValueAt(table1.getSelectedRow(), 0);
		String feature_label = table1.getColumnName(0);
		System.out.println(feature_alternative + " " + feature_label);
		try {
			this.bn_wrapper.removeFeatureAlternative(feature_label, feature_alternative);
		} catch(Exception e) {
			System.out.println(e.toString());
		}
		finally {
			this.displayTableForNodeSelection(this.map_name_mxnode.get(feature_label));
		}
	}

	private void displayTableForEdgeSelected(Object selected) {
		if(add_button != null) {
			panel1.remove(add_button);
		}
		add_button = new JButton("Add dependency alternative");
		
		if(del_button != null) {
			panel1.remove(del_button);
		}
		del_button = new JButton("Delete dependency alternative");
		
		if(table1 != null) {
			panel1.remove(table1);
		}
		BayesianNetworkEdgeWrapper selected_edge = map_mxedge_edge.get(selected);
		label1.setText("P(" +
				selected_edge.getNodeTarget().getFeatureLabelName() + 
				"|" +
				selected_edge.getNodeSource().getFeatureLabelName() + 
				")");
		JTable table = new JTable(new BayesianEdgeTable(selected_edge));
		table.setSelectionMode( ListSelectionModel.SINGLE_SELECTION);
		
		addComboBoxListenersEdge(selected_edge, table);
		
		table1 = table;
		panel1.add(table1);
		panel1.add(add_button);
		panel1.add(del_button);
		
		del_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventEdgeDelFeatureAlternative();
			}
		});
		
		add_button.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				EventEdgeAddFeatureAlternative();
			}
		});
	}
	
	private void EventEdgeDelFeatureAlternative() {
		int selected_row = table1.getSelectedRow();
		int selected_col = table1.getSelectedColumn();
		if(selected_row == -1) {
			return;
		}
		
		String name_source = table1.getModel().getColumnName(1);
		String name_target = table1.getModel().getColumnName(0);
		String alternative = (String) table1.getModel().getValueAt(selected_row, 0);
		String alternative_conditioned = (String) table1.getModel().getValueAt(selected_row, 1);
		try {
			this.bn_wrapper.removeFeatureConditioned(name_source, name_target, alternative, alternative_conditioned);
		} catch(Exception e) {
			JOptionPane.showMessageDialog(null, e.toString());
		}
		this.displayTableForEdgeSelected(this.map_edge_mxnode.get(bn_wrapper.getEdge(name_source, name_target)));
		this.validate();
	}
	
	private void EventEdgeAddFeatureAlternative() {
		String name_source = table1.getModel().getColumnName(1);
		String name_target = table1.getModel().getColumnName(0);
		
		// we need to compute a valid combination of a feature alternative and a
		// corresponding dependent alternative which is not already present in
		// the table
		BayesianNetworkNodeWrapper source = bn_wrapper.getNode(name_source);
		BayesianNetworkNodeWrapper target = bn_wrapper.getNode(name_target);
		
		int number_of_alternatives_source = source.getNumberOfAlternatives();
		int number_of_alternatives_target = target.getNumberOfAlternatives();
		
		Set<String> alternatives_source = new TreeSet<String>(source.getAlternativeNames());
		Set<String> alternatives_target = new TreeSet<String>(target.getAlternativeNames());
		
		for(String alternative_source : alternatives_source) {
			TreeSet<String> used = new TreeSet<String>();
			for(int i = 0; i < table1.getModel().getRowCount(); ++i) {
				if(alternative_source.equals((String)table1.getModel().getValueAt(i, 1))) {
					used.add((String)table1.getModel().getValueAt(i, 0));
				}
			}
			
			TreeSet<String> intersection = new TreeSet<String>(alternatives_target);
			
			intersection.removeAll(used);
			
			if(!intersection.isEmpty()) {
				try {
					bn_wrapper.addFeatureConditioned(name_source, name_target, intersection.first().toString(), alternative_source, new Float(0));
					return;
				} catch(Exception e) {
					JOptionPane.showMessageDialog(null, e.toString());
				}
			}
			this.displayTableForEdgeSelected(this.map_edge_mxnode.get(bn_wrapper.getEdge(name_source, name_target)));
			this.validate();
		}
	}
	
	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - C E
	private mxGraphComponent graph_component;
	private JMenuBar menuBar1;
	private JMenu menu1;
	private JMenuItem menuItem1;
	private JMenuItem menuItem2;
	private JMenu menu2;
	private JPanel panel1;
	private JLabel label1;
	private JTable table1;
	// JFormDesigner - End of variables declaration  //GEN-END:variables
	private JButton add_button;
	private JButton del_button;
	private JButton new_node_button;
	private JButton new_edge_button;
	private JButton send_button;
	private JButton draw_button;
	
	
	Object source_for_new_edge;
}
