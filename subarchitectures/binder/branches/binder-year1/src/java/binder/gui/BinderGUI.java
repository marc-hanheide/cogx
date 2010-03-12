
package binder.gui;

import java.awt.*;
import java.awt.event.*;
import java.util.Vector;

import javax.swing.*;

import org.apache.log4j.Logger;

import binder.autogen.bayesiannetworks.BayesianNetwork;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.components.Binder;
import binder.components.BinderMonitor;
import binder.constructors.ProxyConstructor;
import binder.utils.GenericUtils;

import binder.autogen.featvalues.StringValue;

import binder.gui.BayesianNetworkGUI;
import binder.gui.ProxyInfoGUI;

import cast.core.logging.ComponentLogger;
import cast.core.CASTData;

// import com.jgoodies.forms.layout.*;
/*
 * Created by JFormDesigner on Fri Oct 23 23:10:43 CEST 2009
 */
import com.mxgraph.swing.mxGraphComponent;
import com.mxgraph.util.mxConstants;
import com.mxgraph.view.mxGraph;



/**
 * @author Pierre Lison
 */
public class BinderGUI extends JFrame {
	public boolean LOGGING = true;

	private static Logger logger = ComponentLogger.getLogger(BinderGUI.class);
	
	BinderMonitor bm;
	BayesianNetworkVisualizationGUI bn_gui;
	
	int config_CurrentRank = 1;

	String curSelectedEntityId = "";
	Class curSelectedEntityClass ;
	
	public BinderGUI(BinderMonitor bm) {
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		initComponents();
		this.bm = bm;
	} 
	
	private void menuItem15ActionPerformed(ActionEvent e) {

		if (config_CurrentRank > 1) {
			config_CurrentRank = config_CurrentRank - 1;
			bm.showConfigurationOfRank(config_CurrentRank);
		}
		else {
			log("(BOOOOOONG) maximum rank reached!");
		}

	}
	
	
	public void setCurrentSelection (String entityId, Class type) {
		curSelectedEntityId = entityId;
		curSelectedEntityClass = type;
		
		if (type == null) {
			menuItem7.setEnabled(false);
			menuItem8.setEnabled(false);
			menuItem18.setEnabled(false);
		}
		else if (type.equals(Proxy.class)) {
			menuItem7.setEnabled(true);
			menuItem8.setEnabled(true);
			menuItem18.setEnabled(true);
		}
		else {
			menuItem7.setEnabled(false);
			menuItem8.setEnabled(false);
			menuItem18.setEnabled(false);
		}
	}


	private void menuItem16ActionPerformed(ActionEvent e) {
		if (config_CurrentRank < bm.alternativeConfigs.alterconfigs.length) {
			config_CurrentRank = config_CurrentRank + 1;
			bm.showConfigurationOfRank(config_CurrentRank);
		}
		else {
			log("(BOOOOOONG) minimum rank reached!");
		}
	}

	private void menuItem11ActionPerformed(ActionEvent e) {
		ProxyInfoGUI proxyInfo = new ProxyInfoGUI(this);
		proxyInfo.setVisible(true);
	} 

	public void menuItem8ActionPerformed(ActionEvent e) {
		
		if (!curSelectedEntityId.equals("")) {
			try {
			bm.deleteFromWorkingMemory(curSelectedEntityId);
			}
			catch (Exception ex) {
				ex.printStackTrace();
			}
		}
		
	}

	private void menuItem5ActionPerformed(ActionEvent e) {
		dispose();
	}

	public void menuItem7ActionPerformed(ActionEvent e) {
		
		try {
			Proxy proxy = bm.getMemoryEntry(curSelectedEntityId, Proxy.class);
			ProxyInfoGUI proxyInfo = new ProxyInfoGUI(this, proxy);
			proxyInfo.setVisible(true);
		}
		catch (Exception ex) {
			ex.printStackTrace();
		}

	}

	private void menuItem18ActionPerformed(ActionEvent e) {
		try {
			if (!curSelectedEntityId.equals("")) {
			Proxy proxy = bm.getMemoryEntry(curSelectedEntityId, Proxy.class);
		FeatureInfoGUI featureInfo = new FeatureInfoGUI(this, proxy);
		featureInfo.setVisible(true);
		}
		}
		catch (Exception ex) {
			ex.printStackTrace();
		}
	}

	private void thisMouseClicked(MouseEvent e) {
		// TODO add your code here
	}

	private void menuItem20ActionPerformed(ActionEvent e) {
		try {
			if (!curSelectedEntityId.equals("")) {
			Proxy proxy = bm.getMemoryEntry(curSelectedEntityId, Proxy.class);
			Vector<Feature> feats = new Vector<Feature>();
			for (int i = 0 ; i < proxy.features.length ; i++) {
				if (!proxy.features[i].featlabel.equals("saliency"))
				feats.add(proxy.features[i]);
			}
			proxy.features = new Feature[feats.size()];
			proxy.features = feats.toArray(proxy.features);
			
			StringValue salient = ProxyConstructor.createStringValue("high", 1.0f);
			Feature feat = ProxyConstructor.createFeatureWithUniqueFeatureValue("saliency", salient);
			ProxyConstructor.addFeatureToProxy(proxy, feat);
			bm.overwriteWorkingMemory(proxy.entityID, Binder.BINDER_SA, proxy);
			}
		}
		catch (Exception ex) {
			ex.printStackTrace();
		}
	}

	private void menuItem21ActionPerformed(ActionEvent e) {
		
		try {
			if (!curSelectedEntityId.equals("")) {
			Proxy proxy = bm.getMemoryEntry(curSelectedEntityId, Proxy.class);
			Vector<Feature> feats = new Vector<Feature>();
			for (int i = 0 ; i < proxy.features.length ; i++) {
				if (!proxy.features[i].featlabel.equals("saliency"))
				feats.add(proxy.features[i]);
			}
			proxy.features = new Feature[feats.size()];
			proxy.features = feats.toArray(proxy.features);
			
			StringValue salient = ProxyConstructor.createStringValue("low", 1.0f);
			Feature feat = ProxyConstructor.createFeatureWithUniqueFeatureValue("saliency", salient);
			ProxyConstructor.addFeatureToProxy(proxy, feat);
			bm.overwriteWorkingMemory(proxy.entityID, Binder.BINDER_SA, proxy);	
			}
		}
		catch (Exception ex) {
			ex.printStackTrace();
		}
	}
	
	private void menuItem9ActionPerformed(ActionEvent e) {
		if(this.bn_gui == null) {
			this.bn_gui = new BayesianNetworkVisualizationGUI(bm);
			this.bn_gui.drawBayesianNetwork();
		}
		if(!this.bn_gui.isVisible()) {
			this.bn_gui.setVisible(true);
		}
	}
	
	private void initComponents() {
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - C E
		menuBar1 = new JMenuBar();
		menu1 = new JMenu();
		menuItem4 = new JMenuItem();
		menuItem1 = new JMenuItem();
		menuItem2 = new JMenuItem();
		menuItem6 = new JMenuItem();
		menuItem5 = new JMenuItem();
		menu2 = new JMenu();
		menu6 = new JMenu();
		menuItem11 = new JMenuItem();
		menuItem18 = new JMenuItem();
		menuItem7 = new JMenuItem();
		menuItem8 = new JMenuItem();
		menuItem20 = new JMenuItem();
		menuItem21 = new JMenuItem();
		menu7 = new JMenu();
		menuItem19 = new JMenuItem();
		menuItem3 = new JMenuItem();
		menuItem17 = new JMenuItem();
		menu3 = new JMenu();
		menuItem12 = new JMenuItem();
		menuItem13 = new JMenuItem();
		menuItem15 = new JMenuItem();
		menuItem16 = new JMenuItem();
		menu4 = new JMenu();
		menuItem9 = new JMenuItem();
		menuItem10 = new JMenuItem();
		menu5 = new JMenu();
		menuItem14 = new JMenuItem();

		//======== this ========
		setTitle("Binder GUI");
		Container contentPane = getContentPane();
		contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));

		//======== menuBar1 ========
		{

			//======== menu1 ========
			{
				menu1.setText("File");

				//---- menuItem4 ----
				menuItem4.setText("Restart");
				menuItem4.setEnabled(false);
				menu1.add(menuItem4);
				menu1.addSeparator();

				//---- menuItem1 ----
				menuItem1.setText("Import state");
				menuItem1.setEnabled(false);
				menu1.add(menuItem1);

				//---- menuItem2 ----
				menuItem2.setText("Export state");
				menuItem2.setEnabled(false);
				menu1.add(menuItem2);
				menu1.addSeparator();

				//---- menuItem6 ----
				menuItem6.setText("Snapshot");
				menuItem6.setEnabled(false);
				menu1.add(menuItem6);
				menu1.addSeparator();

				//---- menuItem5 ----
				menuItem5.setText("Exit");
				menuItem5.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem5ActionPerformed(e);
					}
				});
				menu1.add(menuItem5);
			}
			menuBar1.add(menu1);

			//======== menu2 ========
			{
				menu2.setText("Edit");

				//======== menu6 ========
				{
					menu6.setText("Insert");
					menu6.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/add2.png"));

					//---- menuItem11 ----
					menuItem11.setText("New Proxy");
					menuItem11.addActionListener(new ActionListener() {
						public void actionPerformed(ActionEvent e) {
							menuItem11ActionPerformed(e);
						}
					});
					menu6.add(menuItem11);

					//---- menuItem18 ----
					menuItem18.setText("New Feature in Proxy");
					menuItem18.setEnabled(false);
					menuItem18.addActionListener(new ActionListener() {
						public void actionPerformed(ActionEvent e) {
							menuItem18ActionPerformed(e);
						}
					});
					menu6.add(menuItem18);
				}
				menu2.add(menu6);

				//---- menuItem7 ----
				menuItem7.setText("Modify");
				menuItem7.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/modify2.png"));
				menuItem7.setEnabled(false);
				menuItem7.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem7ActionPerformed(e);
					}
				});
				menu2.add(menuItem7);

				//---- menuItem8 ----
				menuItem8.setText("Delete");
				menuItem8.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/delete2.png"));
				menuItem8.setEnabled(false);
				menuItem8.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem8ActionPerformed(e);
					}
				});
				menu2.add(menuItem8);
				menu2.addSeparator();

				//---- menuItem20 ----
				menuItem20.setText("Make salient");
				menuItem20.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem20ActionPerformed(e);
					}
				});
				menu2.add(menuItem20);

				//---- menuItem21 ----
				menuItem21.setText("Make not salient");
				menu2.add(menuItem21);
			}
			menuBar1.add(menu2);

			//======== menu7 ========
			{
				menu7.setText("View");

				//---- menuItem19 ----
				menuItem19.setText("Refresh");
				menuItem19.setEnabled(false);
				menuItem19.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/refresh.png"));
				menu7.add(menuItem19);
				menu7.addSeparator();

				//---- menuItem3 ----
				menuItem3.setText("Zoom in");
				menuItem3.setIcon(UIManager.getIcon("InternalFrame.maximizeIcon"));
				menuItem3.setEnabled(false);
				menu7.add(menuItem3);

				//---- menuItem17 ----
				menuItem17.setText("Zoom out");
				menuItem17.setIcon(UIManager.getIcon("InternalFrame.minimizeIcon"));
				menuItem17.setEnabled(false);
				menu7.add(menuItem17);
			}
			menuBar1.add(menu7);

			//======== menu3 ========
			{
				menu3.setText("Navigate");

				//---- menuItem12 ----
				menuItem12.setText("Previous state");
				menuItem12.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/db_arrow_left.png"));
				menuItem12.setEnabled(false);
				menu3.add(menuItem12);

				//---- menuItem13 ----
				menuItem13.setText("Next state");
				menuItem13.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/db_arrow_right.png"));
				menuItem13.setEnabled(false);
				menu3.add(menuItem13);
				menu3.addSeparator();

				//---- menuItem15 ----
				menuItem15.setText("Higher configuration");
				menuItem15.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/db_arrow_up.png"));
				menuItem15.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem15ActionPerformed(e);
					}
				});
				menu3.add(menuItem15);

				//---- menuItem16 ----
				menuItem16.setText("Lower configuration");
				menuItem16.setIcon(new ImageIcon("/Users/plison/svn.cogx/systems/dfki/subarchitectures/binder/imgs/db_arrow_down.png"));
				menuItem16.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem16ActionPerformed(e);
					}
				});
				menu3.add(menuItem16);
			}
			menuBar1.add(menu3);

			//======== menu4 ========
			{
				menu4.setText("Settings");

				//---- menuItem9 ----
				menuItem9.setText("View Bayesian Network");
				menuItem9.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem9ActionPerformed(e);
					}
				});
				menu4.add(menuItem9);

				//---- menuItem10 ----
				menuItem10.setText("Binding parameters");
				menuItem10.setEnabled(false);
				menu4.add(menuItem10);
			}
			menuBar1.add(menu4);

			//======== menu5 ========
			{
				menu5.setText("Help");

				//---- menuItem14 ----
				menuItem14.setText("About the CogX Binder");
				menuItem14.setEnabled(false);
				menu5.add(menuItem14);
			}
			menuBar1.add(menu5);
		}
		setJMenuBar(menuBar1);
		pack();
		setLocationRelativeTo(getOwner());
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
		setSize(1000, 700);
		setVisible(true);
	}

	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - C E
	private JMenuBar menuBar1;
	private JMenu menu1;
	private JMenuItem menuItem4;
	private JMenuItem menuItem1;
	private JMenuItem menuItem2;
	private JMenuItem menuItem6;
	private JMenuItem menuItem5;
	private JMenu menu2;
	private JMenu menu6;
	private JMenuItem menuItem11;
	private JMenuItem menuItem18;
	private JMenuItem menuItem7;
	private JMenuItem menuItem8;
	private JMenuItem menuItem20;
	private JMenuItem menuItem21;
	private JMenu menu7;
	private JMenuItem menuItem19;
	private JMenuItem menuItem3;
	private JMenuItem menuItem17;
	private JMenu menu3;
	private JMenuItem menuItem12;
	private JMenuItem menuItem13;
	private JMenuItem menuItem15;
	private JMenuItem menuItem16;
	private JMenu menu4;
	private JMenuItem menuItem9;
	private JMenuItem menuItem10;
	private JMenu menu5;
	private JMenuItem menuItem14;
	// JFormDesigner - End of variables declaration  //GEN-END:variables
	
	private void log(String s) {
		if (LOGGING)
			logger.debug("[BinderMonitorGUI]" + s);
	}
}
