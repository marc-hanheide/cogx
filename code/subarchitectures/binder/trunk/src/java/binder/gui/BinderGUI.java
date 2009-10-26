
package binder.gui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import org.apache.log4j.Logger;

import binder.components.BinderMonitor;
import binder.utils.GenericUtils;

import binder.gui.ProxyInfoGUI;

import cast.core.logging.ComponentLogger;

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
	
	
	public boolean LOGGING = false;

	private static Logger logger = ComponentLogger.getLogger(BinderGUI.class);
	
	BinderMonitor bm;
	 	
	int config_CurrentRank = 1;

	String curSelectedEntity = "";
	
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
	
	
	public void setCurSelectedEntity (String entity) {
		curSelectedEntity = entity;
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

	private void menuItem8ActionPerformed(ActionEvent e) {
		
		if (!curSelectedEntity.equals("")) {
		//	System.out.println("GOING TO DELETE: " + curSelectedEntity);
			try {
			bm.deleteFromWorkingMemory(curSelectedEntity);
			}
			catch (Exception ex) {
				ex.printStackTrace();
			}
		}
		
	}

	private void menuItem5ActionPerformed(ActionEvent e) {
		dispose();
	}
	
	
	private void initComponents() {
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - Pierre Lison
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
		menuItem7 = new JMenuItem();
		menuItem8 = new JMenuItem();
		menu7 = new JMenu();
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

					//---- menuItem11 ----
					menuItem11.setText("New Proxy");
					menuItem11.addActionListener(new ActionListener() {
						public void actionPerformed(ActionEvent e) {
							menuItem11ActionPerformed(e);
						}
					});
					menu6.add(menuItem11);
				}
				menu2.add(menu6);

				//---- menuItem7 ----
				menuItem7.setText("Modify");
				menuItem7.setEnabled(false);
				menu2.add(menuItem7);

				//---- menuItem8 ----
				menuItem8.setText("Delete");
				menuItem8.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						menuItem8ActionPerformed(e);
					}
				});
				menu2.add(menuItem8);
			}
			menuBar1.add(menu2);

			//======== menu7 ========
			{
				menu7.setText("View");

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
				menuItem9.setEnabled(false);
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
	// Generated using JFormDesigner Evaluation license - Pierre Lison
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
	private JMenuItem menuItem7;
	private JMenuItem menuItem8;
	private JMenu menu7;
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
